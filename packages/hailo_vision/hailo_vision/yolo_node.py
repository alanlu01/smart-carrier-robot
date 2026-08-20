import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import os
from ament_index_python.packages import get_package_share_directory
from hailo_platform import HEF, VDevice, InferVStreams, InputVStreamParams, OutputVStreamParams, FormatType

# Hailo 專屬套件 (4.24.0 最終版寫法)
from hailo_platform import HEF, VDevice, InferVStreams, InputVStreamParams, OutputVStreamParams

# YOLOv8 設定參數
NUM_CLASSES = 11  
CONF_THRES = 0.5  
IOU_THRES = 0.45

def sigmoid(x):
    return 1 / (1 + np.exp(-x))

def dfl_decode(bbox_tensor, reg_max=16):
    """將 Hailo 吐出的 64 通道張量，透過 Softmax 與積分還原成 [x, y, w, h]"""
    batch, grid_y, grid_x, channels = bbox_tensor.shape
    bbox_tensor = bbox_tensor.reshape(batch, grid_y, grid_x, 4, reg_max)
    exp_x = np.exp(bbox_tensor - np.max(bbox_tensor, axis=-1, keepdims=True))
    softmax_x = exp_x / np.sum(exp_x, axis=-1, keepdims=True)
    weights = np.arange(reg_max, dtype=np.float32)
    decoded = np.sum(softmax_x * weights, axis=-1) 
    return decoded

def process_hailo_outputs(infer_results, original_shape):
    """處理 6 個輸出矩陣並執行 NMS"""
    bbox_names = ['custom_yolov8s/conv41', 'custom_yolov8s/conv52', 'custom_yolov8s/conv62']
    cls_names  = ['custom_yolov8s/conv42', 'custom_yolov8s/conv53', 'custom_yolov8s/conv63']
    strides = [8, 16, 32]
    
    boxes, scores, class_ids = [], [], []
    
    for i, stride in enumerate(strides):
        # 👇 加上 .astype(np.float32) 強制轉型，避免無號整數運算溢位
        bbox_pred = infer_results[bbox_names[i]][0].astype(np.float32)
        cls_pred = infer_results[cls_names[i]][0].astype(np.float32)
        
        # 👇 補上這行：將原始分數壓縮成 0~1 的機率！
        cls_pred = sigmoid(cls_pred)
        
        max_scores = np.max(cls_pred, axis=-1)
        valid_mask = max_scores > CONF_THRES
        
        if not np.any(valid_mask):
            continue
            
        valid_cls = cls_pred[valid_mask]
        valid_bbox = bbox_pred[valid_mask]
        valid_scores = max_scores[valid_mask]
        valid_class_ids = np.argmax(valid_cls, axis=-1)
        
        decoded_bbox = dfl_decode(valid_bbox.reshape(1, -1, 1, 64))[0, :, 0, :]
        grid_y, grid_x = np.where(valid_mask)
        grid_coords = np.stack((grid_x, grid_y), axis=-1).astype(np.float32) + 0.5
        
        lt = decoded_bbox[:, :2]
        rb = decoded_bbox[:, 2:]
        x1y1 = (grid_coords - lt) * stride
        x2y2 = (grid_coords + rb) * stride
        
        w = x2y2[:, 0] - x1y1[:, 0]
        h = x2y2[:, 1] - x1y1[:, 1]
        x = x1y1[:, 0]
        y = x1y1[:, 1]
        
        for j in range(len(x)):
            # 👇 加上這行防護罩，如果萬一算出 NaN 就跳過這個框
            if np.isnan(x[j]) or np.isnan(y[j]) or np.isnan(w[j]) or np.isnan(h[j]):
                continue
            
            boxes.append([int(x[j]), int(y[j]), int(w[j]), int(h[j])])
            scores.append(float(valid_scores[j]))
            class_ids.append(int(valid_class_ids[j]))
            
    indices = cv2.dnn.NMSBoxes(boxes, scores, CONF_THRES, IOU_THRES)
    
    final_results = []
    scale_x = original_shape[1] / 640.0
    scale_y = original_shape[0] / 640.0
    
    if len(indices) > 0:
        for i in indices.flatten():
            box = boxes[i]
            x = int(box[0] * scale_x)
            y = int(box[1] * scale_y)
            w = int(box[2] * scale_x)
            h = int(box[3] * scale_y)
            final_results.append((x, y, w, h, scores[i], class_ids[i]))
            
    return final_results

class HailoYoloNode(Node):
    def __init__(self, infer_pipeline, network_group):
        super().__init__('yolo_node')
        self.infer_pipeline = infer_pipeline
        self.input_name = network_group.get_input_vstream_infos()[0].name
        self.bridge = CvBridge()
        
        # 訂閱相機畫面 (對應 camera_ros 的預設話題)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        
        # 發布畫好綠色辨識框的畫面
        self.publisher_ = self.create_publisher(Image, '/hailo_vision/image_result', 10)
        self.get_logger().info('🚀 Hailo YOLOv8 ROS 2 節點已成功啟動並等待影像...')
        
        self.frame_count = 0
        self.start_time = time.time()
        self.fps = 0.0

    def image_callback(self, msg):
        # 1. 將 ROS 影像轉成 OpenCV 格式
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        original_shape = frame.shape
        
        # 2. 影像前處理
        input_frame = cv2.resize(frame, (640, 640))
        input_frame = cv2.cvtColor(input_frame, cv2.COLOR_BGR2RGB)
        input_data = {self.input_name: np.expand_dims(input_frame, axis=0)}
        
        # 3. Hailo-8 硬體推論
        infer_results = self.infer_pipeline.infer(input_data)
        
        # 4. CPU 解碼與畫框
        detections = process_hailo_outputs(infer_results, original_shape)
        
        for (x, y, w, h, score, cls_id) in detections:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, f"ID:{cls_id} {score:.2f}", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # 算一下 FPS 並印在畫面上
        self.frame_count += 1
        if time.time() - self.start_time >= 1.0:
            self.fps = self.frame_count / (time.time() - self.start_time)
            self.frame_count = 0
            self.start_time = time.time()
        
        cv2.putText(frame, f"ROS 2 FPS: {self.fps:.1f}", (20, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # 5. 將畫好的矩陣轉回 ROS 影像並發布出去
        result_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.publisher_.publish(result_msg)

def main(args=None):
    rclpy.init(args=args)
    
    # 自動尋找 ROS 2 安裝目錄下的模型檔案
    package_share_dir = get_package_share_directory('hailo_vision')
    hef_path = os.path.join(package_share_dir, 'models', 'best.hef')
    hef = HEF(hef_path)
    
    with VDevice() as target:
        network_group = target.configure(hef)[0]
        input_params = InputVStreamParams.make(network_group)
        # 👇 加上 format_type=FormatType.FLOAT32
        output_params = OutputVStreamParams.make(network_group, format_type=FormatType.FLOAT32)
        
        # 👇 關鍵修復：取得啟動參數並「正式喚醒/啟動」神經網路
        network_group_params = network_group.create_params()
        with network_group.activate(network_group_params):
            
            # 在已啟動的神經網路內，建立推論管線並執行 ROS 2 節點
            with InferVStreams(network_group, input_params, output_params) as infer_pipeline:
                node = HailoYoloNode(infer_pipeline, network_group)
                try:
                    rclpy.spin(node)
                except KeyboardInterrupt:
                    pass
                
                node.destroy_node()
                
    rclpy.shutdown()

if __name__ == '__main__':
    main()

