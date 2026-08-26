import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import json
import os
from ament_index_python.packages import get_package_share_directory
from hailo_platform import HEF, VDevice, InferVStreams, InputVStreamParams, OutputVStreamParams, FormatType

from hailo_vision.semantic_protocol import build_semantic_payload

# 分類字典
CLASS_NAMES = {
    0: 'Person', 1: 'Cart', 2: 'Stroller', 3: 'Wheelchair', 4: 'Wheelchair_Person',
    5: 'Basket', 6: 'Trash_Can', 7: 'Sign', 8: 'Chair', 9: 'Table', 10: 'Glass'
}

CONF_THRES = 0.5  
IOU_THRES = 0.45  
CAMERA_HFOV = 67.5  # 🌟 更新為更精準的光學水平視角 67.5 度

def sigmoid(x):
    return 1 / (1 + np.exp(-x))

def dfl_decode(bbox_tensor, reg_max=16):
    batch, grid_y, grid_x, channels = bbox_tensor.shape
    bbox_tensor = bbox_tensor.reshape(batch, grid_y, grid_x, 4, reg_max)
    exp_x = np.exp(bbox_tensor - np.max(bbox_tensor, axis=-1, keepdims=True))
    softmax_x = exp_x / np.sum(exp_x, axis=-1, keepdims=True)
    weights = np.arange(reg_max, dtype=np.float32)
    decoded = np.sum(softmax_x * weights, axis=-1) 
    return decoded

def process_hailo_outputs(infer_results, original_shape):
    bbox_names = ['custom_yolov8s/conv41', 'custom_yolov8s/conv52', 'custom_yolov8s/conv62']
    cls_names  = ['custom_yolov8s/conv42', 'custom_yolov8s/conv53', 'custom_yolov8s/conv63']
    strides = [8, 16, 32]
    boxes, scores, class_ids = [], [], []
    
    for i, stride in enumerate(strides):
        bbox_pred = infer_results[bbox_names[i]][0].astype(np.float32)
        cls_pred = infer_results[cls_names[i]][0].astype(np.float32)   
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

class SemanticVisionNode(Node):
    def __init__(self, infer_pipeline, network_group):
        super().__init__('semantic_node')
        self.infer_pipeline = infer_pipeline
        self.input_name = network_group.get_input_vstream_infos()[0].name
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, '/hailo_vision/semantic_image', 10)
        self.info_pub = self.create_publisher(String, '/vision/semantic_info', 10)
        
        self.get_logger().info('🚀 Hailo 語意導航大腦啟動！開始鎖定目標方位...')
        self.frame_count = 0
        self.sequence = 0
        self.start_time = time.time()
        self.fps = 0.0
        
        # 🌟 瘦身機制：影像發布計數器
        self.image_pub_counter = 0
        self.IMAGE_PUB_INTERVAL = 3  # 每 3 幀才發布一次影像

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        original_shape = frame.shape
        img_w = original_shape[1]
        
        input_frame = cv2.resize(frame, (640, 640))
        input_frame = cv2.cvtColor(input_frame, cv2.COLOR_BGR2RGB)
        input_data = {self.input_name: np.expand_dims(input_frame, axis=0)}
        
        infer_results = self.infer_pipeline.infer(input_data)
        detections = process_hailo_outputs(infer_results, original_shape)
        
        semantic_data_list = []
        
        # 決定這一幀是否要繪圖與發布
        self.image_pub_counter += 1
        should_publish_image = (self.image_pub_counter % self.IMAGE_PUB_INTERVAL == 0)
        
        for (x, y, w, h, score, cls_id) in detections:
            class_name = CLASS_NAMES.get(cls_id, f"Unknown_{cls_id}")
            
            if cls_id == 10 and score < 0.80:
                continue 
            
            x_center = x + (w / 2)
            angle = ((x_center / img_w) - 0.5) * CAMERA_HFOV
            
            target_info = {
                "id": cls_id,
                "class": class_name,
                "angle": round(angle, 2),
                "bbox_width": w,
                "score": round(score, 2)
            }
            semantic_data_list.append(target_info)
            
            # 🌟 只有在應該發布的幀數，才消耗 CPU 資源去畫框框
            if should_publish_image:
                color = (0, 0, 255) if cls_id == 10 else ((255, 0, 0) if cls_id in [0, 2, 3, 4] else (0, 255, 0))
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
                label = f"{class_name} | {angle:+.1f} deg"
                cv2.putText(frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                cv2.circle(frame, (int(x_center), y + h), 5, (0, 255, 255), -1)

        # 🌟 JSON 情報是關鍵神經，每一幀都必須全速發布！
        self.sequence += 1
        semantic_payload = build_semantic_payload(
            semantic_data_list,
            stamp_sec=msg.header.stamp.sec,
            stamp_nanosec=msg.header.stamp.nanosec,
            frame_id=msg.header.frame_id,
            sequence=self.sequence,
        )
        msg_str = String()
        msg_str.data = json.dumps(semantic_payload)
        self.info_pub.publish(msg_str)
        
        # 計算 FPS
        self.frame_count += 1
        if time.time() - self.start_time >= 1.0:
            self.fps = self.frame_count / (time.time() - self.start_time)
            self.frame_count = 0
            self.start_time = time.time()
        
        # 🌟 只有符合區間，才把影像壓縮並發布出去
        if should_publish_image:
            cv2.putText(frame, f"FPS: {self.fps:.1f}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            result_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            result_msg.header = msg.header
            self.image_pub.publish(result_msg)

def main(args=None):
    rclpy.init(args=args)
    package_share_dir = get_package_share_directory('hailo_vision')
    hef_path = os.path.join(package_share_dir, 'models', 'best.hef')
    hef = HEF(hef_path)
    
    with VDevice() as target:
        network_group = target.configure(hef)[0]
        input_params = InputVStreamParams.make(network_group)
        output_params = OutputVStreamParams.make(network_group, format_type=FormatType.FLOAT32)
        
        network_group_params = network_group.create_params()
        with network_group.activate(network_group_params):
            with InferVStreams(network_group, input_params, output_params) as infer_pipeline:
                node = SemanticVisionNode(infer_pipeline, network_group)
                try:
                    rclpy.spin(node)
                except KeyboardInterrupt:
                    pass
                node.destroy_node()
                
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()
