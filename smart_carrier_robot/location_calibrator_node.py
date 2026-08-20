import json

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from smart_carrier_robot.map_validation import OccupancyMap, quaternion_to_yaw, validate_location
from smart_carrier_robot.ros_map import occupancy_map_from_message


class LocationCalibratorNode(Node):
    def __init__(self):
        super().__init__("smart_carrier_location_calibrator")
        self.declare_parameter("location_code", "")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("clearance_m", 0.35)
        self.declare_parameter("max_cost", 0)

        self.location_code = str(self.get_parameter("location_code").value).strip().upper()
        if not self.location_code:
            raise ValueError("location_code is required")

        self.occupancy_map: OccupancyMap | None = None
        self.map_frame = "map"
        self.done = False
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self.on_map,
            map_qos,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("goal_topic").value,
            self.on_goal,
            10,
        )
        self.get_logger().info(
            f"Waiting for RViz 2D Goal Pose for {self.location_code}; "
            "unsafe targets will be rejected"
        )

    def on_map(self, message: OccupancyGrid) -> None:
        self.occupancy_map = occupancy_map_from_message(message)
        self.map_frame = message.header.frame_id or "map"
        self.get_logger().info(
            f"Loaded {self.map_frame} occupancy grid "
            f"{message.info.width}x{message.info.height} at {message.info.resolution:.3f} m/px"
        )

    def on_goal(self, message: PoseStamped) -> None:
        if self.done:
            return
        if self.occupancy_map is None:
            self.get_logger().error("Map has not arrived yet; wait for /map and click again")
            return
        if message.header.frame_id != self.map_frame:
            self.get_logger().error(
                f"Goal frame {message.header.frame_id!r} does not match map frame "
                f"{self.map_frame!r}"
            )
            return

        pose = message.pose
        x = float(pose.position.x)
        y = float(pose.position.y)
        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        result = validate_location(
            self.occupancy_map,
            x,
            y,
            clearance_m=float(self.get_parameter("clearance_m").value),
            max_cost=int(self.get_parameter("max_cost").value),
        )
        if not result.valid:
            self.get_logger().error(
                f"REJECT {self.location_code} ({x:.3f}, {y:.3f}): {result.reason}"
            )
            return

        calibration = {
            "code": self.location_code,
            "x": round(x, 3),
            "y": round(y, 3),
            "yaw": round(yaw, 6),
        }
        self.get_logger().info(f"CALIBRATION {json.dumps(calibration, separators=(',', ':'))}")
        self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = LocationCalibratorNode()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
