import os

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from smart_carrier_robot.api_client import ApiError, SmartCarrierApi
from smart_carrier_robot.map_validation import validate_location
from smart_carrier_robot.ros_map import occupancy_map_from_message


class LocationValidatorNode(Node):
    def __init__(self):
        super().__init__("smart_carrier_location_validator")
        self.declare_parameter(
            "api_url", os.getenv("SMART_CARRIER_API_URL", "http://127.0.0.1:8000")
        )
        self.declare_parameter("robot_id", os.getenv("SMART_CARRIER_ROBOT_ID", "R1"))
        self.declare_parameter("robot_token", os.getenv("SMART_CARRIER_ROBOT_TOKEN", ""))
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("clearance_m", 0.35)
        self.declare_parameter("max_cost", 0)

        self.api = SmartCarrierApi(
            self.get_parameter("api_url").value,
            self.get_parameter("robot_id").value,
            self.get_parameter("robot_token").value,
        )
        self.validated = False
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self.on_map,
            qos,
        )

    def on_map(self, message: OccupancyGrid) -> None:
        if self.validated:
            return
        try:
            locations = self.api.list_locations()
        except ApiError as exc:
            self.get_logger().error(f"Unable to load cloud locations: {exc}")
            return

        occupancy_map = occupancy_map_from_message(message)
        clearance_m = float(self.get_parameter("clearance_m").value)
        max_cost = int(self.get_parameter("max_cost").value)
        failures = 0
        for location in locations:
            code = location["code"]
            if location.get("x") is None or location.get("y") is None:
                failures += 1
                self.get_logger().error(f"{code}: missing x/y coordinates")
                continue
            result = validate_location(
                occupancy_map,
                location["x"],
                location["y"],
                clearance_m=clearance_m,
                max_cost=max_cost,
            )
            message_text = (
                f"{code} ({location['x']:.3f}, {location['y']:.3f}, "
                f"yaw={float(location.get('yaw') or 0.0):.3f}): {result.reason}"
            )
            if result.valid:
                self.get_logger().info(f"PASS {message_text}")
            else:
                failures += 1
                self.get_logger().error(f"FAIL {message_text}")

        self.validated = True
        if failures:
            self.get_logger().error(
                f"Location validation failed: {failures}/{len(locations)} location(s) unsafe"
            )
        else:
            self.get_logger().info(f"Location validation passed: {len(locations)} location(s)")


def main(args=None):
    rclpy.init(args=args)
    node = LocationValidatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
