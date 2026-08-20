import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import String


def yaw_to_quaternion(yaw: float) -> Quaternion:
    quaternion = Quaternion()
    quaternion.z = math.sin(yaw / 2.0)
    quaternion.w = math.cos(yaw / 2.0)
    return quaternion


class CloudNavigator(BasicNavigator):
    def __init__(self):
        super().__init__(node_name="smart_carrier_navigator")
        self.pending_task = None
        self.result_publisher = self.create_publisher(String, "/smart_carrier/task_result", 10)
        self.create_subscription(String, "/smart_carrier/task", self.on_task, 10)

    def on_task(self, message: String) -> None:
        if self.pending_task is not None:
            self.get_logger().warning("Ignoring task while another task is pending")
            return
        try:
            self.pending_task = json.loads(message.data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Invalid task JSON: {exc}")

    def execute_pending(self) -> None:
        task = self.pending_task
        if not task:
            return
        self.pending_task = None
        location = task.get("location") or {}
        if location.get("x") is None or location.get("y") is None:
            self.publish_result(task["id"], "failed", "Location has no map coordinates")
            return

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(location["x"])
        goal.pose.position.y = float(location["y"])
        goal.pose.orientation = yaw_to_quaternion(float(location.get("yaw") or 0.0))
        self.get_logger().info(f"Navigating task {task['id']} to {task['location_code']}")
        self.goToPose(goal)
        while not self.isTaskComplete() and rclpy.ok():
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.05)

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.publish_result(task["id"], "done", "Nav2 goal reached")
        elif result == TaskResult.CANCELED:
            self.publish_result(task["id"], "failed", "Nav2 goal cancelled")
        else:
            self.publish_result(task["id"], "failed", "Nav2 goal failed")

    def publish_result(self, task_id: str, status: str, note: str) -> None:
        message = String()
        message.data = json.dumps({"task_id": task_id, "status": status, "note": note})
        self.result_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    navigator = CloudNavigator()
    navigator.get_logger().info("Waiting for Nav2")
    navigator.waitUntilNav2Active()
    try:
        while rclpy.ok():
            rclpy.spin_once(navigator, timeout_sec=0.2)
            navigator.execute_pending()
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()
