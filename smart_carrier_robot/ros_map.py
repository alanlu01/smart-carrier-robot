from nav_msgs.msg import OccupancyGrid

from smart_carrier_robot.map_validation import OccupancyMap, quaternion_to_yaw


def occupancy_map_from_message(message: OccupancyGrid) -> OccupancyMap:
    origin = message.info.origin
    return OccupancyMap(
        width=message.info.width,
        height=message.info.height,
        resolution=message.info.resolution,
        origin_x=origin.position.x,
        origin_y=origin.position.y,
        origin_yaw=quaternion_to_yaw(
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        ),
        data=message.data,
    )
