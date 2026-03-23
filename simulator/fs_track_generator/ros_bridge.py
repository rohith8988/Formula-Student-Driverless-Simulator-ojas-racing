"""Helpers for writing ros_gz_bridge config files.

The bridge is split into two nodes that run on separate OS threads:
  - control bridge: low-bandwidth state/command topics (cmd_vel, odom, tf, joints, imu, clock)
  - sensors bridge: high-bandwidth sensor topics (lidar scan, point cloud, camera)

This prevents heavy point-cloud/image serialisation from blocking control data.
"""

from __future__ import annotations

from pathlib import Path


def build_control_bridge_config(*, world_name: str, model_name: str) -> str:
    """Low-bandwidth control/state topics. Only the joint_state gz topic is dynamic."""

    joint_state_topic = f"/world/{world_name}/model/{model_name}/joint_state"

    return f"""# Auto-generated ros_gz_bridge control topics
- ros_topic_name: "/cmd_vel"
  gz_topic_name:  "/cmd_vel"
  ros_type_name:  "geometry_msgs/msg/Twist"
  gz_type_name:   "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "/odometry/wheels"
  gz_topic_name:  "/odometry/wheels"
  ros_type_name:  "nav_msgs/msg/Odometry"
  gz_type_name:   "gz.msgs.Odometry"
  direction: GZ_TO_ROS

- ros_topic_name: "/tf"
  gz_topic_name:  "/tf"
  ros_type_name:  "tf2_msgs/msg/TFMessage"
  gz_type_name:   "gz.msgs.Pose_V"
  direction: GZ_TO_ROS

- ros_topic_name: "/joint_states"
  gz_topic_name:  "{joint_state_topic}"
  ros_type_name:  "sensor_msgs/msg/JointState"
  gz_type_name:   "gz.msgs.Model"
  direction: GZ_TO_ROS

- ros_topic_name: "/imu/data"
  gz_topic_name:  "/imu/data"
  ros_type_name:  "sensor_msgs/msg/Imu"
  gz_type_name:   "gz.msgs.IMU"
  direction: GZ_TO_ROS

- ros_topic_name: "/clock"
  gz_topic_name:  "/clock"
  ros_type_name:  "rosgraph_msgs/msg/Clock"
  gz_type_name:   "gz.msgs.Clock"
  direction: GZ_TO_ROS
"""


# Sensor topics never change — fully static, no world/model substitution needed.
SENSORS_BRIDGE_CONFIG = """# Auto-generated ros_gz_bridge sensor topics
- ros_topic_name: "/scan"
  gz_topic_name:  "/lidar/points"
  ros_type_name:  "sensor_msgs/msg/LaserScan"
  gz_type_name:   "gz.msgs.LaserScan"
  direction: GZ_TO_ROS

- ros_topic_name: "/lidar/points"
  gz_topic_name:  "/lidar/points/points"
  ros_type_name:  "sensor_msgs/msg/PointCloud2"
  gz_type_name:   "gz.msgs.PointCloudPacked"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/image_raw"
  gz_topic_name:  "/camera/image_raw"
  ros_type_name:  "sensor_msgs/msg/Image"
  gz_type_name:   "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/camera_info"
  gz_topic_name:  "/camera/camera_info"
  ros_type_name:  "sensor_msgs/msg/CameraInfo"
  gz_type_name:   "gz.msgs.CameraInfo"
  direction: GZ_TO_ROS
"""


def write_bridge_configs(
    control_path: Path,
    sensors_path: Path,
    *,
    world_name: str,
    model_name: str,
) -> None:
    """Write both bridge config files to disk."""
    control_path.write_text(
        build_control_bridge_config(world_name=world_name, model_name=model_name),
        encoding="utf-8",
    )
    sensors_path.write_text(SENSORS_BRIDGE_CONFIG, encoding="utf-8")


# Backward-compat aliases
def build_bridge_config(*, world_name: str, model_name: str) -> str:
    return build_control_bridge_config(world_name=world_name, model_name=model_name)


def write_bridge_config(path: Path, *, world_name: str, model_name: str) -> None:
    path.write_text(
        build_control_bridge_config(world_name=world_name, model_name=model_name),
        encoding="utf-8",
    )
