#!/usr/bin/env python3
"""
Pick and place node combining Cartesian and joint-space moves with smooth joint transitions.
Locks the detected color coordinates before starting the motion.
Now supports picking boxes at any yaw angle by rotating the gripper to match.

ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=R
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=G
ros2 run pymoveit2 pick_and_place.py --ros-args -p target_color:=B
"""

from threading import Thread
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String

from pymoveit2 import MoveIt2, GripperInterface
from pymoveit2.robots import panda

import math

# CHANGE 1: imported Rotation so we can convert yaw angle -> quaternion
from scipy.spatial.transform import Rotation as R


class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")

        # Parameters
        self.declare_parameter("target_color", "R")
        self.target_color = self.get_parameter("target_color").value.upper()

        self.declare_parameter("approach_offset", 0.31)
        self.approach_offset = float(
            self.get_parameter("approach_offset").value
        )

        # Flags
        self.already_moved = False
        self.target_coords = None

        self.callback_group = ReentrantCallbackGroup()

        # Arm MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )

        self.moveit2.max_velocity = 0.1
        self.moveit2.max_acceleration = 0.1

        # Gripper interface
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=panda.gripper_joint_names(),
            open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=panda.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=panda.MOVE_GROUP_GRIPPER,
            callback_group=self.callback_group,
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )

        # Subscriber
        self.sub = self.create_subscription(
            String, "/color_coordinates", self.coords_callback, 10
        )
        self.get_logger().info(f"Waiting for {self.target_color} from /color_coordinates...")

        # Predefined joint positions (in radians)
        self.start_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, math.radians(-125.0)]
        self.home_joints  = [0.0, 0.0, 0.0, math.radians(-90.0), 0.0, math.radians(92.0), math.radians(50.0)]
        self.drop_joints  = [math.radians(-155.0), math.radians(30.0), math.radians(-20.0),
                             math.radians(-124.0), math.radians(44.0), math.radians(163.0), math.radians(7.0)]

        # Move to start joint configuration
        self.moveit2.move_to_configuration(self.start_joints)
        self.moveit2.wait_until_executed()

  
    def yaw_to_quat(self, yaw_rad):
        base = R.from_quat([0.0, 1.0, 0.0, 0.0])  # xyzw format

        
        extra = R.from_euler('z', yaw_rad)
        combined = extra * base

        # Return as [x, y, z, w] list that MoveIt2 expects
        q = combined.as_quat()  
        return [float(q[0]), float(q[1]), float(q[2]), float(q[3])]

    def coords_callback(self, msg):
        if self.already_moved:
            return

        try:
            
            parts = msg.data.split(",")

            if len(parts) == 5:
                color_id, x, y, z, yaw = parts
                box_yaw_rad = float(yaw)
            elif len(parts) == 4:
                
                color_id, x, y, z = parts
                box_yaw_rad = 0.0
                self.get_logger().warn("Received 4-value message (no angle). Using yaw=0.")
            else:
                self.get_logger().error(f"Unexpected message format: {msg.data}")
                return

            color_id = color_id.strip().upper()

            if color_id == self.target_color:
                self.target_coords = [float(x), float(y), float(z)]
                self.get_logger().info(
                    f"Target {self.target_color} locked at: "
                    f"[{self.target_coords[0]:.3f}, {self.target_coords[1]:.3f}, {self.target_coords[2]:.3f}] "
                    f"yaw={math.degrees(box_yaw_rad):.1f} deg"
                )
                self.already_moved = True

                pick_position = [
                    self.target_coords[0],
                    self.target_coords[1],
                    self.target_coords[2] - 0.60
                ]

               
                # NEW:
                quat_xyzw = self.yaw_to_quat(box_yaw_rad)
                self.get_logger().info(f"Gripper quaternion: {[f'{v:.3f}' for v in quat_xyzw]}")

                # --- Pick-and-place sequence (identical steps, just uses rotated quat now) ---

                # 1. Move to home joint configuration
                self.moveit2.move_to_configuration(self.home_joints)
                self.moveit2.wait_until_executed()

                # 2. Move above target (Cartesian)
                self.moveit2.move_to_pose(position=pick_position, quat_xyzw=quat_xyzw)
                self.moveit2.wait_until_executed()

                # 3. Open gripper
                self.gripper.open()
                self.gripper.wait_until_executed()

                # 4. Move down to approach object
                approach_position = [
                    pick_position[0],
                    pick_position[1],
                    pick_position[2] - self.approach_offset
                ]
                self.moveit2.move_to_pose(
                    position=approach_position,
                    quat_xyzw=quat_xyzw,   # same rotated quaternion used here too
                    cartesian=True
                )
                self.moveit2.wait_until_executed()

                # 5. Close gripper
                self.gripper.close()
                self.gripper.wait_until_executed()

                # 6. Move to home joint configuration
                self.moveit2.move_to_configuration(self.home_joints)
                self.moveit2.wait_until_executed()

                # 7. Move to drop joint configuration
                self.moveit2.move_to_configuration(self.drop_joints)
                self.moveit2.wait_until_executed()

                # 8. Open gripper to release
                self.gripper.open()
                self.gripper.wait_until_executed()

                # 9. Close gripper
                self.gripper.close()
                self.gripper.wait_until_executed()

                # 10. Return to start joint configuration
                self.moveit2.move_to_configuration(self.start_joints)
                self.moveit2.wait_until_executed()

                self.get_logger().info("Pick-and-place sequence complete.")
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error parsing /color_coordinates: {e}")


def main():
    rclpy.init()
    node = PickAndPlace()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        executor_thread.join()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()