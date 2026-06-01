#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    PlanningOptions,
    MoveItErrorCodes,
    JointConstraint,
)

from control_msgs.action import GripperCommand


class PandaPlaceBlock(Node):
    def __init__(self):
        super().__init__("panda_place_block")

        self._action_client = ActionClient(self, MoveGroup, "/move_action")

        self._left_gripper_client = ActionClient(
            self,
            GripperCommand,
            "/panda_handleft_controller/gripper_cmd",
        )
        self._right_gripper_client = ActionClient(
            self,
            GripperCommand,
            "/panda_handright_controller/gripper_cmd",
        )

        self.stage = "lift"

        # home 位
        self.home_joint_values = {
            "panda_joint1": 0.0,
            "panda_joint2": -0.785,
            "panda_joint3": 0.0,
            "panda_joint4": -2.356,
            "panda_joint5": 0.0,
            "panda_joint6": 1.571,
            "panda_joint7": 0.785,
        }

        # 固定末端姿态：沿用你抓取成功那组
        self.grasp_qx = 1.0
        self.grasp_qy = 0.0
        self.grasp_qz = 0.0
        self.grasp_qw = 0.0

        # 放置点参数（你自己改）
        self.place_x = 0.20
        self.place_y = -0.30
        self.lift_z = 0.95
        self.place_pre_z = 0.93
        self.place_z = 0.83
        self.retreat_z = 0.95

        self.waiting_gripper_action = None
        self.gripper_done_count = 0

    def send_goal(self):
        self.wait_for_servers()
        self.move_to_named_pose(0.4, 0.4, self.lift_z, "lift")

    def wait_for_servers(self):
        self.get_logger().info("Waiting for move_group action server...")
        self._action_client.wait_for_server()

        self.get_logger().info("Waiting for left gripper action server...")
        self._left_gripper_client.wait_for_server()

        self.get_logger().info("Waiting for right gripper action server...")
        self._right_gripper_client.wait_for_server()

    def move_to_named_pose(self, x, y, z, stage_name):
        goal_msg = MoveGroup.Goal()

        goal_msg.request.group_name = "panda_arm"
        goal_msg.request.num_planning_attempts = 20
        goal_msg.request.allowed_planning_time = 5.0

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "world"
        position_constraint.link_name = "panda_link8"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.005, 0.005, 0.005]

        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.w = 1.0

        bounding_volume = BoundingVolume()
        bounding_volume.primitives.append(box)
        bounding_volume.primitive_poses.append(target_pose)

        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0

        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "world"
        orientation_constraint.link_name = "panda_link8"
        orientation_constraint.orientation.x = self.grasp_qx
        orientation_constraint.orientation.y = self.grasp_qy
        orientation_constraint.orientation.z = self.grasp_qz
        orientation_constraint.orientation.w = self.grasp_qw
        orientation_constraint.absolute_x_axis_tolerance = 0.05
        orientation_constraint.absolute_y_axis_tolerance = 0.05
        orientation_constraint.absolute_z_axis_tolerance = 0.05
        orientation_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)
        goal_msg.request.goal_constraints.append(constraints)

        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = False

        self.stage = stage_name

        self.get_logger().info(
            f"Sending {stage_name} goal: x={x}, y={y}, z={z}"
        )
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def open_gripper(self):
        self.stage = "open_gripper"
        self.waiting_gripper_action = "open"
        self.gripper_done_count = 0

        self.send_single_gripper_goal(
            self._left_gripper_client, 0.04, 20.0, "open_left"
        )
        self.send_single_gripper_goal(
            self._right_gripper_client, 0.04, 20.0, "open_right"
        )

    def send_single_gripper_goal(self, client, position, max_effort, stage_name):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        send_goal_future = client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            lambda future: self.gripper_goal_response_callback(future, stage_name)
        )

    def gripper_goal_response_callback(self, future, stage_name):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"{stage_name} goal rejected")
            rclpy.shutdown()
            return

        self.get_logger().info(f"{stage_name} goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.gripper_result_callback(future, stage_name)
        )

    def gripper_result_callback(self, future, stage_name):
        self.get_logger().info(f"{stage_name} finished")
        self.gripper_done_count += 1

        if self.gripper_done_count < 2:
            return

        time.sleep(1.0)

        if self.waiting_gripper_action == "open":
            self.waiting_gripper_action = None
            self.move_to_named_pose(self.place_x, self.place_y, self.retreat_z, "retreat")
            return

    def send_home_goal(self):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "panda_arm"
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 5.0

        constraints = Constraints()
        for joint_name, joint_value in self.home_joint_values.items():
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = joint_value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)

        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = False

        self.stage = "home"
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"{self.stage} goal rejected")
            rclpy.shutdown()
            return

        self.get_logger().info(f"{self.stage} goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        error_code = result.error_code.val

        if error_code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"{self.stage} failed, MoveIt error code: {error_code}"
            )
            rclpy.shutdown()
            return

        self.get_logger().info(f"{self.stage} succeeded")

        if self.stage == "lift":
            self.move_to_named_pose(self.place_x, self.place_y, self.place_pre_z, "place_pre")
            return

        if self.stage == "place_pre":
            self.move_to_named_pose(self.place_x, self.place_y, self.place_z, "place")
            return

        if self.stage == "place":
            self.open_gripper()
            return

        if self.stage == "retreat":
            self.send_home_goal()
            return

        if self.stage == "home":
            self.get_logger().info("Place sequence finished")
            rclpy.shutdown()
            return


def main(args=None):
    rclpy.init(args=args)
    node = PandaPlaceBlock()
    node.send_goal()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
