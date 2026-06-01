#!/usr/bin/env python3
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
)
#关节角度控制
from moveit_msgs.msg import JointConstraint
# ===== 新增：夹爪 action 接口 =====
from control_msgs.action import GripperCommand
# ===== 新增：用于夹爪动作后短暂停顿 =====
import time  




class PandaPoseGoalClient(Node):
    def __init__(self):
        super().__init__("panda_pose_goal_client")

        self._action_client = ActionClient(self, MoveGroup, "/move_action")

        # ===== 新增：左右夹爪 action client =====
        # 如果你的 action 名不是这个，需要用 ros2 action list | grep panda_hand 查实际名字
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

        # 当前阶段：先回 home，再去 target
        self.stage = "home"

        # Panda 常见 ready/home 位
        # 你可以按自己仿真里的实际初始位再调整
        self.home_joint_values = {
            "panda_joint1": 0.0,
            "panda_joint2": -0.785,
            "panda_joint3": 0.0,
            "panda_joint4": -2.356,
            "panda_joint5": 0.0,
            "panda_joint6": 1.571,
            "panda_joint7": 0.785,
        }

    def send_goal(self):
        # 每次开始先回初始位
        self.send_home_goal()

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
            #角度上下误差
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            #关节重要程度
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg.request.goal_constraints.append(constraints)

        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = False

        self.stage = "home"

        self.get_logger().info("Waiting for move_group action server...")
        self._action_client.wait_for_server()

        self.get_logger().info("Sending HOME goal...")
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def move_to_pregrasp(self):
        goal_msg = MoveGroup.Goal()

        # 1) 规划组名称
        goal_msg.request.group_name = "panda_arm"
        goal_msg.request.num_planning_attempts = 20
        goal_msg.request.allowed_planning_time = 5.0

        # 2) 位置约束：目标位置附近立方体
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "world"
        position_constraint.link_name = "panda_link8"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.005, 0.005, 0.005]

        target_pose = Pose()
        target_pose.position.x = 0.4
        target_pose.position.y = 0.4
        target_pose.position.z = 0.93

        target_pose.orientation.w = 1.0

        bounding_volume = BoundingVolume()
        bounding_volume.primitives.append(box)
        bounding_volume.primitive_poses.append(target_pose)

        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0

        # 3) 姿态约束
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "world"
        orientation_constraint.link_name = "panda_link8"
        orientation_constraint.orientation.x = 0.0
        orientation_constraint.orientation.y = 1.0
        orientation_constraint.orientation.z = 0.0
        orientation_constraint.orientation.w = 0.0
        orientation_constraint.absolute_x_axis_tolerance = 0.05
        orientation_constraint.absolute_y_axis_tolerance = 0.05
        orientation_constraint.absolute_z_axis_tolerance = 0.05
        orientation_constraint.weight = 1.0

        # 4) 组合成目标约束
        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        goal_msg.request.goal_constraints.append(constraints)

        # 5) 规划并执行
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = False

        self.stage = "pregrasp"

        self.get_logger().info("Sending pregrasp pose goal...")
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)


    def move_to_grasp(self):
        goal_msg = MoveGroup.Goal()

        # 1) 规划组名称
        goal_msg.request.group_name = "panda_arm"
        goal_msg.request.num_planning_attempts = 20
        goal_msg.request.allowed_planning_time = 5.0

        # 2) 位置约束：目标位置附近立方体
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "world"
        position_constraint.link_name = "panda_link8"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.005, 0.005, 0.005]

        target_pose = Pose()
        target_pose.position.x = 0.4
        target_pose.position.y = 0.4
        target_pose.position.z = 0.83

        target_pose.orientation.w = 1.0

        bounding_volume = BoundingVolume()
        bounding_volume.primitives.append(box)
        bounding_volume.primitive_poses.append(target_pose)

        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0

        # 3) 姿态约束
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = "world"
        orientation_constraint.link_name = "panda_link8"
        orientation_constraint.orientation.x = 0.0
        orientation_constraint.orientation.y = 1.0
        orientation_constraint.orientation.z = 0.0
        orientation_constraint.orientation.w = 0.0
        orientation_constraint.absolute_x_axis_tolerance = 0.05
        orientation_constraint.absolute_y_axis_tolerance = 0.05
        orientation_constraint.absolute_z_axis_tolerance = 0.05
        orientation_constraint.weight = 1.0

        # 4) 组合成目标约束
        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        goal_msg.request.goal_constraints.append(constraints)

        # 5) 规划并执行
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.look_around = False
        goal_msg.planning_options.replan = False

        self.stage = "target"

        self.get_logger().info("Sending TARGET pose goal...")
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

        if error_code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f"{self.stage} succeeded")

            # home 成功后，再去目标点
            if self.stage == "home":
                self.move_to_pregrasp()
                return
            elif self.stage == "pregrasp":
                self.move_to_grasp()
                return
            else:
                self.get_logger().info("Planning and execution succeeded")
        else:
            self.get_logger().error(
                f"{self.stage} failed, MoveIt error code: {error_code}"
            )

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PandaPoseGoalClient()
    node.send_goal()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
