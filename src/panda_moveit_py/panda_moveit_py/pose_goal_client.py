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
"""
#加障碍物
from moveit_msgs.msg import PlanningScene, CollisionObject
"""


class PandaPoseGoalClient(Node):
    def __init__(self):
        super().__init__("panda_pose_goal_client")

        # 这里默认连接 MoveIt 常见的 action 名
        self._action_client = ActionClient(self, MoveGroup, "/move_action")


        """
        #加障碍物：创建了一个发布器，专门往 /planning_scene 这个话题发 Planning Scene 更新
        self.planning_scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        

    #加障碍物
    def add_box_obstacle(self):
        scene_msg = PlanningScene()
        scene_msg.is_diff = True

        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.id = "box_obstacle"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.2, 0.2, 0.2]   # 长宽高 20cm

        box_pose = Pose()
        box_pose.position.x = 0.3
        box_pose.position.y = 0.3
        box_pose.position.z = 0.5
        box_pose.orientation.w = 1.0

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD

        scene_msg.world.collision_objects.append(collision_object)

        self.get_logger().info("Adding box obstacle to planning scene...")
        self.planning_scene_pub.publish(scene_msg)
    """


   
    def send_goal(self):
        """
        #加障碍物：等一秒障碍物加载
        self.add_box_obstacle()
        self.get_logger().info("Waiting 1 second for planning scene update...")
        import time
        time.sleep(1.0)
        """
        #创建一个moveit目标
        goal_msg = MoveGroup.Goal()

        # 1) 规划组名称
        goal_msg.request.group_name = "panda_arm"
        goal_msg.request.num_planning_attempts = 20
        goal_msg.request.allowed_planning_time = 5.0

        # ===== 新增：速度控制参数 =====
        # 速度缩放因子 (0.0 - 1.0)，1.0 表示全速
        goal_msg.request.max_velocity_scaling_factor = 1.0
        # 加速度缩放因子 (0.0 - 1.0)
        goal_msg.request.max_acceleration_scaling_factor = 1.0


        # 2) 位置约束：目标位置附近 1cm 立方体
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = "world"
        position_constraint.link_name = "panda_link8"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]

        target_pose = Pose()
        target_pose.position.x = 0.4
        target_pose.position.y = 0.0
        target_pose.position.z = 1.2

        # 这里只给位置，姿态约束单独写
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
        orientation_constraint.orientation.x = 1.0
        orientation_constraint.orientation.y = 0.0
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

        self.get_logger().info("Waiting for move_group action server...")
        self._action_client.wait_for_server()

        self.get_logger().info("Sending pose goal...")
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        error_code = result.error_code.val

        if error_code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("Planning and execution succeeded")
        else:
            self.get_logger().error(f"MoveIt failed, error code: {error_code}")

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = PandaPoseGoalClient()
    node.send_goal()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
