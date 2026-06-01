#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import PlanningScene, CollisionObject


class ObstacleManager(Node):
    def __init__(self):
        super().__init__("obstacle_manager")
        self.planning_scene_pub = self.create_publisher(
            PlanningScene, "/planning_scene", 10
        )

    def add_table(
        self,
        object_id="table",
        frame_id="world",
        size=(1.2, 1.2, 0.7),
        position=(0.0 , 0.0 , 0.35)
    ):
        scene_msg = PlanningScene()
        scene_msg.is_diff = True

        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.id = object_id

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [size[0], size[1], size[2]]

        box_pose = Pose()
        box_pose.position.x = position[0]
        box_pose.position.y = position[1]
        box_pose.position.z = position[2]
        box_pose.orientation.w = 1.0

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD

        scene_msg.world.collision_objects.append(collision_object)

        self.get_logger().info(
            f"Adding box obstacle: id={object_id}, "
            f"size={size}, position={position}"
        )
        self.planning_scene_pub.publish(scene_msg)

    def add_box(
        self,
        object_id="box",
        frame_id="world",
        size=(0.2, 0.2, 0.2),
        position=(0.4 , 0.4 , 0.8)
    ):
        scene_msg = PlanningScene()
        scene_msg.is_diff = True

        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.id = object_id

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [size[0], size[1], size[2]]

        box_pose = Pose()
        box_pose.position.x = position[0]
        box_pose.position.y = position[1]
        box_pose.position.z = position[2]
        box_pose.orientation.w = 1.0

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD

        scene_msg.world.collision_objects.append(collision_object)

        self.get_logger().info(
            f"Adding box obstacle: id={object_id}, "
            f"size={size}, position={position}"
        )
        self.planning_scene_pub.publish(scene_msg)


    def add_obstacle1(
        self,
        object_id="obstacle1",
        frame_id="world",
        size=(0.2, 0.2, 0.7),
        position=(0.3 , 0.3 , 1.05)
    ):
        scene_msg = PlanningScene()
        scene_msg.is_diff = True

        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.id = object_id

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [size[0], size[1], size[2]]

        box_pose = Pose()
        box_pose.position.x = position[0]
        box_pose.position.y = position[1]
        box_pose.position.z = position[2]
        box_pose.orientation.w = 1.0

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = CollisionObject.ADD

        scene_msg.world.collision_objects.append(collision_object)

        self.get_logger().info(
            f"Adding box obstacle: id={object_id}, "
            f"size={size}, position={position}"
        )
        self.planning_scene_pub.publish(scene_msg)

    def remove_box(self, object_id="box_obstacle", frame_id="world"):
        scene_msg = PlanningScene()
        scene_msg.is_diff = True

        collision_object = CollisionObject()
        collision_object.id = object_id
        collision_object.header.frame_id = frame_id
        collision_object.operation = CollisionObject.REMOVE

        scene_msg.world.collision_objects.append(collision_object)

        self.get_logger().info(f"Removing obstacle: id={object_id}")
        self.planning_scene_pub.publish(scene_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleManager()

    # 直接运行这个文件时，默认添加一个测试障碍物
    #node.add_box()
    node.add_table()
    node.add_obstacle1()

    # 给 MoveIt 一点时间接收消息
    rclpy.spin_once(node, timeout_sec=1.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
