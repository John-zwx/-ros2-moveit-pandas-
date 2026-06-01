

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

'''
class XxxClient(Node):
    def __init__(self):
        # 1. 创建 ActionClient

    def send_goal(self):
        # 2. 构造 Goal
        # 3. 构造 trajectory
        # 4. 构造 point
        # 5. 发送 goal

    def goal_response_callback(self, future):
        # 6. 看是否 accepted

    def feedback_callback(self, feedback_msg):
        # 7. 看执行中反馈

    def result_callback(self, future):
        # 8. 看最终结果
'''


class PandaArmActionClient(Node):
    def __init__(self):
        super().__init__('panda_arm_action_client')

        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/panda_arm_controller/follow_joint_trajectory'
        )

    def send_goal(self):
        goal_msg = FollowJointTrajectory.Goal()

        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'panda_joint1',
            'panda_joint2',
            'panda_joint3',
            'panda_joint4',
            'panda_joint5',
            'panda_joint6',
            'panda_joint7',
        ]

        trajectory.points = []

        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, -0.2, 0.0, -1.4, 0.0, 1.2, 0.4]
        point1.time_from_start = Duration(sec=5, nanosec=0)

        point2 = JointTrajectoryPoint()
        point2.positions = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        point2.time_from_start = Duration(sec=10, nanosec=0)

        point4 = JointTrajectoryPoint()
        point4.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point4.time_from_start = Duration(sec=15, nanosec=0)

        trajectory.points.append(point1)
        trajectory.points.append(point2)
        trajectory.points.append(point4)

        goal_msg.trajectory = trajectory

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        self.get_logger().info('Sending goal...')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback received, desired positions: {feedback.desired.positions}'
        )

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result error_code: {result.error_code}')
        self.get_logger().info(f'Result error_string: {result.error_string}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = PandaArmActionClient()
    node.send_goal()

    rclpy.spin(node)


if __name__ == '__main__':
    main()
