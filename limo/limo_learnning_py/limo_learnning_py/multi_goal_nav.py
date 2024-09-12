import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.time import Time
import tf_transformations
import time

class MultiGoalNavigator(Node):

    def __init__(self):
        super().__init__('multi_goal_navigator')

        # 创建一个目标点列表
        goals = []
        # 填充目标点列表 (示例中使用两个目标点)
        goals.append(self.create_goal_pose(1.0, 1.0, 0.0))
        goals.append(self.create_goal_pose(2.0, 2.0, 0.0))

        # 导航到多个目标点
        for goal in goals:
            self.navigate_to_goal(goal)

    def create_goal_pose(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation = self.create_quaternion_from_yaw(yaw)

        return goal

    def create_quaternion_from_yaw(self, yaw):
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        pose_orientation = PoseStamped()
        pose_orientation.pose.orientation.x = q[0]
        pose_orientation.pose.orientation.y = q[1]
        pose_orientation.pose.orientation.z = q[2]
        pose_orientation.pose.orientation.w = q[3]

        return pose_orientation.pose.orientation

    def navigate_to_goal(self, goal):
        nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        if not nav_to_pose_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available after waiting')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        self.get_logger().info('Navigating to goal...')

        future = nav_to_pose_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

        rclpy.spin_until_future_complete(self, future)
        # 等待当前目标完成
        # rclpy.sleep(5)  
        time.sleep(5)
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal result: {result}')


def main(args=None):
    rclpy.init(args=args)
    node = MultiGoalNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
