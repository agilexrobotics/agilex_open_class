import rclpy  # 导入ROS 2的Python客户端库
from rclpy.node import Node  # 导入Node类以创建ROS 2节点
from rclpy.action import ActionServer, CancelResponse, GoalResponse  # 导入ActionServer类和相关响应类
from geometry_msgs.msg import Twist  # 导入Twist消息类型，用于发布速度命令
from limo_msgs.action import LimoAction  # 导入LimoAction消息类型
import threading  # 导入线程库，用于并发处理
import time  # 导入时间库，用于延时处理

# 定义LimoActionServer类，继承自Node
class LimoActionServer(Node):

    def __init__(self):  # 构造函数
        super().__init__('limo_action_server')  # 初始化节点，命名为'limo_action_server'
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)  # 创建速度命令发布者，发布到'cmd_vel'话题
        self.limo_action_server = ActionServer(  # 创建动作服务器
            self,  # 当前节点
            LimoAction,  # 动作类型
            'limo_action',  # 动作名称
            execute_callback=self.execute_callback,  # 执行目标的回调函数
        )

    def execute_callback(self, goal_handle):  # 执行目标的回调函数
        self.get_logger().info('Executing goal')  # 记录正在执行目标的信息
        vel_cmd = Twist()  # 创建Twist消息实例
        feedback_msg = LimoAction.Feedback()  # 创建反馈消息实例
        result = LimoAction.Result()  # 创建结果消息实例

        vel_cmd.linear.x = goal_handle.request.x  # 设置线速度X值
        vel_cmd.linear.y = goal_handle.request.y  # 设置线速度Y值
        vel_cmd.angular.z = goal_handle.request.z  # 设置角速度Z值

        if goal_handle.is_active:  # 如果目标句柄处于活动状态
            self.pub_vel.publish(vel_cmd)  # 发布速度命令
            self.get_logger().info('Goal Succeeded')  # 记录目标成功的信息
            goal_handle.succeed()  # 标记目标成功
            result.success = True  # 设置结果成功

            feedback_msg.status = 1  # 设置反馈消息状态
            goal_handle.publish_feedback(feedback_msg)  # 发布反馈消息

            time.sleep(2)  # 延时2秒

            vel_cmd.linear.x = 0.0  # 停止线速度X
            vel_cmd.linear.y = 0.0  # 停止线速度Y
            vel_cmd.angular.z = 0.0  # 停止角速度Z
            self.pub_vel.publish(vel_cmd)  # 发布停止命令
            result.success = True
        return result  # 返回结果消息

def main(args=None):  # 主函数
    rclpy.init(args=args)  # 初始化rclpy
    action_server = LimoActionServer()  # 创建LimoActionServer实例
    rclpy.spin(action_server)  # 保持节点运行，等待和处理ROS 2消息
    rclpy.shutdown()  # 关闭rclpy

if __name__ == '__main__':  # 检查是否为主程序入口
    main()  # 调用主函数
