import rclpy  # 导入ROS 2 Python客户端库
from rclpy.node import Node  # 导入节点类
from rclpy.action import ActionClient  # 导入动作客户端类
# from rclpy.action import ClientGoalHandle  # 导入客户端目标句柄类
from limo_msgs.action import LimoAction  # 导入自定义动作消息类型
import sys  # 导入系统模块

class LimoActionClient(Node):  # 定义LimoActionClient类，继承自Node

    def __init__(self):  # 构造函数
        super().__init__('limo_action_client')  # 初始化节点名称为'limo_action_client'
        self.action_client = ActionClient(self, LimoAction, 'limo_action')  # 创建动作客户端

    def send_goal(self):  # 定义发送目标的函数
        if not self.action_client.wait_for_server(timeout_sec=20.0):  # 等待动作服务器20秒
            self.get_logger().error('Action server not available after waiting')  # 如果服务器不可用，记录错误日志
            rclpy.shutdown()  # 关闭节点
            return  # 返回退出函数

        goal_msg = LimoAction.Goal()  # 创建目标消息
        goal_msg.x = 1.0  # 设置目标x坐标
        goal_msg.y = 0.0  # 设置目标y坐标
        goal_msg.z = 0.5  # 设置目标z坐标

        self.get_logger().info('Sending goal')  # 记录发送目标的日志

        self.action_client.wait_for_server()  # 再次等待动作服务器
        # 异步发送目标，并绑定反馈回调函数
        # self.action_client.send_goal_async(goal_msg, feedback_callback=
        # self.feedback_callback).add_done_callback(self.goal_response_callback)

        self.send_goal_future = self.action_client.send_goal_async(goal_msg,
            feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):  # 定义目标响应回调函数
        goal_handle = future.result()  # 接受结果
        if not goal_handle.accepted:  # 如果目标未被接受
            self.get_logger().error('Client: Goal was rejected by server')  # 记录目标被拒绝的错误日志
            rclpy.shutdown()  # 关闭节点
            return  # 返回退出函数

        self.get_logger().info('Client: Goal accepted by server, waiting for result')  # 记录目标被接受的日志
        # goal_handle.get_result_async().add_done_callback(self.result_callback)  # 异步获取结果，并绑定结果回调函数
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_message):  # 定义反馈回调函数
        feedback = feedback_message.feedback
        self.get_logger().info(f'Client: Received feedback: {feedback.status}')  # 记录收到的反馈信息

    def result_callback(self, future):  # 定义结果回调函数
        result = future.result().result  # 获取结果
        self.get_logger().info('Result: %d' % result.success)

def main(args=None):  # 定义主函数
    rclpy.init(args=args)  # 初始化rclpy
    action_client = LimoActionClient()  # 创建LimoActionClient对象
    action_client.send_goal()  # 发送目标
    rclpy.spin(action_client)  # 保持节点运行

if __name__ == '__main__':  # 如果是主程序入口
    main()  # 调用主函数
