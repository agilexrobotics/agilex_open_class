#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy  # 引入ROS2 Python接口库
from rclpy.node import Node  # 引入ROS2节点类
from geometry_msgs.msg import Twist  # 引入geometry_msgs消息类型中的Twist消息

class LimoTopicCmd(Node):  # 自定义一个LimoTopicCmd类并继承rclpy.Node
    def __init__(self):
        super().__init__('LimoTopicCmd')  # 初始化节点名称为"LimoTopicCmd"

        # 创建速度命令发布者，发布到 /cmd_vel 话题，队列大小为10
        self.pub_vel = self.create_publisher(Twist, '/cmd_vel', 10)

        # 创建一个定时器，每500毫秒调用一次 timer_callback 函数
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.count = 0  # 定义计数器

    def timer_callback(self):  # 定义一个周期性执行的回调函数
        vel_cmd = Twist()  # 创建一个Twist消息对象

        # 检查计数器次数
        if self.count < 4:
            # 填入速度信息
            vel_cmd.linear.x = 0.1
            vel_cmd.linear.y = 0.0
            vel_cmd.linear.z = 0.0
            vel_cmd.angular.z = 0.0
            # 发布话题消息
            self.pub_vel.publish(vel_cmd)
        else:
            # 停止机器人
            vel_cmd.linear.x = 0.0
            vel_cmd.linear.y = 0.0
            vel_cmd.linear.z = 0.0
            vel_cmd.angular.z = 0.0
            # 发布话题消息
            self.pub_vel.publish(vel_cmd)

        # 输出日志信息，提示已经完成话题发布
        self.get_logger().info(f'Publishing: linear.x: {vel_cmd.linear.x}, angular.z: {vel_cmd.angular.z}')

        self.count += 1  # 增加计数器

def main(args=None):
    rclpy.init(args=args)  # 初始化ROS2 Python接口

    # 创建一个LimoTopicCmd节点对象并进入循环，等待和处理ROS2消息
    node = LimoTopicCmd()
    rclpy.spin(node)

    # 关闭ROS2 Python接口
    rclpy.shutdown()

if __name__ == '__main__':
    main()
