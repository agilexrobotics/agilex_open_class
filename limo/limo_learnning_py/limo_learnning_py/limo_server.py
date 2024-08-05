#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from limo_msgs.srv import LimoSrv

class LimoServiceNode(Node):

    def __init__(self):
        super().__init__('limo_srv')
        
        # 创建一个速度命令发布者，发布到 /cmd_vel 话题，队列大小为10
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 创建一个服务，服务名称为 "limo_srv"
        self.srv = self.create_service(LimoSrv, 'limo_srv', self.limo_srv_callback)
        
        self.count = 0
        self.vel_cmd = Twist()

    def limo_srv_callback(self, request, response):
        while rclpy.ok():
            if self.count < 4:
                # 将请求的数据 赋值给vel_cmd
                self.vel_cmd.linear.x = request.x
                self.vel_cmd.linear.y = request.y
                self.vel_cmd.angular.z = request.z
                
                # 发布话题消息
                self.pub_vel.publish(self.vel_cmd)
                
                # 输出日志信息，提示已经完成话题发布
                self.get_logger().info(f'Publishing: linear.x: {self.vel_cmd.linear.x}, angular.z: {self.vel_cmd.angular.z}')
                
                rclpy.sleep(0.5)  # 休眠500毫秒
            else:
                # 停止机器人
                self.vel_cmd.linear.x = 0.0
                self.vel_cmd.linear.y = 0.0
                self.vel_cmd.angular.z = 0.0
                
                # 发布话题消息
                self.pub_vel.publish(self.vel_cmd)
                
                # 输出日志信息，提示已经完成话题发布
                self.get_logger().info(f'Publishing: linear.x: {self.vel_cmd.linear.x}, angular.z: {self.vel_cmd.angular.z}')
                
                response.success = True
                break
            
            self.count += 1
            
            # 打印请求和响应的日志信息
            self.get_logger().info(f'request.x: {request.x}')
            self.get_logger().info(f'request.y: {request.y}')
            self.get_logger().info(f'request.z: {request.z}')
            self.get_logger().info(f'response.success.data: {response.success}')
        
        return response

def main(args=None):
    rclpy.init(args=args)
    
    # 创建一个LimoServiceNode节点对象并进入循环，等待和处理ROS2消息
    node = LimoServiceNode()
    rclpy.spin(node)
    
    # 关闭ROS2 Python接口
    rclpy.shutdown()

if __name__ == '__main__':
    main()
