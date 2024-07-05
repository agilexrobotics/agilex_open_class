#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

geometry_msgs::msg::Twist vel_cmd;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;

std::ofstream outfile; // 声明用于写入txt文件的ofstream对象

void sub_odom_cb(const nav_msgs::msg::Odometry msg)
{
    outfile << "Pose: (x=" << msg.pose.pose.position.x << ", y=" << msg.pose.pose.position.y
          << ", z=" << msg.pose.pose.position.z << ")" << std::endl;
    outfile << "Orientation: (x=" << msg.pose.pose.orientation.x << ", y=" << msg.pose.pose.orientation.y
          << ", z=" << msg.pose.pose.orientation.z << ", w=" << msg.pose.pose.orientation.w << ")" << std::endl;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("topic_cmd");
    pub_vel = node-> create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10); 
    sub_odom = node-> create_subscription<nav_msgs::msg::Odometry>("/odom",10,sub_odom_cb);
    outfile.open("odom_data.txt"); // 打开txt文件，如果文件不存在则创建新文件

    rclcpp::Rate loop_rate(10);
    double count = 0;

    while (rclcpp::ok())
    {
        vel_cmd.linear.x = 0.1;
        vel_cmd.linear.y = 0 ;
        vel_cmd.linear.z = 0 ;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0 ;
        vel_cmd.angular.z = 0.1 ;

        pub_vel->publish(vel_cmd);
        rclcpp::spin_some(node);
        loop_rate.sleep();

    }
    
    outfile.close(); // 关闭txt文件

    return 0;
}