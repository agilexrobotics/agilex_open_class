#include <memory>
#include <vector>
#include <fstream>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class PathFollower : public rclcpp::Node {
public:
  PathFollower() : Node("path_follower"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    // 发布速度指令的Publisher
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // 加载路径点
    loadPathPoints("recorded_path.txt");

    // 定时器用于周期性调用路径跟踪函数
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&PathFollower::followPath, this));
  }

private:
  void loadPathPoints(const std::string &file_path) {
    std::ifstream path_file(file_path);
    if (!path_file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开路径点文件: %s", file_path.c_str());
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    while (path_file >> pose.pose.position.x >> pose.pose.position.y >> pose.pose.position.z
                     >> pose.pose.orientation.x >> pose.pose.orientation.y >> pose.pose.orientation.z >> pose.pose.orientation.w) {
      path_points_.push_back(pose);
    }

    path_file.close();
    RCLCPP_INFO(this->get_logger(), "成功加载 %zu 个路径点。", path_points_.size());
  }

  void followPath() {
    if (path_points_.empty()) {
      RCLCPP_INFO(this->get_logger(), "没有更多的路径点。");
      return;
    }

    // 获取机器人当前位置（使用 TF 变换）
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "无法获取TF变换: %s", ex.what());
      return;
    }
    geometry_msgs::msg::Quaternion quat = transform.transform.rotation;

    // 当前机器人位置
    double robot_x = transform.transform.translation.x;
    double robot_y = transform.transform.translation.y;
    // 提取偏航角
    tf2::Quaternion q;
    tf2::fromMsg(quat,q);
    tf2::Matrix3x3 M(q);
    double r, p, y;
    M.getRPY(r,p,y);
    double robot_yaw = y;

    // 当前目标点
    auto target_pose = path_points_.front();
    double target_x = target_pose.pose.position.x;
    double target_y = target_pose.pose.position.y;

    // 计算距离和方向
    double distance = std::hypot(target_x - robot_x, target_y - robot_y);
    double angle_to_goal = std::atan2(target_y - robot_y, target_x - robot_x);
    double angle_error = angle_to_goal - robot_yaw;

    // 设置速度命令
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.5 * distance;  // 简单比例控制器调整线速度
    cmd_vel.angular.z = 1.0 * angle_error;  // 简单比例控制器调整角速度

    // 发布速度命令
    cmd_vel_pub_->publish(cmd_vel);

    // 如果接近目标点，移到下一个点
    if (distance < 0.1) {
      path_points_.erase(path_points_.begin());
      RCLCPP_INFO(this->get_logger(), "到达一个路径点，切换到下一个目标点。");
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<geometry_msgs::msg::PoseStamped> path_points_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
