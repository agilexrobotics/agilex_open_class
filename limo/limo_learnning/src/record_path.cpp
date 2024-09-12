#include <memory>
#include <vector>
#include <fstream>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class PathRecorder : public rclcpp::Node {
public:
  PathRecorder() : Node("path_recorder"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    // 初始化计时器，每秒执行一次
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&PathRecorder::recordPathPoint, this));
    
    // 打开文件以记录路径点
    path_file_.open("recorded_path.txt", std::ios::out);
    if (!path_file_.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开文件以记录路径点。");
    }
  }

  ~PathRecorder() {
    // 关闭文件
    if (path_file_.is_open()) {
      path_file_.close();
    }
  }

private:
  void recordPathPoint() {
    // 获取机器人当前位置（使用 TF 变换）
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform("map", "base_footprint", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "无法获取TF变换: %s", ex.what());
      return;
    }

    // 创建一个 PoseStamped 消息
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = transform.header.stamp;
    pose.header.frame_id = "map";
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;

    // 打印路径点信息
    RCLCPP_INFO(this->get_logger(), "记录路径点: x=%f, y=%f, z=%f",
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

    // 将路径点保存到文件
    if (path_file_.is_open()) {
      path_file_ << pose.pose.position.x << " "
                 << pose.pose.position.y << " "
                 << pose.pose.position.z << " "
                 << pose.pose.orientation.x << " "
                 << pose.pose.orientation.y << " "
                 << pose.pose.orientation.z << " "
                 << pose.pose.orientation.w << std::endl;
    }
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::ofstream path_file_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathRecorder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
