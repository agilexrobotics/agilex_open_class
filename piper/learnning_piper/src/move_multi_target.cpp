#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char *argv[])
{
  // 初始化 ROS 并创建节点
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "move_multi_target",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // 创建 ROS 日志器
  auto const logger = rclcpp::get_logger("move_multi_target");

  // 创建 MoveIt MoveGroup 接口，用于与机械臂交互
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");

  // 定义多个目标位姿，每个位姿都通过 lambda 函数进行初始化
  std::vector<geometry_msgs::msg::Pose> target_poses = {
      [] {
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;  // 设置姿态四元数的 w 分量（无旋转）
        msg.position.x = 0.28;     // 设置目标位置的 x 坐标
        msg.position.y = -0.2;     // 设置目标位置的 y 坐标
        msg.position.z = 0.5;      // 设置目标位置的 z 坐标
        return msg;
      }(),
      [] {
        geometry_msgs::msg::Pose msg;
        msg.orientation.w = 1.0;
        msg.position.x = 0.27;
        msg.position.y = 0.2;
        msg.position.z = 0.5;
        return msg;
      }()
      };

  // 遍历每个目标位姿，依次规划和执行
  for (const auto &target_pose : target_poses)
  {
    // 设置当前的目标位姿
    move_group_interface.setPoseTarget(target_pose);

    // 创建规划到目标位姿的路径
    auto const [success, plan] = [&move_group_interface]
    {
      moveit::planning_interface::MoveGroupInterface::Plan msg; // 定义一个空的规划消息
      auto const ok = static_cast<bool>(move_group_interface.plan(msg)); // 生成规划并检查是否成功
      return std::make_pair(ok, msg); // 返回是否成功以及规划的消息
    }();

    // 如果规划成功，执行该路径
    if (success)
    {
      RCLCPP_INFO(logger, "Executing plan to target..."); // 输出执行规划的日志信息
      move_group_interface.execute(plan);                 // 执行规划的路径

      // 等待机械臂完成运动
      rclcpp::sleep_for(std::chrono::seconds(1));

      RCLCPP_INFO(logger, "Target pose reached!"); // 目标点达到后输出日志信息
    }
    else
    {
      RCLCPP_ERROR(logger, "Planning to target failed!"); // 如果规划失败，输出错误日志信息
      break; // 如果某个目标点的规划失败，退出循环，停止后续目标点的执行
    }

    // 可选：在每次执行完路径后暂停一会，或提示用户继续
    // rclcpp::sleep_for(std::chrono::seconds(1));
  }

  // 关闭 ROS
  rclcpp::shutdown();
  return 0;
}
