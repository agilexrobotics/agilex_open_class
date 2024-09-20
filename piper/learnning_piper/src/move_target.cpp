#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char *argv[])
{
  // 初始化 ROS 并创建节点
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "move_target",  // 节点名称为 "move_target"
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));  // 自动声明参数，允许从命令行或文件中覆盖

  // 创建 ROS 日志器
  auto const logger = rclcpp::get_logger("move_target");  // 日志器，用于输出日志信息

  // 创建 MoveIt MoveGroup 接口
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "arm");  // "arm" 是规划组的名称

  // 设置目标位姿
  auto const target_pose = []
  {
    geometry_msgs::msg::Pose msg;  // 定义目标位姿的消息
    msg.orientation.w = 1.0;       // 设置姿态的四元数，表示无旋转
    msg.position.x = 0.28;         // 设置目标位置的 x 坐标
    msg.position.y = -0.2;         // 设置目标位置的 y 坐标
    msg.position.z = 0.5;          // 设置目标位置的 z 坐标
    
    return msg;  // 返回目标位姿
  }();
  move_group_interface.setPoseTarget(target_pose);  // 将目标位姿设置为 MoveGroup 的目标

  // 生成到目标位姿的规划
  auto const [success, plan] = [&move_group_interface]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;  // 创建规划消息对象
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));  // 调用规划函数，并检查是否成功
    return std::make_pair(ok, msg);  // 返回规划是否成功及规划消息
  }();

  // 如果规划成功，执行路径
  if (success)
  {
    move_group_interface.execute(plan);  // 执行规划路径
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");  // 如果规划失败，输出错误日志信息
  }

  // 关闭 ROS
  rclcpp::shutdown();
  return 0;
}
