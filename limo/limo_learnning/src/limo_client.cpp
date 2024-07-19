//导入头文件
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <limo_msgs/srv/limo_srv.hpp>
// 使用标准的chrono库中的时间字面量
using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  //创建limo_client节点
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("limo_client");
  //创建一个客户端
  rclcpp::Client<limo_msgs::srv::LimoSrv>::SharedPtr client =
    node->create_client<limo_msgs::srv::LimoSrv>("limo_srv");

  //创建一个服务请求对象，并设置请求参数
  auto request = std::make_shared<limo_msgs::srv::LimoSrv::Request>();
  request->x = 0.5;
  request->y = 0;
  request->z = 0.0;
  //等待服务，每一秒检查一次
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
  //发送请求
  auto result = client->async_send_request(request);
  //等待请求结果
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "success.data: %d", result.get()->success.data);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service limo_srv");
  }

  rclcpp::shutdown();
  return 0;
}