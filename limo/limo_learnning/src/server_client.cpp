#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <limo_msgs/srv/limo_srv.hpp>

#include <memory>
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("server_client");
  rclcpp::Client<limo_msgs::srv::LimoSrv>::SharedPtr client =
    node->create_client<limo_msgs::srv::LimoSrv>("Cmd");
 
  auto request = std::make_shared<limo_msgs::srv::LimoSrv::Request>();
  request->x = 0.5;
  request->y = 0;
  request->z = 0.0;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "success.data: %d", result.get()->success.data);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service server_cmd");
  }

  rclcpp::shutdown();
  return 0;
}