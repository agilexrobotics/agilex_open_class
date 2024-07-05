#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <limo_msgs/srv/limo_srv.hpp>

#include <memory>

geometry_msgs::msg::Twist vel_cmd;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;

void server_cmd(
  const std::shared_ptr<limo_msgs::srv::LimoSrv_Request> request,
  std::shared_ptr<limo_msgs::srv::LimoSrv_Response> response
)
{
  vel_cmd.linear.x = request->x;
  vel_cmd.linear.y = request->y;
  vel_cmd.angular.z = request->z;
  pub_vel->publish(vel_cmd);
  // std::this_thread::sleep_for(std::chrono::milliseconds(500));
  rclcpp::sleep_for(std::chrono::seconds(2));
  if(!response->success.data)
  {
    vel_cmd.linear.x = 0.0;
    vel_cmd.linear.y = 0.0;
    vel_cmd.angular.z = 0.0;
    pub_vel->publish(vel_cmd);

    response->success.data = true;
  
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"request->x: %lf",request->x);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"request->y: %lf",request->y);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"request->z: %lf",request->z);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"response->success.data: %d",response->success.data);

}

int main(int argc, char** argv)
{

  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("server_cmd");

  rclcpp::Service<limo_msgs::srv::LimoSrv>::SharedPtr service= 
    node->create_service<limo_msgs::srv::LimoSrv>("Cmd", &server_cmd);

  pub_vel = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
