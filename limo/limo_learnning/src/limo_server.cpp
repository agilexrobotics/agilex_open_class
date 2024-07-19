// 导入头文件
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <limo_msgs/srv/limo_srv.hpp>
// 导入智能指针库
#include <memory>

// 定义Twist消息对象，用于速度命令
geometry_msgs::msg::Twist vel_cmd;
// 定义一个发布者
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;

int count = 0 ; 

// 定义服务回调函数
void limo_srv(
  const std::shared_ptr<limo_msgs::srv::LimoSrv_Request> request, // 请求
  std::shared_ptr<limo_msgs::srv::LimoSrv_Response> response      //响应
)
{
  while (rclcpp::ok())
  {
 
  if (count < 4)      
    {
        // 将请求的数据 赋值给vel_cmd
        vel_cmd.linear.x = request->x;
        vel_cmd.linear.y = request->y;
        vel_cmd.angular.z = request->z;
        // 发布话题消息       
        pub_vel->publish(vel_cmd); 
        // 输出日志信息，提示已经完成话题发布  
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing: linear.x: %f, angular.z: %f", 
                vel_cmd.linear.x, vel_cmd.angular.z);          
        rclcpp::sleep_for(std::chrono::microseconds(500));

    }
    else
    {
        vel_cmd.linear.x = 0.0;
        vel_cmd.linear.y = 0.0;
        vel_cmd.linear.z = 0.0;
        vel_cmd.angular.z = 0;
        // 发布话题消息       
        pub_vel->publish(vel_cmd); 
        // 输出日志信息，提示已经完成话题发布  
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing: linear.x: %f, angular.z: %f", 
                vel_cmd.linear.x, vel_cmd.angular.z);           
        response->success.data = true;
        break;
    }
    count++;                       

  // 打印请求和响应的日志信息
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"request->x: %lf",request->x);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"request->y: %lf",request->y);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"request->z: %lf",request->z);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"response->success.data: %d",response->success.data);

}
  }
 
int main(int argc, char** argv)
{

  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("limo_srv");
  //创建服务
  rclcpp::Service<limo_msgs::srv::LimoSrv>::SharedPtr service= 
    node->create_service<limo_msgs::srv::LimoSrv>("limo_srv", &limo_srv);

  pub_vel = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
