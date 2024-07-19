//导入头文件
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <limo_msgs/action/limo_action.hpp>

#include <iostream>

using namespace std;
using namespace std::placeholders;

// 定义一个LimoActionServer类，继承自rclcpp::Node
class LimoActionServer : public rclcpp::Node
{

  public:
  // 简化名称
  using limo_action = limo_msgs::action::LimoAction;
  using GoalHandleLimo = rclcpp_action::ServerGoalHandle<limo_action>;
  // 构造函数，初始化节点名称为"limo_action_server"
  explicit LimoActionServer(const rclcpp::NodeOptions & server_options = rclcpp::NodeOptions())
  : Node("limo_action_server",server_options)
  {
    // 创建发布者
    pub_vel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
    // 创建动作服务器
    this->limo_action_server_ = rclcpp_action::create_server<limo_action>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "limo_action",
      std::bind(&LimoActionServer::action_goal,this,_1,_2),
      std::bind(&LimoActionServer::action_cancel,this,_1),
      std::bind(&LimoActionServer::action_accepted,this,_1));
  }
  private:

  geometry_msgs::msg::Twist vel_cmd;
  // 定义发布者指针
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;
  // 定义action 服务器指针
  rclcpp_action::Server<limo_action>::SharedPtr limo_action_server_;

  // 处理目标请求的回调函数
  rclcpp_action::GoalResponse action_goal(
      const rclcpp_action::GoalUUID &uuid,std::shared_ptr<const limo_action::Goal> limo_goal
  )
  {
    (void)uuid;

    RCLCPP_INFO(rclcpp::get_logger("server"), "Got goal request with order X: %f", limo_goal->x);
    RCLCPP_INFO(rclcpp::get_logger("server"), "Got goal request with order Z: %f", limo_goal->z);
    // 限制目标值的大小
    if (limo_goal->x > 1.5 || limo_goal->z > 1.5) 
    {
      // 拒绝请求
      RCLCPP_INFO(rclcpp::get_logger("server"),"REJECT");
      return rclcpp_action::GoalResponse::REJECT;

    }
    // 接受并执行请求
    RCLCPP_INFO(rclcpp::get_logger("server"),"GoalResponse");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

  }
  //用于处理取消目标请求
  rclcpp_action::CancelResponse action_cancel(
      const std::shared_ptr<GoalHandleLimo> limo_cancel
  )
  {
      RCLCPP_INFO(rclcpp::get_logger("server"), "Got request to cancel goal");
      // 忽略参数  
      (void)limo_cancel;
      // 接受请求
      return rclcpp_action::CancelResponse::ACCEPT;
      RCLCPP_INFO(rclcpp::get_logger("server"),"action_cancel");

  }
  // 执行目标请求的回调函数
  void execute(
    const std::shared_ptr<GoalHandleLimo> limo_execute)
  {

    RCLCPP_INFO(rclcpp::get_logger("server"), "Executing goal");
    rclcpp::Rate loop_rate(1);
    // 获取目标
    const auto goal = limo_execute->get_goal();
    // 创建反馈指针
    auto feedback = std::make_shared<limo_action::Feedback>();
    // 获取状态
    auto & sequence = feedback->status;
    // 创建结果指针
    auto result = std::make_shared<limo_action::Result>();
    // 目标信息
    vel_cmd.linear.x = goal.get()->x;
    vel_cmd.linear.y = goal.get()->y;
    vel_cmd.angular.z = goal.get()->z;

    // 检查是否处于执行状态
    if (limo_execute->is_active()) {
      limo_execute->succeed(result);
      pub_vel->publish(vel_cmd);
      limo_execute->publish_feedback(feedback);
      RCLCPP_INFO(rclcpp::get_logger("server"), "Goal Succeeded");
      rclcpp::sleep_for(std::chrono::seconds(2));
      result->success = true; 

      vel_cmd.linear.x = 0.0;
      vel_cmd.linear.y = 0.0;
      vel_cmd.angular.z = 0.0;
      pub_vel->publish(vel_cmd);
    }


  }
  // 处理接受请求的回调函数
  void action_accepted(
      const std::shared_ptr<GoalHandleLimo> limo_accepted
  )
  {
    // 创建一个线程来执行目标请求
    std::thread{std::bind(&LimoActionServer::execute,this,_1),limo_accepted}.detach();

  }

};




int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    //运行action 服务节点
    rclcpp::spin(std::make_shared<LimoActionServer>());   

    rclcpp::shutdown();

    return 0;
}
