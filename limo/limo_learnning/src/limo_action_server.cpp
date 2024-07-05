#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <limo_msgs/action/limo_action.hpp>

#include <memory>
#include <thread>
using namespace std;


using limo_action = limo_msgs::action::LimoAction;
using GoalHandleLimo = rclcpp_action::ServerGoalHandle<limo_action>;

geometry_msgs::msg::Twist vel_cmd;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;

//输入目标参数
rclcpp_action::GoalResponse action_goal(
    const rclcpp_action::GoalUUID &uuid,std::shared_ptr<const limo_action::Goal> limo_goal
)
{
  (void)uuid;

  RCLCPP_INFO(rclcpp::get_logger("server"), "Got goal request with order X: %f", limo_goal->x);
  RCLCPP_INFO(rclcpp::get_logger("server"), "Got goal request with order Z: %f", limo_goal->z);
  if (limo_goal->x > 1.5 || limo_goal->z > 1.5) 
  {
    RCLCPP_INFO(rclcpp::get_logger("server"),"REJECT");
    return rclcpp_action::GoalResponse::REJECT;

  }
  RCLCPP_INFO(rclcpp::get_logger("server"),"GoalResponse");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}
//用于处理取消目标请求
rclcpp_action::CancelResponse action_cancel(
    const std::shared_ptr<GoalHandleLimo> limo_cancel
)
{
    RCLCPP_INFO(rclcpp::get_logger("server"), "Got request to cancel goal");
    (void)limo_cancel;
    return rclcpp_action::CancelResponse::ACCEPT;
    RCLCPP_INFO(rclcpp::get_logger("server"),"action_cancel");

}


void execute(
  const std::shared_ptr<GoalHandleLimo> limo_execute)
{

  RCLCPP_INFO(rclcpp::get_logger("server"), "Executing goal");
  rclcpp::Rate loop_rate(1);
  const auto goal = limo_execute->get_goal();
  auto feedback = std::make_shared<limo_action::Feedback>();
  auto & sequence = feedback->status;
  auto result = std::make_shared<limo_action::Result>();
  auto flag = result->success.data;

  vel_cmd.linear.x = goal.get()->x;
  vel_cmd.linear.y = goal.get()->y;
  vel_cmd.angular.z = goal.get()->z;

  // Check if goal is done
  if (limo_execute->is_active()) {
    limo_execute->succeed(result);
    pub_vel->publish(vel_cmd);
    limo_execute->publish_feedback(feedback);
    RCLCPP_INFO(rclcpp::get_logger("server"), "Goal Succeeded");
    rclcpp::sleep_for(std::chrono::seconds(2));

    vel_cmd.linear.x = 0.0;
    vel_cmd.linear.y = 0.0;
    vel_cmd.angular.z = 0.0;
    pub_vel->publish(vel_cmd);

  }
}

void action_accepted(
    const std::shared_ptr<GoalHandleLimo> limo_accepted
)
{
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{execute, limo_accepted}.detach();
    RCLCPP_INFO(rclcpp::get_logger("server"), "limo_accepted");

}
int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("limo_action");
    pub_vel = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);


    rclcpp_action::Server<limo_action>::SharedPtr action_server;
    
    action_server = rclcpp_action::create_server<limo_action>(
        node,
        "limo_action",
        action_goal,
        action_cancel,
        action_accepted);


    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
