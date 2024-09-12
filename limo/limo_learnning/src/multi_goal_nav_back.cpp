#include <memory>
#include <vector>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using NavigationAction = nav2_msgs::action::NavigateToPose;

class NavToPoseClient : public rclcpp::Node {
 public:
  using NavigationActionClient = rclcpp_action::Client<NavigationAction>;
  using NavigationActionGoalHandle =
      rclcpp_action::ClientGoalHandle<NavigationAction>;

  NavToPoseClient() : Node("nav_to_pose_client") {
    action_client_ = rclcpp_action::create_client<NavigationAction>(
        this, "navigate_to_pose");

    // 添加多个导航目标点到目标队列
    addGoals();
  }

  void sendGoal() {
    // 如果目标队列为空，结束导航
    if (goals_.empty()) {
      RCLCPP_INFO(get_logger(), "所有目标点已完成导航。");
      return;
    }

    // 等待导航动作服务器上线，等待时间为5秒
    while (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_INFO(get_logger(), "等待Action服务上线。");
    }

    // 取出第一个目标点并从队列中移除
    auto goal_msg = goals_.front();
    goals_.erase(goals_.begin());

    auto send_goal_options =
        rclcpp_action::Client<NavigationAction>::SendGoalOptions();
    
    send_goal_options.goal_response_callback =
        [this](NavigationActionGoalHandle::SharedPtr goal_handle) {
          if (goal_handle) {
            RCLCPP_INFO(get_logger(), "目标点已被服务器接收");
          }
        };

    send_goal_options.feedback_callback =
        [this](
            NavigationActionGoalHandle::SharedPtr goal_handle,
            const std::shared_ptr<const NavigationAction::Feedback> feedback) {
          (void)goal_handle;
          RCLCPP_INFO(this->get_logger(), "反馈剩余距离:%f",
                      feedback->distance_remaining);
        };

    send_goal_options.result_callback =
        [this](const NavigationActionGoalHandle::WrappedResult& result) {
          if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(this->get_logger(), "目标点导航成功！");
            // 发送下一个目标点
            this->sendGoal();
          } else {
            RCLCPP_ERROR(this->get_logger(), "目标点导航失败。");
          }
        };

    // 发送导航目标点
    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

 private:
  void addGoals() {
    // 创建并添加第一个目标点
    NavigationAction::Goal goal_1;
    goal_1.pose.header.frame_id = "map";
    goal_1.pose.pose.position.x = 1.0f;
    goal_1.pose.pose.position.y = 1.0f;
    goals_.push_back(goal_1);

    // 创建并添加第二个目标点
    NavigationAction::Goal goal_2;
    goal_2.pose.header.frame_id = "map";
    goal_2.pose.pose.position.x = 2.0f;
    goal_2.pose.pose.position.y = 1.0f;
    goals_.push_back(goal_2);

    // 继续添加更多目标点...
  }

  std::vector<NavigationAction::Goal> goals_;  // 目标点队列
  NavigationActionClient::SharedPtr action_client_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavToPoseClient>();
  node->sendGoal();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
