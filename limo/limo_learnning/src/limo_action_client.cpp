//导入头文件
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <limo_msgs/action/limo_action.hpp>

using namespace std;
using namespace std::placeholders;
// 定义一个LimoActionClient类，继承自rclcpp::Node
class LimoActionClient : public rclcpp::Node
{

  public:
    // 使用别名简化limo_msgs::action::LimoAction
    using Limoaction = limo_msgs::action::LimoAction;
    // 使用别名简化rclcpp_action::ClientGoalHandle<Limoaction>
    using ClientGoal = rclcpp_action::ClientGoalHandle<Limoaction>;
    // 构造函数，初始化节点名称为"limo_action"
    explicit LimoActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
    : Node("limo_action",node_options)
    {
      // 创建动作客户端
      this->action_client = rclcpp_action::create_client<Limoaction>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface()
        , "limo_action");
    }
    //发送目标函数
    void send_goal()
    {
    if (!action_client->wait_for_action_server(std::chrono::seconds(20))) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        return ;
      }

    auto send_goal_opt = rclcpp_action::Client<Limoaction>::SendGoalOptions();
    send_goal_opt.goal_response_callback = 
      std::bind(&LimoActionClient::response_callback,this,_1); // 绑定目标响应回调函数
    send_goal_opt.feedback_callback = 
      std::bind(&LimoActionClient::feedback_callback,this,_1,_2); // 绑定反馈回调函数
    send_goal_opt.result_callback = 
      std::bind(&LimoActionClient::result_callback,this,_1);  // 绑定结果回调函数
    // 创建目标信息，并填上数据
    auto goal_msg = Limoaction::Goal();
    goal_msg.x = 1;
    goal_msg.y = 0;
    goal_msg.z = 0.5;


    RCLCPP_INFO(this->get_logger(), "Sending goal");
    // 发送目标
    this->action_client->async_send_goal(goal_msg,send_goal_opt);
    }

  private:
    rclcpp_action::Client<Limoaction>::SharedPtr action_client;
  // 目标响应函数
  void response_callback (ClientGoal::SharedPtr res_msg)
  {
    if (!res_msg)
    {
        RCLCPP_ERROR(this->get_logger(), "Client: Goal was rejected by server");
        rclcpp::shutdown(); // Shut down client node
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Client: Goal accepted by server, waiting for result");
    }

  }

    // 创建处理周期反馈消息的回调函数
    void feedback_callback(
        ClientGoal::SharedPtr,
        const std::shared_ptr<const Limoaction::Feedback> feedback_message)
      {
          // 将反馈信息写入字符串流
          std::stringstream ss;
          ss << "Client: Received feedback: "<< feedback_message->status;
          RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
      }

      // 创建一个收到最终结果的回调函数
    void result_callback(const ClientGoal::WrappedResult & result_msg)
    {
        switch (result_msg.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Client: Goal was aborted"); // 输出目标被中止信息
                rclcpp::shutdown(); // 关闭客户端节点
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Client: Goal was canceled");  // 输出目标被取消信息
                rclcpp::shutdown(); 
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Client: Unknown result code");  // 输出未知结果码信息
                rclcpp::shutdown();
                return;
        }
            // 输出结果信息
            RCLCPP_INFO(this->get_logger(), "Client: Result received: %s",result_msg.result->success ? "false" : "true");
        rclcpp::shutdown();         
    }

}
;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // 创建limo_action_client节点
  auto action_client = std::make_shared<LimoActionClient>(); 
  // 发送目标
  action_client->send_goal();

  rclcpp::spin(action_client);

  rclcpp::shutdown();

  return 0;
}