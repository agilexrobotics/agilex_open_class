#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class CancelNav : public rclcpp::Node {
public:
    CancelNav() : Node("cancel_navigation_node") {
        // 创建一个动作客户端，连接到导航动作服务器
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this, "navigate_to_pose");

        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&CancelNav::laser_callback, this, std::placeholders::_1)
        );
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // 保存前方激光扫描在 0° 的信息
        laser_forward_ = msg->ranges[359];
        laser_data(laser_forward_);
    }

    void laser_data(const float &msg) {
        // 打印接收到的数据
        RCLCPP_INFO(this->get_logger(), "I receive: %f", msg);
        if (msg<1.0)
        {
            cancel_navigation();
        }
    
    }
    void cancel_navigation() {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "导航Action服务未上线");
            return;
        }

        auto future = action_client_->async_cancel_all_goals();
        std::cout<<"cancel_result: "<<std::endl;
 
        auto cancel_result = future.get()->return_code;
        std::cout<<"cancel_result: "<<cancel_result<<std::endl;
        // 根据返回的结果进行日志输出
        if (cancel_result == 1) {
            RCLCPP_INFO(this->get_logger(), "取消导航目标成功");
        } else {
            RCLCPP_ERROR(this->get_logger(), "取消导航目标失败");
        }
    }

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    float laser_forward_;

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CancelNav>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
