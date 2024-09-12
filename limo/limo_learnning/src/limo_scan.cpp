#include <rclcpp/rclcpp.hpp>
// 导入 ROS2 C++ 库
#include <geometry_msgs/msg/twist.hpp>
// 从 geometry_msgs 接口导入 Twist 模块
#include <sensor_msgs/msg/laser_scan.hpp>
// 从 sensor_msgs 接口导入 LaserScan 模块

class LimoScan : public rclcpp::Node {
public:
    LimoScan() : Node("LimoScan") {
        // 创建发布器对象，发布 Twist 类型的消息到 'cmd_vel' 话题
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        // 创建订阅器对象，订阅 '/scan' 话题，接收 LaserScan 类型的消息
        subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(&LimoScan::laser_callback, this, std::placeholders::_1)
        );
        // 定义计时器周期为 0.5 秒
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&LimoScan::motion, this)
        );
        // 初始化激光前向距离为 0
        laser_forward_ = 0.0;
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // 保存前方激光扫描在 0° 的信息
        laser_forward_ = msg->ranges[359];
    }

    void motion() {
        // 打印接收到的数据
        RCLCPP_INFO(this->get_logger(), "I receive: %f", laser_forward_);
        // 创建一个 Twist 消息对象
        auto cmd = geometry_msgs::msg::Twist();

        // 移动逻辑
        if (laser_forward_ > 5.0) {
            // 如果前方距离大于 5 米，设定前进速度和旋转速度
            cmd.linear.x = 0.5;
            cmd.angular.z = 0.5;
        } else if (laser_forward_ < 5.0 && laser_forward_ >= 0.5) {
            // 如果前方距离在 0.5 米到 5 米之间，只设定前进速度
            cmd.linear.x = 0.2;
            cmd.angular.z = 0.0;
        } else {
            // 如果前方距离小于 0.5 米，停止运动
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        }
        // 将计算出的速度命令发布到 'cmd_vel' 话题
        publisher_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    float laser_forward_;
};

int main(int argc, char** argv) {
    // 初始化 ROS 通信
    rclcpp::init(argc, argv);
    // 创建一个节点对象
    auto node = std::make_shared<LimoScan>();
    // 暂停程序执行，等待节点被杀掉的请求（如按下 ctrl+c）
    rclcpp::spin(node);
    // 关闭 ROS 通信
    rclcpp::shutdown();
    return 0;
}