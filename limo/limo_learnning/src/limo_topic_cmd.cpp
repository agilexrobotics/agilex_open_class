
//导入头文件
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

//自定义一个LimoTopicCmd并继承rclcpp::Node
class LimoTopicCmd : public rclcpp::Node
{
private:
    //定义一个周期性执行的回调函数
    void timer_callback()                                                       
        {
            geometry_msgs::msg::Twist vel_cmd;

            //检查计数器次数
            if (count < 4)      
            {
                //填入速度信息
                vel_cmd.linear.x = 0.1;
                vel_cmd.linear.y = 0.0;
                vel_cmd.linear.z = 0.0;
                vel_cmd.angular.z = 0;
                // 发布话题消息       
                pub_vel->publish(vel_cmd); 
                // 输出日志信息，提示已经完成话题发布  
                RCLCPP_INFO(this->get_logger(), "Publishing: linear.x: %f, angular.z: %f", 
                        vel_cmd.linear.x, vel_cmd.angular.z);           
                
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
                RCLCPP_INFO(this->get_logger(), "Publishing: linear.x: %f, angular.z: %f", 
                        vel_cmd.linear.x, vel_cmd.angular.z);           
                
            }
            count++;                       
        }
    // 定义速度命令消息对象
    geometry_msgs::msg::Twist vel_cmd;  
    // 定义发布者
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;
    // 定义定时器指针
    rclcpp::TimerBase::SharedPtr timer_;    
    // 定义计数器                         
    int count = 0;

public:

    LimoTopicCmd() : Node ("LimoTopicCmd") 
    {
    // 创建速度命令发布者，发布到 /cmd_vel 话题，队列大小为10
    pub_vel = this-> create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10); 
    // 创建一个定时器，每500毫秒调用一次 timer_callback 函数
    timer_ = this->create_wall_timer(
        500ms, std::bind(&LimoTopicCmd::timer_callback, this)); 
    }
    
};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    // 创建一个LimoTopicCmd节点对象并进入循环，等待和处理ROS2消息     
    rclcpp::spin(std::make_shared<LimoTopicCmd>());   

    // 关闭ROS2 C++接口
    rclcpp::shutdown();   

    return 0;
}