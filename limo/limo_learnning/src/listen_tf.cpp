#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("tf_listener");

  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  rclcpp::Rate rate(10); // 10 Hz

  static tf2_ros::StaticTransformBroadcaster static_broadcaster(node);

  while (rclcpp::ok())
  {
    try
    {
      // 获取base_link坐标变换
      geometry_msgs::msg::TransformStamped base_link_transform;
      base_link_transform = tf_buffer.lookupTransform("base_link", "imu_link", tf2::TimePointZero);

      RCLCPP_INFO(node->get_logger(), "Received TF: [%s] -> [%s]",
                  base_link_transform.header.frame_id.c_str(),
                  base_link_transform.child_frame_id.c_str());

      // 虚拟一个跟随base_link的TF坐标，根据base_link的TF进行平移和旋转
      
      geometry_msgs::msg::TransformStamped virtual_transform;
      virtual_transform.header.stamp = node->get_clock()->now();
      virtual_transform.header.frame_id = "base_link";
      virtual_transform.child_frame_id = "virtual_frame";
      virtual_transform.transform.translation.x = sqrt(pow(base_link_transform.transform.translation.x,2))
                                                  +sqrt(pow(base_link_transform.transform.translation.y,2));
      virtual_transform.transform.translation.y = base_link_transform.transform.translation.y + 0.1;
      virtual_transform.transform.translation.z = base_link_transform.transform.translation.z;
      virtual_transform.transform.rotation = base_link_transform.transform.rotation;

      static_broadcaster.sendTransform(virtual_transform);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(node->get_logger(), "%s", ex.what());
    }

    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
