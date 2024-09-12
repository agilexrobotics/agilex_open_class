import rclpy
# 导入 ROS2 Python 库
from rclpy.node import Node
# 从 geometry_msgs 接口导入 Twist 模块
from geometry_msgs.msg import Twist
# 从 sensor_msgs 接口导入 LaserScan 模块
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

# 定义一个类继承自 Node
class Exercise31(Node):

    def __init__(self):
        # 调用父类构造函数，初始化节点
        super().__init__('exercise31')
        # 创建一个发布器对象，发布 Twist 类型的消息到 'cmd_vel' 话题
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        # 创建一个订阅器对象，订阅 '/scan' 话题，接收 LaserScan 类型的消息
        self.subscriber = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.laser_callback, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        # 定义计时器周期为 0.5 秒
        self.timer_period = 0.5
        # 定义变量以保存接收到的激光数据
        self.laser_forward = 0
        # 创建一个 Twist 消息对象
        self.cmd = Twist()
        # 创建一个定时器，每 0.5 秒调用一次 self.motion 方法
        self.timer = self.create_timer(self.timer_period, self.motion)

    def laser_callback(self, msg):
        # 保存前方激光扫描在 0° 的信息
        self.laser_forward = msg.ranges[359]
        
    def motion(self):
        # 打印接收到的数据
        self.get_logger().info('I receive: "%s"' % str(self.laser_forward))
        # 移动逻辑
        if self.laser_forward > 5:
            # 如果前方距离大于 5 米，设定前进速度和旋转速度
            self.cmd.linear.x = 0.5
            self.cmd.angular.z = 0.5
        elif self.laser_forward < 5 and self.laser_forward >= 0.5:
            # 如果前方距离在 0.5 米到 5 米之间，只设定前进速度
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = 0.0         
        else:
            # 如果前方距离小于 0.5 米，停止运动
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
        
        # 将计算出的速度命令发布到 'cmd_vel' 话题
        self.publisher_.publish(self.cmd)

def main(args=None):
    # 初始化 ROS 通信
    rclpy.init(args=args)
    # 声明节点构造函数
    exercise31 = Exercise31()       
    # 暂停程序执行，等待节点被杀掉的请求（如按下 ctrl+c）
    rclpy.spin(exercise31)
    # 显式销毁节点
    exercise31.destroy_node()
    # 关闭 ROS 通信
    rclpy.shutdown()

if __name__ == '__main__':
    main()