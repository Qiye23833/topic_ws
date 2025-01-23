import rclpy
from rclpy.node import Node#导入rclpy库
from geometry_msgs.msg import Twist#导入geometry_msgs消息类型
from turtlesim.msg import Pose#导入turtlesim消息类型
from math import atan2, fabs, sqrt, pow#导入数学库
#Pose 告诉我们"海龟在哪里"
#Twist 告诉海龟"要怎么走"

class TurtleControlNode(Node):
    def __init__(self,node_name):
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(Twist,"turtle1/cmd_vel",10)#创建一个发布者
        self.subscriber_ = self.create_subscription(Pose,"turtle1/pose",self.pose_callback,10)#创建一个订阅者
        self.target_x_ = 1.0#目标位置x
        self.target_y_ = 1.0#目标位置y
        self.k_ = 1.0#比例系数
        self.v_max_ = 3.0#最大速度

    def pose_callback(self,pose):
        print("当前位置:x=%f,y=%f",pose.x,pose.y)
        #1.计算当前位置与目标位置的距离差和角度差
        distance = sqrt(pow(self.target_x_-pose.x,2)+pow(self.target_y_-pose.y,2))#计算距离差
        angle = atan2((self.target_y_-pose.y),(self.target_x_-pose.x))-pose.theta#计算角度差
        #2.控制策略：计算速度和角速度
        msg = Twist()# Twist用于发送运动控制命令
        if distance > 0.1:#如果距离差大于0.1，则进行控制
            if fabs(angle) > 0.2:#如果角度差大于0.2，则进行转向
                msg.angular.z = fabs(angle)  # 控制转向
            else:
                msg.linear.x = self.k_*distance  # 控制前进
        #3.限制线速度最大值
        if msg.linear.x > self.v_max_:
            msg.linear.x = self.v_max_
        #4.发布控制命令
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)#初始化ros2
    node = TurtleControlNode('turtle_control')#创建节点
    rclpy.spin(node)#运行节点
    rclpy.shutdown()#关闭ros2

