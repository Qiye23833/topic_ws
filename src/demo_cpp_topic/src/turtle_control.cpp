#include "rclcpp/rclcpp.hpp"//导入rclcpp库
#include "geometry_msgs/msg/twist.hpp"//导入geometry_msgs消息类型
#include <chrono>//导入时间库
#include "turtlesim/msg/pose.hpp"//导入turtlesim消息类型
using namespace std;
using namespace std::chrono;

class TurtleControlNode : public rclcpp::Node  // 继承rclcpp::Node
{
private:  
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // 创建一个发布者
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_; // 创建一个订阅者
    double target_x_ = 1.0;//目标位置x
    double target_y_ = 1.0;//目标位置y
    double k_=1.0;//比例系数
    double v_max_ = 3.0;//最大速度
    
public:
    explicit TurtleControlNode(const string& node_name):Node(node_name)// 构造函数
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);// 创建一个发布者
        subscriber_ = this->create_subscription<turtlesim::msg::Pose>("turtle1/pose",10,
        bind(&TurtleControlNode::pose_callback,this,placeholders::_1));
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr pose)//回调函数：计算速度和角速度
    {
        //1.获取当前位置
        auto current_x = pose->x;
        auto current_y = pose->y;
        
        RCLCPP_INFO(get_logger(),"当前位置:x=%f,y=%f",current_x,current_y);
        //2.计算当前位置与目标位置的距离差和角度差
        auto distance = sqrt(
            pow(target_x_-current_x,2)+
            pow(target_y_-current_y,2)
        );
        // atan2(y,x)计算的是目标点相对于当前点的方位角(弧度)
        // (target_y_-current_y)和(target_x_-current_x)分别是目标点和当前点的y、x方向差值
        auto angle = atan2((target_y_-current_y),(target_x_-current_x))-pose->theta;
        
        //3.控制策略：计算速度和角速度
        auto msg = geometry_msgs::msg::Twist();
        if(distance>0.1)
        {
            if(fabs(angle)>0.2)
            {
                msg.angular.z = fabs(angle);
            }
            else{
                msg.linear.x =k_*distance;
            }
        }

        //4.限制线速度最大值
        if(msg.linear.x>v_max_)
        {
            msg.linear.x = v_max_;
        }
        //5.发布速度和角速度
        RCLCPP_INFO(get_logger(),"线速度:x=%f,角速度:z=%f",msg.linear.x,msg.angular.z);
        publisher_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = make_shared<TurtleControlNode>("turtle_control_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}