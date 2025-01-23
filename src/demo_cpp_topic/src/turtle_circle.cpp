#include "rclcpp/rclcpp.hpp"  
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
using namespace std;
using namespace std::chrono;

class TurtleCircleNode : public rclcpp::Node  // 继承rclcpp::Node
{
private:  
    rclcpp::TimerBase::SharedPtr timer_;// 创建一个定时器
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_; // 创建一个发布者

public:
    explicit TurtleCircleNode(const string& node_name):Node(node_name)// 构造函数
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);// 创建一个发布者
        timer_ = this->create_wall_timer(1000ms,bind(&TurtleCircleNode::timer_callback,this));// 创建一个定时器
    }

    void timer_callback()// 定时器回调函数
    {
        auto msg = geometry_msgs::msg::Twist();// 创建一个消息
        msg.linear.x =1.0;// 设置线速度
        msg.angular.z = 1.0;// 设置角速度
        publisher_->publish(msg);// 发布消息
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = make_shared<TurtleCircleNode>("turtle_circle_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}