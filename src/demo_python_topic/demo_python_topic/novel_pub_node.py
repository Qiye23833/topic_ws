import rclpy
from rclpy.node import Node
import requests
from example_interfaces.msg import String
from queue import Queue

class NovelPubNode(Node):# 创建一个节点
    def __init__(self,node_name):
        super().__init__(node_name)# 初始化节点
        self.get_logger().info(f'{node_name},启动!')# 打印启动信息
        self.novels_queue_ = Queue() # 先创建一个队列
        self.novel_pub_ = self.create_publisher(String,'novel',10)  # 创建一个发布者
        self.create_timer(5,self.timer_callback)# 创建一个定时器
        
    
    def timer_callback(self):# 定时器回调函数
        self.novel_pub.publish()# 发布消息
        if not self.novels_queue_.empty():# 如果队列不为空
            line = self.novels_queue_.get()# 获取队列中的消息
            msg = String()# 创建一个消息
            msg.data = line# 将消息设置为队列中的消息
            self.novel_pub_.publish(msg)# 发布消息
            self.get_logger().info(f'发布{line}/{msg}')# 打印消息

    def download(self,url):# 下载函数
        response = requests.get(url)# 获取url的响应
        response.encoding ='utf-8'# 设置编码
        text = response.text# 获取响应的文本
        self.get_logger().info(f'下载{url}.{len(text)}')# 打印下载信息
        for line in text.splitlines():# 遍历文本的每一行
            self.novels_queue_.put(line)# 将每一行添加到队列中


def main():
    rclpy.init()# 初始化rclpy
    node = NovelPubNode('novel_pub')# 创建一个节点
    node.download('http://0.0.0.0:8000/novel1.txt')# 下载小说
    rclpy.spin(node)# 运行节点
    rclpy.shutdown()# 关闭节点