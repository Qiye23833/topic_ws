import espeakng
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String
from queue import Queue
import threading
import time

class NovelSubNode(Node):#继承Node类
    def __init__(self,node_name):#初始化
        super().__init__(node_name)#调用父类初始化
        self.get_logger().info(f'{node_name},启动!')#打印启动信息
        self.novel_queue_ = Queue()#创建队列
        self.novel_suber_ = self.create_subscription(String,'novel',self.novel_callback,10)#创建订阅者
        self.reading_thread_ = threading.Thread(target=self.speak_thread)#创建线程
        self.reading_thread_.start()#启动线程

    def novel_callback(self,msg):#回调函数
        self.novel_queue_.put(msg.data)#将消息放入队列

    def speak_thread(self):#线程函数
        speaker=espeakng.Speaker()#创建语音对象
        speaker.voice='zh'#设置语音
        while rclpy.ok():#检测ros上下文是否正常
            if not self.novel_queue_.empty():#如果队列不为空
                text=self.novel_queue_.get()#获取队列中的消息
                self.get_logger().info(f'reading:{text}')#打印消息
                speaker.say(text)#播放消息
                speaker.wait()#等待消息播放完毕
            else:
                time.sleep(1)


def main():
    rclpy.init()#初始化ros2 
    node=NovelSubNode('novel_sub')#创建节点
    rclpy.spin(node)#运行节点
    rclpy.shutdown()#关闭ros2