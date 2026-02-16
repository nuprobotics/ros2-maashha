import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher')
        
        self.declare_parameter('topic_name', '/spgc/receiver')
        self.declare_parameter('text', 'Hello, ROS2!')
        
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.text = self.get_parameter('text').get_parameter_value().string_value
        
        self.get_logger().info(f'Publishing to topic: {topic_name}')
        self.get_logger().info(f'Message text: {self.text}')
        
        self.publisher_ = self.create_publisher(String, topic_name, 10)
        
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = self.text
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    publisher_node = PublisherNode()
    rclpy.spin(publisher_node)
    publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
