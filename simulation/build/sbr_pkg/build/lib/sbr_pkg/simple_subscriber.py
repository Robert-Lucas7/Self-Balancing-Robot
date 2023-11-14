import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class Subscriber(Node):
	def __init__(self):
		super().__init__("subscriber_node")
		self.create_subscription(Int32, "counter", self.display_counter, 10)
	def display_counter(self, msg):
		self.get_logger().info(f'Count: {msg.data}')
		

def main(args=None):
    rclpy.init(args=args)

    sub_node = Subscriber()

    rclpy.spin(sub_node)

    sub_node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
	main()
