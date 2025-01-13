import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray  # Import the message type

class GripperAnglesPublisher(Node):
    def __init__(self):
        super().__init__('gripper_angles_publisher')
        # Create a publisher on the topic 'gripper_angles_topic'
        self.publisher_ = self.create_publisher(Float32MultiArray, 'gripper_angles_topic', 10)
        
        # Set a timer to publish data periodically (e.g., every 2 seconds)
        self.timer = self.create_timer(2.0, self.publish_values)

        # Initialize a message object
        self.msg = Float32MultiArray()

    def publish_values(self):
        # Example values for pot1, pot2, pos1, pos2, pos3
        pot1 = 500.0
        pot2 = 750.0
        pos1 = 300.0
        pos2 = 600.0
        pos3 = 900.0

        # Assign the data to the message's 'data' field
        self.msg.data = [pot1, pot2, pos1, pos2, pos3]
        
        # Publish the message to the topic
        self.publisher_.publish(self.msg)
        self.get_logger().info(f'Published: {self.msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = GripperAnglesPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
