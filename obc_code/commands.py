import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray  # Message type for arrays of float numbers
import termios
import tty
import sys
import select

def get_key():
    """Read a single character from the keyboard."""
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin)
        if select.select([sys.stdin], [], [], 0.1)[0]:
            key = sys.stdin.read(1)
        else:
            key = None
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class KeyboardArrayPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_array_publisher')
        # Create a publisher on the topic 'array_topic'
        self.publisher_ = self.create_publisher(Float32MultiArray, 'array_topic', 10)

        # Initialize an array of 5 float values
        self.data_array = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.get_logger().info('Press some values to send commands')
        
        # Keep track of the currently pressed key
        self.previous_key = None
        self.run()

    def run(self):
        while rclpy.ok():
            key = get_key()

            # Modify values based on key presses
            if key:
                if key == 'w' and self.previous_key != 'w':
                    self.data_array[0] += 20  # Increase first value
                elif key == 's' and self.previous_key != 's':
                    self.data_array[0] -= 20  # Decrease first value
                elif key == 'e' and self.previous_key != 'e':
                    self.data_array[1] += 20  # Increase second value
                elif key == 'd' and self.previous_key != 'd':
                    self.data_array[1] -= 20  # Decrease second value
                elif key == 'r' and self.previous_key != 'r':
                    self.data_array[2] += 20  # Increase third value
                elif key == 'f' and self.previous_key != 'f':
                    self.data_array[2] -= 20  # Decrease third value
                elif key == 't' and self.previous_key != 't':
                    self.data_array[3] += 20  # Increase fourth value
                elif key == 'g' and self.previous_key != 'g':
                    self.data_array[3] -= 20  # Decrease fourth value
                elif key == 'y' and self.previous_key != 'y':
                    self.data_array[4] += 20  # Increase fifth value
                elif key == 'h' and self.previous_key != '':
                    self.data_array[4] -= 20  # Decrease fifth value
                elif key == 'q':  # Quit the loop
                    self.get_logger().info("Quitting...")
                    break

            else:
                self.data_array = [0.0,0.0,0.0,0.0,0.0]
            self.publish_array()
    
            # Reset values when the key is released
            if not key and self.previous_key is not None:
                self.data_array = [0.0, 0.0, 0.0, 0.0, 0.0]
                self.publish_array()
                self.previous_key = None

            self.previous_key = key

    def publish_array(self):
        # Create a Float32MultiArray message and assign the data
        msg = Float32MultiArray()
        msg.data = self.data_array

        # Publish the message to the topic
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published array: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardArrayPublisher()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
