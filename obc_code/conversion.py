
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray  # Assuming the message type is Float32MultiArray

def pot_to_deg(pot):
	angle = pot*180/1023
	return angle

def gripper_rotaion(pos1,pos2):
	gripper_y_rot = pos1-pos2
	gripper_x_rot = (pos1+pos2)
	return [gripper_y_rot,gripper_x_rot]


class GripperAnglesSubscriber(Node):
    def __init__(self):
        super().__init__('gripper_angles_subscriber')
        # Subscribe to the topic that publishes the array of 5 values
        self.subscription = self.create_subscription(
            Float32MultiArray,        # Message type
            'gripper_angles_topic',   # Topic name (replace with your actual topic name)
            self.listener_callback,   # Callback function
            10                         # QoS (Quality of Service)
        )

    def listener_callback(self, msg):
        # Extract the array of values from the message
        if len(msg.data) >= 5:
            pot1, pot2, pos1, pos2, pos3 = msg.data[:5]  # Get the first 5 values
            self.get_logger().info(f'Received: pot1={pot1}, pot2={pot2}, pos1={pos1}, pos2={pos2}, pos3={pos3}')
            
            # Store the values in a Python list or array
            values = [pot1, pot2, pos1, pos2, pos3]
            self.get_logger().info(f'Values array: {values}')

def main(args=None):
    rclpy.init(args=args)
    node = GripperAnglesSubscriber()
    rclpy.spin(node)

    [gripper_y_rot,gripper_x_rot] = gripper_rotaion(pos1,pos2)
	
    fore_angle = pot_to_deg(pot1)
    bicep_angle = pot_to_dep(pot2)
    print("{gripper_x_rot} {gripper_y_rot} {bicep_angle} {fore_angle}")


    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
