#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from servo_controller_msgs.msg import ServosPosition, ServoPosition
from servo_controller_msgs.msg import ServoStateList
import time

class ArmController(Node):
    """
    A ROS2 node for controlling robotic arm movements.
    This class provides methods to control individual joints of the arm using servo motors.
    Input: Commands to move specific joints to desired positions
    Output: Publishes servo position commands to the servo controller topic
    """
    
    def __init__(self):
        """
        Initialize the arm controller node and create necessary publishers
        """
        super().__init__('arm_controller')
        self.publisher_ = self.create_publisher(ServosPosition, 'servo_controller', 10)
        self.get_logger().info('Arm controller node initialized')

    def move_arm_joint(self, joint_id: int, move_pulse: int, duration: float = 1.0):
        """
        Move a specific joint to a desired position
        
        Args:
            joint_id (int): The ID of the joint to move (1-based indexing)
            move_pulse (int): The target position in pulse units
            duration (float): Time in seconds for the movement to complete
        """
        if not isinstance(joint_id, int) or joint_id < 1:
            self.get_logger().error(f'Invalid joint ID: {joint_id}')
            return
            
        msg = ServosPosition()
        msg.position_unit = 'pulse'
        msg.duration = duration
        
        servo = ServoPosition()
        servo.id = joint_id
        servo.position = float(move_pulse)
        
        msg.position.append(servo)
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sent command: Joint {servo.id} to position {servo.position} with duration {msg.duration} seconds")

    def move_arm_joint1(self, move_pulse: int = 500):
        """
        Convenience method to move joint 1 to a specific position
        
        Args:
            move_pulse (int): The target position in pulse units
        """
        self.move_arm_joint(1, move_pulse)

def main(args=None):
    """
    Main function to run the arm controller node
    """
    rclpy.init(args=args)
    node = ArmController()
    
    # Wait for publisher to be ready
    time.sleep(1.0)
    
    try:
        # Move joint 1 to position 700
        node.move_arm_joint1(move_pulse=700)
        # Wait for movement to complete
        time.sleep(1.5)
    except KeyboardInterrupt:
        print("Movement interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 