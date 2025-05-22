#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from servo_controller_msgs.msg import ServosPosition, ServoPosition
from servo_controller_msgs.msg import ServoStateList
import time

class ArmController(Node):
    """
    A ROS2 node for controlling robotic arm movements.
    This class provides methods to control individual joints and multiple joints simultaneously.
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
        
        # For storing the initial joint positions
        self.initial_joint_positions = None
        self.joint_state_received = False

        # Subscribe to the joint state topic
        self.subscription = self.create_subscription(
            ServoStateList,
            'servo_controller/state',  # Adjust this topic if needed
            self.joint_state_callback,
            10
        )
        
        # Define home position for all joints
        self.home_position = {
            1: 500,  # Base joint
            2: 500,  # Shoulder joint
            3: 500,  # Elbow joint
            4: 500,  # Wrist joint
            5: 500,  # Wrist rotation
            6: 500   # Gripper
        }

        # Define positions
        self.fold_position = {
            1: 500,  # Base rotation
            2: 250,  # Shoulder down
            3: 300,  # Elbow in
            4: 300,  # Wrist curled
            5: 500,  # Wrist rotation
            6: 200   # Gripper closed
        }
        self.raised_position = {1: 500, 2: 700, 3: 700, 4: 500, 5: 500, 6: 500} # Example: arm up

    def joint_state_callback(self, msg):
        """
        Callback to store the current joint positions from the robot.
        """
        if not self.joint_state_received:
            # Store the first received joint positions as the initial state
            self.initial_joint_positions = {servo.id: servo.position for servo in msg.servos}
            self.joint_state_received = True
            self.get_logger().info(f"Initial joint positions: {self.initial_joint_positions}")

    def wait_for_initial_joint_state(self, timeout=5.0):
        """
        Wait until the initial joint state is received or timeout.
        """
        start_time = time.time()
        while not self.joint_state_received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.joint_state_received:
            self.get_logger().warn("Did not receive initial joint state in time.")

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

    def move_multiple_joints(self, joint_positions: dict, duration: float = 1.0):
        """
        Move multiple joints simultaneously to their target positions
        
        Args:
            joint_positions (dict): Dictionary mapping joint IDs to their target positions
                                  e.g., {1: 500, 2: 700, 3: 300}
            duration (float): Time in seconds for the movement to complete
        """
        msg = ServosPosition()
        msg.position_unit = 'pulse'
        msg.duration = duration
        
        for joint_id, position in joint_positions.items():
            if not isinstance(joint_id, int) or joint_id < 1:
                self.get_logger().error(f'Invalid joint ID: {joint_id}')
                continue
                
            servo = ServoPosition()
            servo.id = joint_id
            servo.position = float(position)
            msg.position.append(servo)
            
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sent command: Moving {len(joint_positions)} joints simultaneously")

    def return_to_home(self, duration: float = 1.0):
        """
        Return all joints to their home position
        
        Args:
            duration (float): Time in seconds for the movement to complete
        """
        self.move_multiple_joints(self.home_position, duration)
        self.get_logger().info("Returning to home position")

    def move_to_fold(self, duration: float = 1.0):
        """Move all joints to the folded position."""
        self.move_multiple_joints(self.fold_position, duration)
        self.get_logger().info("Moving to fold position")

    def move_to_raised(self, duration: float = 1.0):
        """Move all joints to the raised position."""
        self.move_multiple_joints(self.raised_position, duration)
        self.get_logger().info("Raising arm up")

    # Convenience methods for individual joints
    def move_arm_joint1(self, move_pulse: int = 500):
        """Move joint 1 to a specific position"""
        self.move_arm_joint(1, move_pulse)

    def move_arm_joint2(self, move_pulse: int = 500):
        """Move joint 2 to a specific position"""
        self.move_arm_joint(2, move_pulse)

    def move_arm_joint3(self, move_pulse: int = 500):
        """Move joint 3 to a specific position"""
        self.move_arm_joint(3, move_pulse)

    def move_arm_joint4(self, move_pulse: int = 500):
        """Move joint 4 to a specific position"""
        self.move_arm_joint(4, move_pulse)

    def move_arm_joint5(self, move_pulse: int = 500):
        """Move joint 5 to a specific position"""
        self.move_arm_joint(5, move_pulse)

    def move_arm_joint6(self, move_pulse: int = 500):
        """Move joint 6 to a specific position"""
        self.move_arm_joint(6, move_pulse)

    def move_to_initial_position(self, duration: float = 1.0):
        """
        Move all joints back to the initial position.
        """
        if self.initial_joint_positions:
            self.move_multiple_joints(self.initial_joint_positions, duration)
            self.get_logger().info("Returning to initial joint positions.")
        else:
            self.get_logger().warn("Initial joint positions not available.")

def main(args=None):
    """
    Main function to run the arm controller node with a pattern movement sequence
    """
    rclpy.init(args=args)
    node = ArmController()
    
    # Wait for publisher to be ready
    time.sleep(1.0)
    
    # Wait for the initial joint state from the robot
    node.get_logger().info("Waiting for initial joint state from robot...")
    node.wait_for_initial_joint_state(timeout=5.0)

    try:
        # Print out the initial joint positions
        if node.initial_joint_positions:
            print("Initial joint positions:", node.initial_joint_positions)
        else:
            print("Initial joint positions not received.")

        # 1. Fold first
        node.get_logger().info("Folding arm...")
        node.move_to_fold()
        time.sleep(2.0)

        # 2. Raise up
        node.get_logger().info("Raising arm up...")
        node.move_to_raised()
        time.sleep(2.0)

        # 3. Do the dance patterns (wave, circle, wrist rotation)
        # Start from home position
        node.get_logger().info("Starting from home position...")
        node.return_to_home()
        time.sleep(2.0)

        # Pattern 1: Wave motion
        node.get_logger().info("Pattern 1: Wave motion...")
        for _ in range(3):
            # Move up
            node.move_multiple_joints({
                1: 600,  # Base rotate
                2: 700,  # Shoulder up
                3: 600   # Elbow up
            })
            time.sleep(1.5)
            
            # Move down
            node.move_multiple_joints({
                1: 400,  # Base rotate
                2: 300,  # Shoulder down
                3: 400   # Elbow down
            })
            time.sleep(1.5)

        # Pattern 2: Circular motion
        node.get_logger().info("Pattern 2: Circular motion...")
        positions = [
            (600, 500, 500),  # Right
            (500, 600, 500),  # Up
            (400, 500, 500),  # Left
            (500, 400, 500),  # Down
        ]
        
        for pos in positions:
            node.move_multiple_joints({
                1: pos[0],  # Base
                2: pos[1],  # Shoulder
                3: pos[2]   # Elbow
            })
            time.sleep(1.5)

        # Pattern 3: Wrist rotation
        node.get_logger().info("Pattern 3: Wrist rotation...")
        for _ in range(2):
            node.move_multiple_joints({
                4: 700,  # Wrist up
                5: 700   # Wrist rotate
            })
            time.sleep(1.0)
            
            node.move_multiple_joints({
                4: 300,  # Wrist down
                5: 300   # Wrist rotate
            })
            time.sleep(1.0)

        # 4. Return to initial position
        node.get_logger().info("Returning to initial joint positions...")
        node.move_to_initial_position()
        time.sleep(2.0)
        
    except KeyboardInterrupt:
        print("Movement interrupted by user")
        node.move_to_initial_position()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 