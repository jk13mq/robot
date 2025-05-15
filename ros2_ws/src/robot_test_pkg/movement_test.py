import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class JetRoverControl(Node):
    def __init__(self, namespace):
        super().__init__('jetrover_control_' + namespace)
        self.publisher = self.create_publisher(Twist, f'/{namespace}/cmd_vel', 10)

    def move(self, linear_speed=0.2, angular_speed=0.0, duration=2.0):
        """ Move with specified linear and angular speed for a duration """
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.get_logger().info(f'Moving | Linear: {linear_speed} | Angular: {angular_speed} | Duration: {duration}s')
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(twist)
            time.sleep(0.1)
        self.stop()

    def forward(self, speed=0.2, duration=2.0):
        self.move(linear_speed=speed, duration=duration)

    def backward(self, speed=0.2, duration=2.0):
        self.move(linear_speed=-speed, duration=duration)

    def turn_left(self, speed=0.5, duration=1.0):
        self.move(linear_speed=0.0, angular_speed=speed, duration=duration)

    def turn_right(self, speed=0.5, duration=1.0):
        self.move(linear_speed=0.0, angular_speed=-speed, duration=duration)

    def stop(self):
        """ Stop the robot """
        twist = Twist()
        self.publisher.publish(twist)
        self.get_logger().info('Robot Stopped')


def main():
    rclpy.init()
    rover_a = JetRoverControl('jetrover_a')
    rover_b = JetRoverControl('jetrover_b')

    # Move both forward with different speeds
    rover_a.forward(speed=0.3, duration=3)
    rover_b.forward(speed=0.2, duration=3)

    # Turn Rover A left and Rover B right
    rover_a.turn_left(speed=0.5, duration=1.5)
    rover_b.turn_right(speed=0.5, duration=1.5)

    # Stop both
    rover_a.stop()
    rover_b.stop()

    rover_a.destroy_node()
    rover_b.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
