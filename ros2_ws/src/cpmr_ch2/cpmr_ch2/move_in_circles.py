import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class MoveInCircle(Node):
    def __init__(self):
        super().__init__('move_robot_in_circle')
        self.get_logger().info(f'{self.get_name()} created, moving in circle')

        # Declare parameters for radius and angular velocity
        self.declare_parameter('radius', 1.0)  # Default radius is 1 meter
        self.declare_parameter('linear_velocity', 0.4)  # Default linear velocity

        # Fetch the values of the parameters
        self.radius = self.get_parameter('radius').value
        self.linear_velocity = self.get_parameter('linear_velocity').value

        # Calculate the angular velocity based on the linear velocity and radius
        self.angular_velocity = self.linear_velocity / self.radius

        # Create publisher to command velocity
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 1)

        # Create a timer to call the publish command at regular intervals
        self.timer = self.create_timer(0.1, self.move_in_circle)

    def move_in_circle(self):
        # Create a new Twist message
        twist = Twist()

        # Set the linear velocity in the x direction (forward movement)
        twist.linear.x = self.linear_velocity

        # Set the angular velocity in the z direction (rotation around the z-axis)
        twist.angular.z = self.angular_velocity

        # Publish the twist message to move the robot
        self._publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = MoveInCircle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
