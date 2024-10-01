import math
import numpy as np
import json
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

min_distance_threshold = 0.5

def euler_from_quaternion(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class MoveToGoal(Node):
    def __init__(self, obstacle_file):
        super().__init__('move_robot_to_goal')
        self.get_logger().info(f'{self.get_name()} created')

        self.declare_parameter('_goal_t', 0.0)
        self.declare_parameter('_goal_x', 0.0)
        self.declare_parameter('_goal_y', 0.0)

        self._goal_t = self.get_parameter('_goal_t').value
        self._goal_x = self.get_parameter('_goal_x').value
        self._goal_y = self.get_parameter('_goal_y').value

        # Load obstacles from JSON file
        self.obstacles = self.load_obstacles(obstacle_file)

        self._subscriber = self.create_subscription(Odometry, "/odom", self._listener_callback, 1)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 1)

        # State to determine if the robot is avoiding an obstacle
        self.avoiding_obstacle = False
        self.current_obstacle = None
        self.hexagon_waypoints = []
        self.current_waypoint_index = 0

    def load_obstacles(self, filename):
        with open(filename, 'r') as f:
            data = json.load(f)
        obstacles = [{'x': obj['x'], 'y': obj['y'], 'r': obj['r']} for obj in data.values()]
        return obstacles

    def check_obstacle_collision(self, cur_x, cur_y):
        """Check if the robot is within a minimum distance from any obstacle."""
        for obs in self.obstacles:
            obs_x, obs_y, obs_r = obs['x'], obs['y'], obs['r']
            
            # Calculate the direct Euclidean distance from the robot to the obstacle center
            distance_to_obstacle = math.sqrt((obs_x - cur_x)**2 + (obs_y - cur_y)**2)
            
            # Check if the distance to the obstacle is less than or equal to the obstacle radius plus the safety distance
            if distance_to_obstacle <= (obs_r + min_distance_threshold):
                return True, obs  # Collision likely within the minimum distance
        return False, None  # No collision detected within the minimum distance

    def path_clear_to_goal(self, cur_x, cur_y):
        """Check if the direct path to the goal is clear of obstacles."""
        x_diff = self._goal_x - cur_x
        y_diff = self._goal_y - cur_y
        dist_to_goal = math.sqrt(x_diff**2 + y_diff**2)

        for obs in self.obstacles:
            obs_x, obs_y, obs_r = obs['x'], obs['y'], obs['r']
            
            # Calculate the perpendicular distance from the obstacle to the path to the goal
            num = abs((y_diff) * obs_x - (x_diff) * obs_y + self._goal_x * cur_y - self._goal_y * cur_x)
            denom = math.sqrt(x_diff**2 + y_diff**2)
            dist_to_path = num / denom
            
            # Check if the obstacle intersects the path to the goal
            if dist_to_path <= (obs_r + min_distance_threshold):
                return False
        return True

    def create_hexagon_waypoints(self, obs_x, obs_y, obs_r):
        """Generate six waypoints in a hexagonal pattern around the obstacle with reduced buffer."""
        buffer = 0.3  # Reduced buffer distance
        avoid_radius = obs_r + buffer

        waypoints = []
        for i in range(6):
            angle = i * math.pi / 3  # 60-degree increments (hexagon)
            wp_x = obs_x + avoid_radius * math.cos(angle)
            wp_y = obs_y + avoid_radius * math.sin(angle)
            waypoints.append((wp_x, wp_y))
        return waypoints

    def _listener_callback(self, msg, target_vel=0.4, max_pos_err=0.05):
        pose = msg.pose.pose

        cur_x = pose.position.x
        cur_y = pose.position.y
        o = pose.orientation
        roll, pitch, yaw = euler_from_quaternion(o)
        cur_t = yaw

        x_diff = self._goal_x - cur_x
        y_diff = self._goal_y - cur_y
        dist_to_goal = math.sqrt(x_diff**2 + y_diff**2)

        twist = Twist()

        if dist_to_goal > max_pos_err:
            # If currently avoiding an obstacle, continue moving through waypoints
            if self.avoiding_obstacle and self.hexagon_waypoints:
                wp_x, wp_y = self.hexagon_waypoints[self.current_waypoint_index]

                # Calculate the direction to the current waypoint
                direction_x = wp_x - cur_x
                direction_y = wp_y - cur_y
                direction_norm = math.sqrt(direction_x**2 + direction_y**2)

                # Check if the path to the goal is clear
                if self.path_clear_to_goal(cur_x, cur_y):
                    self.get_logger().info("Path to goal is clear, exiting hexagon avoidance")
                    self.avoiding_obstacle = False
                    self.hexagon_waypoints = []
                    self.current_waypoint_index = 0
                elif direction_norm < max_pos_err:
                    # Move to the next waypoint
                    self.current_waypoint_index += 1
                    if self.current_waypoint_index >= len(self.hexagon_waypoints):
                        self.get_logger().info("Completed hexagon avoidance, resuming goal-directed movement")
                        self.avoiding_obstacle = False
                        self.current_waypoint_index = 0
                        self.hexagon_waypoints = []
                else:
                    twist.linear.x = (direction_x / direction_norm) * target_vel
                    twist.linear.y = (direction_y / direction_norm) * target_vel
            else:
                # Determine the next move direction towards the goal
                next_x = cur_x + (x_diff / dist_to_goal) * target_vel
                next_y = cur_y + (y_diff / dist_to_goal) * target_vel

                # Check if the path to the next point is clear of obstacles
                collision, obs = self.check_obstacle_collision(cur_x, cur_y)
                if collision:
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0

                    self._publisher.publish(twist)

                    self.get_logger().info("Collision detected, generating hexagon waypoints")
                    self.avoiding_obstacle = True
                    self.current_obstacle = obs

                    # Generate waypoints around the obstacle in a hexagonal pattern
                    self.hexagon_waypoints = self.create_hexagon_waypoints(obs['x'], obs['y'], obs['r'])
                    self.current_waypoint_index = 0
                    return

                # Calculate velocity towards the next point
                direction_x = next_x - cur_x
                direction_y = next_y - cur_y
                direction_norm = math.sqrt(direction_x**2 + direction_y**2)
                
                twist.linear.x = (direction_x / direction_norm) * target_vel
                twist.linear.y = (direction_y / direction_norm) * target_vel
        else:
            twist.linear.x = 0.0
            twist.linear.y = 0.0

        self._publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    
    # The JSON file path should be passed as a ROS argument
    json_file_path = '/home/wagner/Projects/CPMR3/ros2_ws/src/cpmr_ch2/maps/default.json'  # Replace with actual path
    node = MoveToGoal(obstacle_file=json_file_path)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
