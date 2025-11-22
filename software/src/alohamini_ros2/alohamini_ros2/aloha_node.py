import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import time
import sys
import os
import threading

# Ensure lerobot is in path if not installed as a package
# This allows running the node from the source workspace without pip installing everything
current_dir = os.path.dirname(os.path.abspath(__file__))
src_root = os.path.abspath(os.path.join(current_dir, '../../..'))
if src_root not in sys.path:
    sys.path.append(src_root)

try:
    from lerobot.robots.alohamini.lekiwi import AlohaMiniDivergent
    from lerobot.robots.alohamini.config_lekiwi import LeKiwiConfig
    
    from johnny5.robot import Johnny5Robot
    from johnny5.config import Johnny5Config
except ImportError as e:
    print(f"Error importing Robot drivers: {e}")
    print("Please ensure you are running from the workspace root or have installed the package.")
    sys.exit(1)

class AlohaNode(Node):
    def __init__(self):
        super().__init__('aloha_node')
        
        # Parameters
        self.declare_parameter('robot_type', 'aloha') # aloha or johnny5
        self.declare_parameter('left_port', '/dev/am_arm_follower_left')
        self.declare_parameter('right_port', '/dev/am_arm_follower_right')
        
        robot_type = self.get_parameter('robot_type').value
        left_port = self.get_parameter('left_port').value
        right_port = self.get_parameter('right_port').value

        # Initialize Robot
        self.get_logger().info(f"Initializing {robot_type} Driver...")
        
        if robot_type == 'johnny5':
            config = Johnny5Config(
                left_bus_port=left_port,
                right_bus_port=right_port
            )
            self.robot = Johnny5Robot(config)
        else:
            config = LeKiwiConfig(
                left_port=left_port,
                right_port=right_port
            )
            self.robot = AlohaMiniDivergent(config)
        
        try:
            self.robot.connect(calibrate=False) # Assume calibrated or handle separately
            self.get_logger().info("AlohaMini Connected Successfully!")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to robot: {e}")
            sys.exit(1)

        # Publishers & Subscribers
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.joint_cmd_sub = self.create_subscription(JointState, 'joint_commands', self.joint_cmd_callback, 10)
        
        # State Loop (30Hz)
        self.timer = self.create_timer(1.0/30.0, self.update)
        
        self.get_logger().info("Aloha Node is Ready.")

    def cmd_vel_callback(self, msg):
        """Handle Twist messages for base movement."""
        # Convert Twist (m/s, rad/s) to Aloha Action Dictionary
        action = {
            "x.vel": msg.linear.x,
            "y.vel": msg.linear.y,
            "theta.vel": msg.angular.z * (180.0 / 3.14159) # Convert rad/s to deg/s for the driver
        }
        self.robot.send_action(action)

    def joint_cmd_callback(self, msg):
        """
        Handle JointState messages for direct arm/lift control.
        Expects joint names to match the Aloha naming convention (e.g., 'arm_left_shoulder_pan')
        or 'lift_joint' for the Z-axis.
        """
        action = {}
        if len(msg.name) != len(msg.position):
            return

        for name, pos in zip(msg.name, msg.position):
            if name == 'lift_joint':
                # Convert meters (ROS) to mm (Aloha)
                action['lift_axis.height_mm'] = pos * 1000.0
            elif name.startswith('arm_'):
                # Pass through arm positions (Ensure your controller sends the correct units expected by the driver)
                action[f"{name}.pos"] = pos
        
        if action:
            self.robot.send_action(action)

    def update(self):
        """Read robot state and publish ROS messages."""
        try:
            # 1. Get Observation
            obs = self.robot.get_observation()
            
            # 2. Publish Joint States
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            
            # Map Aloha state keys to ROS joint names
            for key, value in obs.items():
                if key.endswith('.pos'):
                    # Arm joints
                    joint_name = key.replace('.pos', '')
                    msg.name.append(joint_name)
                    msg.position.append(float(value))
                elif key == 'lift_axis.height_mm':
                    # Lift joint (mm -> meters)
                    msg.name.append('lift_joint')
                    msg.position.append(float(value) / 1000.0)
            
            self.joint_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().warn(f"Update loop error: {e}")

    def destroy_node(self):
        self.get_logger().info("Stopping robot...")
        try:
            self.robot.disconnect()
        except:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = AlohaNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()