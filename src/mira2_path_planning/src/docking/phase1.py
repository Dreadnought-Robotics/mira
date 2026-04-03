# Import Libraries
import rclpy
from rclpy.node import Node
import math
import time 

# Import Message types
from custom_msgs.msg import Telemetry, Commands
from geometry_msgs.msg import Point


class PhaseOneController(Node):
    def __init__(self):
        super().__init__("docking_phase_one_controls")

        # Initialize PID values
        self.kp_depth = 9.05
        self.ki_depth = 0.005
        self.kd_depth = 20.20

        self.kp_yaw = 1.0
        self.kd_yaw = 1.0
        self.ki_yaw = 0.0

        self.kp_lateral = 10.0
        self.kd_lateral = 10.0
        self.ki_lateral = 0.0

        self.dt = 0.1

        # Initialize varibles
        self.stage = 0
        self.goal_heading = None
        self.heading = None
        self.yaw_counter = 0
        self.heading_error = None
        self.heading_threshold_start = 4
        self.heading_threshold_stop = 2
        self.prev_heading_error = None

        self.timer_counter = 0

        self.lateral_threshold = 0.05

        self.depth_target = 1100
        self.depth_threshold = 10

        self.cmd = Commands()
        self.cmd.arm = False
        self.cmd.mode = "ALT_HOLD"
        self.cmd.forward = 1500
        self.cmd.lateral = 1500
        self.cmd.thrust = 1500
        self.cmd.yaw = 1500
        self.cmd.roll = 1500
        self.cmd.pitch = 1500

        # Publishers
        self.cmd_pub = self.create_publisher(Commands, "master/commands", 10)
        
        # Subscribers
        self.telemetry_sub = self.create_subscription(Telemetry, "master/telemetry", self.telemetry_callback, 10)

        # Timer
        self.control_timer = self.create_timer(0.1, self.control_loop)

    
    def telemetry_callback(self, msg):
        if (self.yaw_counter == 0 ):
            self.goal_heading = msg.heading
            self.yaw_counter += 1   
            self.get_logger().info(f"Goal heading locked: {self.goal_heading}")
            self.cmd_pub.publish(self.cmd)
        self.heading = msg.heading
        self.get_logger().info(f"Telemetry heading: {self.heading}")

    def control_loop(self):
        if (self.stage == 0):
            if (self.timer_counter == 0):
                self.cmd.arm = False
                self.cmd_pub.publish(self.cmd)
                self.get_logger().info(f"Timer_counter = {self.timer_counter}, Disarming")
                
            elif (self.timer_counter == 20):
                self.cmd.mode = "ALT_HOLD"
                self.cmd_pub.publish(self.cmd)
                self.get_logger().info(f"Timer_counter = {self.timer_counter}, Changing mode to {self.cmd.mode}")

            elif (self.timer_counter == 40):
                self.cmd.arm = True
                self.cmd_pub.publish(self.cmd)
                self.stage = 1
                self.get_logger().info(f"Timer_counter = {self.timer_counter}, Arming and changing state to {self.stage}")

            self.timer_counter += 1
            return
        
        if self.heading is None or self.goal_heading is None:
            self.get_logger().info("Waiting for telemetry...")
            return

        elif (self.stage == 1):
            self.heading_error = (self.goal_heading - self.heading + 180) % 360 - 180
            if (self.prev_heading_error is None):
                self.derivative = 0.0
            else:
                self.derivative = (self.heading_error - self.prev_heading_error) / self.dt
            self.get_logger().info(f"Stage 1 | Heading: {self.heading} | Error: {self.heading_error}")

            if (abs(self.heading_error) > self.heading_threshold_start):
                self.yaw_cmd = 1500 + (self.heading_error * self.kp_yaw + self.derivative * self.kd_yaw)
                self.cmd.yaw = self.pwm_clamp(self.yaw_cmd, 20)
                self.get_logger().info(f"Yaw PWM: {self.cmd.yaw}")
                self.cmd_pub.publish(self.cmd)
            elif (abs(self.heading_error) < self.heading_threshold_stop):
                self.stage = 2
                self.cmd.yaw = 1500
                self.get_logger().info("Yaw aligned -> Stage 2")
                self.cmd_pub.publish(self.cmd)

        elif (self.stage == 2):
            self.heading_error = (self.goal_heading - self.heading + 180) % 360 - 180
            self.get_logger().info(f"Stage 2 | Heading error: {self.heading_error}")
            if (abs(self.heading_error) > self.heading_threshold_start):
                self.stage = 1
                self.get_logger().info("Heading drifted -> Back to Stage 1")
            else:
                self.cmd.forward = 1600
                self.get_logger().info("Moving forward")
                self.cmd_pub.publish(self.cmd)

        self.prev_heading_error = self.heading_error

    def pwm_clamp(self, pwm, band):
        return int(min(1500 + band, max(1500 - band, pwm)))


def main(args=None):
    rclpy.init(args=args)
    node = PhaseOneController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()