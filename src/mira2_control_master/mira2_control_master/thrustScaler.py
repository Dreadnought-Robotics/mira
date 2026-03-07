#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msgs.msg import Commands, Telemetry
import numpy as np

class BatteryScaler(Node):
    def __init__(self):
        super().__init__('battery_scaler')
        
        # LUT: {Voltage: Scaling_Factor}
        # Example: At 16V, maybe you use 1.0 (full power). 
        # At 12V, you might need to increase the duty cycle to get the same force.
        # Or more commonly: Scale down high-voltage thrust to match low-voltage max thrust.
        self.VOLTAGE_LUT = {
            12.0: 1.25,
            14.0: 1.10,
            16.0: 1.00
        }
        self.current_voltage = 16.0 # Default
        self.neutral_pwm = 1500

        # Subscriptions
        self.telem_sub = self.create_subscription(Telemetry, "/master/telemetry", self.telem_callback, 10)
        self.cmd_sub = self.create_subscription(Commands, "/master/commands", self.cmd_callback, 10)
        
        # Publisher
        self.scaled_pub = self.create_publisher(Commands, "/master/commands_scaled", 10)

    def telem_callback(self, msg):
        self.current_voltage = msg.battery_voltage

    def get_scale_factor(self, voltage):
        # Linear interpolation based on LUT
        v_points = sorted(self.VOLTAGE_LUT.keys())
        f_points = [self.VOLTAGE_LUT[v] for v in v_points]
        return np.interp(voltage, v_points, f_points)

    def scale_pwm(self, val, factor):
        # Shift to 0-center, scale, shift back
        relative = val - self.neutral_pwm
        scaled = int(relative * factor)
        # Clamp to RC limits
        return max(1100, min(1900, self.neutral_pwm + scaled))

    def cmd_callback(self, msg):
        factor = self.get_scale_factor(self.current_voltage)
        
        scaled_msg = Commands()
        scaled_msg.arm = msg.arm
        scaled_msg.mode = msg.mode
        
        # Apply scaling to thruster channels
        scaled_msg.forward = self.scale_pwm(msg.forward, factor)
        scaled_msg.lateral = self.scale_pwm(msg.lateral, factor)
        scaled_msg.thrust = self.scale_pwm(msg.thrust, factor)
        scaled_msg.pitch = self.scale_pwm(msg.pitch, factor)
        scaled_msg.roll = self.scale_pwm(msg.roll, factor)
        scaled_msg.yaw = self.scale_pwm(msg.yaw, factor)
        
        # Servos usually don't need battery scaling
        scaled_msg.servo1 = msg.servo1
        scaled_msg.servo2 = msg.servo2
        
        self.scaled_pub.publish(scaled_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BatteryScaler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
