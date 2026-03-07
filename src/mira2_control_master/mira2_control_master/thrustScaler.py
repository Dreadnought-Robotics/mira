#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msgs.msg import Commands, Telemetry
import numpy as np

class BatteryAwareScaler(Node):
    def __init__(self):
        super().__init__('battery_aware_scaler')
        
        # 1. Reference Point (The voltage we want to mimic)
        self.ref_voltage = 14.0 
        self.neutral_pwm = 1500
        
        # 2. Smoothing Setup (Moving Average)
        self.voltage_history = []
        self.window_size = 20  # Average over last 20 readings (~2 seconds at 10Hz)
        self.current_smoothed_v = 20.0

        # 3. T200 PWM-to-Thrust LUT (Simplified example - replace with your Excel data)
        # We map: [Voltage][PWM] -> Thrust_Kg
        # You should populate this dictionary with your 10V, 12V, 14V, 16V, 18V, 20V sheets.
        self.thrust_lut = {
            14.0: {1100: -3.5, 1500: 0.0, 1900: 4.2}, # Max thrust at 14V is ~4.2kg
            20.0: {1100: -5.1, 1500: 0.0, 1900: 6.5}  # Max thrust at 20V is ~6.5kg
        }

        self.telem_sub = self.create_subscription(Telemetry, "/master/telemetry", self.telem_callback, 10)
        self.cmd_sub = self.create_subscription(Commands, "/master/commands", self.cmd_callback, 10)
        self.scaled_pub = self.create_publisher(Commands, "/master/commands_scaled", 10)

    def telem_callback(self, msg):
        # Apply smoothing to handle sag under load
        self.voltage_history.append(msg.battery_voltage)
        if len(self.voltage_history) > self.window_size:
            self.voltage_history.pop(0)
        self.current_smoothed_v = sum(self.voltage_history) / len(self.voltage_history)

    def get_equivalent_pwm(self, target_thrust, current_v):
        """
        Finds the PWM value at current_v that produces the same thrust 
        as the input command would have produced at ref_voltage.
        """
        # 1. Calculate the 'Desired Thrust' based on 14V curve
        # (This is where you'd use your 14V sheet)
        # thrust = interpolate(raw_pwm, v14_pwms, v14_thrusts)
        
        # 2. Find PWM at 'current_v' that matches that 'thrust'
        # scaled_pwm = interpolate(thrust, current_v_thrusts, current_v_pwms)
        
        # FOR NOW: Simple scaling factor logic for brevity
        # Factor = (Thrust at Ref) / (Thrust at Current)
        # At 20V, the T200 is ~1.5x stronger than at 14V.
        # So at 20V, we reduce the PWM range by 1.5x.
        
        scale_factor = self.ref_voltage / current_v 
        return scale_factor

    def process_pwm(self, raw_pwm, factor):
        rel = raw_pwm - self.neutral_pwm
        scaled = int(rel * factor)
        return self.neutral_pwm + scaled

    def cmd_callback(self, msg):
        # Calculate factor based on smoothed voltage
        factor = self.get_equivalent_pwm(None, self.current_smoothed_v)
        
        scaled_msg = msg # Clone
        scaled_msg.forward = self.process_pwm(msg.forward, factor)
        scaled_msg.lateral = self.process_pwm(msg.lateral, factor)
        scaled_msg.thrust = self.process_pwm(msg.thrust, factor)
        scaled_msg.pitch = self.process_pwm(msg.pitch, factor)
        scaled_msg.roll = self.process_pwm(msg.roll, factor)
        scaled_msg.yaw = self.process_pwm(msg.yaw, factor)
        
        self.scaled_pub.publish(scaled_msg)
