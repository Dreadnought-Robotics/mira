#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Point
from std_msgs.msg import String
from custom_msgs.msg import Commands, Telemetry

class Phase:
    INIT = -1       
    SEARCH = 0
    ALIGN_XY = 1    
    LOCK = 2
    SEARCH2 = 3

PHASE_NAMES = {
    Phase.INIT: "INITIALIZING",
    Phase.SEARCH: "SEARCHING",
    Phase.ALIGN_XY: "ALIGNING_XY",
    Phase.LOCK: "DROPPING",
    Phase.SEARCH2: "EVADING"
}

class BucketControls(Node):

    def __init__(self):
        super().__init__("bucket_control_node")
        
        # Lateral & Surge PID Tuning parameters
        self.kp_sway, self.kd_sway = 150.0, 50.0
        self.kp_surge, self.kd_surge = 150.0, 50.0
        self.kp_yaw = 5.0

        self.pwm_neutral = 1500
        self.pwm_max_effort = 200
        self.search_speed = 1550

        # State Variables
        self.current_phase = Phase.INIT
        self.init_counter = 0
        self.bucket_visible = False
        self.sees_blue = False
        self.sees_orange = False
        
        self.target_nx = 0.0
        self.target_ny = 0.0
        self.last_time_seen = 0.0
        
        self.current_heading = 0.0
        self.locked_target_heading = None 
        
        # Separated error tracking for PIDs
        self.prev_nx_err = 0.0
        self.prev_ny_err = 0.0
        self.last_brain_tick = time.time()
        
        # Timers
        self.xy_hold_timer = None
        self.blind_lock_timer = None
        self.evasion_sweep_start = None
        self.evasion_direction = 1

        self.cmd_pub = self.create_publisher(Commands, "/master/commands", 10)
        self.debug_pub = self.create_publisher(String, "bucket_debug", 10)

        self.create_subscription(Point, "/bucket/p2offset", self.camera_callback, 10)
        self.create_subscription(Telemetry, "/master/telemetry", self.telem_callback, 10)
        self.create_subscription(String, "bucket/p2color", self.color_callback, 10)

        self.create_timer(0.05, self.think_and_act)

    def telem_callback(self, msg):
        self.current_heading = msg.heading

    def camera_callback(self, msg):
        self.last_time_seen = time.time()
        self.bucket_visible = True
        self.target_nx = msg.x
        self.target_ny = msg.y

    def color_callback(self, msg):
        self.sees_blue = (msg.data == "blue")
        self.sees_orange = (msg.data == "orange")

    def change_phase(self, new_phase, reason=""):
        if self.current_phase != new_phase:
            old_name = PHASE_NAMES.get(self.current_phase, "UNKNOWN")
            new_name = PHASE_NAMES.get(new_phase, "UNKNOWN")
            self.get_logger().warn(f"\n>>> STATE CHANGE: {old_name} -> {new_name} | {reason} <<<\n")
            self.current_phase = new_phase

    # ==========================================
    # PID FUNCTIONS
    # ==========================================
    def run_lateral_pid(self, nx, dt):
        """ Handles Sway (Left/Right) """
        derivative = (nx - self.prev_nx_err) / dt if dt > 0 else 0.0
        self.prev_nx_err = nx
        output = int((nx * self.kp_sway) + (derivative * self.kd_sway))
        output = max(min(output, self.pwm_max_effort), -self.pwm_max_effort)
        return self.pwm_neutral - output

    def run_surge_pid(self, ny, dt):
        """ Handles Surge (Forward/Backward) """
        derivative = (ny - self.prev_ny_err) / dt if dt > 0 else 0.0
        self.prev_ny_err = ny
        output = int((ny * self.kp_surge) + (derivative * self.kd_surge))
        output = max(min(output, self.pwm_max_effort), -self.pwm_max_effort)
        return self.pwm_neutral + output

    def calculate_heading_lock(self):
        """ Maintains starting Yaw heading """
        if self.locked_target_heading is None: return self.pwm_neutral
            
        err = self.locked_target_heading - self.current_heading
        if err > 180: err -= 360
        if err < -180: err += 360
        
        output = int(err * self.kp_yaw)
        return self.pwm_neutral + max(min(output, self.pwm_max_effort), -self.pwm_max_effort)


    def think_and_act(self):
        cmd = Commands()
        # Thrust is permanently set to 1500 to let ALT_HOLD maintain current depth natively
        cmd.pitch = cmd.roll = cmd.thrust = cmd.yaw = cmd.forward = cmd.lateral = self.pwm_neutral

        current_time = time.time()
        dt = current_time - self.last_brain_tick
        self.last_brain_tick = current_time

        # ==========================================
        # INITIALIZATION: Disarm -> Set Mode -> Wait 100 -> Arm
        # ==========================================
        if self.current_phase == Phase.INIT:
            #cmd.mode = "ALT_HOLD"
            cmd.arm = False  
            
            self.init_counter += 1
            if self.init_counter % 20 == 0:
                self.get_logger().info(f"Setting Mode... Waiting ({self.init_counter}/100)")

            if self.init_counter >= 100:
                self.change_phase(Phase.SEARCH, "Mode change complete. Arming vehicle.")
            
            self.cmd_pub.publish(cmd)
            return  

        cmd.mode = "ALT_HOLD"
        cmd.arm = True

        # ==========================================
        # FLIGHT LOGIC
        # ==========================================
        if self.bucket_visible and (current_time - self.last_time_seen > 1.0):
            self.get_logger().warn("Target LOST from camera view! (1.0s timeout)")
            self.bucket_visible = self.sees_blue = self.sees_orange = False

        nx = self.target_nx if self.bucket_visible else 0.0
        ny = self.target_ny if self.bucket_visible else 0.0

        if self.current_phase == Phase.SEARCH:
            cmd.forward = self.search_speed

            if self.bucket_visible and self.sees_blue:
                self.locked_target_heading = self.current_heading
                self.change_phase(Phase.ALIGN_XY, "Found BLUE target.")

            elif self.bucket_visible and self.sees_orange:
                self.evasion_sweep_start = current_time
                self.evasion_direction = 1
                self.change_phase(Phase.SEARCH2, "Evading ORANGE target")

        elif self.current_phase == Phase.ALIGN_XY:
            if not self.bucket_visible:
                self.xy_hold_timer = None
                self.change_phase(Phase.SEARCH, "Target lost during alignment")
                return

            cmd.lateral = self.run_lateral_pid(nx, dt)
            cmd.forward = self.run_surge_pid(ny, dt)
            cmd.yaw = self.calculate_heading_lock()

            # HIGHER THRESHOLD (0.25) & BRIEF PERIOD HOLD LOGIC (2.5 seconds)

            if abs(nx) < 0.25 and abs(ny) < 0.25:
                if self.xy_hold_timer is None:
                    self.xy_hold_timer = current_time
                    self.get_logger().info("XY Aligned! Holding lock briefly...")
                elif (current_time - self.xy_hold_timer) > 2.5:
                    self.blind_lock_timer = current_time
                    self.change_phase(Phase.LOCK, "XY lock stabilized. Ready to drop ball.")
            else:
                self.xy_hold_timer = None 

        elif self.current_phase == Phase.LOCK:

            # Continue running X/Y PIDs in LOCK phase so we don't drift away during drop sequence

            if self.bucket_visible:
                cmd.lateral = self.run_lateral_pid(nx, dt)
                cmd.forward = self.run_surge_pid(ny, dt)
                cmd.yaw = self.calculate_heading_lock()
                
            if current_time - self.blind_lock_timer > 3.0:
                self.get_logger().info("Maintaining position over bucket. DROPPING BALL NOW!", throttle_duration_sec=2.0)

        elif self.current_phase == Phase.SEARCH2:
            if self.bucket_visible and self.sees_blue:
                self.change_phase(Phase.ALIGN_XY, "Found BLUE target.")
                return
            
            cmd.lateral = self.pwm_neutral + (self.evasion_direction * 50)
            if (current_time - self.evasion_sweep_start) > 3.0:
                self.evasion_direction *= -1
                self.evasion_sweep_start = current_time

        # Output
        self.log_motion(cmd)
        self.cmd_pub.publish(cmd)
        self.publish_debug(cmd, nx, ny)
        
    # ==========================================
    # DEBUGGING
    # ==========================================
    def log_motion(self, cmd):
        if cmd.lateral > 1530:
            self.get_logger().info(f"-> Moving RIGHT (PWM: {cmd.lateral})", throttle_duration_sec=0.5)
        elif cmd.lateral < 1470:
            self.get_logger().info(f"<- Moving LEFT  (PWM: {cmd.lateral})", throttle_duration_sec=0.5)

        if cmd.forward > 1530:
            self.get_logger().info(f"^ Moving FWD    (PWM: {cmd.forward})", throttle_duration_sec=0.5)
        elif cmd.forward < 1470:
            self.get_logger().info(f"v Moving BACK   (PWM: {cmd.forward})", throttle_duration_sec=0.5)

    def publish_debug(self, cmd, nx, ny):
        state_str = PHASE_NAMES[self.current_phase]
        vis = "YES" if self.bucket_visible else "NO"
        color = "BLUE" if self.sees_blue else ("ORANGE" if self.sees_orange else "NONE")
        
        log_msg = (
            f"[{state_str:<13}] Vis:{vis}({color}) | "
            f"Err(nX,nY): {nx:>5.2f}, {ny:>5.2f} | "
            f"PWM(F,L,Y): {cmd.forward}, {cmd.lateral}, {cmd.yaw}"
        )
        self.debug_pub.publish(String(data=log_msg))
        self.get_logger().info(log_msg, throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = BucketControls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()