import rclpy
from rclpy.node import Node
import serial

from custom_msgs.msg import EmergencyKill

# Serial Configuration
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600  # Make sure this matches your Arduino/Microcontroller code

class KillSwitchPublisher(Node):
    def __init__(self):
        super().__init__('kill_switch_publisher')

        self.publisher_ = self.create_publisher(
            EmergencyKill,
            "/esp/telemetry",
            10
        )

        # Initialize Serial Connection
        try:
            self.ser = serial.Serial(
                SERIAL_PORT, 
                BAUD_RATE, 
                timeout=0.1
            )
            self.get_logger().info(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            # We don't exit here so the node stays alive, 
            # but you might want to add retry logic or exit depending on safety needs.
            self.ser = None

        # Track state: '0' = Safe, '1' = Kill
        self.last_state = '0' 

        self.get_logger().info("Kill switch armed (Serial Mode)")

        # Poll Serial at 50 Hz
        self.timer = self.create_timer(0.02, self.poll_serial)

    def poll_serial(self):
        if self.ser is None or not self.ser.is_open:
            return

        # Check if data is waiting in the buffer
        if self.ser.in_waiting > 0:
            try:
                # Read all available bytes and decode
                data_chunk = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                
                # We iterate through the chunk to handle cases where 
                # multiple characters arrive at once (e.g. "00010")
                for char in data_chunk:
                    if char == '1':
                        # If we receive a '1', and we weren't already in Kill state (or just want to ensure it sends)
                        if self.last_state == '0':
                            self.send_kill()
                        self.last_state = '1'
                        
                    elif char == '0':
                        self.last_state = '0'
                        
            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")

    def send_kill(self):
        msg = EmergencyKill()
        msg.kill_switch = True
        self.publisher_.publish(msg)
        self.get_logger().warn("Sending EmergencyKill signal (received '1' on serial).")

    def destroy_node(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KillSwitchPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
