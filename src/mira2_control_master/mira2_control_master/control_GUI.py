#!/usr/bin/env python3

import sys
import threading
import math
import rclpy
from rclpy.node import Node

# Import your custom messages
from custom_msgs.msg import Commands, Telemetry

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QLabel, 
                             QGridLayout, QGroupBox, QTabWidget, QHBoxLayout)
from PyQt5.QtCore import pyqtSignal, QObject, Qt, QPoint, QRectF
from PyQt5.QtGui import QPainter, QPen, QBrush, QColor, QPolygon, QFont

# -------------------------------------------------------------------------
# Qt Signal Emitter
# -------------------------------------------------------------------------
class RosSignals(QObject):
    command_received = pyqtSignal(Commands)
    telemetry_received = pyqtSignal(Telemetry)

# -------------------------------------------------------------------------
# Custom Compass Widget
# -------------------------------------------------------------------------
class CompassWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(120, 120)
        self.heading = 0.0

    def set_heading(self, angle):
        self.heading = angle
        self.update()  # Trigger a repaint whenever the heading changes

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        width = self.width()
        height = self.height()
        center = QPoint(int(width / 2), int(height / 2))
        radius = min(width, height) / 2 - 10

        # Draw compass background/border
        pen = QPen(QColor("#45475A"), 2)
        painter.setPen(pen)
        painter.setBrush(QBrush(QColor("#181825")))
        painter.drawEllipse(center, int(radius), int(radius))

        # Draw N, E, S, W Labels
        painter.setPen(QPen(QColor("#CDD6F4")))
        painter.setFont(QFont("Segoe UI", 10, QFont.Bold))
        painter.drawText(int(center.x() - 5), int(center.y() - radius + 15), "N")
        painter.drawText(int(center.x() - 4), int(center.y() + radius - 5), "S")
        painter.drawText(int(center.x() + radius - 15), int(center.y() + 5), "E")
        painter.drawText(int(center.x() - radius + 5), int(center.y() + 5), "W")

        # Draw the Needle
        painter.save() # Save the current coordinate system
        painter.translate(center) # Move origin to the center of the widget
        painter.rotate(self.heading) # Rotate the coordinate system by the heading

        # Define the needle polygon (pointing UP relative to the rotated canvas)
        needle_head = QPolygon([
            QPoint(0, int(-radius + 20)),  # Tip
            QPoint(-8, 0),                 # Bottom Left
            QPoint(8, 0)                   # Bottom Right
        ])
        
        needle_tail = QPolygon([
            QPoint(-8, 0),                 # Top Left
            QPoint(8, 0),                  # Top Right
            QPoint(0, int(radius - 30))    # Bottom Tip
        ])

        # Draw North-pointing half (Red)
        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor("#F38BA8"))) 
        painter.drawPolygon(needle_head)

        # Draw South-pointing half (White/Grey)
        painter.setBrush(QBrush(QColor("#A6ADC8")))
        painter.drawPolygon(needle_tail)

        # Draw center pivot dot
        painter.setBrush(QBrush(QColor("#CDD6F4")))
        painter.drawEllipse(QPoint(0,0), 4, 4)

        painter.restore() # Restore the original coordinate system

# -------------------------------------------------------------------------
# ROS 2 Node
# -------------------------------------------------------------------------
class DashboardNode(Node):
    def __init__(self, signals):
        super().__init__("rov_dashboard_node")
        self.signals = signals
        
        self.cmd_sub = self.create_subscription(Commands, "/master/commands", self.cmd_callback, 10)
        self.telem_sub = self.create_subscription(Telemetry, "/master/telemetry", self.telem_callback, 10)
        self.get_logger().info("Dashboard Initialized. Listening to /master/commands and /master/telemetry")

    def cmd_callback(self, msg):
        self.signals.command_received.emit(msg)

    def telem_callback(self, msg):
        self.signals.telemetry_received.emit(msg)

# -------------------------------------------------------------------------
# PyQt5 GUI Application
# -------------------------------------------------------------------------
class ROVDashboard(QWidget):
    def __init__(self):
        super().__init__()
        self.connected_cmd = False
        self.connected_telem = False
        self.initUI()

    def initUI(self):
        self.setWindowTitle("ROV Master Dashboard")
        self.resize(500, 750)
        
        self.setStyleSheet("""
            QWidget { background-color: #1E1E2E; color: #CDD6F4; font-family: 'Segoe UI', Arial, sans-serif; }
            QTabWidget::pane { border: 1px solid #45475A; border-radius: 4px; }
            QTabBar::tab { background: #313244; padding: 8px 20px; margin-right: 2px; border-top-left-radius: 4px; border-top-right-radius: 4px; }
            QTabBar::tab:selected { background: #89B4FA; color: #1E1E2E; font-weight: bold; }
            QGroupBox { border: 1px solid #45475A; border-radius: 6px; margin-top: 15px; font-weight: bold; padding-top: 15px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; color: #89B4FA; }
            .ValueLabel { font-weight: bold; color: #A6E3A1; font-size: 14px; }
            .StatusLabel { font-weight: bold; font-size: 14px; padding: 5px; border-radius: 4px; }
        """)

        main_layout = QVBoxLayout()

        self.lbl_status = QLabel("Status: Waiting for Data...")
        self.lbl_status.setProperty("class", "StatusLabel")
        self.lbl_status.setStyleSheet("color: #F9E2AF; background-color: #313244;")
        self.lbl_status.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(self.lbl_status)

        tabs = QTabWidget()
        self.tab_commands = QWidget()
        self.tab_telemetry = QWidget()
        
        tabs.addTab(self.tab_commands, "Control Inputs")
        tabs.addTab(self.tab_telemetry, "Telemetry & Sensors")
        
        self.setup_commands_tab()
        self.setup_telemetry_tab()
        
        main_layout.addWidget(tabs)
        self.setLayout(main_layout)

    def create_value_label(self, text, color=None):
        label = QLabel(text)
        label.setProperty("class", "ValueLabel")
        if color:
            label.setStyleSheet(f"color: {color};")
        return label

    def setup_commands_tab(self):
        layout = QVBoxLayout()
        status_grp = QGroupBox("Target State"); status_lyt = QGridLayout()
        self.cmd_arm = self.create_value_label("DISARMED", "#F38BA8")
        self.cmd_mode = self.create_value_label("MANUAL", "#F9E2AF")
        status_lyt.addWidget(QLabel("Arm Cmd:"), 0, 0); status_lyt.addWidget(self.cmd_arm, 0, 1, Qt.AlignRight)
        status_lyt.addWidget(QLabel("Mode Cmd:"), 1, 0); status_lyt.addWidget(self.cmd_mode, 1, 1, Qt.AlignRight)
        status_grp.setLayout(status_lyt)

        move_grp = QGroupBox("Target Movement (PWM)"); move_lyt = QGridLayout()
        self.cmd_fwd = self.create_value_label("1500"); self.cmd_lat = self.create_value_label("1500")
        self.cmd_thr = self.create_value_label("1500"); self.cmd_pch = self.create_value_label("1500")
        self.cmd_rol = self.create_value_label("1500"); self.cmd_yaw = self.create_value_label("1500")
        move_lyt.addWidget(QLabel("Forward:"), 0, 0); move_lyt.addWidget(self.cmd_fwd, 0, 1, Qt.AlignRight)
        move_lyt.addWidget(QLabel("Lateral:"), 1, 0); move_lyt.addWidget(self.cmd_lat, 1, 1, Qt.AlignRight)
        move_lyt.addWidget(QLabel("Thrust:"), 2, 0); move_lyt.addWidget(self.cmd_thr, 2, 1, Qt.AlignRight)
        move_lyt.addWidget(QLabel("Pitch:"), 0, 2); move_lyt.addWidget(self.cmd_pch, 0, 3, Qt.AlignRight)
        move_lyt.addWidget(QLabel("Roll:"), 1, 2); move_lyt.addWidget(self.cmd_rol, 1, 3, Qt.AlignRight)
        move_lyt.addWidget(QLabel("Yaw:"), 2, 2); move_lyt.addWidget(self.cmd_yaw, 2, 3, Qt.AlignRight)
        move_grp.setLayout(move_lyt)

        servo_grp = QGroupBox("Target Servos"); servo_lyt = QGridLayout()
        self.cmd_srv1 = self.create_value_label("1500"); self.cmd_srv2 = self.create_value_label("1500")
        servo_lyt.addWidget(QLabel("Servo 1:"), 0, 0); servo_lyt.addWidget(self.cmd_srv1, 0, 1, Qt.AlignRight)
        servo_lyt.addWidget(QLabel("Servo 2:"), 1, 0); servo_lyt.addWidget(self.cmd_srv2, 1, 1, Qt.AlignRight)
        servo_grp.setLayout(servo_lyt)

        layout.addWidget(status_grp); layout.addWidget(move_grp); layout.addWidget(servo_grp); layout.addStretch()
        self.tab_commands.setLayout(layout)

    def setup_telemetry_tab(self):
        layout = QVBoxLayout()
        
        sys_grp = QGroupBox("System Vitals"); sys_lyt = QGridLayout()
        self.tlm_arm = self.create_value_label("DISARMED", "#F38BA8")
        self.tlm_bat = self.create_value_label("0.0 V")
        self.tlm_time = self.create_value_label("0.0 s")
        sys_lyt.addWidget(QLabel("Actual Arm State:"), 0, 0); sys_lyt.addWidget(self.tlm_arm, 0, 1, Qt.AlignRight)
        sys_lyt.addWidget(QLabel("Battery Voltage:"), 1, 0); sys_lyt.addWidget(self.tlm_bat, 1, 1, Qt.AlignRight)
        sys_lyt.addWidget(QLabel("Uptime:"), 0, 2); sys_lyt.addWidget(self.tlm_time, 0, 3, Qt.AlignRight)
        sys_grp.setLayout(sys_lyt)

        env_grp = QGroupBox("Environment & Nav"); env_lyt = QHBoxLayout()
        env_text_lyt = QGridLayout()
        self.tlm_in_press = self.create_value_label("0.0 hPa")
        self.tlm_out_press = self.create_value_label("0.0 hPa")
        self.tlm_heading_txt = self.create_value_label("0°")
        env_text_lyt.addWidget(QLabel("Int. Pressure:"), 0, 0); env_text_lyt.addWidget(self.tlm_in_press, 0, 1, Qt.AlignLeft)
        env_text_lyt.addWidget(QLabel("Ext. Pressure:"), 1, 0); env_text_lyt.addWidget(self.tlm_out_press, 1, 1, Qt.AlignLeft)
        env_text_lyt.addWidget(QLabel("Heading (Raw):"), 2, 0); env_text_lyt.addWidget(self.tlm_heading_txt, 2, 1, Qt.AlignLeft)
        
        # Add the custom visual compass widget
        self.compass_widget = CompassWidget()
        
        env_lyt.addLayout(env_text_lyt)
        env_lyt.addStretch()
        env_lyt.addWidget(self.compass_widget) # Place the compass on the right side
        env_grp.setLayout(env_lyt)

        ori_grp = QGroupBox("Orientation (Euler)"); ori_lyt = QGridLayout()
        self.tlm_roll = self.create_value_label("0.0"); self.tlm_pitch = self.create_value_label("0.0"); self.tlm_yaw = self.create_value_label("0.0")
        self.tlm_rspeed = self.create_value_label("0.0"); self.tlm_pspeed = self.create_value_label("0.0"); self.tlm_yspeed = self.create_value_label("0.0")
        ori_lyt.addWidget(QLabel("Roll (rad):"), 0, 0); ori_lyt.addWidget(self.tlm_roll, 0, 1, Qt.AlignRight)
        ori_lyt.addWidget(QLabel("Pitch (rad):"), 1, 0); ori_lyt.addWidget(self.tlm_pitch, 1, 1, Qt.AlignRight)
        ori_lyt.addWidget(QLabel("Yaw (rad):"), 2, 0); ori_lyt.addWidget(self.tlm_yaw, 2, 1, Qt.AlignRight)
        ori_lyt.addWidget(QLabel("Roll Spd:"), 0, 2); ori_lyt.addWidget(self.tlm_rspeed, 0, 3, Qt.AlignRight)
        ori_lyt.addWidget(QLabel("Pitch Spd:"), 1, 2); ori_lyt.addWidget(self.tlm_pspeed, 1, 3, Qt.AlignRight)
        ori_lyt.addWidget(QLabel("Yaw Spd:"), 2, 2); ori_lyt.addWidget(self.tlm_yspeed, 2, 3, Qt.AlignRight)
        ori_grp.setLayout(ori_lyt)

        thr_grp = QGroupBox("Actual Thruster PWMs"); thr_lyt = QGridLayout()
        self.tlm_pwms = [self.create_value_label("0") for _ in range(8)]
        for i in range(4):
            thr_lyt.addWidget(QLabel(f"M{i+1}:"), i, 0); thr_lyt.addWidget(self.tlm_pwms[i], i, 1, Qt.AlignRight)
            thr_lyt.addWidget(QLabel(f"M{i+5}:"), i, 2); thr_lyt.addWidget(self.tlm_pwms[i+4], i, 3, Qt.AlignRight)
        thr_grp.setLayout(thr_lyt)

        layout.addWidget(sys_grp); layout.addWidget(env_grp); layout.addWidget(ori_grp); layout.addWidget(thr_grp); layout.addStretch()
        self.tab_telemetry.setLayout(layout)

    def check_connection(self):
        if self.connected_cmd or self.connected_telem:
            self.lbl_status.setText("Status: LIVE DATA CONNECTED")
            self.lbl_status.setStyleSheet("color: #1E1E2E; background-color: #A6E3A1;")

    def update_cmd_ui(self, msg: Commands):
        self.connected_cmd = True
        self.check_connection()
        if msg.arm:
            self.cmd_arm.setText("ARMED"); self.cmd_arm.setStyleSheet("color: #A6E3A1;")
        else:
            self.cmd_arm.setText("DISARMED"); self.cmd_arm.setStyleSheet("color: #F38BA8;")
        self.cmd_mode.setText(str(msg.mode) if msg.mode else "MANUAL")
        self.cmd_fwd.setText(str(msg.forward)); self.cmd_lat.setText(str(msg.lateral))
        self.cmd_thr.setText(str(msg.thrust)); self.cmd_pch.setText(str(msg.pitch))
        self.cmd_rol.setText(str(msg.roll)); self.cmd_yaw.setText(str(msg.yaw))
        self.cmd_srv1.setText(str(msg.servo1)); self.cmd_srv2.setText(str(msg.servo2))

    def update_telemetry_ui(self, msg: Telemetry):
        self.connected_telem = True
        self.check_connection()
        
        if msg.arm:
            self.tlm_arm.setText("ARMED"); self.tlm_arm.setStyleSheet("color: #A6E3A1;")
        else:
            self.tlm_arm.setText("DISARMED"); self.tlm_arm.setStyleSheet("color: #F38BA8;")
            
        self.tlm_bat.setText(f"{msg.battery_voltage:.2f} V")
        if msg.battery_voltage < 14.0 and msg.battery_voltage > 1.0:
            self.tlm_bat.setStyleSheet("color: #F38BA8;")
        else:
            self.tlm_bat.setStyleSheet("color: #A6E3A1;")
            
        self.tlm_time.setText(f"{msg.timestamp:.1f} s")
        self.tlm_in_press.setText(f"{msg.internal_pressure:.2f}")
        self.tlm_out_press.setText(f"{msg.external_pressure:.2f}")
        
        # Update raw text AND visual compass needle
        self.tlm_heading_txt.setText(f"{msg.heading}°")
        self.compass_widget.set_heading(float(msg.heading)) # Triggers compass needle to rotate

        self.tlm_roll.setText(f"{msg.roll:.3f}"); self.tlm_pitch.setText(f"{msg.pitch:.3f}"); self.tlm_yaw.setText(f"{msg.yaw:.3f}")
        self.tlm_rspeed.setText(f"{msg.rollspeed:.3f}"); self.tlm_pspeed.setText(f"{msg.pitchspeed:.3f}"); self.tlm_yspeed.setText(f"{msg.yawspeed:.3f}")

        for i in range(8):
            self.tlm_pwms[i].setText(f"{int(msg.thruster_pwms[i])}")

# -------------------------------------------------------------------------
# Main execution
# -------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    ros_signals = RosSignals()
    gui = ROVDashboard()
    
    ros_signals.command_received.connect(gui.update_cmd_ui)
    ros_signals.telemetry_received.connect(gui.update_telemetry_ui)
    
    ros_node = DashboardNode(ros_signals)
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    gui.show()
    exit_code = app.exec_()

    ros_node.get_logger().info("Shutting down GUI...")
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == "__main__":
    main()