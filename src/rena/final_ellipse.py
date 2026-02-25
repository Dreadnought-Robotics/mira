import cv2
import numpy as np
import math
#ros
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from scipy.spatial.transform import Rotation as R
import time

#VIDEO SOURCE
cap = cv2.VideoCapture("ellipsoid_bucket.mp4")

if not cap.isOpened():
    print("Cannot open video")
    exit()

cv2.namedWindow("Frame")
cv2.namedWindow("Mask")
    #Normalize frame
   

cv2.namedWindow("Frame")
cv2.namedWindow("Mask")

# CAMERA CALIBRATION
"""camera_matrix = np.array([
    [985.11136352, 0.0, 648.59492688],
    [0.0, 986.4461004, 481.72760972],
    [0.0, 0.0, 1.0]
], dtype=np.float32)

dist_coeffs = np.array([
    [2.46701940e-01,
     -1.46748990e+00,
     1.22600199e-03,
     5.78468636e-03,
     1.66373263e+00]
], dtype=np.float32)"""

"""camera_matrix = np.array([
 [1.33663561e+03,0.00000000e+00, 6.92881477e+02],
 [0.00000000e+00,1.33913474e+03, 3.56580140e+02],
 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
], dtype=np.float32)

dist_coeffs = np.array([
    [-0.46938636 , 0.44715133 ,-0.00117623 , 0.00088666 ,-0.38332216]
], dtype=np.float32)"""

camera_matrix = np.array([
    [870.096283, 0.0, 325.084678],
    [0.0, 872.828862, 121.313663],
    [0.0, 0.0, 1.0]
], dtype=np.float32)

dist_coeffs = np.array([
    [-0.474217, 0.337795, 0.007489, 0.002760, 0.0]
], dtype=np.float32)

# BUCKET GEOMETRY
Radius = 0.235

object_points = np.array([
    (0, -Radius, 0),
    (Radius, 0, 0),
    (0, Radius, 0),
    (-Radius, 0, 0)
], dtype=np.float32)

#ros
class BucketPerception(Node):

    def __init__(self):
        super().__init__("bucket_perception")

        self.pose_pub = self.create_publisher(
            PoseStamped,
            "ellipsoid_pose",
            10
        )

        self.color_pub = self.create_publisher(
            String,
            "bucket_clr",
            10
        )

# HELPER FUNCTION
def get_ellipse_rim_points(cx, cy, A, B, angle_deg):
    angle = np.deg2rad(angle_deg)
    a = A / 2
    b = B / 2

    cos_t = np.cos(angle)
    sin_t = np.sin(angle)

    ux = cos_t
    uy = sin_t
    vx = -sin_t
    vy = cos_t

    right  = (cx + a * ux, cy + a * uy)
    left   = (cx - a * ux, cy - a * uy)
    top    = (cx + b * vx, cy + b * vy)
    bottom = (cx - b * vx, cy - b * vy)

    return np.array([top, right, bottom, left], dtype=np.float32)

wb = cv2.xphoto.createGrayworldWB()

#ros
rclpy.init()
node = BucketPerception()

#MAIN LOOP
while True:
    ret, frame = cap.read()
    if not ret:
        break
    h, w = frame.shape[:2]
    cv2.normalize(frame,frame, 0, 255, norm_type=cv2.NORM_MINMAX)
    frame = wb.balanceWhite(frame)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    blur = cv2.GaussianBlur(hsv, (11, 11), 0)

    #BLUE MASK
    lower_blue = np.array([90, 110, 0])
    upper_blue = np.array([110, 255, 190])
    mask_blue = cv2.inRange(blur, lower_blue, upper_blue)

    #ORANGE MASK
    lower_orange1 = np.array([0, 80, 30])
    upper_orange1 = np.array([33, 255, 255])

    lower_orange2 = np.array([170, 100, 30])
    upper_orange2 = np.array([179, 255, 255])

    mask_orange1 = cv2.inRange(blur, lower_orange1, upper_orange1)
    mask_orange2 = cv2.inRange(blur, lower_orange2, upper_orange2)

    mask_orange = cv2.bitwise_or(mask_orange1, mask_orange2)

    #COMBINED MASK
    mask = cv2.bitwise_or(mask_blue, mask_orange)

    # Morphological cleaning
    kernel = np.ones((9, 9), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    cv2.imshow("Mask", mask)

    #CONTOURS
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best_area = 0
    best_ellipse = None
    detected_color = None


    # CONTOUR SELECTION
    for cnt in contours:
        if len(cnt) < 5:
            continue

        area = cv2.contourArea(cnt)

        try:
            ellipse = cv2.fitEllipse(cnt)
        except:
            continue
        if area < 2000 or (1-(ellipse[1][0]**2/ellipse[1][1]**2))**0.5 > 0.60:
            continue

        # Color detection inside contour
        mask_roi = np.zeros_like(mask)
        cv2.drawContours(mask_roi, [cnt], -1, 255, -1)

        blue_pixels = cv2.countNonZero(cv2.bitwise_and(mask_blue, mask_roi))
        orange_pixels = cv2.countNonZero(cv2.bitwise_and(mask_orange, mask_roi))

        if best_ellipse is None:
            best_ellipse = ellipse
            best_area = area
            detected_color = "Blue Bucket" if blue_pixels > orange_pixels else "Orange Bucket"
        else:
            if area > best_area:
                best_ellipse = ellipse
                best_area = area
                detected_color = "Blue Bucket" if blue_pixels > orange_pixels else "Orange Bucket"

    # IF ELLIPSE FOUND 
    if best_ellipse is not None:
        (cx, cy), (A, B), angle = best_ellipse

        cv2.ellipse(frame, best_ellipse, (0, 255, 0), 3)

        image_points = get_ellipse_rim_points(cx, cy, A, B, angle)

        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE
        )

        if success:
            tx, ty, tz = tvec.flatten()
            print(f"{detected_color} | tx={tx:.3f}  ty={ty:.3f}  tz={tz:.3f}")

            cv2.putText(frame,
                        f"tz = {tz:.2f} m",
                        (50, 90),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 255, 0),
                        2)
            
             # ros
            # convert to quaternion
            rot_matrix, _ = cv2.Rodrigues(rvec)
            quat = R.from_matrix(rot_matrix).as_quat()

            pose_msg = PoseStamped()
            pose_msg.header.stamp = node.get_clock().now().to_msg()
            pose_msg.header.frame_id = "camera_link"

            pose_msg.pose.position.x = float(tx)
            pose_msg.pose.position.y = float(ty)
            pose_msg.pose.position.z = float(tz)

            pose_msg.pose.orientation.x = float(quat[0])
            pose_msg.pose.orientation.y = float(quat[1])
            pose_msg.pose.orientation.z = float(quat[2])
            pose_msg.pose.orientation.w = float(quat[3])

            node.pose_pub.publish(pose_msg)

            # publish bucket colour
            color_msg = String()
            color_msg.data = "blue" if detected_color == "Blue Bucket" else "orange"
            node.color_pub.publish(color_msg)
            # end of ros

        if detected_color:
            cv2.putText(frame,
                        detected_color,
                        (50, 50),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 255, 0),
                        2)
        # ros
        rclpy.spin_once(node, timeout_sec=0.0)

    cv2.imshow("Frame", frame)

    if cv2.waitKey(30) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()