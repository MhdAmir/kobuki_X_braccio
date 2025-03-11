#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np

# Inisialisasi MediaPipe
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

# Inisialisasi CvBridge
bridge = CvBridge()

# Callback untuk subscriber ROS
def image_callback(color_msg):
    try:
        # Convert ROS Image message to OpenCV image
        color_image = bridge.imgmsg_to_cv2(color_msg, "bgr8")
    except Exception as e:
        print(f"Error converting image: {e}")
        return

    # Turunkan resolusi gambar untuk mempercepat pemrosesan
    scale_percent = 50  # Resolusi 50% dari aslinya
    width = int(color_image.shape[1] * scale_percent / 100)
    height = int(color_image.shape[0] * scale_percent / 100)
    color_image = cv2.resize(color_image, (width, height))

    # Process the image with MediaPipe Pose
    with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
        # Convert the BGR image to RGB
        image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

        # Process the image and detect the pose
        results = pose.process(image_rgb)

        # Gambar landmark di frame
        if results.pose_landmarks:
            print("Landmark terdeteksi!")
            # Gambar landmark dan koneksi di frame
            mp_drawing.draw_landmarks(
                color_image,  # Gambar di frame asli (BGR)
                results.pose_landmarks,  # Landmark yang terdeteksi
                mp_pose.POSE_CONNECTIONS,  # Koneksi antara landmark
                mp_drawing.DrawingSpec(color=(0, 255, 0)),  # Warna landmark
                mp_drawing.DrawingSpec(color=(0, 0, 255)))  # Warna koneksi
        else:
            print("Tidak ada landmark yang terdeteksi.")

        # Tampilkan frame dengan landmark
        cv2.imshow("RealSense Frame with Pose Detection", color_image)
        cv2.waitKey(1)

def main():
    rospy.init_node('mediapipe_realsense', anonymous=True)

    # Subscribe ke topic gambar warna dengan queue size yang lebih besar
    rospy.Subscriber('/camera2/camera/color/image_raw', Image, image_callback, queue_size=0)

    print("Node telah berjalan. Menunggu data dari RealSense...")

    # Batasi frekuensi pemrosesan
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        rate.sleep()

    # Tutup semua OpenCV windows saat node dihentikan
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()