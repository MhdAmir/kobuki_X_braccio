#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import mediapipe as mp
import math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import signal
from custom_msgs.msg import HumanPose

mp_pose = mp.solutions.pose
mp_draw = mp.solutions.drawing_utils
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
bridge = CvBridge()


# Landmark yang ingin diambil koordinatnya
landmark_ids = [23, 11, 13, 15]

pose_pub = rospy.Publisher("/human_pose", HumanPose, queue_size=10)

def calculate_angles(shoulder, wrist):
    """ Menghitung sudut arah berdasarkan koordinat shoulder dan wrist """
    dx = wrist[0] - shoulder[0]
    dy = wrist[1] - shoulder[1]
    dz = wrist[2] - shoulder[2]

    azimuth = math.degrees(math.atan2(dx, dz))  # Rotasi di bidang XZ
    elevation = math.degrees(math.atan2(dy, math.sqrt(dx**2 + dz**2)))
    
    return azimuth, elevation

def is_hand_raised(shoulder, wrist):
    """ Mendeteksi apakah tangan terangkat berdasarkan posisi wrist dan shoulder """
    return wrist[1] < shoulder[1]  # Jika y wrist lebih kecil dari y shoulder, tangan terangkat

def get_depth_coords(landmarks, landmark_id, depth_image, shape):
    """ Mendapatkan koordinat 3D menggunakan depth image dari RealSense """
    h, w = shape
    landmark = landmarks[landmark_id]
    x, y = int(landmark.x * w), int(landmark.y * h)
    
    if 0 <= x < w and 0 <= y < h:
        # Konversi depth dari mm ke meter
        z = depth_image[y, x] / 1000.0 if depth_image[y, x] > 0 else 0
        return np.array([landmark.x, landmark.y, z])
    
    return np.array([landmark.x, landmark.y, 0])

def image_callback(color_msg, depth_msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(color_msg, "bgr8")
        depth_image = bridge.imgmsg_to_cv2(depth_msg, "16UC1")
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")
        return

    rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    results = pose.process(rgb_image)

    pose_msg = HumanPose()


    if results.pose_landmarks:
        mp_draw.draw_landmarks(cv_image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        h, w, _ = cv_image.shape
        landmarks = results.pose_landmarks.landmark
        
        # Display depth values for selected landmarks
        for landmark_id in landmark_ids:
            landmark = landmarks[landmark_id]
            x, y = int(landmark.x * w), int(landmark.y * h)
            
            if 0 <= x < w and 0 <= y < h:
                z = depth_image[y, x] / 1000.0  # Konversi depth dari mm ke meter
                cv2.putText(cv_image, f"{landmark_id}: {z:.2f}m", (x, y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # Mendapatkan koordinat dari MediaPipe
        right_shoulder_mp = np.array([
            landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].x,
            landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].y,
            landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].z
        ])
        
        right_wrist_mp = np.array([
            landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].x,
            landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].y,
            landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].z
        ])
        
        left_shoulder_mp = np.array([
            landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].x,
            landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].y,
            landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].z
        ])
        
        left_wrist_mp = np.array([
            landmarks[mp_pose.PoseLandmark.LEFT_WRIST].x,
            landmarks[mp_pose.PoseLandmark.LEFT_WRIST].y,
            landmarks[mp_pose.PoseLandmark.LEFT_WRIST].z
        ])

        right_elbow_mp = np.array([
            landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].x,
            landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].y,
            landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].z
        ])
        
        left_elbow_mp = np.array([
            landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].x,
            landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].y,
            landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].z
        ])
        
        # Mendapatkan koordinat dari RealSense depth camera
        right_shoulder_rs = get_depth_coords(landmarks, mp_pose.PoseLandmark.RIGHT_SHOULDER, depth_image, (h, w))
        right_wrist_rs = get_depth_coords(landmarks, mp_pose.PoseLandmark.RIGHT_WRIST, depth_image, (h, w))
        left_shoulder_rs = get_depth_coords(landmarks, mp_pose.PoseLandmark.LEFT_SHOULDER, depth_image, (h, w))
        left_wrist_rs = get_depth_coords(landmarks, mp_pose.PoseLandmark.LEFT_WRIST, depth_image, (h, w))
        
        # Hitung azimuth awal menggunakan data MediaPipe untuk menentukan metode
        initial_azimuth, _ = calculate_angles(right_shoulder_mp, right_wrist_mp)
        
        # Pilih metode pengukuran berdasarkan nilai azimuth awal
        if 30 < abs(initial_azimuth) < 150:
            # Gunakan data RealSense untuk koordinat 3D
            right_shoulder_xyz = right_shoulder_rs
            right_wrist_xyz = right_wrist_rs
            left_shoulder_xyz = left_shoulder_rs
            left_wrist_xyz = left_wrist_rs
            source = "RealSense"
        else:
            # Gunakan data MediaPipe bawaan untuk koordinat 3D
            right_shoulder_xyz = right_shoulder_mp
            right_wrist_xyz = right_wrist_mp
            left_shoulder_xyz = left_shoulder_mp
            left_wrist_xyz = left_wrist_mp
            source = "MediaPipe"

        # Deteksi apakah tangan kanan atau kiri terangkat
        right_hand_raised = is_hand_raised(right_elbow_mp, right_wrist_xyz)
        left_hand_raised = is_hand_raised(left_elbow_mp, left_wrist_xyz)

        # Tampilkan teks di layar berdasarkan kondisi tangan
        if right_hand_raised and left_hand_raised:
            cv2.putText(cv_image, "Both Hands Raised", (50, 120), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        elif right_hand_raised:
            cv2.putText(cv_image, "Right Hand Raised", (50, 120), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        elif left_hand_raised:
            cv2.putText(cv_image, "Left Hand Raised", (50, 120), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(cv_image, "No Hands Raised", (50, 120), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Hitung sudut azimuth dan elevasi dari tangan kanan
        azimuth, elevation = calculate_angles(right_shoulder_xyz, right_wrist_xyz)

        cv2.putText(cv_image, f'Azimuth: {azimuth:.2f}° ({source})', (50, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        cv2.putText(cv_image, f'Elevation: {elevation:.2f}°', (50, 80), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        pose_msg = HumanPose()
        if right_hand_raised and left_hand_raised:
            pose_msg.detected_pose = 3
        elif right_hand_raised:
            pose_msg.detected_pose = 1
        elif left_hand_raised:
            pose_msg.detected_pose = 2
        else:
            pose_msg.detected_pose = 0
        
        if abs(elevation) < 25:
            pose_msg.azimuth = azimuth
    else :
        pose_msg.detected_pose = 0
        pose_msg.azimuth = 0
    
    pose_pub.publish(pose_msg)
    cv2.imshow("Pose Tracking", cv_image)
    cv2.waitKey(1)

    # Pastikan jendela OpenCV tertutup saat program dihentikan
    if cv2.getWindowProperty("Pose Tracking", cv2.WND_PROP_VISIBLE) < 1:
        rospy.signal_shutdown("Closing OpenCV window")
        cv2.destroyAllWindows()

def shutdown_handler(sig, frame):
    """ Menangani shutdown ketika Ctrl+C ditekan """
    rospy.loginfo("Shutting down...")
    cv2.destroyAllWindows()
    rospy.signal_shutdown("Ctrl+C pressed")

def cleanup():
    """ Fungsi untuk membersihkan sebelum program berhenti """
    rospy.loginfo("Cleaning up...")
    cv2.destroyAllWindows()

def main():
    rospy.init_node("mediapipe_pose_tracking", anonymous=True)

    # Tangani sinyal Ctrl+C
    signal.signal(signal.SIGINT, shutdown_handler)
    rospy.on_shutdown(cleanup)
    
    image_sub = message_filters.Subscriber("/camera2/camera/color/image_raw", Image)
    depth_sub = message_filters.Subscriber("/camera2/camera/aligned_depth_to_color/image_raw", Image)
    
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], queue_size=10, slop=0.1)
    ts.registerCallback(image_callback)
    
    rospy.spin()

if __name__ == "__main__":
    main()