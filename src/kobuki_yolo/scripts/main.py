#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import mediapipe as mp
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters

mp_pose = mp.solutions.pose
mp_draw = mp.solutions.drawing_utils
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

bridge = CvBridge()

# Daftar landmark yang ingin diambil koordinatnya
landmark_ids = [23, 11, 13, 15]

def image_callback(color_msg, depth_msg):
    try:
        cv_image = bridge.imgmsg_to_cv2(color_msg, "bgr8")
        depth_image = bridge.imgmsg_to_cv2(depth_msg, "16UC1")  # Depth dalam format 16-bit
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error: {e}")
        return

    rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
    results = pose.process(rgb_image)

    if results.pose_landmarks:
        mp_draw.draw_landmarks(cv_image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        h, w, _ = cv_image.shape

        for landmark_id in landmark_ids:
            landmark = results.pose_landmarks.landmark[landmark_id]
            x, y = int(landmark.x * w), int(landmark.y * h)

            if 0 <= x < w and 0 <= y < h:
                z = depth_image[y, x] / 1000.0  # Konversi depth dari mm ke meter
                rospy.loginfo(f"Landmark {landmark_id}: (X: {x}, Y: {y}, Z: {z:.3f} m)")

                cv2.putText(cv_image, f"{landmark_id}: {z:.2f}m", (x, y), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow("Pose Tracking", cv_image)
    cv2.waitKey(1)

def main():
    rospy.init_node("mediapipe_pose_tracking", anonymous=True)
    
    image_sub = message_filters.Subscriber("/camera2/camera/color/image_raw", Image)
    depth_sub = message_filters.Subscriber("/camera2/camera/aligned_depth_to_color/image_raw", Image)
    
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], queue_size=10, slop=0.1)
    ts.registerCallback(image_callback)
    
    rospy.loginfo("MediaPipe Pose Tracking with Depth Node Started")
    rospy.spin()

if __name__ == "__main__":
    main()