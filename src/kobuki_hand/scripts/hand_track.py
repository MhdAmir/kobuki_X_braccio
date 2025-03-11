#!/usr/bin/env python3

import cv2
import mediapipe as mp
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import MotorPower, BumperEvent

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7)
mp_drawing = mp.solutions.drawing_utils

rospy.init_node('hand_gesture_control', anonymous=True)
velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
motor_power_publisher = rospy.Publisher('/mobile_base/commands/motor_power', MotorPower, queue_size=1)
rate = rospy.Rate(10)

bridge = CvBridge()
bumper_hit = False
current_speed = 0.0
max_speed = 0.3
acceleration = 0.005

def bumper_callback(msg):
    global bumper_hit
    bumper_hit = msg.state == BumperEvent.PRESSED

rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, bumper_callback)

def image_callback(msg):
    try:
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logerr(f"CvBridge Error: {e}")
        return

    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_frame)

    twist_msg = Twist()
    motor_msg = MotorPower()

    if bumper_hit:
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        motor_msg.state = MotorPower.OFF
        global current_speed
        current_speed = 0.0
    elif results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            if hand_landmarks.landmark[9].x > 0.55:
                twist_msg.angular.z = 0.66 if hand_landmarks.landmark[9].x > 0.8 else 0.33
                motor_msg.state = MotorPower.ON
            elif hand_landmarks.landmark[9].x < 0.45:
                twist_msg.angular.z = -0.66 if hand_landmarks.landmark[9].x < 0.2 else -0.33
                motor_msg.state = MotorPower.ON

            if abs(hand_landmarks.landmark[5].x - hand_landmarks.landmark[13].x) > 0.05:
                twist_msg.linear.x = 0.0
                current_speed = 0.0
            else:
                if current_speed < max_speed:
                    current_speed += acceleration
                twist_msg.linear.x = current_speed
                twist_msg.angular.z = min(twist_msg.angular.z, 0.33)
                motor_msg.state = MotorPower.ON

    velocity_publisher.publish(twist_msg)
    motor_power_publisher.publish(motor_msg)

    cv2.imshow('Hand Gesture Control', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("User requested shutdown")

rospy.Subscriber("/camera2/camera/color/image_raw", Image, image_callback)
rospy.on_shutdown(lambda: cv2.destroyAllWindows())
rospy.spin()
