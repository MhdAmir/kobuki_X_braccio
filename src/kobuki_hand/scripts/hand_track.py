#!/usr/bin/env python3

import cv2
import mediapipe as mp
import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import MotorPower, BumperEvent

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7)
mp_drawing = mp.solutions.drawing_utils

rospy.init_node('hand_gesture_control', anonymous=True)
velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
motor_power_publisher = rospy.Publisher('/mobile_base/commands/motor_power', MotorPower, queue_size=1)
rate = rospy.Rate(10)

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    rospy.logerr("Cannot Open Camera")
    exit(1)

# Flag to check if bumper is hit
bumper_hit = False
current_speed = 0.0  # Initialize speed for acceleration handling
max_speed = 0.3  # Maximum backward speed
acceleration = 0.005  # Acceleration rate

def bumper_callback(msg):
    global bumper_hit
    if msg.state == BumperEvent.PRESSED:
        bumper_hit = True
    else:
        bumper_hit = False

rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, bumper_callback)

def shutdown_hook():
    rospy.loginfo("Shutting down")
    cap.release()
    cv2.destroyAllWindows()

rospy.on_shutdown(shutdown_hook)

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        rospy.logwarn("Ignoring empty camera frame.")
        continue

    frame = cv2.flip(frame, 1)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_frame)

    twist_msg = Twist()
    motor_msg = MotorPower()

    if bumper_hit:
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        motor_msg.state = MotorPower.OFF
        current_speed = 0.0  # Reset speed when stopping
    elif results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            if hand_landmarks.landmark[9].x > 0.55:
                if hand_landmarks.landmark[9].x > 0.8:
                    twist_msg.angular.z = 0.66
                else:
                    twist_msg.angular.z = 0.33
                motor_msg.state = MotorPower.ON

            elif hand_landmarks.landmark[9].x < 0.45:
                if hand_landmarks.landmark[9].x < 0.2:
                    twist_msg.angular.z = -0.66
                else:
                    twist_msg.angular.z = -0.33
                motor_msg.state = MotorPower.ON

            if abs(hand_landmarks.landmark[5].x - hand_landmarks.landmark[13].x) > 0.05:
                twist_msg.linear.x = 0.0
                current_speed = 0.0  # Reset speed when stopping
            else:
                if current_speed < max_speed:
                    current_speed += acceleration
                twist_msg.linear.x = current_speed
                if twist_msg.angular.z > 0.33:
                    twist_msg.angular.z = 0.33
                motor_msg.state = MotorPower.ON

    velocity_publisher.publish(twist_msg)
    motor_power_publisher.publish(motor_msg)

    cv2.imshow('Hand Gesture Control', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("User requested shutdown")
        break

    rate.sleep()

cap.release()
cv2.destroyAllWindows()
