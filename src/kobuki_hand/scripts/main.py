#!/usr/bin/env python3

import socket
import rospy
import re
import select
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import MotorPower, BumperEvent

rospy.init_node('udp_control', anonymous=True)
velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
motor_power_publisher = rospy.Publisher('/mobile_base/commands/motor_power', MotorPower, queue_size=1)
rate = rospy.Rate(10)

UDP_IP = "0.0.0.0"
UDP_PORT = 12345
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

bumper_hit = False
current_speed = 0.0  # Initialize speed for acceleration handling
max_speed = 0.1  # Maximum forward speed
acceleration = 0.02  # Acceleration rate
last_command = "stop"  # Store last valid command

valid_commands = {"up", "down", "left", "right", "stop"}

def bumper_callback(msg):
    global bumper_hit
    if msg.state == BumperEvent.PRESSED:
        bumper_hit = True
    else:
        bumper_hit = False

rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, bumper_callback)

def shutdown_hook():
    rospy.loginfo("Shutting down UDP control...")
    sock.close()

rospy.on_shutdown(shutdown_hook)

while not rospy.is_shutdown():
    ready, _, _ = select.select([sock], [], [], 0.1)  # Timeout 100ms
    if ready:
        data, addr = sock.recvfrom(1024)
        received_text = data.decode().strip()
    else:
        received_text = ""

    if received_text:  # Hanya proses jika ada data baru
        command_match = re.findall(r'\b(up|down|left|right|stop)\b', received_text)
        command = command_match[0] if command_match else last_command
    else:
        command = last_command  # Gunakan command terakhir jika tidak ada data baru

    print(f"Loop running... Received command: {command}")

    twist_msg = Twist()
    motor_msg = MotorPower()

    print(f"Raw received data: {received_text}")  
    print(f"Received command: {command}")  

    if command != last_command:
        print(f"Command changed: {last_command} -> {command}")

    print("Loop running...")

    
    if bumper_hit:
        # Stop immediately if bumper is pressed
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        motor_msg.state = MotorPower.OFF
        command = "stop"
        current_speed = 0.0  # Reset speed when stopping
    else:
        if command == "up":
            if current_speed < max_speed:
                current_speed += acceleration
            twist_msg.linear.x = current_speed  # Accelerate forward
            motor_msg.state = MotorPower.ON
        elif command == "down":
            if current_speed > -max_speed:
                current_speed -= acceleration
            twist_msg.linear.x = current_speed  # Accelerate backward
            motor_msg.state = MotorPower.ON
        elif command == "right":
            twist_msg.angular.z = -0.33  # Turn right
            motor_msg.state = MotorPower.ON
        elif command == "left":
            twist_msg.angular.z = 0.33  # Turn left
            motor_msg.state = MotorPower.ON
        elif command == "stop":
            twist_msg.linear.x = 0.0  # Stop
            twist_msg.angular.z = 0.0
            motor_msg.state = MotorPower.OFF
            current_speed = 0.0  # Reset speed when stopping

        last_command = command  # Save the last valid command

    velocity_publisher.publish(twist_msg)
    motor_power_publisher.publish(motor_msg)
    
    rate.sleep()

sock.close()



# import cv2
# import mediapipe as mp
# import rospy
# from geometry_msgs.msg import Twist
# from kobuki_msgs.msg import MotorPower, BumperEvent

# mp_hands = mp.solutions.hands
# hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7)
# mp_drawing = mp.solutions.drawing_utils

# rospy.init_node('hand_gesture_control', anonymous=True)
# velocity_publisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
# motor_power_publisher = rospy.Publisher('/mobile_base/commands/motor_power', MotorPower, queue_size=1)
# rate = rospy.Rate(10)  

# cap = cv2.VideoCapture(0)
# if not cap.isOpened():
#     rospy.logerr("Failed to open camera.")
#     exit(1)

# bumper_hit = False
# current_speed = 0.0  # Initialize speed for acceleration handling
# max_speed = 0.1  # Maximum forward speed
# acceleration = 0.02  # Acceleration rate

# def bumper_callback(msg):
#     global bumper_hit
#     if msg.state == BumperEvent.PRESSED:
#         bumper_hit = True
#     else:
#         bumper_hit = False

# rospy.Subscriber("/mobile_base/events/bumper", BumperEvent, bumper_callback)

# def count_fingers(hand_landmarks):
#     tip_ids = [4, 8, 12, 16, 20]  # Thumb, index, middle, ring, pinky
#     fingers = []

#     if hand_landmarks.landmark[tip_ids[0]].x > hand_landmarks.landmark[tip_ids[0] - 1].x:
#         fingers.append(1)  # Open
#     else:
#         fingers.append(0)  # Closed

#     for i in range(1, 5):
#         if hand_landmarks.landmark[tip_ids[i]].y < hand_landmarks.landmark[tip_ids[i] - 2].y:
#             fingers.append(1)  # Open
#         else:
#             fingers.append(0)  # Closed

#     return fingers.count(1)  # Count open fingers

# def shutdown_hook():
#     rospy.loginfo("Shutting down hand gesture control...")
#     cap.release()
#     cv2.destroyAllWindows()

# rospy.on_shutdown(shutdown_hook)

# while not rospy.is_shutdown():
#     ret, frame = cap.read()
#     if not ret:
#         rospy.logwarn("Ignoring empty camera frame.")
#         continue

#     frame = cv2.flip(frame, 1)  # Mirror effect
#     rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
#     results = hands.process(rgb_frame)

#     twist_msg = Twist()
#     motor_msg = MotorPower()

#     if bumper_hit:
#         twist_msg.linear.x = 0.0
#         twist_msg.angular.z = 0.0
#         motor_msg.state = MotorPower.OFF
#         current_speed = 0.0  # Reset speed when stopping
#     elif results.multi_hand_landmarks:
#         for hand_landmarks in results.multi_hand_landmarks:
#             mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

#             finger_count = count_fingers(hand_landmarks)

#             if finger_count == 1:
#                 if current_speed < max_speed:
#                     current_speed += acceleration
#                 twist_msg.linear.x = current_speed  # Accelerate forward
#                 motor_msg.state = MotorPower.ON
#             elif finger_count == 2:
#                 if current_speed > -max_speed:
#                     current_speed -= acceleration
#                 twist_msg.linear.x = current_speed  # Accelerate backward
#                 motor_msg.state = MotorPower.ON
#             elif finger_count == 3:
#                 twist_msg.angular.z = -0.33  # Turn right
#                 motor_msg.state = MotorPower.ON
#             elif finger_count == 4:
#                 twist_msg.angular.z = 0.33  # Turn left
#                 motor_msg.state = MotorPower.ON
#             elif finger_count == 5:
#                 twist_msg.linear.x = 0.0  # Stop
#                 twist_msg.angular.z = 0.0
#                 motor_msg.state = MotorPower.OFF
#                 current_speed = 0.0  # Reset speed when stopping

#             cv2.putText(frame, f"Fingers: {finger_count}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

#     velocity_publisher.publish(twist_msg)
#     motor_power_publisher.publish(motor_msg)

#     cv2.imshow('Hand Gesture Control', frame)

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         rospy.signal_shutdown("User requested shutdown")
#         break

#     rate.sleep()

# cap.release()
# cv2.destroyAllWindows()