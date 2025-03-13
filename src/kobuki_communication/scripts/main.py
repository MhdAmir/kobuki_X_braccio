#!/usr/bin/env python3

import rospy
from custom_msgs.msg import Comm  # Import pesan kustom
import socket
import threading
import time

class RobotUDPServer:
    def __init__(self, host='0.0.0.0', port=5006):
        rospy.init_node('communication_hp', anonymous=True)
        self.pub = rospy.Publisher('/communication_hp', Comm, queue_size=10)
        
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Tambahkan ini
        self.sock.bind((host, port))
        self.client_addr = None
        self.running = True
        self.current_mode = 0  
        
        self.linear_velocity = 0.0  # Linear velocity
        self.angular_velocity = 0.0  # Angular velocity

    def start(self):
        rospy.loginfo(f"UDP Server started on {self.host}:{self.port}")
        
        threading.Thread(target=self.send_status_updates, daemon=True).start()

        try:
            while self.running and not rospy.is_shutdown():
                data, addr = self.sock.recvfrom(1024)
                if self.client_addr is None or addr != self.client_addr:
                    rospy.loginfo(f"New client connected from {addr}")
                self.client_addr = addr
                self.handle_command(data, addr)
        except rospy.ROSInterruptException:
            rospy.loginfo("Server shutting down...")
        finally:
            self.sock.close()

    def handle_command(self, data, addr):
        if len(data) < 2:
            rospy.logwarn(f"Received invalid data from {addr}")
            return
            
        mode = data[0]
        command = data[1]
        rospy.loginfo(f"Received data from {addr}: Mode={mode}, Command={command}")
        
        if mode == 99:
            rospy.loginfo(f"Received ping from {addr}")
            self.send_response(addr)
            return
            
        self.current_mode = mode
        if mode == 2:  # Manual mode
            moves = [(0, 0), (0.3, 0.0), (0, 0.3), (0, -0.3), (-0.3, 0)]
            if 0 <= command < len(moves):
                if command == 0:
                    self.linear_velocity = moves[command][0]  # Linear velocity
                    self.angular_velocity = moves[command][1]  # Angular velocity

                else:
                    self.linear_velocity += moves[command][0]  # Linear velocity
                    self.angular_velocity += moves[command][1]  # Angular velocity
        elif mode in [0, 1, 3]:
            statuses = {0: "Taking Cup (Automatic)", 1: "Following Human", 3: "Going Home & Disconnecting"}
            rospy.loginfo(f"Mode: {statuses.get(mode, 'Unknown Mode')}")
            
        self.send_response(addr)
        self.publish_status()

    def send_response(self, addr):
        response = bytearray([self.current_mode])
        self.sock.sendto(response, addr)
    
    def send_status_updates(self):
        while self.running and not rospy.is_shutdown():
            if self.client_addr:
                self.publish_status()
            time.sleep(1)
    
    def publish_status(self):
        msg = Comm()
        msg.mode = self.current_mode
        msg.linear_velocity = self.linear_velocity
        msg.angular_velocity = self.angular_velocity
        self.pub.publish(msg)
        rospy.loginfo(f"Published: Mode={self.current_mode}, Linear={self.linear_velocity}, Angular={self.angular_velocity}")
    
    def stop(self):
        self.running = False

if __name__ == "__main__":
    server = RobotUDPServer()
    try:
        server.start()
    except rospy.ROSInterruptException:
        server.stop()
        rospy.loginfo("Server stopped")
