import socket
import threading
import time
import netifaces  # You may need to install this: pip install netifaces

def get_ip_and_broadcast():
    """Get the IP address and broadcast address of the current machine"""
    # Find the default interface
    gateways = netifaces.gateways()
    default_interface = gateways['default'][netifaces.AF_INET][1]
    
    # Get addresses for that interface
    addresses = netifaces.ifaddresses(default_interface)
    ip_info = addresses[netifaces.AF_INET][0]
    
    ip_address = ip_info['addr']
    netmask = ip_info['netmask']
    
    # Calculate broadcast address
    # This is a simple way; for more complex networks you might need a different approach
    ip_parts = [int(part) for part in ip_address.split('.')]
    mask_parts = [int(part) for part in netmask.split('.')]
    broadcast_parts = [(ip_parts[i] & mask_parts[i]) | (~mask_parts[i] & 255) for i in range(4)]
    broadcast_address = '.'.join(str(part) for part in broadcast_parts)
    
    return ip_address, broadcast_address

class UDPCommunicator:
    def __init__(self, receive_port=5000, send_port=5001):
        self.receive_port = receive_port
        self.send_port = send_port
        self.running = False
        self.local_ip, self.broadcast_ip = get_ip_and_broadcast()
        print(f"Local IP: {self.local_ip}")
        print(f"Broadcast IP: {self.broadcast_ip}")
        
    def start(self):
        self.running = True
        
        # Start receiver thread
        self.receiver_thread = threading.Thread(target=self.receive_udp)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()
        
        # Start sender thread
        self.sender_thread = threading.Thread(target=self.send_udp)
        self.sender_thread.daemon = True
        self.sender_thread.start()
        
    def stop(self):
        self.running = False
        
    def receive_udp(self):
        """Thread function for receiving UDP packets"""
        # Create a UDP socket for receiving
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver_socket:
            # Allow reuse of address/port
            receiver_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            # Bind to all interfaces (0.0.0.0) to receive data sent directly to this machine's IP
            receiver_socket.bind(('0.0.0.0', self.receive_port))
            print(f"UDP Receiver started on port {self.receive_port}")
            
            # Set a timeout so we can check if we should stop
            receiver_socket.settimeout(1.0)
            
            while self.running:
                try:
                    data, addr = receiver_socket.recvfrom(1024)
                    print(f"Received message: {data.decode('utf-8')} from {addr}")
                    
                    # Process the received data here
                    # Example: send a response back
                    response = f"Received your message: {data.decode('utf-8')}"
                    receiver_socket.sendto(response.encode('utf-8'), addr)
                    
                except socket.timeout:
                    # This is just a timeout to check if we should stop
                    continue
                except Exception as e:
                    print(f"Error in receiver: {e}")
    
    def send_udp(self):
        """Thread function for sending UDP broadcast packets"""
        # Create a UDP socket for sending
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender_socket:
            # Set socket options to allow broadcast
            sender_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            print(f"UDP Sender started, broadcasting to {self.broadcast_ip}:{self.send_port}")
            
            counter = 0
            while self.running:
                try:
                    message = f"Broadcast message {counter} from Python"
                    sender_socket.sendto(message.encode('utf-8'), (self.broadcast_ip, self.send_port))
                    print(f"Sent broadcast: {message}")
                    counter += 1
                    
                    # Sleep for a bit before sending the next message
                    time.sleep(2)
                except Exception as e:
                    print(f"Error in sender: {e}")
                    time.sleep(1)

if __name__ == "__main__":
    try:
        # Create and start the UDP communicator
        communicator = UDPCommunicator(receive_port=5000, send_port=5001)
        communicator.start()
        
        # Keep the main thread running
        print("UDP Communicator running. Press Ctrl+C to stop.")
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("Stopping UDP Communicator...")
        if 'communicator' in locals():
            communicator.stop()