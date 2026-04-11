import socket
import threading
import time
from datetime import datetime

class UDPCommunicator:
    def __init__(self):
        # Hardcoded settings
        self.pico_ip = "192.168.50.50"
        self.pico_port = 5001
        self.listen_port = 5000  # Port to listen for messages from Pico
        
        self.running = False
        self.sock = None
    
    def start(self):
        """Start the UDP communication"""
        # Create and bind the socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.listen_port))
        self.running = True
        
        # Start receiver thread
        self.receiver_thread = threading.Thread(target=self._receive_messages)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()
        
        print(f"[{self._get_timestamp()}] Started UDP communication")
        print(f"Listening on port {self.listen_port}")
        print(f"Sending to Pico W at {self.pico_ip}:{self.pico_port}")
        print("\nType your messages (or 'quit' to exit):")
        
        self._handle_user_input()
    
    def stop(self):
        """Stop the UDP communication"""
        self.running = False
        if self.sock:
            self.sock.close()
    
    def _get_timestamp(self):
        """Get current timestamp for logging"""
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    
    def _receive_messages(self):
        """Handle incoming messages from Pico"""
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                message = data.decode('utf-8')
                print(f"\n[{self._get_timestamp()}] Received from {addr[0]}: {message}")
                print("> ", end='', flush=True)  # Restore input prompt
                
            except socket.error as e:
                if self.running:  # Only print error if we're still meant to be running
                    print(f"\nSocket error: {e}")
                break
            except Exception as e:
                print(f"\nError: {e}")
    
    def send_message(self, message):
        """Send message to Pico"""
        try:
            self.sock.sendto(message.encode('utf-8'), (self.pico_ip, self.pico_port))
            print(f"[{self._get_timestamp()}] Sent: {message}")
        except Exception as e:
            print(f"Error sending message: {e}")
    
    def _handle_user_input(self):
        """Handle user input for sending messages"""
        while self.running:
            try:
                message = input("> ")
                
                if message.lower() == 'quit':
                    self.stop()
                    break
                
                if message:  # Only send if message is not empty
                    self.send_message(message)
                    
            except KeyboardInterrupt:
                self.stop()
                break
            except Exception as e:
                print(f"Error: {e}")

def main():
    communicator = UDPCommunicator()
    
    try:
        communicator.start()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        communicator.stop()
        print("Goodbye!")

if __name__ == "__main__":
    main()