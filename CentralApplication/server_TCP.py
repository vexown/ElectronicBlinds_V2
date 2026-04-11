"""
server_TCP.py

This script implements a simple TCP server using Python's socket and threading modules. 
The server listens for incoming connections, accepts messages from connected clients, 
and allows the user to send messages back to the clients through a command-line interface.

Usage:
1. Run the script using Python 3.
   Example: python server_TCP.py
2. The server will start and listen on the specified host and port (default: 0.0.0.0:8080).
3. To send a message to the connected client, type the message in the command prompt and press Enter.
4. To exit the server, type 'quit' and press Enter.

Key Features:
- Accepts only one client connection at a time.
- Receives and displays messages from the client.
- Allows the user to send messages to the connected client.
- Handles disconnections and errors gracefully.

Dependencies:
- Requires Python 3.x.
- No external libraries are needed beyond the standard library.

Note:
- You can modify the 'host' and 'port' parameters when creating the TCPServer instance
  to customize the server's listening address and port.
"""

import socket
import threading
import time

class TCPServer:
    def __init__(self, host='0.0.0.0', port=8080):
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        self.running = False
        
    def start(self):
        """Start the TCP server"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Allow port reuse
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        self.running = True
        
        print(f"Server listening on {self.host}:{self.port}")
        
        # Start receive thread
        receive_thread = threading.Thread(target=self.receive_messages)
        receive_thread.daemon = True
        receive_thread.start()
        
        # Start command input thread
        command_thread = threading.Thread(target=self.command_loop)
        command_thread.daemon = True
        command_thread.start()
        
    def receive_messages(self):
        """Handle incoming connections and messages"""
        while self.running:
            try:
                # Wait for connection
                print("Waiting for connection...")
                self.client_socket, self.client_address = self.server_socket.accept()
                print(f"Connected to client: {self.client_address}")
                
                # Receive messages
                while self.running:
                    try:
                        data = self.client_socket.recv(1024)
                        if not data:
                            print("Client disconnected")
                            break
                        
                        message = data.decode('utf-8')
                        print(f"Received: {message}")
                        
                    except ConnectionResetError:
                        print("Connection reset by client")
                        break
                    except Exception as e:
                        print(f"Error receiving message: {e}")
                        break
                
                # Clean up client socket
                if self.client_socket:
                    self.client_socket.close()
                    self.client_socket = None
                    
            except Exception as e:
                print(f"Connection error: {e}")
                if self.client_socket:
                    self.client_socket.close()
                    self.client_socket = None
                time.sleep(1)  # Wait before retrying
                
    def send_message(self, message):
        """Send a message to the connected client"""
        if self.client_socket:
            try:
                self.client_socket.send(message.encode('utf-8'))
                print(f"Sent: {message}")
                return True
            except Exception as e:
                print(f"Error sending message: {e}")
                return False
        else:
            print("No client connected")
            return False
            
    def command_loop(self):
        """Handle user input for sending messages"""
        while self.running:
            try:
                message = input("Enter message to send (or 'quit' to exit): ")
                if message.lower() == 'quit':
                    self.stop()
                    break
                
                if message:
                    self.send_message(message)
                    
            except Exception as e:
                print(f"Error in command loop: {e}")
                
    def stop(self):
        """Stop the TCP server"""
        self.running = False
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()
        print("Server stopped")

if __name__ == "__main__":
    # Create and start server
    server = TCPServer(host='192.168.50.194', port=8080)
    try:
        server.start()
        # Keep main thread alive
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down server...")
        server.stop()


