"""
client_TCP.py
This script is a simple TCP client that connects to a server running on a Pico W board.
It sends messages to the server and prints any messages received from the server.
To use this script, replace the server_ip and server_port variables with the IP address
and port of the Pico W server you want to connect to. Then run the script using Python 3.
Example:
    python client_TCP.py
Dependencies:
- Requires Python 3.x.
- No external libraries are needed beyond the standard library.
Note:
- This script is intended to be used with the server_TCP.py script running on a Pico W board.
- The server_ip and server_port variables should be set to the IP address and port of the server.
- The client will connect to the server and send messages entered by the user in the command prompt.
- Messages received from the server will be printed to the console.
- Don't try to run this script on WSL, use a native Windows or Linux environment.
"""
import socket
import threading
def receive_messages(client_socket):
    while True:
        try:
            data = client_socket.recv(1024)
            if data:
                print(f"Received: {data.decode('utf-8')}")
            else:
                break
        except:
            break
def start_tcp_client(server_ip, server_port):
    # Create a TCP/IP socket
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    # Connect the socket to the server's address and port
    client_socket.connect((server_ip, server_port))
    print(f"Connected to {server_ip}:{server_port}")
    
    # Start a thread to receive messages
    receive_thread = threading.Thread(target=receive_messages, args=(client_socket,))
    receive_thread.daemon = True
    receive_thread.start()
    
    try:
        while True:
            # Send data to the server if needed
            message = input("Enter message to send (or press Enter to skip): ")
            if message:
                client_socket.sendall(message.encode('utf-8'))
    finally:
        # Clean up the connection
        client_socket.close()
if __name__ == "__main__":
    server_ip = '192.168.50.50'  # Replace with the IP address of your Pico W
    server_port = 8080          # Replace with the port your Pico W server is listening on
    start_tcp_client(server_ip, server_port)