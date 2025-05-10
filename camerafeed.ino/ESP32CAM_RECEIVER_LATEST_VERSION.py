import socket
import numpy as np
import cv2
import time
from collections import defaultdict

# UDP settings
UDP_IP = "0.0.0.0"  # Listen on all available interfaces
UDP_PORT = 4210     # Same port as ESP32 is sending to
BUFFER_SIZE = 2048  # Slightly larger than the ESP32's packet size

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.5)  # 500ms timeout for packet reception

# Tracking partially received frames
frame_buffers = defaultdict(lambda: {})
last_complete_frame = None

print(f"UDP video client listening on port {UDP_PORT}")

while True:
    try:
        # Receive data
        data, addr = sock.recvfrom(BUFFER_SIZE)
        
        if len(data) < 12:  # Minimum header size
            continue
            
        # Parse header
        frame_id = int.from_bytes(data[0:4], byteorder='little')
        total_packets = int.from_bytes(data[4:6], byteorder='little')
        packet_index = int.from_bytes(data[6:8], byteorder='little')
        packet_size = int.from_bytes(data[8:12], byteorder='little')
        
        # Store packet data
        frame_buffers[frame_id][packet_index] = data[12:12+packet_size]
        
        # Check if we have a complete frame
        if len(frame_buffers[frame_id]) == total_packets:
            # Reconstruct the complete frame
            # First, determine total size
            total_bytes = sum(len(packet_data) for packet_data in frame_buffers[frame_id].values())
            frame_data = bytearray(total_bytes)
            
            # Copy data in correct order
            offset = 0
            for i in range(total_packets):
                if i in frame_buffers[frame_id]:
                    packet_data = frame_buffers[frame_id][i]
                    frame_data[offset:offset+len(packet_data)] = packet_data
                    offset += len(packet_data)
            
            # Decode JPEG image
            img = cv2.imdecode(np.frombuffer(frame_data, dtype=np.uint8), cv2.IMREAD_COLOR)
            
            if img is not None:
                # Display the frame
                cv2.imshow('ESP32-CAM Stream', img)
                last_complete_frame = frame_id
                
                # Clean up old frames
                for old_frame_id in list(frame_buffers.keys()):
                    if old_frame_id <= last_complete_frame:
                        del frame_buffers[old_frame_id]
            
            # Wait for key press (1ms) to check for exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except socket.timeout:
        # No data received within timeout period
        pass
    except Exception as e:
        print(f"Error: {e}")

# Clean up
cv2.destroyAllWindows()
sock.close()