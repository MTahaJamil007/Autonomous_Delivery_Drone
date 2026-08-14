# version 2: with UDP broadcasting of error data for high-speed transmission to the flight controller.
import cv2
import numpy as np
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
import time
import socket

# Setup UDP Socket for high-speed transmission
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

CENTER_X = 160
CENTER_Y = 120

def image_cb(msg):
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    
    corners, ids, rejected = detector.detectMarkers(img)
    
    if ids is not None:
        c = corners[0][0]
        marker_x = int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4)
        marker_y = int((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4)

        err_x = marker_x - CENTER_X
        err_y = marker_y - CENTER_Y
        
        # Grab the specific ID of the marker (e.g., 0, 1, 14)
        detected_id = ids[0][0]

        # Transmit the error data AND the ID
        message = f"{err_x},{err_y},{detected_id}"
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))

        cv2.circle(img, (marker_x, marker_y), 5, (0, 255, 0), -1)
        cv2.aruco.drawDetectedMarkers(img, corners, ids)
    else:
        # Broadcast that the target is lost
        sock.sendto("None".encode(), (UDP_IP, UDP_PORT))

    # Draw crosshairs for visual reference
    cv2.line(img, (160, 0), (160, 240), (255, 0, 0), 1)
    cv2.line(img, (0, 120), (320, 120), (255, 0, 0), 1)
    
    cv2.imshow("Drone Downward Camera", img)
    cv2.waitKey(1)

def main():
    node = Node()
    topic = "/camera/image"
    if node.subscribe(Image, topic, image_cb):
        print("✅ Vision Broadcaster Online on Port 5005!")
        try:
            while True:
                time.sleep(0.1)
        except KeyboardInterrupt:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()