import cv2
import numpy as np
import serial
import time

def detect_red_circles(frame):
    # Convert the frame to the HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define the range of the red color in HSV
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([179, 255, 255])

    # Create a mask for the red color
    mask1 = cv2.inRange(hsv, lower_red, upper_red)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)
    
    # Apply GaussianBlur to the mask to reduce noise
    blurred = cv2.GaussianBlur(mask, (11, 11), 0)
    
    # Use HoughCircles to detect circles in the mask
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
                               param1=50, param2=30, minRadius=20, maxRadius=100)
    
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
            cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
            return x, y  # Return the coordinates of the detected circle

    return None, None

# Initialize the video stream
cap = cv2.VideoCapture(2)

# Initialize serial communication
ser = serial.Serial('/dev/ttyUSB0', 9600)  # Adjust 'COM3' to your Arduino's port
time.sleep(2)  # Wait for Arduino to reset

# Constants
frame_width = 640  # Adjust based on your camera resolution
frame_height = 480  # Adjust based on your camera resolution
center_x = frame_width // 2
center_y = frame_height // 2
tolerance = 20  # Pixel tolerance for the red circle to be considered centered
speed_factor = 1.5  # Speed factor for servo adjustments

# Current servo positions
current_servo_x = 90
current_servo_y = 90

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    x, y = detect_red_circles(frame)
    
    if x is not None and y is not None:
        # Calculate the difference from the center
        diff_x = x - center_x
        diff_y = y - center_y
        
        # Adjust servo positions based on the difference
        if abs(diff_x) > tolerance:
            if diff_x > 0:
                current_servo_x -= speed_factor  # Move left
            else:
                current_servo_x += speed_factor  # Move right
        
        if abs(diff_y) > tolerance:
            if diff_y > 0:
                current_servo_y += speed_factor  # Move up
            else:
                current_servo_y -= speed_factor  # Move down
        
        # Constrain servo positions to valid range
        current_servo_x = np.clip(current_servo_x, 0, 180)
        current_servo_y = np.clip(current_servo_y, 0, 180)
        
        # Send the current servo positions to the Arduino
        ser.write(f"{current_servo_x},{current_servo_y}\n".encode())
        
        # Print the coordinates to the terminal
        print(f"Detected red circle at (x, y): ({x}, {y})")
        print(f"Current servo angles: X={current_servo_x}, Y={current_servo_y}")
    
    # Display the frame with detected circles
    cv2.imshow('Frame', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
ser.close()  # Close the serial connection when done
