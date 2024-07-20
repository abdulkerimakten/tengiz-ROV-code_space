import cv2
import numpy as np

def detect_color(frame, lower_color, upper_color):
    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Create a mask for the specified color range
    mask = cv2.inRange(hsv, lower_color, upper_color)
    
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Draw green boxes around detected objects
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 500:  # Adjust this value to filter out small noise
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    
    return frame

# Initialize the camera
cap = cv2.VideoCapture(0)

# Define the color range for dark blue in HSV
lower_dark_blue = np.array([100, 150, 0])
upper_dark_blue = np.array([140, 255, 100])

while True:
    # Read a frame from the camera
    ret, frame = cap.read()
    if not ret:
        break
    
    # Detect dark blue color and draw boxes
    result = detect_color(frame, lower_dark_blue, upper_dark_blue)
    
    # Display the result
    cv2.imshow('Dark Blue Color Detection', result)
    
    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close windows
cap.release()
cv2.destroyAllWindows()