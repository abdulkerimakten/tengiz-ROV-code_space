import cv2
import time
import smbus2
from rov import *
from lidar import Lidar
from ultralytics import YOLO

# Initialize LiDAR
lidar = Lidar()

def rovUpdate(commandData: CommandData, receivedData: ReceivedData) -> CommandData:
    

    ##--  VARIABLES THAT AFFET THE ROV'S MISSSION --##

    distance = lidar.read_distance() # Read LiDAR data
    print(f"Distance to obstacle: {distance} cm")

    
    number_of_launced_torpedos = 0 # if it is 5 then the mission is completed and rov will stop
    isCircleDetected = False

    


    #-  Avoid obstacles and travel under water untill a circle is detected -#
    
    ## too close to the obstacle
    if distance <= 20 and isCircleDetected==False:  # If an obstacle is closer than 20 cm
        rov.move(0, 500, 0)  # Strafe right
        time.sleep(1)
        rov.move(0, 0, 0)  # Stop strafing
    
    ## a little close to the obstacle
    elif 20 < distance <= 50 and isCircleDetected==False:  # If an obstacle is between 20 cm and 50 cm
        rov.move(0, 0, 100)  # Surge forward a little bit
        time.sleep(1)
        rov.move(0, 300, 0)  # Strafe right slowly
        time.sleep(1)
        rov.move(0, 0, 0)  # Stop strafing

    ## a little far from the obstacle
    elif 50 < distance <= 200 and isCircleDetected==False:  # If an obstacle is between 50 cm and 200 cm
        rov.move(0, 0, 200)  # Surge forward
        time.sleep(1)
        rov.move(0, 300, 0)  # trafe right slowly
        time.sleep(1)
        rov.move(0, 0, 0)  # Stop strafing
    
    ## too far from the obstacle
    elif distance > 200 and isCircleDetected==False:  # If an obstacle is farther than 250 cm and no circle is detected
        rov.move(0, 0, 500)  # Surge forward
        time.sleep(1)
        rov.move(0, 500, 0)  # Strafe right swiftly
        time.sleep(1)
        rov.move(0, 0, 0)  # Stop strafing


    while True:
        # Process video frames and detect objects
        success, frame = cap.read()

        if success:
            frame = cv2.resize(frame, (width, height))
            
            current_time = time.time()
            fps = 1 / (current_time - prev_time)
            prev_time = current_time

            result = model.predict(source=frame, conf=0.6)
            annotated_frame = result[0].plot()

            for r in result:
                if r.boxes:
                    box = r.boxes[0]
                    class_id = int(box.cls)
                    object_name = classes[class_id]

                    if object_name == "circle":
                        isCircleDetected = True
                        # Get the center coordinates and size of the detected circle
                        x_center = int((box.xyxy[0][0] + box.xyxy[0][2]) / 2)
                        y_center = int((box.xyxy[0][1] + box.xyxy[0][3]) / 2)
                        # Draw the center point on the frame
                        cv2.circle(annotated_frame, (x_center, y_center), 5, (0, 0, 255), -1)

                        # Compute the distance to the circle (assume 10cm at a specific width)
                        distance_to_circle = 10 / width

                        # actual_width_cm = 10  # Real-world width of the circle in cm
                        # focal_length = 615  # Focal length of the camera in pixels (this is an example value)
                        # distance_to_circle = (actual_width_cm * focal_length) / width
                        print(f"Distance to circle: {distance_to_circle:.2f} cm")

                        if distance_to_circle > 10:
                            # Move forward towards the circle
                            rov.move(0, 0, 100)
                        else:
                            # Stop and launch torpedo
                            rov.move(0, 0, 0)
                            launch_torpedo()
                    
            cv2.putText(annotated_frame, f"FPS: {int(fps)}", (10, 20), font, 0.5, (0, 255, 0), 2)
            cv2.imshow("YOLOv8 Detection", annotated_frame)
            
            if cv2.waitKey(1) & 0xFF == 27: # Press ESC to exit
                break
        else:
            print("\nNo frame from camera!\n")
            break

    return commandData

def launch_torpedo():
    """
    
    
    Servo motor control codde goes here.
    
    
    """
    print("Torpedo launched!")
    # number_of_launced_torpedos += 1 # Increment the number of launched torpedos. This part will work when we have more torpedos.

try:
    model = YOLO('./weights/best_cember_v0.pt')
    classes = model.names
    print(classes)

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    prev_time = time.time()
    width, height = 640, 480
    font = cv2.FONT_HERSHEY_SIMPLEX

    logging.getLogger('ultralytics').setLevel(logging.WARNING)

    # Initialize ROV
    rov = Rov(port='/dev/cu.usbmodem101', rate_hz=20)
    clear_buffers(rov.link)

    # Run the main loop
    rov.run(rovUpdate)


except Exception as e:
    import traceback
    traceback.print_exc()
finally:
    rov.close()
    cap.release()
    cv2.destroyAllWindows()