import cv2
import numpy as np
import time
from pymavlink import mavutil

# Constants
UPDATE_RATE = 10  # Hz
SAFETY_DISTANCE = 0.5  # meters

# Connect to Pixhawk
mavlink_connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# Initialize camera
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

def process_frame(frame):
    """Process the camera frame to detect obstacles."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)

    # Find contours
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    obstacles = []
    for contour in contours:
        if cv2.contourArea(contour) > 500:  # Minimum area threshold
            x, y, w, h = cv2.boundingRect(contour)
            obstacles.append((x + w / 2, y + h / 2, w, h))
    return obstacles

def avoid_obstacles(obstacles, frame_width, frame_height):
    """Generate avoidance commands based on obstacle positions."""
    if not obstacles:
        return None

    # Simple avoidance strategy: move away from the closest obstacle
    closest = min(obstacles, key=lambda o: o[2] * o[3])  # Use area as a measure of closeness
    x, y, w, h = closest
    center_x, center_y = frame_width / 2, frame_height / 2

    if abs(x - center_x) < SAFETY_DISTANCE * frame_width and abs(y - center_y) < SAFETY_DISTANCE * frame_height:
        # Move in opposite direction of the obstacle
        vx = -1 if x < center_x else 1
        vy = -1 if y < center_y else 1
        return (vx, vy)
    
    return None

def send_velocity_command(vx, vy, vz):
    """Send velocity command to Pixhawk."""
    mavlink_connection.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms
        mavlink_connection.target_system,
        mavlink_connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions
        vx, vy, vz,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration
        0, 0)

def main():
    print("Starting indoor navigation system...")

    try:
        while True:
            start_time = time.time()

            # Capture frame from camera
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame.")
                break

            # Process frame to detect obstacles
            obstacles = process_frame(frame)

            # Generate avoidance command
            frame_height, frame_width = frame.shape[:2]
            avoid_command = avoid_obstacles(obstacles, frame_width, frame_height)

            if avoid_command:
                vx, vy = avoid_command
                send_velocity_command(vx, vy, 0)
            else:
                # No obstacles, continue with mission or hover
                send_velocity_command(0, 0, 0)

            # Display the frame with obstacles marked
            for (x, y, w, h) in obstacles:
                cv2.rectangle(frame, (x - w // 2, y - h // 2), (x + w // 2, y + h // 2), (0, 255, 0), 2)
            cv2.imshow('Frame', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            # Maintain update rate
            elapsed_time = time.time() - start_time
            if elapsed_time < 1.0 / UPDATE_RATE:
                time.sleep(1.0 / UPDATE_RATE - elapsed_time)

    except KeyboardInterrupt:
        print("Stopping indoor navigation system...")
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
