import time
import math
import numpy as np
from pymavlink import mavutil
from rplidar import RPLidar

# Constants
UPDATE_RATE = 10  # Hz
MAX_DISTANCE = 5.0  # meters
SAFETY_DISTANCE = 0.5  # meters

# Connect to Pixhawk
mavlink_connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# Connect to RPLidar
lidar = RPLidar('/dev/ttyUSB0')

def get_lidar_data():
    """Get LiDAR scan data"""
    scan = lidar.iter_scans()
    return next(scan)

def process_lidar_data(scan):
    """Process LiDAR data to detect obstacles"""
    obstacles = []
    for _, angle, distance in scan:
        if distance > 0 and distance < MAX_DISTANCE:
            x = distance * math.cos(math.radians(angle))
            y = distance * math.sin(math.radians(angle))
            obstacles.append((x, y))
    return obstacles

def avoid_obstacles(obstacles):
    """Generate avoidance commands based on obstacle positions"""
    if not obstacles:
        return None

    # Simple avoidance strategy: move away from the closest obstacle
    closest = min(obstacles, key=lambda o: math.hypot(o[0], o[1]))
    distance = math.hypot(closest[0], closest[1])
    
    if distance < SAFETY_DISTANCE:
        angle = math.atan2(closest[1], closest[0])
        avoid_angle = angle + math.pi  # Move in opposite direction
        return (math.cos(avoid_angle), math.sin(avoid_angle))
    
    return None

def send_velocity_command(vx, vy, vz):
    """Send velocity command to Pixhawk"""
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

            # Get LiDAR data
            scan = get_lidar_data()

            # Process LiDAR data
            obstacles = process_lidar_data(scan)

            # Generate avoidance command
            avoid_command = avoid_obstacles(obstacles)

            if avoid_command:
                vx, vy = avoid_command
                send_velocity_command(vx, vy, 0)
            else:
                # No obstacles, continue with mission or hover
                send_velocity_command(0, 0, 0)

            # Maintain update rate
            elapsed_time = time.time() - start_time
            if elapsed_time < 1.0 / UPDATE_RATE:
                time.sleep(1.0 / UPDATE_RATE - elapsed_time)

    except KeyboardInterrupt:
        print("Stopping indoor navigation system...")
    finally:
        lidar.stop()
        lidar.disconnect()

if __name__ == "__main__":
    main()
