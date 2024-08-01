import time
import math
from pymavlink import mavutil
import serial

# Constants
UPDATE_RATE = 10  # Hz
NUM_UWB_ANCHORS = 4  # Number of UWB anchors
ANCHOR_POSITIONS = [
    (0, 0, 0),    # Anchor 1 position (x, y, z)
    (5, 0, 0),    # Anchor 2 position
    (5, 5, 0),    # Anchor 3 position
    (0, 5, 0)     # Anchor 4 position
]

# Connect to Pixhawk
try:
    mavlink_connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
except Exception as e:
    print(f"Error connecting to Pixhawk: {e}")
    exit(1)

# Connect to UWB module
try:
    uwb_serial = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
except Exception as e:
    print(f"Error connecting to UWB module: {e}")
    exit(1)

def read_uwb_distances():
    """Read distances from UWB PulseON modules."""
    distances = []
    for _ in range(NUM_UWB_ANCHORS):
        try:
            line = uwb_serial.readline().decode('utf-8').strip()
            distance = float(line)
            distances.append(distance)
        except (ValueError, serial.SerialException) as e:
            print(f"Error reading UWB distance: {e}")
            distances.append(None)
    return distances

def trilaterate(distances):
    """Perform trilateration to estimate position."""
    # Simple trilateration algorithm (this is a placeholder and needs to be improved)
    x, y, z = 0, 0, 0
    valid_measurements = 0
    
    for i, distance in enumerate(distances):
        if distance is not None:
            x += ANCHOR_POSITIONS[i][0] * distance
            y += ANCHOR_POSITIONS[i][1] * distance
            z += ANCHOR_POSITIONS[i][2] * distance
            valid_measurements += 1
    
    if valid_measurements > 0:
        x /= valid_measurements
        y /= valid_measurements
        z /= valid_measurements
    
    return x, y, z

def send_position_target(x, y, z):
    """Send position target to Pixhawk."""
    try:
        mavlink_connection.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            mavlink_connection.target_system,
            mavlink_connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,  # type_mask (only positions enabled)
            x, y, z,  # x, y, z positions
            0, 0, 0,  # x, y, z velocity
            0, 0, 0,  # x, y, z acceleration
            0, 0)  # yaw, yaw_rate
    except Exception as e:
        print(f"Error sending position target: {e}")

def obstacle_avoidance(x, y, z):
    """Basic obstacle avoidance (placeholder)."""
    # Implement obstacle avoidance logic here
    # For now, just return the same coordinates
    return x, y, z

def path_planning(x, y, z):
    """Basic path planning (placeholder)."""
    # Implement path planning algorithm here
    # For now, just return the same coordinates
    return x, y, z

def main():
    print("Starting indoor navigation system...")

    try:
        while True:
            start_time = time.time()

            # Read UWB distances
            distances = read_uwb_distances()
            print(f"UWB Distances: {distances}")

            # Estimate position using trilateration
            x, y, z = trilaterate(distances)
            print(f"Estimated position: ({x:.2f}, {y:.2f}, {z:.2f})")

            # Apply obstacle avoidance
            x, y, z = obstacle_avoidance(x, y, z)

            # Apply path planning
            x, y, z = path_planning(x, y, z)

            # Send position target to Pixhawk
            send_position_target(x, y, z)

            # Maintain update rate
            elapsed_time = time.time() - start_time
            if elapsed_time < 1.0 / UPDATE_RATE:
                time.sleep(1.0 / UPDATE_RATE - elapsed_time)

    except KeyboardInterrupt:
        print("Stopping indoor navigation system...")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        uwb_serial.close()

if __name__ == "__main__":
    main()
