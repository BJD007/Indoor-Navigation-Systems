import time
import RPi.GPIO as GPIO
from pymavlink import mavutil

# Constants
UPDATE_RATE = 10  # Hz
SAFETY_DISTANCE = 0.5  # meters

# GPIO setup for sonar
TRIG = 23
ECHO = 24
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Connect to Pixhawk
mavlink_connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# Wait for the connection to be established
mavlink_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % 
      (mavlink_connection.target_system, mavlink_connection.target_component))

def get_sonar_distance():
    """Measure distance using the sonar sensor."""
    GPIO.output(TRIG, False)
    time.sleep(0.1)
    
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
    
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Convert to cm
    distance = distance / 100  # Convert to meters
    
    return distance

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
        0, 0)  # yaw, yaw_rate

def main():
    print("Starting indoor navigation system...")

    try:
        while True:
            start_time = time.time()

            # Get sonar distance
            distance = get_sonar_distance()
            print(f"Distance: {distance:.2f} meters")

            # Simple obstacle avoidance
            if distance < SAFETY_DISTANCE:
                # Move backward if too close to an obstacle
                send_velocity_command(-0.5, 0, 0)
            else:
                # Move forward if safe
                send_velocity_command(0.5, 0, 0)

            # Maintain update rate
            elapsed_time = time.time() - start_time
            if elapsed_time < 1.0 / UPDATE_RATE:
                time.sleep(1.0 / UPDATE_RATE - elapsed_time)

    except KeyboardInterrupt:
        print("Stopping indoor navigation system...")
    finally:
        # Stop the drone
        send_velocity_command(0, 0, 0)
        GPIO.cleanup()

if __name__ == "__main__":
    main()
