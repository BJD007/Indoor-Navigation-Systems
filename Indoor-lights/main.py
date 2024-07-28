import time
import math
from pymavlink import mavutil

# Constants
UPDATE_RATE = 20  # Hz
MAX_VELOCITY = 0.5  # m/s
KP = 0.5  # Proportional gain for line following
KI = 0.1  # Integral gain for line following
KD = 0.2  # Derivative gain for line following

# Connect to Pixhawk
mavlink_connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# Wait for the connection to be established
mavlink_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % 
      (mavlink_connection.target_system, mavlink_connection.target_component))

def request_message_interval(message_id, frequency_hz):
    """Request a message at a particular frequency."""
    mavlink_connection.mav.command_long_send(
        mavlink_connection.target_system, mavlink_connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        1e6 / frequency_hz, # The interval between two messages in microseconds
        0, 0, 0, 0, # Unused parameters
        0 # Target address of message stream (0=flight controller)
    )

def send_velocity_command(vx, vy, vz, yaw_rate):
    """Send velocity command to Pixhawk."""
    mavlink_connection.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms
        mavlink_connection.target_system,
        mavlink_connection.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000011111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        vx, vy, vz,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not used)
        0, yaw_rate)  # yaw, yaw_rate

def main():
    print("Starting indoor navigation system...")

    # Request OPTICAL_FLOW message at desired rate
    request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW, UPDATE_RATE)

    last_error = 0
    integral = 0

    try:
        while True:
            start_time = time.time()

            # Wait for an OPTICAL_FLOW message
            msg = mavlink_connection.recv_match(type='OPTICAL_FLOW', blocking=True)

            if msg:
                # Extract flow in X and Y directions
                flow_x = msg.flow_x
                flow_y = msg.flow_y

                # Assuming the line is vertical, we're interested in the X flow
                # Positive flow_x means the line is to the right, negative means to the left
                error = -flow_x  # We want to minimize this error

                # PID control
                proportional = KP * error
                integral += KI * error
                derivative = KD * (error - last_error)

                # Calculate correction (this will be our Y velocity)
                correction = proportional + integral + derivative
                correction = max(-MAX_VELOCITY, min(MAX_VELOCITY, correction))

                # Set forward velocity (X) and correction (Y)
                vx = MAX_VELOCITY
                vy = correction

                # Send velocity command
                send_velocity_command(vx, vy, 0, 0)

                last_error = error

                print(f"Flow X: {flow_x:.2f}, Flow Y: {flow_y:.2f}, Correction: {correction:.2f}")

            # Maintain update rate
            elapsed_time = time.time() - start_time
            if elapsed_time < 1.0 / UPDATE_RATE:
                time.sleep(1.0 / UPDATE_RATE - elapsed_time)

    except KeyboardInterrupt:
        print("Stopping indoor navigation system...")
    finally:
        # Stop the drone
        send_velocity_command(0, 0, 0, 0)

if __name__ == "__main__":
    main()
