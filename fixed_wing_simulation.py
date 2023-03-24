import math
import time
from pymavlink import mavutil

# Function to check if the vehicle is armed
def is_vehicle_armed(vehicle):
    armed_status = vehicle.recv_match(type='HEARTBEAT', blocking=True).base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    return bool(armed_status)

# Function to set the target position in meters relative to the current position
def set_target_position(vehicle, north, east, alt):
    vehicle.mav.mission_item_send(
        vehicle.target_system,
        vehicle.target_component,
        0,  # Sequence number
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,  # Current waypoint
        1,  # Autocontinue
        0, 0, 0, 0,
        north, east, -alt  # Use negative altitude for NED frame
    )

# Function to set drone speed
def set_drone_speed(vehicle, speed):
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
        0,
        0,  # Airspeed type (0 = ground speed, 1 = true airspeed)
        speed,  # Target speed in m/s
        -1,  # Throttle (set to -1 to ignore throttle)
        0, 0, 0, 0  # Unused parameters
    )

# Function to change the vehicle mode
def change_mode(vehicle, mode_id):
    print("Changing mode to GUIDED")
    vehicle.mav.set_mode_send(vehicle.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)

    while True:
        current_mode = vehicle.recv_match(type='HEARTBEAT', blocking=True).custom_mode
        if current_mode == mode_id:
            print("Mode changed to GUIDED successfully!")
            break
        else:
            print("Mode change failed. Retrying...")
            vehicle.mav.set_mode_send(vehicle.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
            time.sleep(1)

# Function to arm the vehicle
def arm_vehicle(vehicle):
    arming_attempts = 0
    max_arming_attempts = 15

    while not is_vehicle_armed(vehicle) and arming_attempts < max_arming_attempts:
        print(f"Arming the vehicle (attempt {arming_attempts + 1})")
        vehicle.mav.command_long_send(
            vehicle.target_system,
            vehicle.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )

        arming_attempts += 1
        time.sleep(2)  # Increase the sleep time to give more time for the vehicle to arm

    return is_vehicle_armed(vehicle)

# Function to disarm the vehicle
def disarm_vehicle(vehicle):
    print("Disarming the vehicle.")
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )

def proportional_controller(distance_to_target, max_speed, gain):
    speed = gain * distance_to_target
    return min(speed, max_speed)

# Function to fly the square
def fly_square(vehicle, waypoints, start_alt, max_speed, gain, waypoint_tolerance=2, waypoint_timeout=30):
    # Adjust the altitude of the waypoints so that the starting altitude is start_alt
    waypoints = [(north, east, start_alt) for north, east, _ in waypoints]

    for waypoint in waypoints:
        north, east, alt = waypoint
        print(f"Setting target location to N: {north}, E: {east}, Alt: {alt}")
        set_target_position(vehicle, north, east, alt)

        # Monitor the vehicle's position
        start_time = time.time()
        while True:
            position_msg = vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True)
            current_north = position_msg.x
            current_east = position_msg.y
            current_down = -position_msg.z  # Convert to altitude
            print(f"Current position: N: {current_north}, E: {current_east}, A:{current_down}")

            distance_to_target_horizontal = math.sqrt((current_north - north) ** 2 + (current_east - east) ** 2)
            distance_to_target_vertical = abs(current_down - alt)

            # Calculate the desired speed based on the distance to the target
            desired_speed_horizontal = proportional_controller(distance_to_target_horizontal, max_speed, gain)
            desired_speed_vertical = proportional_controller(distance_to_target_vertical, max_speed, gain)

            # Update the drone speed (use the minimum of horizontal and vertical speeds)
            desired_speed = min(desired_speed_horizontal, desired_speed_vertical)
            set_drone_speed(vehicle, desired_speed)

            if distance_to_target_horizontal < waypoint_tolerance and distance_to_target_vertical < waypoint_tolerance:
                print("Arrived at target location!")
                break

            # Check if the waypoint timeout has been reached
            elapsed_time = time.time() - start_time
            if elapsed_time > waypoint_timeout:
                print(f"Timeout reached for waypoint: N: {north}, E: {east}, Alt: {alt}. Moving to the next waypoint.")
                break

            time.sleep(1)

    # Add a waypoint at altitude 0
    set_target_position(vehicle, 0, 0, 0)

    # Monitor the vehicle's position
    while True:
        position_msg = vehicle.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        current_down = -position_msg.z  # Convert to altitude
        print(f"Current altitude: {current_down}")

        if abs(current_down) < 1:
            print("Reached altitude 0!")
            break
        time.sleep(1)

# Main function to execute the mission
def main():
    # Create a connection to the simulated plane
    connection_string = "udp:127.0.0.1:14550"
    print(f"Connecting to the simulated plane at {connection_string}")
    vehicle = mavutil.mavlink_connection(connection_string)

    # Wait for a heartbeat message to establish a connection
    print("Waiting for heartbeat...")
    vehicle.wait_heartbeat()
    print("Heartbeat received. Connection established.")

    # Define the square's side length (in meters)
    square_side = 5

    # Define the square's side length (in meters)
    square_side = 3

    # Define the waypoints for the square at a lower altitude
    waypoints = [
        (square_side, 0, 5),
        (square_side, square_side, 5),
        (0, square_side, 5),
        (0, 0, 5)
    ]

    # Read and print telemetry data
    msg = vehicle.recv_match(type='ATTITUDE', blocking=True)
    print(f"Roll: {msg.roll}, Pitch: {msg.pitch}, Yaw: {msg.yaw}")

    # Change the vehicle mode to GUIDED
    change_mode(vehicle, 4)

    # Set drone speed
    max_speed = 10  # m/s
    gain = 0.5  # Proportional gain

    # Arm the vehicle
    if arm_vehicle(vehicle):
        print("Vehicle armed successfully!")

        # Fly the square
        fly_square(vehicle, waypoints, start_alt=5, max_speed=max_speed, gain=gain)

        # Disarm the vehicle
        disarm_vehicle(vehicle)

        # Close the connection
        print("Closing connection.")
        vehicle.close()

        print("Mission completed.")

if __name__ == '__main__':
    main()