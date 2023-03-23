import math
import time
from pymavlink import mavutil

# Function to check if the vehicle is armed
def is_vehicle_armed(vehicle):
    armed_status = vehicle.recv_match(type='HEARTBEAT', blocking=True).base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    return bool(armed_status)

# Function to set the target position
def set_target_position(vehicle, lat, lon, alt):
    vehicle.mav.mission_item_send(
        vehicle.target_system,
        vehicle.target_component,
        0,  # Sequence number
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,  # Current waypoint
        1,  # Autocontinue
        0, 0, 0, 0,
        lat, lon, alt
    )

# Create a connection to the simulated plane
connection_string = "udp:127.0.0.1:14550"
print(f"Connecting to the simulated plane at {connection_string}")
vehicle = mavutil.mavlink_connection(connection_string)

# Wait for a heartbeat message to establish a connection
print("Waiting for heartbeat...")
vehicle.wait_heartbeat()
print("Heartbeat received. Connection established.")

# Read and print telemetry data
while True:
    msg = vehicle.recv_match(type='ATTITUDE', blocking=True)
    print(f"Roll: {msg.roll}, Pitch: {msg.pitch}, Yaw: {msg.yaw}")

    # Change the vehicle mode to GUIDED
    print("Changing mode to GUIDED")
    vehicle.mav.set_mode_send(vehicle.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)

    # Monitor the mode change
    while True:
        current_mode = vehicle.recv_match(type='HEARTBEAT', blocking=True).custom_mode
        if current_mode == 4:
            print("Mode changed to GUIDED successfully!")
            break
        else:
            print("Mode change failed. Retrying...")
            vehicle.mav.set_mode_send(vehicle.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
            time.sleep(1)

    # Arm the vehicle
    arming_attempts = 0
    max_arming_attempts = 15

    while not is_vehicle_armed(vehicle):
        if arming_attempts >= max_arming_attempts:
            print("Failed to arm vehicle after multiple attempts. Exiting.")
            break

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

    if is_vehicle_armed(vehicle):
        print("Vehicle armed successfully!")
        break  # Exit the loop since the vehicle is armed
    else:
        print("Arming failed. Retrying...")

    # Set target altitude and location
    target_latitude = 47.397741
    target_longitude = 8.545593
    target_altitude = 50 # in meters

    print(f"Setting target location to lat: {target_latitude}, lon: {target_longitude}, alt: {target_altitude}")
    set_target_position(vehicle, target_latitude, target_longitude, target_altitude)

    # Fly to the target location
    print("Flying to target location")
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0,
        0, 0, 0, 0, target_latitude, target_longitude, target_altitude
    )

    # Monitor the vehicle's altitude and location
    while True:
        altitude_msg = vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_altitude = altitude_msg.relative_alt / 1000 # Convert from millimeters to meters
        current_latitude = altitude_msg.lat / 1e7
        current_longitude = altitude_msg.lon / 1e7
        print(f"Current altitude: {current_altitude} m, latitude: {current_latitude}, longitude: {current_longitude}")

        distance_to_target = math.sqrt((current_latitude - target_latitude)**2 + (current_longitude - target_longitude)**2)
        if distance_to_target < 5 and abs(current_altitude - target_altitude) < 1:
            print("Arrived at target location!")
            break
        time.sleep(1)

    print("Mission completed. Disarming the vehicle.")
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0
    )

    print("Vehicle disarmed. Closing connection.")
    vehicle.close()