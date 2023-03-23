import time
from pymavlink import mavutil

def is_vehicle_armed(vehicle):
    armed_status = vehicle.recv_match(type='HEARTBEAT', blocking=True).base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    return bool(armed_status)

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
    max_arming_attempts = 5

    while not is_vehicle_armed(vehicle):
        if arming_attempts >= max_arming_attempts:
            print("Failed to arm vehicle after multiple attempts. Exiting.")
            break

        print("Arming the vehicle (attempt {})".format(arming_attempts + 1))
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
