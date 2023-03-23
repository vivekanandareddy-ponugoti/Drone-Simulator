import time
from pymavlink import mavutil

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
    print("Arming the vehicle")
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0
    )

    time.sleep(1)

    # Monitor the arming status
    print("Checking arming status")
    arming_status = vehicle.recv_match(type='HEARTBEAT', blocking=True).base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    if arming_status:
        print("Vehicle armed successfully!")
    else:
        print("Arming failed. Retrying...")