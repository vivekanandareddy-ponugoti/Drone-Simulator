# Drone Simulator Prototype with MAVLink Protocol
This project showcases a drone simulator prototype that utilizes the MAVLink protocol and the pymavlink Python library to communicate with MAVLink-enabled autopilots. The script offers a series of functions to interact with and control the drone's behavior.

## Functions Overview
1. is_vehicle_armed(vehicle): Determines if the vehicle is armed, returning True or False accordingly.
2. set_target_position(vehicle, north, east, alt): Defines the target position in meters relative to the vehicle's current position using North, East, Down (NED) coordinates.
3. set_drone_speed(vehicle, speed): Adjusts the drone's speed in meters per second.
4. change_mode(vehicle, mode_id): Modifies the vehicle's mode to GUIDED mode.
5. arm_vehicle(vehicle): Arms the vehicle with multiple attempts, incorporating a sleep time between each.
6. disarm_vehicle(vehicle): Disarms the vehicle.
7. proportional_controller(distance_to_target, max_speed, gain): Implements a simple proportional controller that calculates the desired speed based on the target distance.
8. fly_square(vehicle, waypoints, start_alt, max_speed, gain, waypoint_tolerance=2, waypoint_timeout=30): Controls the drone to fly in a square pattern using the provided waypoints.
9. main(): Coordinates the entire process, including establishing a connection to the drone, waiting for a heartbeat message, changing the vehicle's mode, setting the drone speed, arming and disarming the vehicle, flying the square, and closing the connection.

## Command Line Usage
To use the drone simulator, run the following commands:
<code>python3 sim_vehicle.py -v ArduPlane --console --map --out 127.0.0.1:14550</code>
<code>python3 fixed_wing_simulation.py</code>

With this drone simulator prototype, users can explore the potential of the MAVLink protocol and pymavlink library in controlling drones and managing their various functions.