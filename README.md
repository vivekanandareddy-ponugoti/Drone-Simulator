# Drone-Simulator
Drone simulator prototype using mavlink protocol

python3 sim_vehicle.py -v ArduPlane --console --map --out 127.0.0.1:14550

This modified script now sets a target altitude and location for the simulated plane, flies to the target location, and monitors the vehicle's altitude and location. When the vehicle arrives at the target location, the script will disarm the vehicle and close the connection.
