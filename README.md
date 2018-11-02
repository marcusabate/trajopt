# trajopt
Catkin package to generate trajectories for a px4 drone based on high-level user determined waypoints.

This package utilizes the mav_trajectory_generation package from ethz-asl (https://github.com/ethz-asl/mav_trajectory_generation)
mav_trajectory_generation is used to optimize trajectories through defined waypoints for both actuator limitations and fast execution.

Trajectory information is then sent to a low level controller developed as a part of MIT's 16.s398 VNAV 2018 Class (https://github.mit.edu/VNAV2018)
The controller can be used either in simulation or directly onboard the px4 drone system.

Simulation/visualization is done via yunchang's gazebo simulator (https://github.mit.edu/yunchang/aero_quadsim)

# Install Instructions

wip


# Usage Instructions

1. roscore
2. rosrun tf static_transform_publisher 0 0 0 0 0 0 1 world map 10
3. rosrun rviz rviz
4. rosrun trajopt manual_waypoint.py
4. rosrun trajopt traj_generator
