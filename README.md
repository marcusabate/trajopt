# trajopt

Generate trajectories for a px4 drone based on high-level user determined waypoints.

This package utilizes the mav_trajectory_generation package from ethz-asl (https://github.com/ethz-asl/mav_trajectory_generation)
mav_trajectory_generation is used to optimize trajectories through defined waypoints for both actuator limitations and fast execution.

Trajectory information is then sent to a low level controller developed as a part of MIT's 16.s398 VNAV 2018 Class (https://github.mit.edu/VNAV2018)
The controller can be used either in simulation or directly onboard the px4 drone system.

Simulation/visualization is done via yunchang's gazebo simulator (https://github.mit.edu/yunchang/aero_quadsim)


## Install Instructions

WORK IN PROGRESS but essentially follow the instal instructions for both ```mav_trajectory_generation``` and ```aero_quadsim```, and then clone this repo into your workspace.


## Trajectory Generation

``manual_waypoint.py`` is a script that allows you to set desired waypoints for future trajectory optimization.

``traj_generator.cpp`` is where the optimization actually occurs. Waypoints are picked up from the /waypoints rostopic (published to by manual_waypoint.py) and then used to generate an optimal trajectory.

The optimizer is nonlinear and has many parameters that as of now can only be set within the source code. This happens in get_trajectory(). Soon these will become optional launch parameters.
The segments of the trajectory are all checked for input feasibility using drone parameters that, again, are set within the source code. This happens in get_feasibility_result(). These will also soon be launch parameters or simply set to reflect the dynamics of the Intel Aero platform.
The resultant trajectory is published in three different ways. First, it is published as a ```MarkerArray``` for use in RVIZ for simple visualization. Then, it is published as a ```MultiDOFJointTrajectoryPoint``` message (the entire trajectory is published at once) for use with the fast low-level controller written in ```controller_node_sol.cpp```. Finally, it is published as a ```PolynomialTrajectory4D``` message to be deciphered by a conversion node at another point. This method is superfluous and therefore has been commented out in the source code, but can be run in main using trajectory_publish().

WORK IN PROGRESS:
``controller_node_sol.cpp`` is a fast controller for the drone that takes the generated trajectory and directly publishes motor commands to the drone for each time step. Currently, it does not interface with the gazebo simulator.

## Simple RVIZ Trajectory Visualization

To cleanly generate and then visualize trajectories through desired waypoints in RVIZ, first edit ```scripts/manual_waypoint.py``` to include your desired waypoints in the ```waypoints``` list. Note that these waypoints are defined as 3-coordinate points in space; they do not include yaw or other states.

Run the launch file provided:
```
roslaunch trajopt rviz_visualization.launch
```

or, start the nodes as follows:
1. roscore
2. rosrun tf static_transform_publisher 0 0 0 0 0 0 1 world map 10
3. rosrun rviz rviz
4. rosrun trajopt manual_waypoint.py
4. rosrun trajopt traj_generator

RVIZ should start and immediately plot the trajectory generated by ```traj_generator.cpp``` based on your waypoints.


## Gazebo Trajectory Simulation

WORK IN PROGRESS

Using yunchang's simulator for the px4 in Gazebo, the trajectory can be simulated more realistically. Currently, control is done by the onboard px4 default controller. The trajectory generated by the system is sampled to produce 'waypoints' that are spaced very close together along the trajectory. These 'waypoints' are then fed to the px4 controller as mission waypoints. The controller is slow, and will soon be replaced by a more aggressive controller that can accurately track the trajectory and use the generated angular rates.

First make sure to set up the simulator correctly. Navigate to aero_quadsim for setup:
```
cd ~/catkin_ws/src/aero_quadsim
source .simbash
roslaunch aero_quadsim aero_sitl.launch
```

Once the simulator has fully loaded, run the simulation:
```
roslaunch trajopt traj_sim_minimal.launch
```

NOTE: traj_sim.launch is the same as traj_sim_minimal.launch but <include>'s the aero_sitl.launch file. Feel free to try it, my results have been poor.


## Gazebo Simulation with Low-Level Controller

WORK IN PROGRESS
