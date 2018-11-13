# trajopt

Generate and simulate trajectories for a px4 drone based on high-level user determined waypoints.

This package utilizes the [mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation) package from ethz-asl.
``mav_trajectory_generation`` is used to optimize trajectories through defined waypoints for both actuator limitations and fast execution.

Trajectory information is then sent to a low level controller developed as a part of MIT's [16.s398 VNAV 2018 Class](https://github.mit.edu/VNAV2018)
The controller can be used either in simulation or directly onboard the px4 drone system.

Simulation/visualization is done via yunchang's gazebo simulator [aero_quadsim](https://github.mit.edu/yunchang/aero_quadsim)


## Install Instructions

Follow install instructions for the following packages:
* [mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation)
* [aero_quadsim](https://github.mit.edu/yunchang/aero_quadsim)

**OUTSIDE OF YOUR CATKIN WS**, clone the VNAV 2018 Labs repo:
* [Labs](https://github.mit.edu/VNAV2018/Labs)

Then copy the unity bridge package from Lab_3 into your catkin workspace:
```
cp -r ${VNAV_LABS_PATH}/Lab_3/unity_bridge ${CATKIN_WS}/src
catkin build
```

Finally, clone this repo into your catkin workspace. Perform a final ``catkin build``.


## Package Description

The trajopt package trajectory generation and visualization programs are modularized into different nodes for each task. Here are brief descriptions of each of the files:

``waypoint_publisher_node.cpp`` is a ROS node that publishes the user-defined waypoints over which to perform optimization. Refer to the comments in the file to find the waypoint definition. Waypoints are published to the ``/waypoints`` topic.
This node also starts the simulation at user command. When writing launch files, be sure to include the following arguments:
``output="screen" launch-prefix="gnome-terminal --command"``
This is required as user input is needed to run the waypoint publish function.

``traj_generator_node.cpp`` is where optimization occurs.  Waypoints are picked up from the ```/waypoints``` rostopic and then used to generate an optimal trajectory.
The optimizer is nonlinear and has many parameters that as of now can only be set within the source code. This happens in ```get_trajectory()```. Soon these will become optional launch parameters.
The segments of the trajectory are all checked for input feasibility using drone parameters that, again, are set within the source code. These will also soon be launch parameters or simply set to reflect the dynamics of the Intel Aero platform within a config file.

``state_publisher_node.cpp`` is the node responsible for picking up the generated trajectory from the ``/trajectory`` topic and then slicing the trajectory into ``MultiDOFJointTrajectoryPoint`` messages reflecting the desired state of the drone at each time step. These messages are published to the ``/desired_state`` topic at each time step.

``controller_node.cpp`` is the controller of the drone. It takes as input the desired_state messages published to ``/desired_state`` as well as the current state of the drone from ``/current_state``. It then computes the required control inputs and publishes them as rotor commands to ``/rotor_speed_cmds``. The Unity simulator then performs state estimation and runs the dynamics of the drone to compute the current state of the drone after control input is applied. This is published to ``/current_state``.

``rviz_publisher_node.cpp`` is the visualization node (beyond the Unity engine visualization and the gazebo simulator). This node publishes the trajectory as well as the current state of the drone at each time step so that the user can visualize the drone's trajectory tracking performance. Be sure to load the rviz config file ``default.rviz`` located in the ``/rviz`` folder in the package. Note that multiple MarkerArray rostopics are used.

Older code is included in the ``/src/legacy`` folder for now.


## Unity and RVIZ Trajectory Visualization

A test launch file is provided that generates a simple trajectory and then visualizes it in RVIZ. The trajectory is a straight line. To run the test, run the launch file:
```
roslaunch trajopt test_traj.launch
```

To generate and then visualize trajectories through desired waypoints in RVIZ, first edit ```waypoint_publisher_node.cpp``` to include your desired waypoints. Note that these waypoints are defined as 3-coordinate points in space; they do not include yaw or other states.

Visualization happens both in the Unity engine and in RVIZ. To view only the Unity engine's output, run the following provided launch file:
```
roslaunch trajopt traj_sim_unity.launch
```

To include RVIZ trajectory and state visualization for a more accurate measure of performance, run the following provided launch file:
```
roslaunch trajopt traj_sim_rviz.launch
```

For both of these launch files, the user must manually start the simulation by pressing any key in the waypoint_publisher_node node terminal. This is done to allow preparation of the RVIZ config as well as give time for the Unity engine to load before running the visualization.


## Gazebo Trajectory Simulation

**WORK IN PROGRESS**

Using yunchang's simulator for the px4 in Gazebo, the trajectory can be simulated more realistically. Currently, control is done by the onboard px4 default controller. The trajectory generated by the system is sampled to produce 'waypoints' that are spaced very close together along the trajectory. These 'waypoints' are then fed to the px4 controller as mission waypoints. The controller is slow, and will soon be replaced by a more aggressive controller that can accurately track the trajectory and use the generated angular rates.

First make sure to set up the simulator correctly. Navigate to aero_quadsim for setup:
```
cd ~/catkin_ws/src/aero_quadsim
source .simbash
roslaunch aero_quadsim aero_sitl.launch
```

Once the simulator has fully loaded, run the simulation (in another terminal):
```
roslaunch trajopt traj_sim_minimal.launch
```

*NOTE*: ```traj_sim.launch``` is the same as ```traj_sim_minimal.launch``` but includes the ```aero_sitl.launch``` file. Feel free to try it, my results have been poor.


## Gazebo Trajectory Simulation with Low-Level Controller

**WORK IN PROGRESS**
