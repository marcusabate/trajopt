# trajectory_test_pkg
Catkin package to generate trajectories for px4 drone as a test

This package tests usage of mav_trajectory_generation from ethz-asl (found at: https://github.com/ethz-asl/mav_trajectory_generation)
It also tests visualization via yunchang's gazebo simulator (found at: https://github.mit.edu/yunchang/aero_quadsim)

This project will conclude with a working implementation of UNCO in c++ used in place of nlopt, as is currently used in the
mav_trajectory_generation package.
