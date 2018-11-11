/*
    simulate_trajectory.cpp

    Marcus Abate

    This script grabs the MarkerArray of poses given by traj_generator.cpp
    and converts each pose to a setpoint that is then given to aero_quadsim
    for use as a "waypoint" during simulation. The onboard px4 controller
    determines actual execution in this implementation: a faster controller
    is NOT implemented here.

    The executable here calls on aero_ctrl.cpp in the aero_quadsim package
    to perform navigation.

    CURRENTLY THE EXECUTABLE VERSION OF THIS FILE IS LOCATED IN THE
    aero_quadsim DIRECTORY BECAUSE THE aero_ctrl.h FILE IS THERE.
        SINCE HAS BEEN MADE TO WORK HERE, BUT UNTESTED.
*/

#include "../../aero_quadsim/src/aero_ctrl.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


visualization_msgs::MarkerArray markers;

void marker_sub_cb(const visualization_msgs::MarkerArray::ConstPtr& msg) {
  markers = *msg;
}

int main(int arc, char** argv)
{
  // ROS and control setup:
  ros::init(arc, argv, "simulate_trajectory");
  ROS_INFO("Simulate optimized trajectory through desired waypoints");
  aero_ctrl control;

  // get the drone in the air:
  control.arm();
  float flight_altitude = 1.0; // inital flight altitude at takeoff
  control.takeoff(flight_altitude);

  // Grab desired 'waypoints' (Markers):
  ros::NodeHandle nh;
  ros::Subscriber marker_sub = nh.subscribe<visualization_msgs::MarkerArray>
          ("/visualization_marker_array", 1, marker_sub_cb);
  ros::spin(); // use spin() instead of spinOnce() b/c we need one message only.

  // Parse the message appropriately:
  unsigned int marker_array_size = markers.markers.size(); // ->?
  visualization_msgs::Marker path_marker = markers.markers[marker_array_size-1];
  std::vector<geometry_msgs::Point> positions = path_marker.points;

  // navigate to each pose in succession.
  /*
      note: as is, this implementation does not wait AT ALL between each publish
      so when this is actually simulated it might not hit all the waypoints.
  */
  for (int i=0; ros::ok() && i<positions.size(); i++)
  {
    float yaw = 0.0; // can probably get from trajectory.
    control.navigate_to_pose(positions[i].x, positions[i].y, positions[i].z, yaw);
    sleep(5);
  }

  // Land and end sim.
  control.land();
  ros::Duration(5.0).sleep();
  return 0;
}
