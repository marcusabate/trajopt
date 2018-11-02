/* waypoint_pub.cpp
 *
 * This program publishes a set of waypoints to the rostopic /waypoints.
 * The waypoints are user defined here. This is a placeholder script for another
 * module that will use RRT* to do proper planning.
 *
 * The waypoints are picked up by the traj_generator node and used to generate
 * optimal trajectories.
 *
 * Only publishes waypoints once, so rerun if waypoints are changed.
 *
 * NOTE that waypoints are position only; angular rates are computed at the
 * control stage.
 *
 * An additional script to allow on the fly waypoint publishing via user input
 * would be a nice addition.
 */

 #include <geometry_msgs/PoseArray.h>
 #include "ros/ros.h"
 using namespace std;

 geometry_msgs::PoseArray define_waypoints()
 /* Creates a geometry_msgs::PoseArray consisting of all the user-requested
  * waypoints in the path.
  * Define and/or change requested waypoints here.
  */
 {

 }

 void publish(geometry_msgs::PoseArray poses)
 /* Creates a publisher to the /waypoints rostopic
  * Publishes input 'poses' as waypoints to the rostopic.
  */
 {

 }

 int main(int argc, char** argv)
 {
   waypoints = define_waypoints()

   publish(waypoints)
 }
