/* rviz_publisher_node.cpp
 *
 * This node grabs trajectory information from /trajectory and converts it to
 * a mav_trajectory_generation::Trajectory object. It then uses tools from
 * the mav_trajectory_generation package to draw the trajectory in RVIZ.
 *
 * Additionally, it grabs the desired and current state of the drone after
 * the controller has attempted to track the trajectory from the
 * /desired_state and the /current_state topics and visualizes them in RVIZ.
 *
 */

 //   NOTE: instead of a commandTimerCallback, just listen to /desired_state
 //         and to /current_state and publish a message every single time.

#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include "ros/ros.h"

 class rvizPublisherNode
 {
   ros::NodeHandle nh;
   ros::Timer publish_timer;
   ros::Publisher marker_pub;
   ros::Subscriber trajectory_sub;
   ros::Time start_time;
   ros::Rate r;

   double dt;
   double current_sample_time;

   mav_trajectory_generation::Trajectory trajectory;

   visualization_msgs::MarkerArray traj_markers;
   double distance = 1.0; // Distance by which to seperate additional markers.
   std::string frame_id = "world";

   rvizPublisherNode(const ros::NodeHandle& n)
    :nh(n),
     dt(0.01),
     current_sample_time(0.0),
   {
     marker_pub = nh.advertise<visualization_msgs::MarkerArray>(
    				"visualization_marker_array", 0);
     trajectory_sub = nh.subscribe("trajectory", 10, trajCallback);
     r = r(1);
     publish_timer = nh.createTimer(ros::Duration(dt), &controlTimerCallback);
   }

  void trajCallback(const mav_planning_msgs::PolynomialTrajectory4D& segments_message)
  {
    if (segments_message.segments.empty()) {
      ROS_WARN("Trajectory sampler: received empty waypoint message");
      return;
    }
    else {
      ROS_INFO("Trajectory sampler: received %lu waypoints",
               segments_message.segments.size());
    }

    bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(
        segments_message, &trajectory);
    if (!success) {
      ROS_WARN("Failure to Convert Trajectory Message");
      return;
    }
    publish_timer.start();
    current_sample_time = 0.0;
    start_time = ros::Time::now();

    rviz_publish();
 }

 void controlTimerCallback(const ros::TimerEvent&)
 {
   // publish desired and current state of drone to RVIZ
 }

 void rviz_publish()
 {
 	// Trajectory visualization in RVIZ:
 	mav_trajectory_generation::drawMavTrajectory(trajectory, distance,
 																								frame_id, &traj_markers);
 	//publishing the visualization:
 	while(marker_pub.getNumSubscribers() < 1) {
 		ROS_WARN_ONCE("Please create a subscriber to the MarkerArray");
 		sleep(1);
 	}
 	marker_pub.publish(traj_markers);
 	//sleep(1);
 }

} // class rvizPublisherNode

 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "traj_generator_node");
   ros::Nodehandle nh;
   rvizPublisherNode rviz_publisher_node(nh);
   ROS_INFO("Initialized RVIZ Publisher Node");
   ros::spin();
 }
