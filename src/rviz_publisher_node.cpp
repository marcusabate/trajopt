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

#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include "ros/ros.h"

 class rvizPublisherNode
 {
 public:
    ros::NodeHandle nh;
    ros::Publisher traj_pub;
    ros::Publisher current_state_pub;
    ros::Subscriber trajectory_sub;
    ros::Subscriber current_state_sub;
    // ros::Time start_time;
    //ros::Rate r;

    // double dt;
    // double current_sample_time;

    mav_trajectory_generation::Trajectory trajectory;

    visualization_msgs::MarkerArray traj_markers;
    visualization_msgs::MarkerArray current_state_markers;
    int state_count = 0; // for the marker.id of the current_state_markers
    double distance = 1.0; // Distance by which to seperate additional markers.
    std::string frame_id = "world";

    rvizPublisherNode(const ros::NodeHandle& n)
    :nh(n)
    {
      traj_pub = nh.advertise<visualization_msgs::MarkerArray>(
      	"visualization_marker_array_1", 0);
      current_state_pub = nh.advertise<visualization_msgs::MarkerArray>(
        "visualization_marker_array_2", 0);
      trajectory_sub = nh.subscribe("trajectory", 10,
        &rvizPublisherNode::trajCallback, this);
      current_state_sub = nh.subscribe("current_state", 100,
        &rvizPublisherNode::currentStateSubCallback, this);
      ros::Rate r(1);
    }

    void trajCallback(const mav_planning_msgs::PolynomialTrajectory4D& segments_message)
    {
      if (segments_message.segments.empty()) {
        ROS_WARN("RVIZ Publisher: received empty waypoint message");
        return;
      }
      else {
        ROS_INFO("RVIZ Publisher: received %lu waypoints",
          segments_message.segments.size());
      }

      bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(
          segments_message, &trajectory);
      if (!success) {
        ROS_WARN("Failure to Convert Trajectory Message");
        return;
      }
      rviz_traj_publish();
    }

    void currentStateSubCallback(const nav_msgs::Odometry& current_state)
    {
      geometry_msgs::Pose pose = current_state.pose.pose;
      std_msgs::Header header = current_state.header;

      state_count += 1;

      visualization_msgs::Marker new_state_marker;
      new_state_marker.pose = pose;
      new_state_marker.header = header;
      new_state_marker.id = state_count;
      new_state_marker.type = visualization_msgs::Marker::SPHERE;
      new_state_marker.action = visualization_msgs::Marker::ADD;
      new_state_marker.scale.x = 0.2;
      new_state_marker.scale.y = 0.2;
      new_state_marker.scale.z = 0.2;
      new_state_marker.color.a = 1;
      new_state_marker.color.r = 0;
      new_state_marker.color.g = 1;
      new_state_marker.color.b = 0;
      new_state_marker.lifetime = ros::Duration(0.0);

      current_state_markers.markers.push_back(new_state_marker);
      current_state_pub.publish(current_state_markers);
    }

    void rviz_traj_publish()
    {
    	// Trajectory visualization in RVIZ:
    	mav_trajectory_generation::drawMavTrajectory(trajectory, distance,
    																								frame_id, &traj_markers);
    	//publishing the visualization:
    	while(traj_pub.getNumSubscribers() < 1) {
    		ROS_WARN_ONCE("Please create a subscriber to the MarkerArray");
    		sleep(1);
    	}
    	traj_pub.publish(traj_markers);
    	//sleep(1);
    }
  }; // class rvizPublisherNode

 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "traj_generator_node");
   ros::NodeHandle nh;
   rvizPublisherNode rviz_publisher_node(nh);
   ROS_INFO("Initialized RVIZ Publisher Node");
   ros::spin();
 }
