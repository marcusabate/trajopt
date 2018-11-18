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
#include "std_msgs/String.h"
#include "ros/ros.h"

 class rvizPublisherNode
 {
 public:
    ros::NodeHandle nh;
    ros::Publisher traj_pub;
    ros::Publisher current_state_pub;
    ros::Publisher desired_state_pub;
    ros::Publisher flag_pub;
    ros::Subscriber trajectory_sub;
    ros::Subscriber current_state_sub;
    ros::Subscriber desired_state_sub;
    ros::Subscriber flag_sub;

    bool PUBLISH_CURRENT_STATE = true;
    bool PUBLISH_DESIRED_STATE = true;

    mav_trajectory_generation::Trajectory trajectory;

    visualization_msgs::MarkerArray traj_markers;
    visualization_msgs::MarkerArray current_state_markers;
    visualization_msgs::MarkerArray desired_state_markers;
    int current_state_count = 0; // for the marker.id of the current_state_markers
    int desired_state_count = 0;
    double distance = 1.0; // Distance by which to seperate additional markers.
    std::string frame_id = "world";

    rvizPublisherNode(const ros::NodeHandle& n)
    :nh(n)
    {
      traj_pub = nh.advertise<visualization_msgs::MarkerArray>(
      	"visualization_marker_array_1", 0);
      current_state_pub = nh.advertise<visualization_msgs::MarkerArray>(
        "visualization_marker_array_2", 0);
      // current_state_pub = nh.advertise<nav_msgs::Odometry>(
      //   "odometry_1", 0);
      desired_state_pub = nh.advertise<visualization_msgs::MarkerArray>(
        "visualization_marker_array_3", 0);
      flag_pub = nh.advertise<std_msgs::String>("flag_chatter", 1);
      trajectory_sub = nh.subscribe("trajectory", 10,
        &rvizPublisherNode::trajCallback, this);
      current_state_sub = nh.subscribe("current_state", 1000,
        &rvizPublisherNode::currentStateSubCallback, this);
      desired_state_sub = nh.subscribe("desired_state", 1000,
        &rvizPublisherNode::desiredStateSubCallback, this);
      flag_sub = nh.subscribe("flag_chatter", 10,
        &rvizPublisherNode::flagCallback, this);
      ros::Rate r(1);

      std_msgs::String unity_msg;
      unity_msg.data = "UNITY_SIM";
      flag_pub.publish(unity_msg);
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
        ROS_WARN("RVIZ Publisher: Failure to Convert Trajectory Message");
        return;
      }
      rviz_traj_publish();

      std_msgs::String flag_msg;
      flag_msg.data = "START_STATE_PULBISH";
      flag_pub.publish(flag_msg);
    }

    void currentStateSubCallback(const nav_msgs::Odometry& current_state)
    {
      if (PUBLISH_CURRENT_STATE)
      {
        geometry_msgs::Pose pose = current_state.pose.pose;
        std_msgs::Header header = current_state.header;

        current_state_count += 1;

        visualization_msgs::Marker new_state_marker;
        new_state_marker.pose = pose;
        new_state_marker.header = header;
        new_state_marker.id = current_state_count;
        new_state_marker.type = visualization_msgs::Marker::SPHERE;
        new_state_marker.action = visualization_msgs::Marker::ADD;
        new_state_marker.scale.x = 0.1;
        new_state_marker.scale.y = 0.1;
        new_state_marker.scale.z = 0.1;
        new_state_marker.color.a = 1;
        new_state_marker.color.r = 0;
        new_state_marker.color.g = 1;
        new_state_marker.color.b = 0;
        new_state_marker.lifetime = ros::Duration(0.0);

        current_state_markers.markers.push_back(new_state_marker);
        current_state_pub.publish(current_state_markers);

        // nav_msgs::Odometry odometry = current_state;
        // current_state_pub.publish(odometry);
      }
    }

    void desiredStateSubCallback(
      const trajectory_msgs::MultiDOFJointTrajectoryPoint& desired_state)
    {
      if (PUBLISH_DESIRED_STATE)
      {
        geometry_msgs::Pose pose;
        pose.position.x = desired_state.transforms[0].translation.x;
        pose.position.y = desired_state.transforms[0].translation.y;
        pose.position.z = desired_state.transforms[0].translation.z;
        pose.orientation = desired_state.transforms[0].rotation;

        desired_state_count += 1;

        visualization_msgs::Marker new_state_marker;
        new_state_marker.header.frame_id = frame_id;
        new_state_marker.pose = pose;
        new_state_marker.id = desired_state_count;
        new_state_marker.type = visualization_msgs::Marker::SPHERE;
        new_state_marker.action = visualization_msgs::Marker::ADD;
        new_state_marker.scale.x = 0.1;
        new_state_marker.scale.y = 0.1;
        new_state_marker.scale.z = 0.1;
        new_state_marker.color.a = 0.2;
        new_state_marker.color.r = 0;
        new_state_marker.color.g = 0;
        new_state_marker.color.b = 1;
        new_state_marker.lifetime = ros::Duration(0.0);

        desired_state_markers.markers.push_back(new_state_marker);
        desired_state_pub.publish(desired_state_markers);
      }
    }

    void flagCallback(const std_msgs::String& msg)
    {
      if (msg.data == "END_DESIRED_STATE_PUBLISH")
      {
        PUBLISH_CURRENT_STATE = false;
        PUBLISH_DESIRED_STATE = false;
      }
    }

    void rviz_traj_publish()
    {
    	mav_trajectory_generation::drawMavTrajectory(trajectory, distance,
    																								frame_id, &traj_markers);
    	while(traj_pub.getNumSubscribers() < 1) {
    		ROS_WARN_ONCE("RVIZ Publisher: Please create a subscriber to the MarkerArray");
    		sleep(1);
    	}
    	traj_pub.publish(traj_markers);
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
