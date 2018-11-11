/* state_publisher_node.cpp
 *
 * This ROS node subscribes to the /trajectory rostopic and grabs the generated
 * trajectory. It then cuts it into MultiDOFJointTrajectoryPoint messages for
 * each time step and publishes these messages to the /desired_state rostopic.
 *
 * This node is responsible for communicating with the controller_node, which
 * subscribes to the /desired_state rostopic and gets these messages.
 */

#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include "ros/ros.h"

class StatePublisherNode
{
  ros::NodeHandle nh;
  ros::Timer publish_timer;
  ros::Publisher command_pub;
  ros::Subscriber trajectory_sub;
  ros::Time start_time;
  ros::Rate r;

  double dt;
  double current_sample_time;

  mav_trajectory_generation::Trajectory;

  StatePublisherNode(const ros::Nodehandle& n)
   :nh(n),
    dt(0.01)
    current_sample_time(0.0),
  {
    command_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      "desired_state", 1);
    trajectory_sub = nh.subscribe("trajectory", 10, trajCallback);
    publish_timer = nh.createTimer(ros::Duration(dt), &commandTimerCallback);
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
  }

  void commandTimerCallback(const ros::TimerEvent&)
  {
    if (current_sample_time <= trajectory.getMaxTime())
    {
      trajectory_msgs::MultiDOFJointTrajectory msg;
      mav_msgs::EigenTrajectoryPoint flat_state;
      bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
        trajectory, current_sample_time, %flat_state);
      if (!success) {
        ROS_WARN("Failure to sample trajectory at current time");
        publish_timer.stop();
      }
      mav_msgs::msgMultiDofJointTrajectoryFromEigen(flat_state, &msg);
      msg.points[0].time_from_start = ros::Duration(current_sample_time);
      command_pub.publish(msg);
      current_sample_time += dt;
    }
    else
    {
      ROS_INFO("Full trajectory published to /desired_state.")
      publish_timer.stop();
    }
  }

} // class StatePublisherNode

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_publisher_node");
  ros::NodeHandle nh;
  StatePublisherNode state_publisher_node(nh);
  ROS_INFO("Initialized State Publisher Node");
  ros::spin();
}
