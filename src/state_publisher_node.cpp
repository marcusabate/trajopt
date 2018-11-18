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
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include "std_msgs/String.h"
#include "ros/ros.h"

 class StatePublisherNode
 {
 public:
    ros::NodeHandle nh;
    ros::Timer publish_timer;
    ros::Publisher command_pub;
    ros::Publisher flag_pub;
    ros::Subscriber trajectory_sub;
    ros::Subscriber flag_sub;
    ros::Time start_time;

    double dt;
    double current_sample_time;

    mav_trajectory_generation::Trajectory trajectory;

    StatePublisherNode(const ros::NodeHandle& n)
     :nh(n),
      dt(0.01),
      current_sample_time(0.0)
    {
      command_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
        "desired_state", 1);
      flag_pub = nh.advertise<std_msgs::String>("flag_chatter", 1);
      trajectory_sub = nh.subscribe("trajectory", 10,
        &StatePublisherNode::trajCallback, this);
      flag_sub = nh.subscribe("flag_chatter", 10,
        &StatePublisherNode::flagCallback, this);
      publish_timer = nh.createTimer(ros::Duration(dt),
        &StatePublisherNode::commandTimerCallback, this);
      ros::Rate r(1);
    }

    void trajCallback(const mav_planning_msgs::PolynomialTrajectory4D& segments_message)
    {
      if (segments_message.segments.empty()) {
        ROS_WARN("State Publisher: received empty waypoint message");
        return;
      }
      else {
        ROS_INFO("State Publisher: received %lu waypoints",
                 segments_message.segments.size());
      }

      bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(
          segments_message, &trajectory);
      if (!success) {
        ROS_WARN("State Publisher: Failure to Convert Trajectory Message");
        return;
      }
    }

    void commandTimerCallback(const ros::TimerEvent&)
    {
      if (current_sample_time <= trajectory.getMaxTime())
      {
        trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
        mav_msgs::EigenTrajectoryPoint flat_state;
        bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
          trajectory, current_sample_time, &flat_state);
        if (!success) {
          ROS_WARN("State Publisher: Failure to sample trajectory at current time");
          publish_timer.stop();
        }
        mav_msgs::msgMultiDofJointTrajectoryPointFromEigen(flat_state, &msg);
        msg.time_from_start = ros::Duration(current_sample_time);
        command_pub.publish(msg);
        current_sample_time += dt;
      }
      else
      {
        ROS_INFO("State Publisher: Full trajectory published to /desired_state.");
        std_msgs::String msg;
        msg.data = "END_DESIRED_STATE_PUBLISH";
        flag_pub.publish(msg);

        publish_timer.stop();
      }
    }

    void flagCallback(const std_msgs::String msg)
		{
			if (msg.data == "START_STATE_PULBISH")
      {
        publish_timer.start();
        current_sample_time = 0.0;
        start_time = ros::Time::now();
      }
		}
 }; // class StatePublisherNode

 int main(int argc, char** argv)
 {
    ros::init(argc, argv, "state_publisher_node");
    ros::NodeHandle nh;
    StatePublisherNode state_publisher_node(nh);
    ROS_INFO("Initialized State Publisher Node");
    ros::spin();
 }
