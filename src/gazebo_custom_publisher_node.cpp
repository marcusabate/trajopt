/* gazebo_custom_publisher_node.cpp
 *
 * Grab setpoint information from /desired_state and publish it to gazebo
 * aero_quadsim is used to visualize the px4 drone and its response to the given
 * setpoint.
 * This implementation uses the controller_node controller to generate actuator
 * commands.
 *
 */

 #include <math.h>
 #include "../../aero_quadsim/src/aero_ctrl.h"
 #include <ros/ros.h>
 #include <ros/console.h>
 #include <geometry_msgs/Pose.h>
 #include <trajectory_msgs/MultiDOFJointTrajectory.h>
 #include <tf/transform_datatypes.h>
 #include "std_msgs/String.h"

  class GazeboController
  {
  public:
    ros::NodeHandle nh;
    //ros::Publisher setpoint_pub;
    ros::Publisher flag_pub;
    ros::Subscriber desired_state_sub;
    ros::Subscriber current_state_sub;
    ros::Subscriber flag_sub;

    geometry_msgs::Pose current_pose;
    geometry_msgs::Pose desired_pose;

    aero_ctrl controller;
    float start_alt;
    double eps;

    GazeboController(ros::NodeHandle& n)
    :nh(n)
    {
      //setpoint_pub = nh.advertise<
      flag_pub = nh.advertise<std_msgs::String>("flag_chatter", 1);
      desired_state_sub = nh.subscribe("desired_state", 1000,
        &GazeboController::desiredStateSubCallback, this);
      current_state_sub = nh.subscribe("mavros/local_position/pose", 1000,
        &GazeboController::currentStateSubCallback, this);
      flag_sub = nh.subscribe("flag_chatter", 10,
        &GazeboController::flagCallback, this);

      start_alt = 1.0;
      eps = 0.08;

      controller.arm();
      controller.takeoff(start_alt);

      std_msgs::String gazebo_msg;
      gazebo_msg.data = "GAZEBO_SIM";
      flag_pub.publish(gazebo_msg);

      std_msgs::String flag_msg;
      flag_msg.data = "START_STATE_PULBISH";
      flag_pub.publish(flag_msg);
    }

    void currentStateSubCallback(const geometry_msgs::PoseStamped msg)
    {
      current_pose = msg.pose;
    }

    void desiredStateSubCallback(
      const trajectory_msgs::MultiDOFJointTrajectoryPoint& desired_state)
    {
      // ROS_INFO("Received Desired State");
      desired_pose.position.x = desired_state.transforms[0].translation.x;
      desired_pose.position.y = desired_state.transforms[0].translation.y;
      desired_pose.position.z = desired_state.transforms[0].translation.z;
      desired_pose.orientation = desired_state.transforms[0].rotation;
    }

    void flagCallback(const std_msgs::String msg)
		{
			if (msg.data == "END_DESIRED_STATE_PUBLISH")
      {
        ros::Rate rate(30);

        while(sqrt((current_pose.position.x - desired_pose.position.x)*(current_pose.position.x - desired_pose.position.x) +
                   (current_pose.position.y - desired_pose.position.y)*(current_pose.position.y - desired_pose.position.y) +
                   (current_pose.position.z - desired_pose.position.z)*(current_pose.position.z - desired_pose.position.z)) > eps)
        {
          controller.set_setpt(desired_pose.position.x,
                               desired_pose.position.y,
                               desired_pose.position.z,
                               desired_pose.orientation);
          rate.sleep();
        }

        controller.land();
      }
		}

  }; // class GazeboController

  int main(int argc, char** argv)
  {
   ros::init(argc, argv, "gazebo_custom_publisher_node");
   ros::NodeHandle nh;
   GazeboController gazebo_custom_publisher_node(nh);
   ROS_INFO("Initialized Gazebo Custom Publisher Node");
   ros::spin();
  }
