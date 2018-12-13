/* gazebo_stock_publisher_node.cpp
 *
 * Grab setpoint information from /desired_state and publish it to gazebo
 * aero_quadsim is used to visualize the px4 drone and its response to the given
 * setpoint. The onboard controller is used in this implementation.
 *
 */

 #include <math.h>
 #include "../../aero_control/src/aero_ctrl.h"
 #include <ros/ros.h>
 #include <ros/console.h>
 #include <geometry_msgs/Pose.h>
 #include <trajectory_msgs/MultiDOFJointTrajectory.h>
 #include <tf/transform_datatypes.h>
 #include "std_msgs/String.h"
 #include <mavros_msgs/PositionTarget.h>

  class GazeboController
  {
  public:
    ros::NodeHandle nh;
    //ros::Publisher setpoint_pub;
    ros::Publisher flag_pub;
    ros::Publisher position_target_pub;
    ros::Publisher velocity_target_pub;
    ros::Subscriber desired_state_sub;
    ros::Subscriber current_state_sub;
    ros::Subscriber flag_sub;

    geometry_msgs::Pose current_pose;
    geometry_msgs::Pose desired_pose;
    mavros_msgs::PositionTarget position_target;
    geometry_msgs::Twist velocity_target;

    aero_ctrl controller;
    float start_alt;
    double eps;

    GazeboController(ros::NodeHandle& n)
    :nh(n)
    {
      //setpoint_pub = nh.advertise<
      flag_pub = nh.advertise<std_msgs::String>("flag_chatter", 1);
      position_target_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);
      velocity_target_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped", 1);
      desired_state_sub = nh.subscribe("desired_state", 1,
        // &GazeboController::desiredStateSubCallback_vel_cmd, this);
        &GazeboController::desiredStateSubCallback_raw_local, this);
        // &GazeboController::desiredStateSubCallback, this);
      current_state_sub = nh.subscribe("mavros/local_position/pose", 1,
        &GazeboController::currentStateSubCallback, this);
      flag_sub = nh.subscribe("flag_chatter", 10,
        &GazeboController::flagCallback, this);

      start_alt = 1.0;
      eps = 0.2;

      controller.arm();
      controller.takeoff(start_alt);

      std_msgs::String unity_msg;
      unity_msg.data = "UNITY_SIM";
      flag_pub.publish(unity_msg);

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

      // ros::Rate rate(30);
      //
      // while(sqrt((current_pose.position.x - desired_pose.position.x)*(current_pose.position.x - desired_pose.position.x) +
      //            (current_pose.position.y - desired_pose.position.y)*(current_pose.position.y - desired_pose.position.y) +
      //            (current_pose.position.z - desired_pose.position.z)*(current_pose.position.z - desired_pose.position.z)) > eps)
      // {
      //   controller.set_setpt(desired_pose.position.x,
      //                        desired_pose.position.y,
      //                        desired_pose.position.z,
      //                        desired_pose.orientation);
      //   rate.sleep();
      // }

      // double x = desired_state.transforms[0].translation.x;
      // double y = desired_state.transforms[0].translation.y;
      // double z = desired_state.transforms[0].translation.z;
      // tf::Quaternion quat(desired_state.transforms[0].rotation.x,
      //                     desired_state.transforms[0].rotation.y,
      //                     desired_state.transforms[0].rotation.z,
      //                     desired_state.transforms[0].rotation.w);
      // tf::Matrix3x3 m(quat);
      // double roll, pitch, yaw;
      // m.getRPY(roll, pitch, yaw);
      //
      // controller.navigate_to_pose(x, y, z, yaw);

      controller.set_setpt(desired_pose.position.x,
                            desired_pose.position.y,
                            desired_pose.position.z,
                            desired_pose.orientation);
    }

    void desiredStateSubCallback_raw_local(
      const trajectory_msgs::MultiDOFJointTrajectoryPoint& desired_state)
    {
      // Publishes to the /mavros/setpoint_raw/local topic
      position_target.header.stamp = ros::Time::now();
      // position_target.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE | mavros_msgs::PositionTarget::IGNORE_YAW;
      position_target.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE |
        mavros_msgs::PositionTarget::IGNORE_YAW |
        // mavros_msgs::PositionTarget::IGNORE_PX |
        // mavros_msgs::PositionTarget::IGNORE_PY |
        // mavros_msgs::PositionTarget::IGNORE_PZ |
        mavros_msgs::PositionTarget::IGNORE_VX |
        mavros_msgs::PositionTarget::IGNORE_VY |
        mavros_msgs::PositionTarget::IGNORE_VZ |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ;

      position_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
      position_target.position.x = desired_state.transforms[0].translation.x;
      position_target.position.y = desired_state.transforms[0].translation.y;
      position_target.position.z = desired_state.transforms[0].translation.z;
      position_target.velocity.x = desired_state.velocities[0].linear.x;
      position_target.velocity.y = desired_state.velocities[0].linear.y;
      position_target.velocity.z = desired_state.velocities[0].linear.z;
      position_target.acceleration_or_force.x = desired_state.accelerations[0].linear.x;
      position_target.acceleration_or_force.y = desired_state.accelerations[0].linear.y;
      position_target.acceleration_or_force.z = desired_state.accelerations[0].linear.z;

      // for use in landing after final state is reached:
      desired_pose.position.x = desired_state.transforms[0].translation.x;
      desired_pose.position.y = desired_state.transforms[0].translation.y;
      desired_pose.position.z = desired_state.transforms[0].translation.z;
      desired_pose.orientation = desired_state.transforms[0].rotation;

      // position_target.yaw_rate = desired_state.velocities[0].angular.z;

      // tf::Quaternion quat(desired_state.transforms[0].rotation.x,
      //                     desired_state.transforms[0].rotation.y,
      //                     desired_state.transforms[0].rotation.z,
      //                     desired_state.transforms[0].rotation.w);
      // tf::Matrix3x3 m(quat);
      // double roll, pitch, yaw;
      // m.getRPY(roll, pitch, yaw);
      //
      // position_target.yaw = yaw;

      position_target_pub.publish(position_target);
    }

    void desiredStateSubCallback_vel_cmd(
      const trajectory_msgs::MultiDOFJointTrajectoryPoint& desired_state)
    {
      // Publishes to the /mavros/setpoint_velocity/cmd_vel_unstamped topic
      velocity_target.linear = desired_state.velocities[0].linear;
      velocity_target.angular = desired_state.velocities[0].angular;

      // for use in landing after final state is reached:
      desired_pose.position.x = desired_state.transforms[0].translation.x;
      desired_pose.position.y = desired_state.transforms[0].translation.y;
      desired_pose.position.z = desired_state.transforms[0].translation.z;
      desired_pose.orientation = desired_state.transforms[0].rotation;

      velocity_target_pub.publish(velocity_target);
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
   ros::init(argc, argv, "gazebo_stock_publisher_node");
   ros::NodeHandle nh;
   GazeboController gazebo_stock_publisher_node(nh);
   ROS_INFO("Initialized Gazebo Stock Publisher Node");
   ros::spin();
  }
