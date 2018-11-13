/* gazebo_stock_publisher_node.cpp
 *
 * Grab setpoint information from /desired_state and publish it to gazebo
 * aero_quadsim is used to visualize the px4 drone and its response to the given
 * setpoint. The onboard controller is used in this implementation.
 *
 */

 #include <math.h>
 #include "../../aero_quadsim/src/aero_ctrl.h"
 #include <ros/ros.h>
 #include <ros/console.h>
 #include <geometry_msgs/Pose.h>
 #include <trajectory_msgs/MultiDOFJointTrajectory.h>
 #include <tf/transform_datatypes.h>

  class GazeboController
  {
  public:
    ros::NodeHandle nh;
    //ros::Publisher setpoint_pub;
    ros::Subscriber desired_state_sub;
    ros::Subscriber current_state_sub;

    geometry_msgs::Pose current_pose;
    geometry_msgs::Pose desired_pose;


    aero_ctrl controller;
    float start_alt;
    double eps;

    GazeboController(ros::NodeHandle& n)
    :nh(n)
    {
      //setpoint_pub = nh.advertise<
      desired_state_sub = nh.subscribe("desired_state", 1000,
        &GazeboController::desiredStateSubCallback, this);
      current_state_sub = nh.subscribe("mavros/local_position/pose", 1000,
        &GazeboController::currentStateSubCallback, this);

      start_alt = 1.0;
      eps = 0.08;

      controller.arm();
      controller.takeoff(start_alt);
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

  }; // class GazeboController

  int main(int argc, char** argv)
  {
   ros::init(argc, argv, "gazebo_stock_publisher_node");
   ros::NodeHandle nh;
   GazeboController gazebo_stock_publisher_node(nh);
   ROS_INFO("Initialized Gazebo Stock Publisher Node");
   ros::spin();
  }
