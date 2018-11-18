/* convert_desired_to_actuator.cpp
 *
 * Converts messages sent to the /desired_state topic as
 * MultiDOFJointTrajectoryPoint into mavros_msgs::ActuatorControl messages to
 * be published to the /mavros/actuator_control topic.
 *
 * This conversion bypasses controller_node entirely as well as the onboard
 * px4 controller. It gets roll, pitch, yaw and thrust from the trajectory
 * point the drone should be at. This is not a control loop; there is no
 * incorporation of state estimation here.
 *
 */

#include <ros/ros.h>
#include <mavros_msgs/ActuatorControl.h>
#include <geometry_msgs/Pose.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <tf/transform_datatypes.h>

#define PX4_MIX_FLIGHT_CONTROL 0

  class ConvertDesiredToActuator
  {
    ros::NodeHandle nh;
    ros::Publisher control_pub;
    ros::Subscriber desired_sub;
    ros::Subscriber flag_sub;

    float[] angular_velocities;
    std_msgs::Header header;

    ConvertDesiredToActuator(ros::NodeHandle& n)
    :nh(n)
    {
      control_pub = nh.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 1);
      desired_sub = nh.subscribe("desired_state", 1,
        &ConvertDesiredToActuator::desiredStateSubCallback, this);
      flag_sub = nh.subscribe("flag_chatter", 10,
        &ConvertDesiredToActuator::flagCallback, this);
    }

    void desiredStateSubCallback(
      const trajectory_msgs::MultiDOFJointTrajectoryPoint& msg)
    {
      mavros_msgs::ActuatorControl control_msg;
      control_msg.header = msg.header;
      control_msg.group_mix = PX4_MIX_FLIGHT_CONTROL;

      double x = desired_state.transforms[0].translation.x;
      double y = desired_state.transforms[0].translation.y;
      double z = desired_state.transforms[0].translation.z;
      tf::Quaternion quat(desired_state.transforms[0].rotation.x,
                          desired_state.transforms[0].rotation.y,
                          desired_state.transforms[0].rotation.z,
                          desired_state.transforms[0].rotation.w);
      tf::Matrix3x3 m(quat);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      control_msg.controls[0] = roll;
      control_msg.controls[1] = pitch;
      control_msg.controls[2] = yaw;
      control_msg.controls[4] = 1; // this isn't going to work...

      control_pub.publish(control_msg);
    }

    void flagCallback(const std_msgs::String& msg)
    {

    }

  }; // class ConvertRotorToActuator

  int main(int argc, char** argv)
  {
    ros::init(argc, argv, "convert_desired_to_actuator");
    ros::NodeHandle nh;
    ConvertDesiredToActuator convert_desired_to_actuator_node(nh);
    ROS_INFO("Initialized Actuator Conversion Node");
    ros::spin();
  }
