/* convert_rotor_to_actuator.cpp
 *
 * Converts messages sent to the /rotor_speed_cmds topic as mav_msgs::Actuators
 * into mavros_msgs::ActuatorControl messages to be published to the
 * /mavros/actuator_control topic.
 *
 * This conversion allows controller_node to be less cluttered and publish
 * its control input to the same topic regardless of which simulator is being
 * used. If the gazebo simulator is being used with the custom controller_node,
 * then this node will convert those messages over to a px4-friendly type.
 *
 */

#include <ros/ros.h>
#include <mav_msgs/Actuators.h>
#include <mavros_msgs/ActuatorControl.h>

#define PX4_MIX_FC_MC_VIRT 4

  class ConvertRotorToActuator
  {
    ros::NodeHandle nh;
    ros::Publisher control_pub;
    ros::Subscriber rotor_sub;
    ros::Subscriber flag_sub;

    float[] angular_velocities;
    std_msgs::Header header;

    ConvertRotorToActuator(ros::NodeHandle& n)
    :nh(n)
    {
      control_pub = nh.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 1);
      rotor_sub = nh.subscribe("rotor_speed_cmds", 1,
        &ConvertRotorToActuator::rotorCallback, this);
      flag_sub = nh.subscribe("flag_chatter", 10,
        &ConvertRotorToActuator::flagCallback, this);
    }

    void rotorCallback(const mav_msgs::Actuators& msg)
    {
      mavros_msgs::ActuatorControl control_msg;
      control_msg.header = msg.header;
      control_msg.group_mix = PX4_MIX_FC_MC_VIRT;
      control_msg.
    }

    void flagCallback(const std_msgs::String& msg)
    {

    }

  }; // class ConvertRotorToActuator
