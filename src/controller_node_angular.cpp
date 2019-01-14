#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <mav_msgs/Actuators.h>
#include <mavros_msgs/AttitudeTarget.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <math.h>
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#define PI M_PI

//  Implementation of the geometric controller described in the paper below:
//
//  [1] Lee, Taeyoung, Melvin Leoky, N. Harris McClamroch. "Geometric tracking
//      control of a quadrotor UAV on SE (3)." Decision and Control (CDC),
//      49th IEEE Conference on. IEEE, 2010
//
//  We use variable names as close as possible to the conventions found in the
//  paper, however, we have slightly different conventions for the aerodynamic
//  coefficients of the propellers (refer to the lecture notes for these).
//  Additionally, watch out for the different conventions on reference frames
//  (see Lab 3 Handout for more details).
//
//  The include below is strongly suggested [but not mandatory if you have
//  better alternatives in mind :)]. Eigen is a c++ library for linear algebra
//  that will help you significantly with the implementation. Check the
//  quick reference page to learn the basics:
//
//  https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
#include <eigen3/Eigen/Dense>

// This implementation publishes angular rates, obtained after torques
// are calculated. It also uses px4 parameters such as mass and inertia.

class ControllerNode{
  ros::NodeHandle nh;
  ros::Subscriber desired_state_sub;
  ros::Subscriber current_state_sub;
  // ros::Subscriber current_pos_sub;
  // ros::Subscriber current_vel_sub;
  ros::Subscriber flag_sub;
  ros::Publisher attitudeTargetPub;
  ros::Timer utimer;
  ros::Time time_last_update;

  double kx, kv, kr, komega; // controller gains - [1] eq (15), (16)

  // Physical constants (we will set them below)
  double m;              // mass of the UAV
  double g;              // gravity acceleration
  double d;              // distance from the center of propellers to the c.o.m.
  double cf,             // Propeller lift coefficient
         cd;             // Propeller drag coefficient
  double f_max,          // Maximum thrust of the drone
         a_max;          // Maximum acceleration of the drone
  Eigen::Matrix3d J;     // Inertia Matrix
  Eigen::Vector3d e3;    // [0,0,1]
  Eigen::MatrixXd F2W;   // Wrench-rotor speeds map


  // Current state
  Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R;     // current orientation of the UAV
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame
  Eigen::Vector3d omega_cmd; // angular velocity command for UAV in *body* frame
  Eigen::Vector3d omega_cmd_old; // past angular velocity command
  Eigen::Vector3d M;     // moments about body axes (command)

  // Desired state
  Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
  Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame
  Eigen::Vector3d omegad;// desired angular velocity of the UAV's c.o.m in the world frame
  Eigen::Matrix3d Rd;    // desired orientation of the UAV

  double yawd;           // desired yaw angle
  double hz;             // frequency of the main control loop

  static Eigen::Vector3d Vee(const Eigen::Matrix3d& in){
    Eigen::Vector3d out;
    out << in(2,1), in(0,2), in(1,0);
    return out;
  }

  public:
    ControllerNode():e3(0,0,1),F2W(4,4),hz(30.0){
      desired_state_sub = nh.subscribe("/desired_state", 1, &ControllerNode::onDesiredState, this);
      current_state_sub = nh.subscribe("/mavros/local_position/odom", 1, &ControllerNode::onCurrentState, this);
      // current_pos_sub = nh.subscribe("/mavros/local_position/pose", 1, &ControllerNode::onCurrentPose, this);
      // current_vel_sub = nh.subscribe("/mavros/local_position/velocity", 1, &ControllerNode::onCurrentVel, this);
      flag_sub = nh.subscribe("/flag_chatter", 10, &ControllerNode::flagCallback, this);
      attitudeTargetPub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 1);
      // utimer = nh.createTimer(ros::Duration(1.0/hz), &ControllerNode::controlLoop, this);

      // kx = 7.0;
      // kv = 4.0;
      // kr = 8.0;
      // komega = 1.0;
      kx = 12.7;
      kv = 5.8;
      kr = 8.8;
      komega = 1.15;
      // kx = 13.0;
      // kv = 2.0;
      // kr = 5.0;
      // komega = 1.15;


      m = 1.0;
      cd = 1e-5;
      cf = 1e-3;
      // g = -9.81;
      g = 9.81;
      d = 0.3;
      J << 1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0;

      a_max = 8.0; // m/s, based on empirical data
      f_max = (m*a_max) + (m*g);

      time_last_update = ros::Time::now();
    }

    void flagCallback(const std_msgs::String& msg){
      if (msg.data == "START_STATE_PULBISH"){
        utimer = nh.createTimer(ros::Duration(1.0/hz), &ControllerNode::controlLoop, this);
      }
      // if (msg.data == "END_DESIRED_STATE_PUBLISH"){
      //   utimer.stop();
      // }
    }

    void onDesiredState(const trajectory_msgs::MultiDOFJointTrajectoryPoint& des_state){
      xd << des_state.transforms[0].translation.x, des_state.transforms[0].translation.y, des_state.transforms[0].translation.z;
      vd << des_state.velocities[0].linear.x, des_state.velocities[0].linear.y, des_state.velocities[0].linear.z;
      ad << des_state.accelerations[0].linear.x, des_state.accelerations[0].linear.y, des_state.accelerations[0].linear.z;
      yawd = tf::getYaw(des_state.transforms[0].rotation);

      Eigen::Quaterniond q;
      q.x() = des_state.transforms[0].rotation.x;
      q.y() = des_state.transforms[0].rotation.y;
      q.z() = des_state.transforms[0].rotation.z;
      q.w() = des_state.transforms[0].rotation.w;
      Rd = q.toRotationMatrix();
      Eigen::Vector3d omegad_world;
      omegad_world << des_state.velocities[0].angular.x, des_state.velocities[0].angular.y, des_state.velocities[0].angular.z;
      omegad = Rd.transpose()*omegad_world;
    }

    void onCurrentState(const nav_msgs::Odometry& cur_state){
        x << cur_state.pose.pose.position.x, cur_state.pose.pose.position.y, cur_state.pose.pose.position.z;
        v << cur_state.twist.twist.linear.x, cur_state.twist.twist.linear.y, cur_state.twist.twist.linear.z;

        Eigen::Quaterniond q;
        q.x() = cur_state.pose.pose.orientation.x;
        q.y() = cur_state.pose.pose.orientation.y;
        q.z() = cur_state.pose.pose.orientation.z;
        q.w() = cur_state.pose.pose.orientation.w;
        R = q.toRotationMatrix();
        Eigen::Vector3d omega_world;
        omega_world << cur_state.twist.twist.angular.x, cur_state.twist.twist.angular.y, cur_state.twist.twist.angular.z;
        omega = R.transpose()*omega_world;
    }

    // void onCurrentPose(const geometry_msgs::PoseStamped& cur_state){
    //   x << cur_state.pose.position.x, cur_state.pose.position.y, cur_state.pose.position.z;
    //   Eigen::Quaterniond q;
    //   q.x() = cur_state.pose.orientation.x;
    //   q.y() = cur_state.pose.orientation.y;
    //   q.z() = cur_state.pose.orientation.z;
    //   q.w() = cur_state.pose.orientation.w;
    //   R = q.toRotationMatrix();
    // }
    //
    // void onCurrentVel(const geometry_msgs::TwistStamped& cur_state){
    //   v << cur_state.twist.linear.x, cur_state.twist.linear.y, cur_state.twist.linear.z;
    //   Eigen::Vector3d omega_world;
    //   omega_world << cur_state.twist.angular.x, cur_state.twist.angular.y, cur_state.twist.angular.z;
    //   omega = R.transpose()*omega_world;
    // }

    // TODO:implement this callback so that the control loop rate is more finely controlled

    // void onControlRate(const ...){
    //   hz = ...; // set the main control loop rate to be twice the state estimation rate
    //   utimer.setPeriod(1.0/hz, reset=true);
    // }

    void controlLoop(const ros::TimerEvent& t){
      Eigen::Vector3d ex, ev, er, eomega;
      ex = x-xd;
      ev = v-vd;

      Eigen::Vector3d b1d, b1dprime, b2d, b3d;
      // b3d = -(-kx*ex-kv*ev+m*g*e3+m*ad);
      b3d = (-kx*ex-kv*ev+m*g*e3+m*ad);
      b3d.normalize();
      // ... computing the b2d vector ...
      b1dprime << cos(yawd), sin(yawd), 0;
      b1dprime.normalize();
      b2d = b3d.cross(b1dprime).normalized();
      b1d = b2d.cross(b3d).normalized(); // normalized might be redundant here
      b2d = b3d.cross(b1d).normalized();
      // ... assembling the Rd matrix ...
      Eigen::Matrix3d Rd, erhat;
      Rd << b1d, b2d, b3d;

      erhat = 0.5*(Rd.transpose()*R-R.transpose()*Rd);
      er = Vee(erhat); // vee operator
      eomega = omega-omegad;

      // f must be normalized to 1.0.
      // double f = -(-kx*ex-kv*ev+m*g*e3+m*ad).dot(R*e3);
      double f = (-kx*ex-kv*ev+m*g*e3+m*ad).dot(R*e3);

      // f_max = 10*m;
      double f_limit = 1.0;

      // std::cout << "f_orig:" << f << std::endl;
      if (f > f_max){
        // f_max = f;     // reset the maximum thrust if higher values are detected
        f = f_max;      // thrust cannot exceed our maximum constraint
      }
      if (f < 0){
        f = 0;          // thrust cannot be negative
      }
      f = (f-0.0)/(f_max-0.0)*f_limit; // normalize thrust to 1.0
      // std::cout << "f_norm:" << f << std::endl;

      double dt = (ros::Time::now() - time_last_update).toSec();
      M = -kr*er-komega*eomega+omega.cross(J*omega);
      // std::cout << "dw/dt:\n" << J.inverse()*M << "\n" << std::endl;
      omega_cmd = omega + dt*(J.inverse()*M);

      double max_cmd = 15;
      double omega_limit = 6.28;

      for (int i=0; i<3; i++){
        omega_cmd(i) = (omega_cmd(i)-0.0)/(max_cmd-0.0)*omega_limit;
        if (omega_cmd(i) > omega_limit){
          omega_cmd(i) = omega_limit;
        }
        else if (omega_cmd(i) < -omega_limit){
          omega_cmd(i) = -omega_limit;
        }
      }

      // std::cout << "\nex:\n" << ex << std::endl;
      // std::cout << "\nomega_cmd:\n" << omega_cmd << std::endl;
      // std::cout << "\nomega:\n" << omega << std::endl;
      // std::cout << "\neomega:\n" << eomega << std::endl;
      // std::cout << "f:" << f << "\n" << std::endl;

      mavros_msgs::AttitudeTarget attitudeTarget;
      // attitudeTarget.header.stamp = ros::Time::now();
      // attitudeTarget.header.frame_id = 0;
      attitudeTarget.body_rate.x = omega_cmd(0);
      attitudeTarget.body_rate.y = -omega_cmd(1);
      attitudeTarget.body_rate.z = -omega_cmd(2);
      attitudeTarget.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
      attitudeTarget.thrust = f;
      attitudeTargetPub.publish(attitudeTarget);

      omega_cmd_old = omega_cmd;
      time_last_update = ros::Time::now();

      // Eigen::VectorXd fM(4); fM << f,(M);
      // Eigen::VectorXd W(4);
      // auto signedsqrt = [](double in){
      //     double res = sqrt(abs(in));
      //     return (in<0)?-res:res;
      // };
      // constexpr double cs45 = 1.0/sqrt(2);
      // F2W <<        cf,       cf,        cf,        cf,
      //        cs45*d*cf,cs45*d*cf,-cs45*d*cf,-cs45*d*cf,
      //       -cs45*d*cf,cs45*d*cf, cs45*d*cf,-cs45*d*cf,
      //               cd,      -cd,        cd,       -cd;
      // W = (F2W.inverse()*fM).unaryExpr(signedsqrt);
      //
      // mav_msgs::Actuators rotor_speeds;
      // rotor_speeds.header.stamp = ros::Time::now();
      // rotor_speeds.angular_velocities.resize(4);
      // rotor_speeds.angular_velocities[0] = W[0];
      // rotor_speeds.angular_velocities[1] = W[1];
      // rotor_speeds.angular_velocities[2] = W[2];
      // rotor_speeds.angular_velocities[3] = W[3];
      // rotor_speeds_pub.publish(rotor_speeds);
    }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "controller_node");
  ControllerNode n;
  ros::spin();
}
