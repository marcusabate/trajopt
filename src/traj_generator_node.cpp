/* traj_generator_node.cpp
 *
 * This code grabs waypoints from the /waypoints rostopic
 * and then computes a trajectory between them using
 * the standard nlopt methods given by the
 * mav_trajectory_generation package. It then publishes
 * this trajectory to /trajectory as a PolynomialSegment4D message.
 *
 * Marcus Abate
 */

#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

#include <mav_trajectory_generation_ros/feasibility_analytic.h>
#include <mav_trajectory_generation_ros/feasibility_base.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/default_topics.h>
#include <geometry_msgs/PoseArray.h>
#include "std_msgs/String.h"
#include "ros/ros.h"
//using namespace std;

 class TrajGeneratorNode
 {
 public:
		ros::NodeHandle nh;
		ros::Publisher traj_pub;
		ros::Publisher flag_pub;
		ros::Subscriber waypoint_sub;
		ros::Subscriber flag_sub;

		mav_trajectory_generation::Vertex::Vector waypoints;
		mav_trajectory_generation::NonlinearOptimizationParameters parameters;
		mav_trajectory_generation::Segment::Vector segments;
		mav_trajectory_generation::Trajectory trajectory;

		std::vector<double> segment_times;
		const double v_max;
		const double a_max;
		static const int N = 10;

		const int dimension;
		const int derivative_to_optimize;
		float temp_x, temp_y, temp_z;

		mav_trajectory_generation::InputConstraints input_constraints;
		std::vector<mav_trajectory_generation::InputFeasibilityResult>
					feasibility_result;
		bool isFeasible;

		TrajGeneratorNode(const ros::NodeHandle& n)
		:nh(n),
     // N(10),
     dimension(3),
     //v_max(2.0),
     //a_max(2.0),
     v_max(15.0),
     a_max(15.0),
     derivative_to_optimize(mav_trajectory_generation::derivative_order::SNAP)
		{
			traj_pub = nh.advertise<mav_planning_msgs::PolynomialTrajectory4D>(
				"trajectory", 1);
			flag_pub = nh.advertise<std_msgs::String>("flag_chatter", 1);
			waypoint_sub = nh.subscribe("waypoints", 10,
				&TrajGeneratorNode::waypointCallback, this);
			flag_sub = nh.subscribe("flag_chatter", 10,
				&TrajGeneratorNode::flagCallback, this);
			ros::Rate r(1);

      // N = 10;
      // dimension = 3;
      // v_max = 2.0;
      // a_max = 2.0;
      // derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;

			parameters.max_iterations = 1000;
			parameters.f_rel = 0.05;
			parameters.x_rel = 0.1;
			//parameters.time_penalty = 500.0;
      parameters.time_penalty = 10000.0;
			parameters.initial_stepsize_rel = 0.1;
			parameters.inequality_constraint_tolerance = 0.1;

			isFeasible = true;
			typedef mav_trajectory_generation::InputConstraintType ICT;

			input_constraints.addConstraint(ICT::kFMin, 0.5 * 9.81); // minimum acceleration in [m/s/s].
			// input_constraints.addConstraint(ICT::kFMax, 1.5 * 9.81); // maximum acceleration in [m/s/s].
      input_constraints.addConstraint(ICT::kFMax, 15 * 9.81);
			//input_constraints.addConstraint(ICT::kVMax, 3.5); // maximum velocity in [m/s].
      input_constraints.addConstraint(ICT::kVMax, 15);
			input_constraints.addConstraint(ICT::kOmegaXYMax, M_PI / 2.0); // maximum roll/pitch rates in [rad/s].
			input_constraints.addConstraint(ICT::kOmegaZMax, M_PI / 2.0); // maximum yaw rates in [rad/s].
			input_constraints.addConstraint(ICT::kOmegaZDotMax, M_PI); // maximum yaw acceleration in [rad/s/s].
 		}

		void flagCallback(const std_msgs::String msg)
		{
			// what to do here
		}

		void waypointCallback(const geometry_msgs::PoseArray& pose_array)
		{
			int pose_size = pose_array.poses.size();

			mav_trajectory_generation::Vertex start(dimension), end(dimension);
			start.makeStartOrEnd(Eigen::Vector3d(pose_array.poses[0].position.x,
																					 pose_array.poses[0].position.y,
																					 pose_array.poses[0].position.z),
																					 derivative_to_optimize);
			waypoints.push_back(start);
			for (int i=1; i<pose_size-1; i++)
			{
				mav_trajectory_generation::Vertex middle(dimension); // a middle waypoint
				middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
								Eigen::Vector3d(pose_array.poses[i].position.x,
																pose_array.poses[i].position.y,
																pose_array.poses[i].position.z));
				waypoints.push_back(middle);
			}
			end.makeStartOrEnd(Eigen::Vector3d(pose_array.poses[pose_size-1].position.x,
																				 pose_array.poses[pose_size-1].position.y,
																				 pose_array.poses[pose_size-1].position.z),
																				 derivative_to_optimize);
			waypoints.push_back(end);

			get_trajectory();
			check_feasibility();
      publish_trajectory();
		}

		void get_trajectory()
		{
			segment_times = mav_trajectory_generation::estimateSegmentTimes(
				waypoints, v_max, a_max);

      mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(
        dimension, parameters);
			opt.setupFromVertices(waypoints, segment_times, derivative_to_optimize);
			opt.addMaximumMagnitudeConstraint(
				mav_trajectory_generation::derivative_order::VELOCITY, v_max);
			opt.addMaximumMagnitudeConstraint(
				mav_trajectory_generation::derivative_order::ACCELERATION, a_max);
			opt.optimize();
			opt.getPolynomialOptimizationRef().getSegments(&segments);
			opt.getTrajectory(&trajectory);
		}

		void get_feasibility_result()
		{
			mav_trajectory_generation::FeasibilityAnalytic feasibility_check(
        input_constraints);
			feasibility_check.settings_.setMinSectionTimeS(0.01);

			feasibility_result.reserve(segments.size());
			for (unsigned i; i<segments.size(); i++)
			{
				feasibility_result[i] = feasibility_check.checkInputFeasibility(
          segments[i]);
			}
		}

		void check_feasibility()
		{
			get_feasibility_result();
			for (unsigned i; i<feasibility_result.size(); i++)
			{
				std::string seg_result =
          mav_trajectory_generation::getInputFeasibilityResultName(
					feasibility_result[i]);
				if (seg_result != "Feasible")
				{
					isFeasible = false;
				}
			}
		}

		void publish_trajectory()
		{
			if (isFeasible)
			{
				mav_planning_msgs::PolynomialTrajectory4D traj_msg;
				bool success = mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(
														trajectory, &traj_msg);
				if (success)
				{
					while(traj_pub.getNumSubscribers() < 1) {
						ROS_WARN_ONCE("Trajectory Generator: Please create a subscriber to the /trajectory rostopic");
						sleep(1);
					}
          ROS_INFO("Trajectory Generator: Generated and published trajectory.");
					traj_pub.publish(traj_msg);
				}
				else
				{
					ROS_WARN("Trajectory Generator: Unable to convert trajectory to message format");
				}
			}
			else
			{
				ROS_ERROR("Trajectory Generator: Infeasible trajectory segment.");
			}
		}
	}; // TrajGeneratorNode class

	int main(int argc, char **argv)
	{
		ros::init(argc, argv, "traj_generator_node");
    ros::NodeHandle nh;
    TrajGeneratorNode traj_generator_node(nh);
    ROS_INFO("Initialized Trajectory Generator Node");
    ros::spin();
	}
