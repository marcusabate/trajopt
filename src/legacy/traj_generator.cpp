/* traj_generator.cpp
 *
 * This code grabs waypoints from the /waypoints rostopic
 * and then computes a trajectory between them using
 * the standard nlopt methods given by the
 * mav_trajectory_generation package. It then publishes
 * this trajectory to rviz as well as poses for the drone
 * to follow either on hardware on in gazebo
 *
 * Marcus Abate
 */

#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/feasibility_analytic.h>
#include <mav_trajectory_generation_ros/feasibility_base.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/default_topics.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PoseArray.h>
#include "ros/ros.h"
//using namespace std;

const int dimension = 3;
const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
float temp_x, temp_y, temp_z;

/*
void waypointCallback(const geometry_msgs::Point::ConstPtr& msg)
{
	temp_x = msg->x;
	temp_y = msg->y;
	temp_z = msg->z;

  	ROS_INFO("I heard: [%s]", msg->data.c_str());
}
*/

mav_trajectory_generation::Vertex::Vector get_waypoints()
{
	// First we create a subscriber to the /waypoints topic to pick them up:
	//ros::NodeHandle n;
	//ros::Subscriber sub = n.subscribe("waypoints", 1000, waypointCallback);
	//ros::spin();
	boost::shared_ptr<geometry_msgs::PoseArray const> pose_array_ptr =
							ros::topic::waitForMessage<geometry_msgs::PoseArray>("/waypoints");
	geometry_msgs::PoseArray pose_array = *pose_array_ptr;

	int pose_size = pose_array.poses.size();

	mav_trajectory_generation::Vertex::Vector waypoints;
	mav_trajectory_generation::Vertex start(dimension), end(dimension); // the two guaranteed waypoints

	//First waypoint picked up is the start
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

	//Final waypoint in the list is the end
	end.makeStartOrEnd(Eigen::Vector3d(pose_array.poses[pose_size-1].position.x,
																		 pose_array.poses[pose_size-1].position.y,
																		 pose_array.poses[pose_size-1].position.z),
																		 derivative_to_optimize);
	waypoints.push_back(end);

	return waypoints;
}

mav_trajectory_generation::Trajectory get_trajectory(
					mav_trajectory_generation::Vertex::Vector waypoints)
{
	// The following is standard optimization and segment production from the usual examples:

	std::vector<double> segment_times;
	const double v_max = 2.0;
	const double a_max = 2.0;
	// const double magic_fabian_constant = 6.5; // A tuning parameter. Seems deprecated
	// segment_times = estimateSegmentTimes(waypoints, v_max, a_max, magic_fabian_constant);
	segment_times = estimateSegmentTimes(waypoints, v_max, a_max);

	mav_trajectory_generation::NonlinearOptimizationParameters parameters;
	parameters.max_iterations = 1000;
	parameters.f_rel = 0.05;
	parameters.x_rel = 0.1;
	parameters.time_penalty = 500.0;
	parameters.initial_stepsize_rel = 0.1;
	parameters.inequality_constraint_tolerance = 0.1;

	const int N = 10;
	mav_trajectory_generation::PolynomialOptimizationNonLinear<N> opt(dimension, parameters);
	opt.setupFromVertices(waypoints, segment_times, derivative_to_optimize);
	opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, v_max);
	opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, a_max);
	opt.optimize();

	mav_trajectory_generation::Segment::Vector segments;
	opt.getPolynomialOptimizationRef().getSegments(&segments);

	//Now compute the trajectory:
	mav_trajectory_generation::Trajectory trajectory;
	opt.getTrajectory(&trajectory);

	return trajectory;
}

void rviz_publish(mav_trajectory_generation::Trajectory trajectory)
{
	//visualization:
	ros::Time::init();

	visualization_msgs::MarkerArray markers;
	double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
	std::string frame_id = "world";

	// From Trajectory class:
	mav_trajectory_generation::drawMavTrajectory(trajectory, distance,
																								frame_id, &markers);

	//std::cout << "\n\nmarkers: \n" << markers << "\n";

	//publishing the visualization:
	ros::NodeHandle n;
	ros::Rate r(1);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>(
				"visualization_marker_array", 0);

	while(marker_pub.getNumSubscribers() < 1)
	{
		//if(!ros::ok()) { return 0; }
		ROS_WARN_ONCE("Please create a subscriber to the MarkerArray");
		sleep(1);
	}

	marker_pub.publish(markers);
	sleep(1); // I had to include this or for some reason the data didn't get published. Maybe main loop closes too quickly
			  // if there is no sleep?
}

void trajectory_command_publish(mav_trajectory_generation::Trajectory trajectory)
{
	// publish the entire trajectory command at once.
	// this will be a MultiDOFJointTrajectory message for use with controller_node
	// see trajectory_sampler_node.cpp for other methods

	ros::Time::init();

	const double sampling_time = 0.1;

	mav_msgs::EigenTrajectoryPoint::Vector flat_states;
	mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_time,
																									 &flat_states);
	trajectory_msgs::MultiDOFJointTrajectory cmd_msg;
	msgMultiDofJointTrajectoryFromEigen(flat_states, &cmd_msg);

	ros::NodeHandle nh_cmd;
	ros::Rate r(1);
	ros::Publisher command_pub =
			nh_cmd.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
				//	mav_msgs::default_topics::COMMAND_TRAJECTORY, 1);
					"desired_state", 1);

	while(command_pub.getNumSubscribers() < 1)
	{
		ROS_WARN_ONCE("Please create a subscriber to the MarkerArray");
		sleep(1);
	}

	command_pub.publish(cmd_msg);
	sleep(1);
}

void trajectory_publish(mav_trajectory_generation::Trajectory trajectory)
{
	// publish the trajectory as a polynomialTrajectoryMsg to be back-converted
	// to a trajectory by the trajectory_sampler_node
	std::cout << "instantiating msg now:" << std::endl;
	mav_planning_msgs::PolynomialTrajectory4D* traj_msg;

	// std::string frame_id = "world";
	// std_msgs::Header header;
	// header.frame_id = frame_id;
	// header.stamp = ros::Time::now();
	//
	// traj_msg->header = header;
	//traj_msg->segments.reserve(trajectory.segments().size());

	std::cout << "header set" << std::endl;
	bool success = mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(
											trajectory, traj_msg);

	ros::Time::init();
	ros::NodeHandle nh_traj;
	ros::Rate r(1);
	ros::Publisher trajectory_pub =
			nh_traj.advertise<mav_planning_msgs::PolynomialTrajectory4D>(
					"path_segments", 1);

	if (success) {
		while(trajectory_pub.getNumSubscribers() < 1)
		{
			//if(!ros::ok()) { return 0; }
			ROS_WARN_ONCE("Please create a subscriber to the MarkerArray");
			sleep(1);
		}

		trajectory_pub.publish(*traj_msg);
	}
	else {
		std::cout << "Unable to convert trajectory to message format" << std::endl;
		ROS_WARN("Unable to convert trajectory to message format");
	}
	sleep(1);
}

std::vector<mav_trajectory_generation::InputFeasibilityResult>
		get_feasibility_result(mav_trajectory_generation::Trajectory trajectory)
{
	// Check input feasibility for generated trajectory.

	// Create input constraints:
	typedef mav_trajectory_generation::InputConstraintType ICT;
	mav_trajectory_generation::InputConstraints input_constraints;
	input_constraints.addConstraint(ICT::kFMin, 0.5 * 9.81); // minimum acceleration in [m/s/s].
	input_constraints.addConstraint(ICT::kFMax, 1.5 * 9.81); // maximum acceleration in [m/s/s].
	input_constraints.addConstraint(ICT::kVMax, 3.5); // maximum velocity in [m/s].
	input_constraints.addConstraint(ICT::kOmegaXYMax, M_PI / 2.0); // maximum roll/pitch rates in [rad/s].
	input_constraints.addConstraint(ICT::kOmegaZMax, M_PI / 2.0); // maximum yaw rates in [rad/s].
	input_constraints.addConstraint(ICT::kOmegaZDotMax, M_PI); // maximum yaw acceleration in [rad/s/s].

	mav_trajectory_generation::FeasibilityAnalytic feasibility_check(input_constraints);
	feasibility_check.settings_.setMinSectionTimeS(0.01);

	mav_trajectory_generation::Segment::Vector segments = trajectory.segments();
	int vec_size = segments.size();
	std::vector<mav_trajectory_generation::InputFeasibilityResult> result (vec_size);

	for (unsigned i; i<vec_size; i++)
	{
		result[i] = feasibility_check.checkInputFeasibility(segments[i]);
	}

	return result;
}

int main(int argc, char **argv)
{
	//Listen to /waypoints to get waypoints:
	//ros::init(argc, argv, "waypoint_listener");
	ros::init(argc, argv, "traj_generator");
	mav_trajectory_generation::Vertex::Vector waypoints = get_waypoints();

	//Get Trajectory:
	mav_trajectory_generation::Trajectory trajectory;
	trajectory = get_trajectory(waypoints);

	// Check input feasibility:
	std::vector<mav_trajectory_generation::InputFeasibilityResult>
				feasibility_result = get_feasibility_result(trajectory);

	for (unsigned i; i<feasibility_result.size(); i++)
	{
		std::string seg_result = mav_trajectory_generation::getInputFeasibilityResultName(
			feasibility_result[i]);
		std::cout << "Feasibility result segment " << i << ": " << seg_result << std::endl;
		if (seg_result != "Feasible")
		{
			// ROS_ERROR('Infeasible Trajectory');
			ROS_INFO("Infeasible trajectory segment. Exiting.");
			return 0;
		}
	}

	// Visualization in rviz:
	rviz_publish(trajectory);
	std::cout << "rviz publish done" << std::endl;

	// Control publishing:
	//trajectory_publish(trajectory);
	//std::cout << "traj publish done" << std::endl;
	trajectory_command_publish(trajectory);
	std::cout << "cmd publish done" << std::endl;

	return 0;
}
