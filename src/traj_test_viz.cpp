// testing trajectory generation and visualization using linear method

#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <typeinfo>
using namespace std;

int main(int argc, char** argv)
{
	mav_trajectory_generation::Vertex::Vector vertices;
	const int dimension = 3;
	const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
	mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);

	start.makeStartOrEnd(Eigen::Vector3d(0,0,1), derivative_to_optimize);
	vertices.push_back(start);

	middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1,2,3));
	vertices.push_back(middle);

	end.makeStartOrEnd(Eigen::Vector3d(2,1,5), derivative_to_optimize);
	vertices.push_back(end);

	std::vector<double> segment_times;
	const double v_max = 2.0;
	const double a_max = 2.0;
	const double magic_fabian_constant = 6.5; // A tuning parameter.
	segment_times = estimateSegmentTimes(vertices, v_max, a_max, magic_fabian_constant);

	const int N = 10;
	mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
	opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
	opt.solveLinear();


	mav_trajectory_generation::Segment::Vector segments;
	opt.getSegments(&segments);

	//Now compute the trajectory:
	mav_trajectory_generation::Trajectory trajectory;
	opt.getTrajectory(&trajectory);

	// Single sample:
	double sampling_time = 2.0;
	int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
	Eigen::VectorXd sample = trajectory.evaluate(sampling_time, derivative_order);

	//visualization:
	ros::Time::init();
	
	visualization_msgs::MarkerArray markers;
	double distance = 1.0; // Distance by which to seperate additional markers. Set 0.0 to disable.
	std::string frame_id = "world";

	// From Trajectory class:
	mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);

	//cout << "\n\nmarkers: \n" << markers << "\n";

	//publishing the visualization:
	ros::init(argc, argv, "traj_test_viz");
	ros::NodeHandle n;
	ros::Rate r(1);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 0);

	while(marker_pub.getNumSubscribers() < 1)
	{
		if(!ros::ok()) { return 0; }
		ROS_WARN_ONCE("Please create a subscriber to the marker");
		sleep(1);
	}

	cout << "\n\ntype of markers: " << typeid(markers).name() << endl;

	marker_pub.publish(markers);

	sleep(1); // I had to include this or for some reason the data didn't get published. Maybe main loop closes too quickly
			  // if there is no sleep?
}