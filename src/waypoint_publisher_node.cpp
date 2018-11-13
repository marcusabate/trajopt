/* waypoint_publisher_node.cpp
 *
 * User places desired waypoints in the 'wps' vector. This class then
 * publishes these waypoints for use in trajectory generation.
 *
 * Place waypoints in the 'wps' vector, located in main()
 *
 * Waypoints are published to the /waypoints rostopic.
 *
 * These waypoints only have xyz coordinates and do not specify other states.
 * Therefore, the quaternion is identity for the pose that gets published.*
 *
 */

  #include "std_msgs/String.h"
  #include "ros/ros.h"
  #include <geometry_msgs/PoseArray.h>

  class WaypointPublisherNode
  {
  public:
    ros::NodeHandle nh;
    ros::Publisher waypoint_pub;
    ros::Subscriber flag_sub;

    std::vector< std::vector<double> > waypoints;
    geometry_msgs::Quaternion rotation;

    bool publishedOnce;

    WaypointPublisherNode(const ros::NodeHandle& n,
      std::vector< std::vector<double> > wps)
    :nh(n),
     publishedOnce(false)
    {
      waypoint_pub = nh.advertise<geometry_msgs::PoseArray>(
        "waypoints", 100);
      // flag_sub = nh.subscribe("flag_chatter", 10,
      //   &WaypointPublisherNode::flagCallback, this);

      waypoints = wps;
      rotation.x = 0;
      rotation.y = 0;
      rotation.z = 0;
      rotation.w = 1;
    }

    void publish_info()
    {
        if (!publishedOnce)
        {
          geometry_msgs::PoseArray poseArray;

          for (int i=0; i<waypoints.size(); i++)
          {
            geometry_msgs::Pose this_pose;
            this_pose.position.x = waypoints[i][0];
            this_pose.position.y = waypoints[i][1];
            this_pose.position.z = waypoints[i][2];
            this_pose.orientation = rotation;

            poseArray.poses.push_back(this_pose);
          }

          while (waypoint_pub.getNumSubscribers() < 1)
          {
            ROS_WARN_ONCE("Waypoint Publisher: Please create a subscriber to the /waypoints rostopic");
            ros::Duration(1).sleep();
          }

          waypoint_pub.publish(poseArray);
          publishedOnce = true;
        }
        else
        {
          ROS_WARN("Waypoint Publisher: Requested start sim, but already published once");
        }
    }
  }; // class WaypointPublisherNode


  int main(int argc, char** argv)
  {
    ros::init(argc, argv, "waypoint_publisher_node");
    ros::NodeHandle nh;

    std::vector< std::vector<double> > wps =
    { {1,1,1},{2,2,2},{3,3,3},{4,4,4},{5,5,5},{5,5,0} };

    std::cout << "Press any key to start simulation" << std::endl;
    std::string input;
    std::cin >> input;
    std::cout << "Starting sim" << std::endl;

    WaypointPublisherNode waypoint_publisher_node(nh, wps);
    ROS_INFO("Initialized Waypoint Publisher Node");
    waypoint_publisher_node.publish_info();

    // ros::spin(); // only to keep node alive.

    return 0;
 }
