#!/usr/bin/env python

"""

	manual_waypoint.py

	Marcus Abate

	This script publishes waypoints to rostopic /waypoints so that 
	the trajectory optimizer can use them. 
	The user can edit these waypoints in this script at the top 
	and they'll be published constantly to the rostopic so that the 
	user can run the trajectory optimziation software at any time 
	and it'll be able to subscribe to the /waypoints topic and 
	grab the waypoints.

	This is also meant as a way to simulate the waypoints being
	generated by RRT* externally, which will be the final 
	implementation.

"""

import rospy
from geometry_msgs.msg import Point, Quaternion, Pose, PoseArray

# points defined below; adjust to change waypoints:
point_list = []
point_list.append(Point(0,0,0))
point_list.append(Point(1,1,1))
point_list.append(Point(2,3,6))
point_list.append(Point(3,3,8))
point_list.append(Point(4,7,20))
point_list.append(Point(5,5,15))

orientation = Quaternion(0,0,0,1)


def talker():
	# convert point_list into a PoseArray:
	Poses = PoseArray()
	for point in point_list:
		Poses.poses.append(Pose(point, orientation))

	pub = rospy.Publisher('waypoints', PoseArray, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	while not rospy.is_shutdown():
		rospy.loginfo(Poses)
		pub.publish(Poses)
		rate.sleep()

if __name__ == '__main__':
    try:
		talker()
    except rospy.ROSInterruptException:
        pass