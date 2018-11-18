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

	Note that waypoints at this high level do not include angular rates; they
	are just xyz positions. Therefore, Quaternion for every pose is identity.

	This is also meant as a way to simulate the waypoints being
	generated by RRT* externally, which will be the final
	implementation.

"""

import rospy
from geometry_msgs.msg import Point, Quaternion, Pose, PoseArray

# points defined below; adjust to change waypoints:
waypoints = [[0,0,0], [1,1,1], [2,2,2], [3,3,3], [4,4,4], [5,5,5], [5,5,0]]

orientation = Quaternion(0,0,0,1)
frame_id = None # may need to define as project gets more complex


def build_point_list(wp):
	point_list = []
	for point in wp:
		point_list.append(Point(point[0], point[1], point[2]))
	return point_list

def talker(point_list):
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
		talker(build_point_list(waypoints))
	except rospy.ROSInterruptException:
    	pass
