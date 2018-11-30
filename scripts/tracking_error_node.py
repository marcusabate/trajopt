#!/usr/bin/env python

"""
    tracking_error_node.py

    Marcus Abate

    This script gathers data on the desired state and the current state of the
    drone at every time step during the gazebo simulation. The tracking error
    is then computed in several important states for each time step and
    visualized graphically post flight.
    The script also calculates a measure of tracking error overall. This is the
    integral of the error between desired and current position states across
    the whole trajectory. This can be used to compare different controller
    implementations or trajectory generation parameters.
"""

import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import matplotlib.pyplot as plt

class TrackingErrorNode:
    def __init__(self):
        self.flag_sub = rospy.Subscriber('flag_chatter', String, self.flagSubCallback)
        self.desired_state_sub = rospy.Subscriber('desired_state', MultiDOFJointTrajectoryPoint, self.desiredStateSubCallback)
        # self.current_state_sub = rospy.Subscriber('mavros/local_position/pose', PoseStamped, self.currentStateSubCallback) # use with gazebo
        self.current_state_sub = rospy.Subscriber('current_state', Odometry, self.currentStateSubCallback) # use with unity
        self.des_x = []
        self.des_y = []
        self.des_z = []
        self.cur_x = []
        self.cur_y = []
        self.cur_z = []
        self.x_error = []
        self.y_error = []
        self.z_error = []
        self.tracking_error = []
        # more states to be handled soon

    def desiredStateSubCallback(self, desired_state):
        if len(self.des_x) <= len(self.cur_x) or len(self.des_x) == 0:
            self.des_x.append(desired_state.transforms[0].translation.x)
            self.des_y.append(desired_state.transforms[0].translation.y)
            self.des_z.append(desired_state.transforms[0].translation.z)

    def currentStateSubCallback(self, current_state):
        if len(self.cur_x) <= len(self.des_x) or len(self.cur_x) == 0:
            # for use with gazebo (PoseStamped message):
            # self.cur_x.append(current_state.pose.position.x)
            # self.cur_y.append(current_state.pose.position.y)
            # self.cur_z.append(current_state.pose.position.z)

            # for use with unity (Odometry message):
            self.cur_x.append(current_state.pose.pose.position.x)
            self.cur_y.append(current_state.pose.pose.position.y)
            self.cur_z.append(current_state.pose.pose.position.z)

    def flagSubCallback(self, msg):
        if msg.data == 'END_DESIRED_STATE_PUBLISH':
            self.desired_state_sub.unregister()
            self.current_state_sub.unregister()
            self.calc_errors()
            self.show_final_visuals()

        if msg.data == 'START_STATE_PULBISH':
            pass

    def calc_errors(self):
        # if len(self.desired_states) != 0 and len(self.current_states) != 0:
        for i in range(min(len(self.cur_x), len(self.des_x))):
            self.x_error.append(self.cur_x[i] - self.des_x[i])
            self.y_error.append(self.cur_y[i] - self.des_y[i])
            self.z_error.append(self.cur_z[i] - self.des_z[i])
            track_er = math.sqrt(self.x_error[i]**2 + self.y_error[i]**2 + self.z_error[i]**2)
            self.tracking_error.append(track_er)

            # self.update_visuals()

    def show_final_visuals(self):
        ''' Display plots of tracking error. '''

        plt.figure(1)
        plt.subplot(411)
        plt.plot(self.x_error)
        plt.title('X error')

        plt.subplot(412)
        plt.plot(self.y_error)
        plt.title('Y error')

        plt.subplot(413)
        plt.plot(self.z_error)
        plt.title('Z error')

        plt.subplot(414)
        plt.plot(self.tracking_error)
        plt.title('Overall Position Tracking Error')


        legend = ['Desired', 'Actual']
        plt.figure(2)
        plt.subplot(311)
        plt.plot(self.des_x)
        plt.plot(self.cur_x)
        plt.legend(legend)
        plt.title('X Trajectory')

        plt.subplot(312)
        plt.plot(self.des_y)
        plt.plot(self.cur_y)
        plt.legend(legend)
        plt.title('Y Trajectory')

        plt.subplot(313)
        plt.plot(self.des_z)
        plt.plot(self.cur_z)
        plt.legend(legend)
        plt.title('Z Trajectory')


        plt.show()

    def update_visuals(self):
        ''' Update plots in real time. '''
        raise NotImplementedError

if __name__ == '__main__':
    rospy.init_node('TrackingErrorNode', anonymous=True)
    tracking_error_node = TrackingErrorNode()
    rospy.spin()
