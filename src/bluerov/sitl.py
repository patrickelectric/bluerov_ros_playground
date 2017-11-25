#!/usr/bin/env python

import cv2
import math
import mavros_msgs
import rospy
import signal
import sys
import time

try:
    import pubs
    import subs
except:
    import bluerov.pubs as pubs
    import bluerov.subs as subs

from gazebo_msgs.msg import ModelState
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped


class SITL(object):

    """Class to handle with SITL

    Attributes:
        pub (TYPE): ROS publisher
        sub (TYPE): ROS subscriber
    """

    def __init__(self):
        super(SITL, self).__init__()

        self.arm()

        self.sub = subs.Subs()
        self.pub = pubs.Pubs()

        self.sub.subscribe_topic('/mavros/local_position/pose', PoseStamped)
        self.pub.subscribe_topic('/gazebo/set_model_state', ModelState, 1)

    def run(self):
        """ Sendo SITL information to gazebo
        """
        while not rospy.is_shutdown():
            time.sleep(0.1)

            # Get ROV position and send it to gazebo
            try:
                pose = self.sub.get_data()['mavros']['local_position']['pose']['pose']

                model_state = ModelState()
                model_state.model_name = 'BlueRov2'
                # Set position
                model_state.pose.position.x = pose['position']['x']
                model_state.pose.position.y = pose['position']['y']
                model_state.pose.position.z = pose['position']['z']

                # Set rotation
                q = [q1*q2 for q1, q2 in zip(
                    [pose['orientation']['x'], pose['orientation']['y'], pose['orientation']['z'], pose['orientation']['w']],
                    [math.sin(3.1415/2), 0, 0, math.cos(3.1415/2)])]
                model_state.pose.orientation.x = q[0]
                model_state.pose.orientation.y = q[1]
                model_state.pose.orientation.z = q[2]
                model_state.pose.orientation.w = q[3]

                self.pub.set_data('/gazebo/set_model_state', model_state)
            except Exception as error:
                print('Get data error:', error)

    def arm(self):
        """ Arm the vehicle and trigger the disarm
        """
        rospy.wait_for_service('/mavros/cmd/arming')

        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.arm_service(True)

        # Disarm is necessary when shutting down
        rospy.on_shutdown(self.disarm)

        # Set to guided mode
        rospy.wait_for_service('/mavros/set_mode')
        mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        mode_service(custom_mode='GUIDED')

    def disarm(self):
        self.arm_service(False)


if __name__ == "__main__":
    try:
        rospy.init_node('bluerov_sitl', log_level=rospy.DEBUG)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)
    bluerov_sitl = SITL()
    bluerov_sitl.run()
