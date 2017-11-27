#!/usr/bin/env python

import math
import numpy as np
import rospy
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
        self.pub.subscribe_topic('/gazebo/set_model_state', ModelState)

    @staticmethod
    def quaternion_from_axis(axis, angle=0):
        """ Create quaternion from axis angle
            http://www.popflock.com/learn?s=Conversion_between_quaternions_and_Euler_angles
        Args:
            axis (list): Versor
            angle (int, optional): Versor rotation angle

        Returns:
            TYPE: Description
        """
        s = math.sin(angle/2)
        return [math.cos(angle/2)] + [i*s for i in axis]

    @staticmethod
    def quaternion_multiply(q1, q0):
        """ Ordered list form multiplication
            https://en.wikipedia.org/wiki/Quaternion

        Args:
            q1 (TYPE): Quaternion (a1, b1, c1, d1)
            q0 (TYPE): Quaternion (a2, b2, c2, d2)

        Returns:
            TYPE: multiplication between q1 and q0
        """
        return np.array([ q1[0] * q0[0] - q1[1] * q0[1] - q1[2] * q0[2] - q1[3] * q0[3],
                          q1[0] * q0[1] + q1[1] * q0[0] + q1[2] * q0[3] - q1[3] * q0[2],
                          q1[0] * q0[2] - q1[1] * q0[3] + q1[2] * q0[0] + q1[3] * q0[1],
                          q1[0] * q0[3] + q1[1] * q0[2] - q1[2] * q0[1] + q1[3] * q0[0]], dtype=np.float64)

    @staticmethod
    def q2e(q):
        """ Return euler (rpy) from quaternion (wxyz)

        Args:
            q (list): Quaternion

        Returns:
            list: rpy list
        """
        # roll (x-axis rotation)
        sinr = 2.0 * (q[3] * q[0] + q[1] * q[2])
        cosr = 1.0 - 2.0 * (q[0] * q[0] + q[1] * q[1])
        roll = math.atan2(sinr, cosr)

        # pitch (y-axis rotation)
        sinp = +2.0 * (q[3] * q[1] - q[2] * q[0])
        if math.fabs(sinp) >= 1:
            # use 90 degrees if out of range
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny = 2.0 * (q[3] * q[2] + q[0] * q[1])
        cosy = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
        yaw = math.atan2(siny, cosy)

        return [roll, pitch, yaw]

    @staticmethod
    def e2q(rpy):
            """ Return quaternion (wxyz) from euler (rpy)

            Args:
                rpy (list): roll pitch yaw list

            Returns:
                list: Quaternion
            """
            cy = math.cos(rpy[2] / 2)
            sy = math.sin(rpy[2] / 2)
            cr = math.cos(rpy[0] / 2)
            sr = math.sin(rpy[0] / 2)
            cp = math.cos(rpy[1] / 2)
            sp = math.sin(rpy[1] / 2)
            q = [0 , 0, 0, 0]
            q[0] = cy * cr * cp + sy * sr * sp
            q[1] = cy * sr * cp - sy * cr * sp
            q[2] = cy * cr * sp + sy * sr * cp
            q[3] = sy * cr * cp - cy * sr * sp
            return q

    def run(self):
        """ Send SITL information to gazebo
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

                # Rotate axis to gazebo orientation
                q1 = [q for q in [pose['orientation']['w'], pose['orientation']['z'], pose['orientation']['x'], pose['orientation']['y']]]

                q2 = self.quaternion_from_axis([1, 0, 0], math.pi)
                q = self.quaternion_multiply(q1, q2)
                q1 = self.quaternion_multiply(q1, q2)
                q2 = self.quaternion_from_axis([1, 0, 0], -math.pi/2)
                q = self.quaternion_multiply(q2, q1)

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
        mode_service(custom_mode='MANUAL')

    def disarm(self):
        """ Disarm vehicle
        """
        self.arm_service(False)


if __name__ == "__main__":
    try:
        rospy.init_node('bluerov_sitl', log_level=rospy.DEBUG)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)
    bluerov_sitl = SITL()
    bluerov_sitl.run()
