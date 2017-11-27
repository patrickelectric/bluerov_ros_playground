#!/usr/bin/env python

import rospy
import time

try:
    import pubs
    import subs
except:
    import bluerov.pubs as pubs
    import bluerov.subs as subs

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, Joy


class GazeboTeleop(object):

    """Class to handle with gazebo teleop

    Attributes:
        pub (TYPE): ROS publisher
        sub (TYPE): ROS subscriber
    """

    def __init__(self):
        super(GazeboTeleop, self).__init__()

        self.sub = subs.Subs()
        self.pub = pubs.Pubs()

        self.pub.subscribe_topic('/BlueRov2/thruster_command', JointState)

        self.sub.subscribe_topic('/BlueRov2/state', Odometry)
        self.sub.subscribe_topic('/joy', Joy)

    def run(self):
        """ Run Gazebo Teleop
        """
        while not rospy.is_shutdown():
            time.sleep(0.1)
            # Try to get data
            try:
                # Get ROV position
                rospy.loginfo(self.sub.get_data()['BlueRov2']['state']['pose']['pose'])
            except Exception as error:
                print('Get data error:', error)

            try:
                # Get joystick data and send it to Gazebo model
                joy = self.sub.get_data()['joy']['axes']

                forces = [0 for u in range(6)]

                # joystick analog enum
                #  left stick: 1 = up down; 0 = right left
                #  right stick: 4 = up down; 3 = right left
                forces[0] = joy[1] +joy[0]
                forces[1] = joy[1] -joy[0]
                forces[2] = -joy[1] -joy[0]
                forces[3] = -joy[1] +joy[0]
                forces[4] = joy[4] +joy[3]
                forces[5] = joy[4] -joy[3]

                joint = JointState()
                joint.name = ["thr{}".format(u + 1) for u in range(6)]
                joint.position = [pwm for pwm in forces]

                self.pub.set_data('/BlueRov2/thruster_command', joint)
            except Exception as error:
                print('rc error:', error)


if __name__ == "__main__":
    try:
        rospy.init_node('gazebo_teleop', log_level=rospy.DEBUG)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)
    gazebo_teleop = GazeboTeleop()
    gazebo_teleop.run()
