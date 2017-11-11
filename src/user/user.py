#!/usr/bin/env python

import cv2
import rospy
import threading
import time

try:
    import user.pubs as pubs
    import user.subs as subs
    import user.video as video
except:
    import pubs
    import subs
    import video

from sensor_msgs.msg import JointState


class Code(threading.Thread):
    def __init__(self):
        super(Code, self).__init__()
        self.daemon = True
        self.sub = subs.Subs()
        self.pub = pubs.Pubs()
        rospy.init_node('user_node', log_level=rospy.DEBUG)
        self.sub.subscribe_topics()
        self.pub.subscribe_topics()

        self.cam = video.Video()
        self.cam.start()

        self. i = 0

        while not self.cam.frame_available():
            pass

    def pwm_to_thrust(self, pwm):
        # Equation from
        #    https://colab.research.google.com/notebook#fileId=1CEDW9ONTJ8Aik-HVsqck8Y_EcHYLg0zK
        return -3.04338931856672e-13*pwm**5 \
            + 2.27813523978448e-9*pwm**4 \
            - 6.73710647138884e-6*pwm**3 \
            + 0.00983670053385902*pwm**2 \
            - 7.08023833982539*pwm \
            + 2003.55692021905


    def run(self):
        while True:
            # Try to get data
            try:
                rospy.loginfo(self.sub.get_data()['mavros']['battery']['voltage'])
                rospy.loginfo(self.sub.get_data()['mavros']['rc']['in']['channels'])
                rospy.loginfo(self.sub.get_data()['mavros']['rc']['out']['channels'])
            except Exception as error:
                print('Get data error:', error)

            try:
                joy = self.sub.get_data()['joy']['axes']

                # pwm run between 1100 and 2000, a joy command is between -1.0 and 1.0
                override = [int(val*400 + 1500) for val in joy]
                self.pub.set_data('/mavros/rc/override', override)
            except Exception as error:
                print('joy error:', error)

            try:
                rc = self.sub.get_data()['mavros']['rc']['out']['channels']
                joint = JointState()
                joint.name = ["thr{}".format(u + 1) for u in range(5)]
                joint.position = [self.pwm_to_thrust(pwm) for pwm in rc]

                self.pub.set_data('/BlueRov2/body_command', joint)
            except Exception as error:
                print('rc error:', error)

            try:
                _, frame = self.cam.frame()
                cv2.imshow('frame', frame)
                cv2.waitKey(1)
            except Exception as error:
                print('imshow error:', error)

            time.sleep(0.1)


if __name__ == "__main__":
    thread = Code()
    thread.start()
