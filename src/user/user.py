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
        self.cam.start_capture()

    def run(self):
        # User code
        try:
            rospy.loginfo(self.sub.get_data()['mavros']['battery']['voltage'])
            rospy.loginfo(self.sub.get_data()['mavros']['rc']['in']['channels'])
            self.pub.set_data('/mavros/rc/override',
                              [1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200])

            ret, frame = self.cam.frame()
            cv2.imshow('frame', frame)
            cv2.waitKey(1)
        except:
            pass

        time.sleep(0.1)
        self.run()


if __name__ == "__main__":
    thread = Code()
    thread.start()
