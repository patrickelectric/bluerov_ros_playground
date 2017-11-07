#!/usr/bin/env python
import rospy
import threading

import mavros_msgs.msg

from sensor_msgs.msg import JointState


class Pubs(object):
    def __init__(self):
        # Dict with all data
        self.data = {}
        # Get data from topic list
        self.topics = [
            # Sensors
            ['/mavros/rc/override',
                mavros_msgs.msg.OverrideRCIn,
                1
             ],
            ['/BlueRov2/body_command',
                JointState,
                1
             ]
        ]

    def get_data(self):
        return self.data

    def set_data(self, path, value={}, pub=None):
        # The first item will be empty
        keys = path.split('/')[1:]
        current_level = self.data
        for part in keys:
            # If dict don't have the path, create it !
            if part not in current_level:
                current_level[part] = {}
            current_level = current_level[part]

        if value is not {} and 'pub' in current_level:
            current_level['pub'].publish(value)
        if pub is not None:
            current_level.update({'pub': pub})

    def subscribe_topics(self, init=False):
        # Create dict to access publisher
        for topic, msg_type, queue in self.topics:
            self.set_data(topic, pub=rospy.Publisher(
                topic, msg_type, queue_size=queue))

        try:
            if init:
                rospy.init_node('set_mav_data')
            thread = threading.Thread(target=lambda: rospy.spin())
            thread.start()
        except rospy.ROSInterruptException:
            print(e)

    def callback(self, data, topic):
        self.set_data(topic, data)


if __name__ == '__main__':
    pub = Pubs()
    pub.subscribe_topics(True)

    def rc():
        pub.set_data('/mavros/rc/override',
                     [1201, 1200, 1200, 1200, 1200, 1200, 1200, 1205])
        thread = threading.Timer(1.0, rc)
        thread.daemon = True
        thread.start()
    rc()
