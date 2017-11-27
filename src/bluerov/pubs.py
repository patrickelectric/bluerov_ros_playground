#!/usr/bin/env python
"""Publish data to ROS topic
"""
import mavros_msgs.msg
import rospy
import time


class Pubs(object):
    """Class that control publish data to ROS

    Attributes:
        data (dict): Dict that contains all data available of all topics
        topics (list): list of topics structs
    """
    def __init__(self):
        # Dict with all data
        self.data = {}
        # Get data from topic list
        self.topics = []

        self.subscribe_topics()

    def get_data(self):
        """Return data dict

        Returns:
            dict: Data from all topics
        """
        return self.data

    def set_data(self, path, value={}, pub=None):
        """Add topic to dict and add data on it

        Args:
            path (string): Topic
            value (dict, optional): Data of topic
            pub (None, optional): rospy.Publisher
        """

        # The first item will be empty
        keys = path.split('/')[1:]
        current_level = self.data
        for part in keys:
            # If dict don't have the path, create it !
            if part not in current_level:
                current_level[part] = {}
            current_level = current_level[part]

        # Publish data (if it exist) to ros topic (path)
        if value is not {} and 'pub' in current_level:
            try:
                current_level['pub'].publish(value)
            except Exception as error:
                print(error)

        # Add publisher to dict
        if pub is not None:
            current_level.update({'pub': pub})

    def subscribe_topic(self, topic, msg_type, queue=1):
        """Update dict with topic

        Args:
            topic (string): Topic path
            msg_type (ros msg type): Message type
            queue (int, optional): number of messages in queue
        """
        self.set_data(topic, pub=rospy.Publisher(
                topic, msg_type, queue_size=queue))

    def subscribe_topics(self):
        """Create dict to access publisher

        Args:
            init (bool, optional): init node
        """

        # Get item in topics and populate dict with publisher
        for topic, msg_type, queue in self.topics:
            self.subscribe_topic(topic, msg_type, queue)

    def callback(self, data, topic):
        """ROS callback

        Args:
            data (string): Data from ROS topic
            topic (string): ROS topic name
        """
        self.set_data(topic, data)


if __name__ == '__main__':
    try:
        rospy.init_node('set_mav_data')
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)

    pub = Pubs()
    pub.subscribe_topic('/mavros/rc/override', mavros_msgs.msg.OverrideRCIn)

    def rc():
        pub.set_data('/mavros/rc/override',
                     [1201, 1200, 1200, 1200, 1200, 1200, 1200, 1205])

    while not rospy.is_shutdown():
        rc()
        time.sleep(1)
