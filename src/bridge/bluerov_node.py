#!/usr/bin/env python

from __future__ import division

import json
import math
import re
import rospy
import sys
import time

from bridge import Bridge

try:
    from pubs import Pubs
    from subs import Subs
    from video import Video
except:
    from bluerov.pubs import Pubs
    from bluerov.subs import Subs
    from bluerov.video import Video

# convert opencv image to ros image msg
from cv_bridge import CvBridge

# msgs type
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import UInt16

class BlueRov(Bridge):
    def __init__(self, device='udp:192.168.2.1:14550', baudrate=115200):
        """ BlueRov ROS Bridge

        Args:
            device (str, optional): mavproxy device description
            baudrate (int, optional): Serial baudrate
        """
        super(BlueRov, self).__init__(device, baudrate)
        self.pub = Pubs()
        self.sub = Subs()
        self.ROV_name = 'BlueRov2'
        self.model_base_link = '/base_link'

        self.video = Video()
        self.video_bridge = CvBridge()

        self.pub_topics = [
            [
                self._create_battery_msg,
                '/battery',
                BatteryState,
                1
            ],
            [
                self._create_camera_msg,
                '/camera/image_raw',
                Image,
                1
            ],
            [
                self._create_ROV_state,
                '/state',
                String,
                1
            ],
            [
                self._create_imu_msg,
                '/imu/data',
                Imu,
                1
            ],
            [
                self._create_odometry_msg,
                '/odometry',
                Odometry,
                1
            ],
        ]

        self.sub_topics= [
            [
                self._setpoint_velocity_cmd_vel_callback,
                '/setpoint_velocity/cmd_vel',
                TwistStamped,
                1
            ],
            [
                self._set_servo_callback,
                '/servo{}/set_pwm',
                UInt16,
                1,
                [1, 2, 3, 4, 5, 6, 7, 8]
            ],
            [
                self._set_rc_channel_callback,
                '/rc_channel{}/set_pwm',
                UInt16,
                1,
                [1, 2, 3, 4, 5, 6, 7, 8]
            ],
            [
                self._set_mode_callback,
                '/mode/set',
                String,
                1
            ],
            [
                self._arm_callback,
                '/arm',
                Bool,
                1
            ],
        ]

        self.mavlink_msg_available = {}

        for _, topic, msg, queue in self.pub_topics:
            self.mavlink_msg_available[topic] = 0
            self._pub_subscribe_topic(topic, msg, queue)

        for topic in self.sub_topics:
            if len(topic) <= 4:
                callback, topic_name, msg, queue = topic
                self._sub_subscribe_topic(topic_name, msg, queue, callback)
            else:
                callback, topic_name, msg, queue, arg = topic
                for name in arg:
                    self._sub_subscribe_topic(topic_name.format(name), msg, queue, callback)

    @staticmethod
    def _callback_from_topic(topic):
        """ Create callback function name

        Args:
            topic (str): Topic name

        Returns:
            str: callback name
        """
        return topic.replace('/', '_') + '_callback'

    def _pub_subscribe_topic(self, topic, msg, queue_size=1):
        """ Subscribe to a topic using the publisher

        Args:
            topic (str): Topic name
            msg (TYPE): ROS message type
            queue_size (int, optional): Queue size
        """
        self.pub.subscribe_topic(self.ROV_name + topic, msg, queue_size)

    def _sub_subscribe_topic(self, topic, msg, queue_size=1, callback=None):
        """ Subscribe to a topic using the subscriber

        Args:
            topic (str): Topic name
            msg (TYPE): ROS message type
            queue_size (int, optional): Queue size
            callback (None, optional): Callback function
        """
        self.sub.subscribe_topic(self.ROV_name + topic, msg, queue_size, callback)

    def _set_servo_callback(self, msg, topic):
        """ Set servo from topic

        Args:
            msg (TYPE): ROS message
            topic (TYPE): Topic name

        Returns:
            None: Description
        """
        paths = topic.split('/')
        servo_id = None
        for path in paths:
            if 'servo' in path:
                servo_id = int(re.search('[0-9]', path).group(0)) + 1
                # Found valid id !
                break
        else:
            # No valid id
            return

        self.set_servo_pwm(servo_id, msg.data)

    def _set_rc_channel_callback(self, msg, topic):
        """ Set RC channel from topic

        Args:
            msg (TYPE): ROS message
            topic (TYPE): Topic name

        Returns:
            TYPE: Description
        """
        paths = topic.split('/')
        channel_id = None
        for path in paths:
            if 'rc_channel' in path:
                channel_id = int(re.search('[0-9]', path).group(0))  - 1
                # Found valid id !
                break
        else:
            # No valid id
            return

        self.set_rc_channel_pwm(channel_id, msg.data)

    def _set_mode_callback(self, msg, _):
        """ Set ROV mode from topic

        Args:
            msg (TYPE): Topic message
            _ (TYPE): Description
        """
        self.set_mode(msg.data)

    def _arm_callback(self, msg, _):
        """ Set arm state from topic

        Args:
            msg (TYPE): ROS message
            _ (TYPE): Description
        """
        self.arm_throttle(msg.data)

    def _setpoint_velocity_cmd_vel_callback(self, msg, _):
        """ Set angular and linear velocity from topic

        Args:
            msg (TYPE): ROS message
            _ (TYPE): Description
        """
        #http://mavlink.org/messages/common#SET_POSITION_TARGET_GLOBAL_INT
        params = [
            None,
            None,
            None,
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
            None,
            None,
            None,
            None,
            None,
            ]
        self.set_position_target_local_ned(params)

        #http://mavlink.org/messages/common#SET_ATTITUDE_TARGET
        params = [
            None,
            None,
            None,
            None,
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z,
            None,
            ]
        self.set_attitude_target(params)

    def _create_header(self, msg):
        """ Create ROS message header

        Args:
            msg (ROS message): ROS message with header
        """
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.model_base_link

    def _create_odometry_msg(self):
        """ Create odometry message from ROV information

        Raises:
            Exception: No data to create the message
        """

        # Check if data is available
        if 'LOCAL_POSITION_NED' not in self.get_data():
            raise Exception('no LOCAL_POSITION_NED data')

        if 'ATTITUDE' not in self.get_data():
            raise Exception('no ATTITUDE data')

        #TODO: Create class to deal with BlueRov state
        msg = Odometry()

        self._create_header(msg)

        #http://mavlink.org/messages/common#LOCAL_POSITION_NED
        local_position_data = self.get_data()['LOCAL_POSITION_NED']
        xyz_data = [local_position_data[i]  for i in ['x', 'y', 'z']]
        vxyz_data = [local_position_data[i]  for i in ['vx', 'vy', 'z']]
        msg.pose.pose.position.x = xyz_data[0]
        msg.pose.pose.position.y = xyz_data[1]
        msg.pose.pose.position.z = xyz_data[2]
        msg.twist.twist.linear.x = vxyz_data[0]/100
        msg.twist.twist.linear.y = vxyz_data[1]/100
        msg.twist.twist.linear.z = vxyz_data[2]/100

        #http://mavlink.org/messages/common#ATTITUDE
        attitude_data = self.get_data()['ATTITUDE']
        orientation = [attitude_data[i] for i in ['roll', 'pitch', 'yaw']]
        orientation_speed = [attitude_data[i] for i in ['rollspeed', 'pitchspeed', 'yawspeed']]

        #https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_to_Quaternion_Conversion
        cy = math.cos(orientation[2] * 0.5)
        sy = math.sin(orientation[2] * 0.5)
        cr = math.cos(orientation[0] * 0.5)
        sr = math.sin(orientation[0] * 0.5)
        cp = math.cos(orientation[1] * 0.5)
        sp = math.sin(orientation[1] * 0.5)

        msg.pose.pose.orientation.w = cy * cr * cp + sy * sr * sp
        msg.pose.pose.orientation.x = cy * sr * cp - sy * cr * sp
        msg.pose.pose.orientation.y = cy * cr * sp + sy * sr * cp
        msg.pose.pose.orientation.z = sy * cr * cp - cy * sr * sp
        msg.twist.twist.angular.x = orientation_speed[0]
        msg.twist.twist.angular.y = orientation_speed[1]
        msg.twist.twist.angular.z = orientation_speed[2]

        self.pub.set_data('/odometry', msg)

    def _create_imu_msg(self):
        """ Create imu message from ROV data

        Raises:
            Exception: No data available
        """

        # Check if data is available
        if 'ATTITUDE' not in self.get_data():
            raise Exception('no ATTITUDE data')

        #TODO: move all msgs creating to msg
        msg = Imu()

        self._create_header(msg)

        #http://mavlink.org/messages/common#SCALED_IMU
        imu_data = None
        for i in ['', '2', '3']:
            try:
                imu_data = self.get_data()['SCALED_IMU{}'.format(i)]
                break
            except Exception as e:
                pass

        if imu_data is None:
            raise Exception('no SCALED_IMUX data')

        acc_data = [imu_data['{}acc'.format(i)]  for i in ['x', 'y', 'z']]
        gyr_data = [imu_data['{}gyro'.format(i)] for i in ['x', 'y', 'z']]
        mag_data = [imu_data['{}mag'.format(i)]  for i in ['x', 'y', 'z']]

        #http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
        msg.linear_acceleration.x = acc_data[0]/100
        msg.linear_acceleration.y = acc_data[1]/100
        msg.linear_acceleration.z = acc_data[2]/100
        msg.linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        msg.angular_velocity.x = gyr_data[0]/1000
        msg.angular_velocity.y = gyr_data[1]/1000
        msg.angular_velocity.z = gyr_data[2]/1000
        msg.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        #http://mavlink.org/messages/common#ATTITUDE
        attitude_data = self.get_data()['ATTITUDE']
        orientation = [attitude_data[i] for i in ['roll', 'pitch', 'yaw']]

        #https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_to_Quaternion_Conversion
        cy = math.cos(orientation[2] * 0.5)
        sy = math.sin(orientation[2] * 0.5)
        cr = math.cos(orientation[0] * 0.5)
        sr = math.sin(orientation[0] * 0.5)
        cp = math.cos(orientation[1] * 0.5)
        sp = math.sin(orientation[1] * 0.5)

        msg.orientation.w = cy * cr * cp + sy * sr * sp
        msg.orientation.x = cy * sr * cp - sy * cr * sp
        msg.orientation.y = cy * cr * sp + sy * sr * cp
        msg.orientation.z = sy * cr * cp - cy * sr * sp

        msg.orientation_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        self.pub.set_data('/imu/data', msg)

    def _create_battery_msg(self):
        """ Create battery message from ROV data

        Raises:
            Exception: No data available
        """

        # Check if data is available
        if 'SYS_STATUS' not in self.get_data():
            raise Exception('no SYS_STATUS data')

        if 'BATTERY_STATUS' not in self.get_data():
            raise Exception('no BATTERY_STATUS data')

        bat = BatteryState()
        self._create_header(bat)

        #http://docs.ros.org/jade/api/sensor_msgs/html/msg/BatteryState.html
        bat.voltage = self.get_data()['SYS_STATUS']['voltage_battery']/1000
        bat.current = self.get_data()['SYS_STATUS']['current_battery']/100
        bat.percentage = self.get_data()['BATTERY_STATUS']['battery_remaining']/100
        self.pub.set_data('/battery', bat)

    def _create_camera_msg(self):
        if not self.video.frame_available():
            return
        frame = self.video.frame()
        image_msg = Image()
        self._create_header(image_msg)
        height, width, channels = frame.shape
        image_msg.width = width
        image_msg.height = height
        image_msg.encoding = 'bgr8'
        image_msg.data = frame
        msg = self.video_bridge.cv2_to_imgmsg(frame, "bgr8")
        self._create_header(msg)
        msg.step = int(msg.step)
        self.pub.set_data('/camera/image_raw', msg)

    def _create_ROV_state(self):
        """ Create ROV state message from ROV data

        Raises:
            Exception: No data available
        """

        # Check if data is available
        if 'SERVO_OUTPUT_RAW' not in self.get_data():
            raise Exception('no SERVO_OUTPUT_RAW data')

        if 'HEARTBEAT' not in self.get_data():
            raise Exception('no HEARTBEAT data')

        servo_output_raw_msg = self.get_data()['SERVO_OUTPUT_RAW']
        servo_output_raw = [servo_output_raw_msg['servo{}_raw'.format(i+1)] for i in range(8)]
        motor_throttle = [servo_output_raw[i] - 1500 for i in range(6)]
        # 1100 -> -1 and 2000 -> 1
        for throttle in motor_throttle:
            if throttle < 0:
                throttle = throttle/400
            else:
                throttle = throttle/500

        light_on = (servo_output_raw[6] - 1100) / 8
        #need to check
        camera_angle = servo_output_raw[7] - 1500

        # Create angle from pwm
        camera_angle = -45*camera_angle/400

        base_mode = self.get_data()['HEARTBEAT']['base_mode']
        custom_mode = self.get_data()['HEARTBEAT']['custom_mode']

        mode, arm = self.decode_mode(base_mode, custom_mode)

        state = {
            'motor': motor_throttle,
            'light': light_on,
            'camera_angle': camera_angle,
            'mode': mode,
            'arm': arm
        }

        string = String()
        string.data = str(json.dumps(state, ensure_ascii=False))

        self.pub.set_data('/state', string)

    def publish(self):
        """ Publish the data in ROS topics
        """
        self.update()
        for sender, topic, _, _ in self.pub_topics:
            try:
                if time.time() - self.mavlink_msg_available[topic] > 1:
                    sender()
            except Exception as e:
                self.mavlink_msg_available[topic] = time.time()
                print(e)

if __name__ == '__main__':
    try:
        rospy.init_node('user_node', log_level=rospy.DEBUG)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)

    bluerov = BlueRov(device='udp:localhost:14550')

    while not rospy.is_shutdown():
        bluerov.publish()
