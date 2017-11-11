#!/usr/bin/env python

import json
import rospy
import threading
import yaml

import diagnostic_msgs.msg
import geometry_msgs.msg
import mavros_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg
import std_msgs.msg


class Subs(object):
    def __init__(self):
        # Dict with all data
        self.data = {}
        # Get data from topic list
        self.topics = [
            # Sensors
            ['/mavros/battery',
                sensor_msgs.msg.BatteryState,
                self.callback
             ],
            ['/mavros/global_position/global',
                sensor_msgs.msg.NavSatFix,
                self.callback
             ],
            ['/mavros/global_position/raw/fix',
                sensor_msgs.msg.NavSatFix,
                self.callback
             ],
            ['/mavros/imu/atm_pressure',
                sensor_msgs.msg.FluidPressure,
                self.callback
             ],
            ['/mavros/imu/data',
                sensor_msgs.msg.Imu,
                self.callback
             ],
            ['/mavros/imu/data_raw',
                sensor_msgs.msg.Imu,
                self.callback
             ],
            ['/mavros/imu/mag',
                sensor_msgs.msg.MagneticField,
                self.callback
             ],
            ['/mavros/imu/temperature',
                sensor_msgs.msg.Temperature,
                self.callback
             ],
            ['/mavros/time_reference',
                sensor_msgs.msg.TimeReference,
                self.callback
             ],
            # Generic
            ['/mavlink/from',
                mavros_msgs.msg.Mavlink,
                self.callback
             ],
            ['/mavros/altitude',
                mavros_msgs.msg.Altitude,
                self.callback
             ],
            ['/mavros/extended_state',
                mavros_msgs.msg.ExtendedState,
                self.callback
             ],
            ['/mavros/global_position/compass_hdg',
                std_msgs.msg.Float64,
                self.callback
             ],
            ['/mavros/global_position/rel_alt',
                std_msgs.msg.Float64,
                self.callback
             ],
            ['/mavros/hil_actuator_controls',
                mavros_msgs.msg.HilActuatorControls,
                self.callback
             ],
            ['/mavros/manual_control/control',
                mavros_msgs.msg.ManualControl,
                self.callback
             ],
            ['/mavros/mission/waypoints',
                mavros_msgs.msg.WaypointList,
                self.callback
             ],
            ['/mavros/radio_status',
                mavros_msgs.msg.RadioStatus,
                self.callback
             ],
            ['/mavros/rc/in',
                mavros_msgs.msg.RCIn,
                self.callback
             ],
            ['/mavros/rc/out',
                mavros_msgs.msg.RCOut,
                self.callback
             ],
            ['/mavros/setpoint_raw/target_attitude',
                mavros_msgs.msg.AttitudeTarget,
                self.callback
             ],
            ['/mavros/setpoint_raw/target_global',
                mavros_msgs.msg.GlobalPositionTarget,
                self.callback
             ],
            ['/mavros/setpoint_raw/target_local',
                mavros_msgs.msg.PositionTarget,
                self.callback
             ],
            ['/mavros/state',
                mavros_msgs.msg.State,
                self.callback
             ],
            # Position
            ['/mavros/local_position/pose',
                geometry_msgs.msg.PoseStamped,
                self.callback
             ],
            ['/mavros/wind_estimation',
                geometry_msgs.msg.TwistStamped,
                self.callback
             ],
            ['/mavros/local_position/velocity',
                geometry_msgs.msg.TwistStamped,
                self.callback
             ],
            ['/mavros/global_position/local',
                nav_msgs.msg.Odometry,
                self.callback
             ],
            ['/mavros/global_position/odom',
                nav_msgs.msg.Odometry,
                self.callback
             ],
            ['/mavros/global_position/raw/gps_vel',
                geometry_msgs.msg.TwistStamped,
                self.callback
             ],
            # Diagnostics
            ['/diagnostics',
                diagnostic_msgs.msg.DiagnosticArray,
                self.callback
             ],
            ['/joy',
                sensor_msgs.msg.Joy,
                self.callback
             ]
        ]

    def get_data(self):
        return self.data

    def set_data(self, path, value={}):
        # The first item will be empty
        keys = path.split('/')[1:]
        current_level = self.data
        for part in keys:
            # If dict don't have the path, create it !
            if part not in current_level:
                current_level[part] = {}
            current_level = current_level[part]
        if value is not {}:
            current_level.update(yaml.load(str(value)))

    def subscribe_topics(self, init=False):
        for topic, msg_type, callback in self.topics:
            self.set_data(topic)
            rospy.Subscriber(topic, msg_type, callback, callback_args=topic)

        try:
            if init:
                rospy.init_node('get_mav_data')
            thread = threading.Thread(target=lambda: rospy.spin())
            thread.start()
        except rospy.ROSInterruptException:
            print(e)

    def callback(self, data, topic):
        self.set_data(topic, data)

    def print_data(self):
        print(self.data['mavros']['battery']['voltage'])


if __name__ == '__main__':
    sub = Subs()
    sub.subscribe_topics(True)

    def print_voltage():
        try:
            rospy.loginfo(sub.get_data()['mavros']['battery']['voltage'])
        except Exception as error:
            print(error)

        thread = threading.Timer(1.0, print_voltage)
        thread.daemon = True
        thread.start()
    print_voltage()
