#!/usr/bin/env python

from pymavlink import mavutil
from bridge import Bridge
import json
import rospy
import sys

from cv_bridge import CvBridge

sys.path.append("../bluerov")
from pubs import Pubs
from video import Video

# msgs type
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import Image
from std_msgs.msg import String

class BlueRov(Bridge):
    def __init__(self, device='udp:192.168.2.1:14550', baudrate=None):
        super().__init__(device, baudrate)
        self.pub = Pubs()
        self.ROV_name = 'BlueRov2'
        self.model_base_link = '/base_link'

        self.video = Video()
        self.video_bridge = CvBridge()

        self.topics = [
            [
                self._create_camera_msg,
                '/battery',
                BatteryState,
                1
            ],
            [
                self._create_battery_msg,
                '/camera',
                Image,
                1
            ],
            [
                self._create_ROV_state,
                '/state',
                String,
                1
            ],
        ]

        for _, topic, msg, queue in self.topics:
            self._subscribe_topic(topic, msg, queue)

    def _subscribe_topic(self, topic, msg, queue_size=1):
        self.pub.subscribe_topic(self.ROV_name + topic, msg, queue_size)

    def _create_header(self, msg):
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.model_base_link

    def _create_battery_msg(self):
            '''
            Header  header
            float32 voltage                 # Voltage in Volts (Mandatory)
            float32 current                 # Negative when discharging (A)  (If unmeasured NaN)
            float32 charge                  # Current charge in Ah  (If unmeasured NaN)
            float32 capacity                # Capacity in Ah (last full capacity)  (If unmeasured NaN)
            float32 design_capacity         # Capacity in Ah (design capacity)  (If unmeasured NaN)
            float32 percentage              # Charge percentage on 0 to 1 range  (If unmeasured NaN)
            uint8   power_supply_status     # The charging status as reported. Values defined above
            uint8   power_supply_health     # The battery health metric. Values defined above
            uint8   power_supply_technology # The battery chemistry. Values defined above
            bool    present                 # True if the battery is present

            float32[] cell_voltage          # An array of individual cell voltages for each cell in the pack
                                            # If individual voltages unknown but number of cells known set each to NaN
            string    location              # The location into which the battery is inserted. (slot number or plug)
            string    serial_number         # The best approximation of the battery serial number
            '''
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
        print('new frame')
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
        self.pub.set_data('/camera', msg)


    def _create_ROV_state(self):
        servo_output_raw_msg = self.get_data()['SERVO_OUTPUT_RAW']
        servo_output_raw = [servo_output_raw_msg['servo{}_raw'.format(i+1)] for i in range(8)]
        motor_throttle = [servo_output_raw[i] - 1500 for i in range(6)]
        # 1100 -> -1 and 2000 -> 1
        for throttle in motor_throttle:
            if throttle < 0:
                throttle = throttle/400
            else:
                throttle = throttle/500

        light_on = servo_output_raw[6] > 1500
        #need to check
        camera_angle = servo_output_raw[7] - 1500

        if camera_angle < 0:
            camera_angle = 45*camera_angle/400
        else:
            camera_angle = 45*camera_angle/500

        state = {
            'motor': motor_throttle,
            'light': light_on,
            'camera_angle': camera_angle
        }

        string = String()
        print(string)

        self.pub.set_data('/state', str(json.dumps(state, ensure_ascii=False)))


    def publish(self):
        self.update()
        for sender, _, _, _ in self.topics:
            try:
                sender()
            except Exception as e:
                print(e)

if __name__ == '__main__':
    try:
        rospy.init_node('user_node', log_level=rospy.DEBUG)
    except rospy.ROSInterruptException as error:
        print('pubs error with ROS: ', error)
        exit(1)

    #bridge = Bridge()
    #bluerov = BlueRov(device='udp:localhost:14550')
    bluerov = BlueRov()
    while not rospy.is_shutdown():
        bluerov.publish()




'''
from pymavlink import mavutil

# create a mavlink serial instance
master = mavutil.mavlink_connection('udp:192.168.2.1:14550')
#master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

#wait_heartbeat(master)

#print("Sending all message types")

master.recv_match(type='HEARTBEAT', blocking=True)
print("Heartbeat from APM (system %u component %u)" % (master.target_system, master.target_system))

#msg = master.recv_match(type='RC_CHANNELS_RAW', blocking=True)
#print(msg)
master.set_mode_manual()
while True:
    m = master.recv_match()
    if m is not None:
        print(m)
        print(m.get_type())
'''
