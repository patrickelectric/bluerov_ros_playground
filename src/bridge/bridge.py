#!/usr/bin/env python

from pymavlink import mavutil

class Bridge():
    def __init__(self, device='udp:192.168.2.1:14550', baudrate=None):
        self.conn = mavutil.mavlink_connection(device, baud=baudrate)
        self.data = {}

    def get_data(self):
        return self.data

    def get_all_msgs(self):
        msgs = []
        while True:
            msg = self.conn.recv_match()
            if msg != None:
                msgs.append(msg)
            else:
                break
        return msgs

    def update(self):
        # Get all messages
        msgs = self.get_all_msgs()
        # Update dict
        for msg in msgs:
            self.data[msg.get_type()] = msg.to_dict()

    def print(self):
        print(self.data)

    def set_mode(self, mode):
        mode = mode.upper()
        if mode not in self.conn.mode_mapping():
            print('Unknown mode : {}'.format(mode))
            print('Try:', list(self.conn.mode_mapping().keys()))
            return
        mode_id = self.conn.mode_mapping()[mode]
        self.conn.set_mode(mode_id)

    def set_guided_mode(self):
        #https://github.com/ArduPilot/pymavlink/pull/128
        params = [mavutil.mavlink.MAV_MODE_GUIDED, 0, 0, 0, 0, 0, 0]
        self.send_command_long(mavutil.mavlink.MAV_CMD_DO_SET_MODE, params)

    def send_command_long(self, command, params=[0, 0, 0, 0, 0, 0, 0], confirmation=0):
        self.conn.mav.command_long_send(
            self.conn.target_system,                # target system
            self.conn.target_component,             # target component
            command,                                # mavlink command
            confirmation,                           # confirmation
            params[0],                              # params
            params[1],
            params[2],
            params[3],
            params[4],
            params[5],
            params[6]
        )

    def set_position_target_local_ned(self, param=[]):
        if len(param) != 11:
            print('SET_POISITION_TARGET_GLOBAL_INT need 11 params')
        mask = 0
        for i, value in enumerate(param):
            if value is not None:
                mask += 1<<i
            else:
                param[i] = 0.0

        #http://mavlink.org/messages/common#SET_POSITION_TARGET_GLOBAL_INT
        self.conn.mav.set_position_target_local_ned_send(
            0,                                              # system time in milliseconds
            self.conn.target_system,                        # target system
            0,                                              # target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,             # frame
            mask,                                           # mask
            param[0], param[1], param[2],                   # position x,y,z
            param[3], param[4], param[5],                   # velocity x,y,z
            param[6], param[7], param[8],                   # accel x,y,z
            param[9], param[10])                            # yaw, yaw rate

    def set_attitude_target(self, param=[]):
        if len(param) != 8:
            print('SET_ATTITUDE_TARGET need 8 params')
        mask = 0
        for i, value in enumerate(param[4:-1]):
            if value is not None:
                mask += 1<<i
            else:
                param[i+3] = 0.0

        if param[7] is not None:
            mask += 1<<6
        else:
            param[7] = 0.0

        q = param[:4]

        if q != [None, None, None, None]:
            mask += 1<<7
        else:
            q = [1.0, 0.0, 0.0, 0.0]

        self.conn.mav.set_attitude_target_send(0,   # system time in milliseconds
            self.conn.target_system,                # target system
            self.conn.target_component,             # target component
            mask,                                   # mask
            q,                                      # quaternion attitude
            param[4],                               # body roll rate
            param[5],                               # body pitch rate
            param[6],                               # body yaw rate
            param[7])                               # thrust

    def set_servo_pwm(self, id, pwm=1500):
        #http://mavlink.org/messages/common#MAV_CMD_DO_SET_SERVO
        self.conn.mav.command_long_send(
            self.conn.target_system,                # target_system
            self.conn.target_component,             # target_component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,   # command
            0,                                      # confirmation
            id,                                     # servo id
            pwm,                                    # pwm 1000-2000
            0, 0, 0, 0, 0)                          # empty

    def set_rc_channel_pwm(self, id, pwm=1100):
        rc_channel_values = [65535 for _ in range(8)]
        rc_channel_values[id] = pwm
        #http://mavlink.org/messages/common#RC_CHANNELS_OVERRIDE
        self.conn.mav.rc_channels_override_send(
            self.conn.target_system,                # target_system
            self.conn.target_component,             # target_component
            *rc_channel_values)                     # RC channel list, in microseconds.

    def arm_throttle(self, arm_throttle):
        if arm_throttle:
            self.conn.arducopter_arm()
        else:
            self.conn.mav.command_long_send(
                self.conn.target_system,                        # target_system
                self.conn.target_component,                     # target_component
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,   # command
                0,                                              # confirmation
                0,                                              # param1 (0 to indicate disarm)
                0, 0, 0, 0, 0, 0)                               # Reserved (all remaining params)

if __name__ == '__main__':
    bridge = Bridge()
    #bridge = Bridge(device='udp:localhost:14550')
    while True:
        bridge.update()
        bridge.print()