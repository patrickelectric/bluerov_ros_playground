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

    def set_guided_mode(self):
        #https://github.com/ArduPilot/pymavlink/pull/128
        params = [mavlink.MAV_MODE_GUIDED, 0, 0, 0, 0, 0, 0]
        self.send_command_long(mavlink.MAV_CMD_DO_SET_MODE, params)

    def send_command_long(self, command, params=[0, 0, 0, 0, 0, 0, 0], confirmation=0):
        self.conn.mav.command_long_send(self.conn.target_system, self.conn.target_component,
            #command, confirmation
            command, confirmation,
            #params
            params[0],
            params[1],
            params[2],
            params[3],
            params[4],
            params[5],
            params[6],
            params[7]
        )

    def set_position_target_local_ned(self, param=[]):
        print(param)
        if len(param) != 11:
            print('SET_POISITION_TARGET_GLOBAL_INT need 11 params')
        mask = 0
        print(mask, param)
        for i, value in enumerate(param):
            if value is not None:
                mask += 1<<i
            else:
                param[i] = 0.0
        print(mask, param)

        #http://mavlink.org/messages/common#SET_POSITION_TARGET_GLOBAL_INT
        self.conn.mav.set_position_target_local_ned_send(0, # system time in milliseconds
            self.conn.target_system,                        # target system
            0,                                              # target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,             # frame
            mask,                                           # mask
            param[0], param[1], param[2],                   # position x,y,z
            param[3], param[4], param[5],                   # velocity x,y,z
            param[6], param[7], param[8],                   # accel x,y,z
            param[9], param[10])                            # yaw, yaw rate

if __name__ == '__main__':
    bridge = Bridge()
    #bridge = Bridge(device='udp:localhost:14550')
    while True:
        bridge.update()
        bridge.print()