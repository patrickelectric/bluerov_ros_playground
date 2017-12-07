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

if __name__ == '__main__':
    bridge = Bridge()
    #bridge = Bridge(device='udp:localhost:14550')
    while True:
        bridge.update()
        bridge.print()