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

if __name__ == '__main__':
    bridge = Bridge()
    #bridge = Bridge(device='udp:localhost:14550')
    while True:
        bridge.update()
        bridge.print()