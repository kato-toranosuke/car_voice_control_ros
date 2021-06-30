#!/usr/bin/env python3
# encoding: utf-8

import rospy
import os
import socket


class JuliusReceiver:
    def __init__(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.connect(("localhost", 10500))
                break
            except:
                rate.sleep()

        rospy.on_shutdown(self.sock.close)

        print("JuliusReceiver is initialized.\n")

    def get_line(self):
        line = ""
        while not rospy.is_shutdown():
            v = self.sock.recv(1).decode(encoding='UTF-8', errors='ignore')
            # print(v)
            if v == '\n':
                return line
            line += v


if __name__ == '__main__':
    rospy.init_node("voice_to_command")
    julius = JuliusReceiver()
    while not rospy.is_shutdown():
        print(julius.get_line())
