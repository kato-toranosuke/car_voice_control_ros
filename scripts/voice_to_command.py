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

    def get_command(self, th):
        line = self.get_line()

        if "WHYPO" not in line:
            return None

        score_str = line.split('CM="')[-1].split('"')[0]
        if float(score_str) < th:
            return None

        command = None
        if "zenshin" in line:
            command = 'f'
        elif "koutai" in line:
            command = 'b'
        elif "usetsu" in line:
            command = 'r'
        elif "sasetsu" in line:
            command = 'l'

        return command


if __name__ == '__main__':
    rospy.init_node("voice_to_command")
    julius = JuliusReceiver()
    while not rospy.is_shutdown():
        com = julius.get_command(0.8)
        if com != None:
            print(com)
