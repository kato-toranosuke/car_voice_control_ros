#!/usr/bin/env python3
# coding: utf-8

import rospy
import os


def kill():
    os.system("killall julius")


os.chdir(os.path.dirname(__file__) + "/../etc")
rospy.init_node("julius")
rospy.on_shutdown(kill)
os.system("julius -C command.jconf")

rospy.spin()
