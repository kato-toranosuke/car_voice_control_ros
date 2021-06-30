#!/usr/bin/python3
# coding: utf-8

import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import String

#######################
### Initial Setting ###
#######################

# GPIOではないので注意！
PIN_IN1_R = 12
PIN_IN2_R = 16
PIN_PWM_R = 7
PIN_IN1_L = 32
PIN_IN2_L = 36
PIN_PWM_L = 40
PIN_STBY = 13

# デフォルトのモータの駆動時間
DEF_DRIVE_TIME = 2


###################################
### functions for motor driving ###
###################################

class Motor:
    def __init__(self, in1, in2, pwm, stby=PIN_STBY):
        self.in1 = in1
        self.in2 = in2
        self.pwm = pwm
        self.stby = stby

    def pinSetup(self):
        # GPIO番号指定モードの設定
        GPIO.setmode(GPIO.BOARD)
        # GPIO出力設定
        GPIO.setup(self.in1, GPIO.OUT)
        GPIO.setup(self.in2, GPIO.OUT)
        GPIO.setup(self.pwm, GPIO.OUT)
        GPIO.setup(self.stby, GPIO.OUT)

        # pwm, stby pinの設定
        GPIO.output(self.pwm, 1)
        GPIO.output(self.stby, 1)

    # rotation
    def rotate(self):
        GPIO.output(self.in1, 1)
        GPIO.output(self.in2, 0)
        print("rotate")

    # inverse rotation
    def invRotate(self):
        GPIO.output(self.in1, 0)
        GPIO.output(self.in2, 1)
        print("inv rotate")

    # break
    def breaking(self):
        GPIO.output(self.in1, 1)
        GPIO.output(self.in2, 1)
        print("break")

    # stop
    def stop(self):
        GPIO.output(self.in1, 0)
        GPIO.output(self.in2, 0)
        print("stop")


class Car:

    def __init__(self, mr, ml):
        self.motorR = mr
        self.motorL = ml
        self.pinSetup()

    def pinSetup(self):
        self.motorR.pinSetup()
        self.motorL.pinSetup()

    def goForward(self, t=DEF_DRIVE_TIME):
        self.motorR.invRotate()
        self.motorL.invRotate()

        time.sleep(t)

        # self.motorR.breaking()
        # self.motorL.breaking()
        self.motorR.stop()
        self.motorL.stop()

    def goBack(self, t=DEF_DRIVE_TIME):
        self.motorR.rotate()
        self.motorL.rotate()

        time.sleep(t)

        self.motorR.stop()
        self.motorL.stop()
        # self.motorR.breaking()
        # self.motorL.breaking()

    def rotateRight(self, t=DEF_DRIVE_TIME):
        self.motorR.rotate()
        self.motorL.invRotate()

        time.sleep(0.25)

        self.motorR.stop()
        self.motorL.stop()
        # self.motorR.breaking()
        # self.motorL.breaking()

    def rotateLeft(self, t=DEF_DRIVE_TIME):
        self.motorR.invRotate()
        self.motorL.rotate()

        time.sleep(0.25)

        self.motorR.stop()
        self.motorL.stop()
        # self.motorR.breaking()
        # self.motorL.breaking()


class MotorDriver:
    def __init__(self, car):
        self.car = car
        self.sub = rospy.Subscriber('voice_command', String, self.recvCom)

    def recvCom(self, data):
        com = data.data
        rospy.loginfo(com)

        if com == 'f':
            self.car.goForward()
        elif com == 'b':
            self.car.goBack()
        elif com == 'r':
            self.car.rotateRight()
        elif com == 'l':
            self.car.rotateLeft()


if __name__ == "__main__":
    rospy.init_node("motor_control")

    motorR = Motor(PIN_IN1_R, PIN_IN2_R, PIN_PWM_R, PIN_STBY)
    motorL = Motor(PIN_IN1_L, PIN_IN2_L, PIN_PWM_L, PIN_STBY)

    car = Car(motorR, motorL)

    motor_driver = MotorDriver(car)

    rospy.spin()
