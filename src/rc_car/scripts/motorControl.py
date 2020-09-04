#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 22 09:13:00 2019
@author: marvin
    motorpins = {"MOTOR4":{"config":{1:{"e":32,"f":24,"r":26},2:{"e":32,"f":26,"r":24}},"arrow":1},
                 "MOTOR3":{"config":{1:{"e":19,"f":21,"r":23},2:{"e":19,"f":23,"r":21}}, "arrow":2},
                 "MOTOR2":{"config":{1:{"e":22,"f":16,"r":18},2:{"e":22,"f":18,"r":16}}, "arrow":3},
                 "MOTOR1":{"config":{1:{"e":11,"f":15,"r":13},2:{"e":11,"f":13,"r":15}},"arrow":4}}
"""

import rospy
import RPi.GPIO as GPIO
from rc_car_msgs.msg import MotorCommand

class MotorControlNode:
    
    MOTOR_TOPIC = "motor_message"
    
    def __init__(self):
        self._mcu = MotorControlUnit()
        self._motorSub = rospy.Subscriber(self.MOTOR_TOPIC, MotorCommand,
                                          self.receiveMotorMessage,
                                          queue_size=3)
        rospy.init_node('MotorNode', anonymous=False)
        
    def receiveMotorMessage(self, msg):
        self._mcu.setSpeed(msg.speedMotorA, msg.speedMotorB)
        
    def startNode(self):
        rospy.spin()
    
    def cleanUp(self):
        self._mcu.cleanUp()
    
class MotorControlUnit:
    
    
    def __init__(self):
        self._MotorA = Motor(24,32,26)
        self._MotorB = Motor(23,19,21)
        
    def setSpeed(self, speedA, speedB):
        self._MotorA.setDirectionNdDutyCycle(speedA)
        self._MotorB.setDirectionNdDutyCycle(speedB)

    def cleanUp(self):
        self._MotorA.stopPWM()
        self._MotorB.stopPWM()
        GPIO.cleanup()        
    
    
class Motor:
    
    def __init__(self, pinPWM, pinIN1, pinIN2, pwmFreq=150,
                 gpioMode=GPIO.BOARD):
        self._pinPWM = pinPWM
        self._pinIN1 = pinIN1
        self._pinIN2 = pinIN2
        self._pwmFrequency = pwmFreq
        self._gpioMode = gpioMode
        self._pwm = None
        self._initializeGPIO()
        
    def _initializeGPIO(self):
        GPIO.setmode(self._gpioMode)
        GPIO.setup(self._pinPWM, GPIO.OUT)
        GPIO.setup(self._pinIN1, GPIO.OUT)
        GPIO.setup(self._pinIN2, GPIO.OUT)
        # Initalizierung des PWM Objects
        self._pwm = GPIO.PWM(self._pinPWM, self._pwmFrequency)
        self._pwm.start(0)
        
        
    def setDirectionNdDutyCycle(self, numb):
        abs_numb = abs(numb)
        if abs_numb<=100:
            if numb >=0:
                # Forward
                GPIO.output(self._pinIN1, GPIO.HIGH)
                GPIO.output(self._pinIN2, GPIO.LOW)
            else:
                # Backward
                GPIO.output(self._pinIN1, GPIO.LOW)
                GPIO.output(self._pinIN2, GPIO.HIGH)
            if abs_numb >10:
                self._pwm.ChangeDutyCycle(abs_numb)
            else:
                self._pwm.ChangeDutyCycle(0)
        else:
            raise ValueError("The input needs to be in the range +-100")
            
    def stopPWM(self):
        self._pwm.stop()
        
    
if __name__ == "__main__":
    try:
        motorControl = MotorControlNode()
        motorControl.startNode()
    except:
        print ("Program closed ...")
motorControl.cleanUp()
