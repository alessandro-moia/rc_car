#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
This node file contains to classes: 
 - 
 
 
@author: Marvin Kiefer
"""


# -*- coding: utf-8 -*-
"""
Created on Wed Nov 20 09:46:51 2019
@author: marvin
"""



from evdev import InputDevice, list_devices
import rospy
import numpy as np
from rc_car_msgs.msg import MotorCommand, ServoCommand

class RemoteControlNode:
    
    MOTOR_TOPIC = "motor_message"
    SERVO_TOPIC = "servo_message"
    
    def __init__(self, refreshRate=15):
        self._ps4 = PS4Interpreter()
        self._refreshRate = refreshRate
        self._lastMotorSpeed = [None, None]
        self._lastServoAngle = None
        self._motorPub = rospy.Publisher(self.MOTOR_TOPIC, MotorCommand,
                                         queue_size=3)
        self._servoPub = rospy.Publisher(self.SERVO_TOPIC, ServoCommand,
                                         queue_size=3)
        rospy.init_node('DS4Node', anonymous=False)
        
    def startNode(self):
        rate = rospy.Rate(self._refreshRate)
        while not rospy.is_shutdown():
            self._ps4.refreshInput()
            msg_Motor = self._convertLeftStickInput2Msg()
            msg_Servo = self._convertRightStickInput2Msg()
            if (self._newMotorCommand(msg_Motor)):
                self._lastMotorSpeed[0] = msg_Motor.speedMotorA
                self._lastMotorSpeed[1] = msg_Motor.speedMotorB
                self._motorPub.publish(msg_Motor) 
            if (self._newServoCommand(msg_Servo)):
                self._lastServoAngle = msg_Servo.servoAngle
                self._servoPub.publish(msg_Servo)
            rate.sleep()
            
    
    def _convertLeftStickInput2Msg(self):
        xy_value = self._ps4.analog_LeftStick
        maxVal = self._ps4.getResolutionSteps()**2
        # Geschwindigkeitsberechnung:
        # - Auslenkung in Y-Richtung: Interpretiert als V_Gerade 
        # - Auslenkung in X-Richtung: Interpretiert als einseitige 
#                                     Abweichung von V_Gerade
        # Normieren auf 100% - Quadratische Funktion
        v_linear = 100./maxVal*np.sign(xy_value["Y"])*xy_value["Y"]**2
        v_turn = 100./maxVal*np.sign(xy_value["X"])*xy_value["X"]**2
        motorA = v_linear - v_turn/2.0
        motorB = v_linear + v_turn/2.0
        # MAX TURN  -100 vs +100
        if abs(v_turn)-3 >= 97 and abs(v_linear)<=3:
            motorA = -100*np.sign(v_turn)
            motorB = 100*np.sign(v_turn)
        # Limiting 
        if np.abs(motorA) > 100:
            motorA = np.sign(motorA)*100
        if np.abs(motorB)  > 100:
            motorB = np.sign(motorB)*100
        msg = MotorCommand()
        msg.speedMotorA = round(motorA)
        msg.speedMotorB = round(motorB)
        return msg
        
    def _convertRightStickInput2Msg(self):
        xy_value = self._ps4.analog_RightStick
        maxVal = self._ps4.getResolutionSteps()
        # Stick muss sich am äußeren oberen Anschlag befinden
        if( xy_value["Y"] >= 0 
        and np.linalg.norm([xy_value["X"],xy_value["Y"]])>=maxVal):
            circle_y = np.sqrt(maxVal**2 - xy_value["X"]**2)
            angle = (np.arctan2(circle_y, xy_value["X"])*(180./np.pi))
            angle -=90
            msg = ServoCommand()
            msg.servoAngle= round(angle)
            return msg
        elif np.linalg.norm([xy_value["X"],xy_value["Y"]])<maxVal/2:
            return ServoCommand(0)
        else:
            return None
                
        
    def _newMotorCommand(self, msg):
        if (self._lastMotorSpeed[0] is None or 
            self._lastMotorSpeed[1] is None):
                return True
        elif self._lastMotorSpeed[0]-msg.speedMotorA !=0:
                return True
        elif self._lastMotorSpeed[1]-msg.speedMotorB !=0:
                return True
        return False
            
    def _newServoCommand(self, msg):
        if msg is None:
            return False
        elif self._lastServoAngle is None:
            return True
        elif (self._lastServoAngle - msg.servoAngle) != 0:
            return True
        return False
                                   
class PS4Interpreter:

    ANALOG_STICK_RESOLUTION = 256 # [0 255]
    TRIGGER_RESOLUTION = 256  # [0 255]
    
    
    def __init__(self, device_dir ="/dev/input" ):
        self._ds4= self._findDS4Controller(device_dir)
        self._stepNumber = 16
        self._halfStickResolution = float(self.ANALOG_STICK_RESOLUTION/2)
        self._keyMapping = {}
        self._categoryMapping = {}
        # Buttons
        self.analog_LeftStick = {"X":0, "Y":0} # Orientierung - 4Quadranten schema
        self.analog_RightStick = {"X":0, "Y":0} # Orientierung - 4Quadranten schema
        self.analog_TriggerL2 = 0 #Left Trigger Negativ Right positiv
        self.analog_TriggerR2 = 0 #Left Trigger Negativ Right positiv
        self.btn_ArrowUp = 0
        self.btn_ArrowDown = 0
        self.btn_ArrowLeft = 0
        self.btn_ArrowRight = 0
        self.btn_South = None #pressed 1 - not pressed 0
        self.btn_North = None #pressed 1 - not pressed 0
        self.btn_East = None #pressed 1 - not pressed 0
        self.btn_West = None #pressed 1 - not pressed 0
        self.btn_L1 = None #pressed 1 - not pressed 0
        self.btn_R1 = None #pressed 1 - not pressed 0
        self.btn_L2 = None #pressed 1 - not pressed 0
        self.btn_R2 = None #pressed 1 - not pressed 0
        self.btn_Options  = None #pressed 1 - not pressed 0
        self.btn_Share  = None #pressed 1 - not pressed 0
        self.btn_PS4 = None # pressed 1 - not pressed 0
#        self._printDeviceConfig()
        self._mapDeviceButtons()
        if self._ds4 is not None:
            self.refreshInput()
        else:
            raise IOError("DS4 Controller is not connected")
            
        
    def refreshInput(self):
        try:
            for event in self._ds4.read():
#            for event in self._ds4.read_loop():
                if event.type == self._categoryMapping["DISCRETE"]:
                    if event.code == self._keyMapping["SOUTH_BTN_D"]:
                        self.btn_South = event.value
                    elif event.code == self._keyMapping["EAST_BTN_D"]:
                        self.btn_East = event.value
                    elif event.code == self._keyMapping["NORTH_BTN_D"]:
                        self.btn_North = event.value
                    elif event.code == self._keyMapping["WEST_BTN_D"]:
                        self.btn_West = event.value
                    elif event.code == self._keyMapping["OPTIONS_BTN_D"]:
                        self.btn_Options = event.value
                    elif event.code == self._keyMapping["SHARE_BTN_D"]:
                        self.btn_Share = event.value
                    elif event.code == self._keyMapping["R1_BTN_D"]:
                        self.btn_R1 = event.value
                    elif event.code == self._keyMapping["L1_BTN_D"]:
                        self.btn_L1 = event.value
                    elif event.code == self._keyMapping["R2_BTN_D"]:
                        self.btn_R2 = event.value
                    elif event.code == self._keyMapping["L2_BTN_D"]:
                        self.btn_L2 = event.value
                    elif event.code == self._keyMapping["PS4_BTN_D"]:
                        self.btn_PS4 = event.value
                elif event.type == self._categoryMapping["ABSOLUTE"]:                    
                    if (event.code == self._keyMapping["LS_X_ABS"] or
                        event.code == self._keyMapping["LS_Y_ABS"]):
                        self._processLeftStickEvent(event)
                    elif (event.code ==self._keyMapping["RS_X_ABS"] or 
                          event.code == self._keyMapping["RS_Y_ABS"]):
                        self._processRightStickEvent(event)
                    elif (event.code == self._keyMapping["LT_ABS"]):
                        self.analog_TriggerL2 = event.value
                    elif (event.code == self._keyMapping["RT_ABS"]):
                        self.analog_TriggerR2 = event.value
                    elif (event.code == self._keyMapping["ARROW_LR_ABS"]):
                        if event.value == 0:
                            self.btn_ArrowLeft = 0
                            self.btn_ArrowRight = 0
                        elif event.value == -1:
                            self.btn_ArrowLeft = 1
                            self.btn_ArrowRight = 0                        
                        else:
                            self.btn_ArrowLeft = 0
                            self.btn_ArrowRight = 1
                    elif (event.code == self._keyMapping["ARROW_UD_ABS"]):
                        if event.value == 0:
                            self.btn_ArrowUp = 0
                            self.btn_ArrowDown = 0
                        elif event.value == -1:
                            self.btn_ArrowUp = 1
                            self.btn_ArrowDown = 0
                        else:
                            self.btn_ArrowUp = 0
                            self.btn_ArrowDown = 1

#                self._printBtnValues()
            return True
        except Exception as e:
            return False
            
    def getResolutionSteps(self):
        return self._stepNumber

                
    def _processArrowKeys(self, event):
        value = event.value 
        if event.code == self._keyMapping["ARROW_UD_ABS"]:
            self.btn_ArrowUpDown = value*-1
        else:
            self.btn_ArrowLeftRight = value      


                
    def _processLeftStickEvent(self, event):
        value = event.value
        stepSize = (self._halfStickResolution/self._stepNumber)
        if event.code == self._keyMapping["LS_X_ABS"]:
            self.analog_LeftStick["X"] = round((value-self._halfStickResolution)/stepSize)
        else:
            self.analog_LeftStick["Y"] = round((value-self._halfStickResolution)/stepSize)*-1
            
            
    def _processRightStickEvent(self, event):
        value = event.value
        stepSize = (self._halfStickResolution/self._stepNumber)
        if event.code == self._keyMapping["RS_X_ABS"]:
            self.analog_RightStick["X"] = round((value-self._halfStickResolution)/stepSize)
        else:
            self.analog_RightStick["Y"] = round((value-self._halfStickResolution)/stepSize)*-1
            
            
    def _mapDeviceButtons(self):
        cpList = self._ds4.capabilities(verbose=True)
        if ("EV_KEY",1) in cpList.keys():
            self._categoryMapping["DISCRETE"] = 1
            for mapTup in cpList[("EV_KEY",1)]:
                if isinstance(mapTup[0], list):
                    if "BTN_SOUTH" in mapTup[0]:
                        self._keyMapping["SOUTH_BTN_D"] = mapTup[1]
                    elif "BTN_EAST" in mapTup[0]:
                        self._keyMapping["EAST_BTN_D"] = mapTup[1]
                    elif "BTN_NORTH" in mapTup[0]:
                        self._keyMapping["NORTH_BTN_D"] = mapTup[1]
                    elif "BTN_WEST" in mapTup[0]:
                        self._keyMapping["WEST_BTN_D"] = mapTup[1]
                    elif "BTN_TL" in mapTup[0]:
                        self._keyMapping["L1_BTN_D"] = mapTup[1]
                    elif "BTN_TR" in mapTup[0]:
                        self._keyMapping["R1_BTN_D"] = mapTup[1]
                    elif "BTN_TL2" in mapTup[0]:
                        self._keyMapping["L2_BTN_D"] = mapTup[1]
                    elif "BTN_TR2" in mapTup[0]:
                        self._keyMapping["R2_BTN_D"] = mapTup[1]
                    elif "BTN_SELECT" in mapTup[0]:
                        self._keyMapping["SHARE_BTN_D"] = mapTup[1]
                    elif "BTN_START" in mapTup[0]:
                        self._keyMapping["OPTIONS_BTN_D"] = mapTup[1]
                    elif "BTN_MODE" in mapTup[0]:
                        self._keyMapping["PS4_BTN_D"] = mapTup[1]
                    elif "BTN_THUMBL" in mapTup[0]:
                        self._keyMapping["L3_BTN_D"] = mapTup[1]
                    elif "BTN_THUMBR" in mapTup[0]:
                        self._keyMapping["R3_BTN_D"] = mapTup[1]
                else:
                    if "BTN_SOUTH" == mapTup[0]:
                        self._keyMapping["SOUTH_BTN_D"] = mapTup[1]
                    elif "BTN_EAST" == mapTup[0]:
                        self._keyMapping["EAST_BTN_D"] = mapTup[1]
                    elif "BTN_NORTH" == mapTup[0]:
                        self._keyMapping["NORTH_BTN_D"] = mapTup[1]
                    elif "BTN_WEST" == mapTup[0]:
                        self._keyMapping["WEST_BTN_D"] = mapTup[1]
                    elif "BTN_TL" == mapTup[0]:
                        self._keyMapping["L1_BTN_D"] = mapTup[1]
                    elif "BTN_TR" == mapTup[0]:
                        self._keyMapping["R1_BTN_D"] = mapTup[1]
                    elif "BTN_TL2" == mapTup[0]:
                        self._keyMapping["L2_BTN_D"] = mapTup[1]
                    elif "BTN_TR2" == mapTup[0]:
                        self._keyMapping["R2_BTN_D"] = mapTup[1]
                    elif "BTN_SELECT" == mapTup[0]:
                        self._keyMapping["SHARE_BTN_D"] = mapTup[1]
                    elif "BTN_START" == mapTup[0]:
                        self._keyMapping["OPTIONS_BTN_D"] = mapTup[1]
                    elif "BTN_MODE" == mapTup[0]:
                        self._keyMapping["PS4_BTN_D"] = mapTup[1]
                    elif "BTN_THUMBL" == mapTup[0]:
                        self._keyMapping["L3_BTN_D"] = mapTup[1]
                    elif "BTN_THUMBR" == mapTup[0]:
                        self._keyMapping["R3_BTN_D"] = mapTup[1]
                        
        if ('EV_ABS', 3) in cpList.keys():
            self._categoryMapping["ABSOLUTE"] = 3
            for mapTup in cpList[('EV_ABS', 3)]:
                if "ABS_X" == mapTup[0][0]:
                    self._keyMapping["LS_X_ABS"] = mapTup[0][1]
                elif "ABS_Y" == mapTup[0][0]:
                    self._keyMapping["LS_Y_ABS"] = mapTup[0][1]
                elif "ABS_RX" == mapTup[0][0]:
                    self._keyMapping["RS_X_ABS"] = mapTup[0][1]
                elif "ABS_RY" == mapTup[0][0]:
                    self._keyMapping["RS_Y_ABS"] = mapTup[0][1]
                elif "ABS_Z" == mapTup[0][0]:
                    self._keyMapping["LT_ABS"] = mapTup[0][1]
                elif "ABS_RZ" == mapTup[0][0]:
                    self._keyMapping["RT_ABS"] = mapTup[0][1]
                elif "ABS_HAT0X" == mapTup[0][0]:
                    self._keyMapping["ARROW_LR_ABS"] = mapTup[0][1]
                elif "ABS_HAT0Y" == mapTup[0][0]:
                    self._keyMapping["ARROW_UD_ABS"] = mapTup[0][1]
                    
                         
    def _printBtnValues(self):
        print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
        print( "Left Stick:  ",self.analog_LeftStick)# Orientierung - 4Quadranten schema
        print( "Right Stick: ",self.analog_RightStick)
        print( "Trigger L2:  ",self.analog_TriggerL2) 
        print( "Trigger R2:  ",self.analog_TriggerR2)
        print( "Arrow Up:    ",self.btn_ArrowUp)
        print( "Arrow Down:  ",self.btn_ArrowDown)
        print( "Arrow Left:  ",self.btn_ArrowLeft)
        print( "Arrow Right: ",self.btn_ArrowRight)
        print( "South:       ",self.btn_South)
        print( "North:       ",self.btn_North)
        print( "East:        ",self.btn_East)
        print( "West:        ",self.btn_West)
        print( "L1:          ",self.btn_L1)
        print( "R1:          ",self.btn_R1)
        print( "L2:          ",self.btn_L2)
        print( "R2:          ",self.btn_R2)
        print( "OPTIONS:     ",self.btn_Options)
        print( "SHARE:       ",self.btn_Share)
        print( "PS4:         ",self.btn_PS4)
        print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
        

    def _printMapping2Event(self, event):
        for key in self._keyMapping:
            if self._keyMapping[key] == event.code:
                print("Btn: ", key)                           
            
    def _printDeviceConfig(self):
        cpList = self._ds4.capabilities(verbose=True)
        for capkey in cpList:
            print("")
            print("Keys:", capkey)
            print("-"*80)
            for i,sub in enumerate(cpList[capkey]):
                print(i,": ", sub)
        
        
    def _findDS4Controller(self, device_dir):        
        devices = list_devices(device_dir)
        for path in devices:
            device = InputDevice(path)
            if device.name == "Wireless Controller":
                return device
                
if __name__ == "__main__":
    try:
       ds4 = RemoteControlNode()
       ds4.startNode()
    except rospy.ROSInterruptException:
        pass
