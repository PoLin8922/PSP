#!/usr/bin/env python3

import rospy
import sys
import time
import os
import datetime
from vision_capture2.srv import VisionCapture
from path_planning_ver1.srv import path_planning_ver1
from communication.srv import ModbusPLC, ModbusRobot, Ftp
from std_msgs.msg import Int32

request = False

class controller ():
    
    def __init__ ( self ):
        rospy.init_node ( 'controller' )
        self.plcState = 0
        
        self.capture_proxy = rospy.ServiceProxy ( 'vision_capture', VisionCapture )
        self.pp_proxy = rospy.ServiceProxy ( 'path_planning_ver1', path_planning_ver1 )
        self.ftp_proxy = rospy.ServiceProxy('ftp_transfer', Ftp)
        self.funuc_proxy = rospy.ServiceProxy('modbus_robot_control', ModbusRobot)
        self.plc_proxy = rospy.ServiceProxy('modbus_plc_control', ModbusPLC)
        self.plc_sub = rospy.Subscriber("plc_state", Int32, self.plc_callback)

    def plc_callback( self, data):
        self.plcState = data.data

    def capture( self, request ):
        rospy.wait_for_service( 'vision_capture' )

        try:
            response = self.capture_proxy( request )
            if response.scan_state:
                print( '\033[94m' + "[SERVER] Vision Capture Successfully Completed." + '\033[0m' )
                request = False
                return True
            else:
                request = False
                return False
                        
        except rospy.ServiceException as e:
            print( '\033[91m' + " [SERVER] Error Sending Vision Capture Request." +  '\033[0m' )
            self.LOGGING(   "   [SERVER] Error Sending Vision Capture Request" )

    def planning(self, request):
        rospy.wait_for_service('path_planning_ver1')

        try:
            response = self.pp_proxy( request )
            if response:
                print( '\033[94m' + "[SERVER] Path Planning Successfully Completed." + '\033[0m' )
                request = False
        
        except rospy.ServiceException as e:
            print( '\033[91m' + " [SERVER] Error Sending Path Planning Request." +  '\033[0m' )
            self.LOGGING(   "   [SERVER] Error Sending Path Planning Request" )

    def fileTf(self, host, path):  
        rospy.wait_for_service('ftp_transfer')

        try:
            response = self.ftp_proxy(host, path)
            if response:
                print( '\033[94m' + "[SERVER] Transfer LS File to funuc Successfully Completed." + '\033[0m' )
                
        except rospy.ServiceException as e:
            print( '\033[91m' + " [SERVER] Error Transfer LS File Request." +  '\033[0m' )
            self.LOGGING(   "   [SERVER] Error Transfer LS File Request" )
            self.plcControl(5)
            retry_num = 3
            while not response and retry_num:
                response = self.ftp_proxy(host, path)
                retry_num -=1
            if not retry_num:
                print("Transfer LS File error")
            else:
                self.plcControl(1)


    def fanucStart(self, request):
        rospy.wait_for_service('modbus_robot_control')

        try:
            response = self.funuc_proxy(request)
            if response:
                print( '\033[94m' + "[SERVER] Execute FUNUC Successfully Completed." + '\033[0m' )
        
        except rospy.ServiceException as e:
            print( '\033[91m' + " [SERVER] Error Execute FUNUC Request." +  '\033[0m' )
            self.LOGGING(   "   [SERVER] Error Execute FUNUC Request" )

            self.plcControl(5)
            retry_num = 3
            while not response and retry_num:
                response = self.funuc_proxy(request)
                retry_num -=1
            if not retry_num:
                print("Execute FUNUC error")
            else:
                self.plcControl(1)

    def plcControl(self, request):
        rospy.wait_for_service('modbus_plc_control')

        try:
            response = self.plc_proxy(request)
            if response:
                print( '\033[94m' + "[SERVER] Set PLC Successfully Completed." + '\033[0m' )
        
        except rospy.ServiceException as e:
            print( '\033[91m' + " [SERVER] Error Set PLC Request." +  '\033[0m' )
            self.LOGGING(   "   [SERVER] Error Set PLC Request" )
            self.plcControl(5)
            retry_num = 3
            while not response and retry_num:
                response = self.plc_proxy(request)
                retry_num -=1
            if not retry_num:
                print("Set PLC error")
            else:
                self.plcControl(1)

    def run(self):
        # self.plcControl(2)
        if self.plcState == 1:
            print("funuc start moving")
            time.sleep(1)
            if not self.capture(True):
                self.plcControl(4)
            else:
                print("contorller.py run")
                self.planning(True)
                self.fileTf('192.168.255.200', '/home/honglang/PSP/files/H001.LS')
                self.plcControl(8)
                self.fanucStart(True)
                self.plcControl(2)
                print("funuc stop moving")
            

    def LOGGING ( self, state ):
        homeDir = os.getenv( "HOME" )
        if homeDir is None:
            sys.stderr.write( "Failed to get the home directory.\n" )

        current_datetime = datetime.datetime.now()
        #file_loc = homeDir + "/PSP/logfile/" + "logging_file.txt"
        file_loc = "/home/honglang/PSP/logfile/logging_file.txt"
        
        with open( file_loc, 'a' ) as file: 
            if file.tell() != 0: 
                file.write( '\n' ) 
            file.write( state )


if __name__ == '__main__':
    homeDir = os.getenv( "HOME" )
    if homeDir is None:
        sys.stderr.write( "Failed to get the home directory.\n" )

    current_datetime = datetime.datetime.now()
    #file_loc = homeDir + "/PSP/logfile/" + "logging_file.txt"
    # file_loc = "/home/honglang/PSP/logfile/logging_file.txt"

    current_datetime = datetime.datetime.now()

    # with open( file_loc, 'a' ) as file: 
    #     if file.tell() != 0: 
    #         file.write( '\n' ) 
    #     file.write( "[Time] " + current_datetime.strftime("%Y-%m-%d_%H-%M-%S") )
    #     file.write( "[Start] ")

    ###
    c = controller()
    while(1):
        c.run()
    ###

    # with open( file_loc, 'a' ) as file: 
    #     if file.tell() != 0: 
    #         file.write( '\n' ) 

    #     file.write( "[End] ")