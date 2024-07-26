#!/usr/bin/env python3

import cv2
import os
import sys
import numpy as np
import rospy
import json
from sensor_msgs.msg import PointCloud2
# from outsole_path.srv import GetPath, GetPathResponse

TF_Z_BIAS = 0;
TF_X_BIAS = 0;
TF_Y_BIAS = 0;
velocity = 300;
PLASMA_DIA = 0.05;
CLOUD_SEARCHING_RANGE = 0.0014;
rounds=3;
mode = 0;

def read_parameters():
    global rounds
    filePath = "/home/honglang/PSP/tuning/outsole_path.json"
    if not os.path.exists(filePath):
        sys.exit("Error: File not found - " + filePath)
    try:
        with open(filePath, 'r') as file:
            parameters = json.load(file)
        
        global mode, PLASMA_DIA, TF_Z_BIAS, TF_X_BIAS, TF_Y_BIAS, velocity, rounds
        mode = parameters["mode"]
        PLASMA_DIA = parameters["PLASMA_DIA"]
        TF_Z_BIAS = parameters["TF_Z_BIAS"]
        TF_X_BIAS = parameters["TF_X_BIAS"]
        TF_Y_BIAS = parameters["TF_Y_BIAS"]
        velocity = parameters["velocity"]
        rounds = parameters["rounds"]
        
        return True
    
    except Exception as e:
        print("Failed to read the parameters:", e)
        return False

def get_path():

    # Define the boundary points
    point_2 = [420.000 + TF_X_BIAS, -180.000 + TF_Y_BIAS, -290.000 + TF_Z_BIAS]  #  P[2]
    point_3 = [420.000 + TF_X_BIAS, 180.000 + TF_Y_BIAS, -290.000 + TF_Z_BIAS]  #  P[3]
    point_14 = [600.000 + TF_X_BIAS, -180.000 + TF_Y_BIAS, -290.000 + TF_Z_BIAS]  #  P[14]
    point_15 = [600.000 + TF_X_BIAS, 180.000 + TF_Y_BIAS, -290.000 + TF_Z_BIAS]  #  P[15]

    # Calculate the offset on x-axis around the rounds
    total_distance_on_x = point_14[0] - point_2[0]  # or point_15[0] - point_3[0]
    offset_x = total_distance_on_x / (rounds - 1)

    # Append points into array named path
    path = [[420.246 + TF_X_BIAS, 0.000 + TF_Y_BIAS, 53.417 + TF_Z_BIAS]]  # origin position P[1]
    path.append(point_2)  # P[2]
    path.append(point_3)  # P[3]

    # Get points beetween each rounds
    for i in range(2, rounds + 1):
        direction = -1 if i % 2 == 1 else 1  # odd --> left  even --> right
        next_round_point1 = [
            point_3[0] + (i - 1) * offset_x,  # calculate new x
            direction * point_3[1] ,  # y will diferent from variable direction(-1 or 1)
            -290 + TF_Z_BIAS  # z is const.
        ]
        next_round_point2 = [
            next_round_point1[0],  #  Through a round ,next_round_point2[0] = next_round_point1[0]
            next_round_point1[1] * (-1),  #  next_round_point2[0] = next_round_point1[0] * -1
            -290 + TF_Z_BIAS  # z is const.
        ]
        path.append(next_round_point1)
        path.append(next_round_point2)

    path.append([420.246 + TF_X_BIAS, 0.000 + TF_Y_BIAS, 53.417 + TF_Z_BIAS])  # 確保結束於 P[16]，即 P[1] 的位置

    return path

# points = [
#     [420.246, 0.000, 53.417],  # P[1]
#     [420.000, -180.000, -290.000],  # P[2]
#     [420.000, 180.000, -290.000],  # P[3]
#     [450.000, 180.000, -290.000],  # P[4]
#     [450.000, -180.000, -290.000],  # P[5]
#     [480.000, -180.000, -290.000],  # P[6]
#     [480.000, 180.000, -290.000],  # P[7]
#     [510.000, 180.000, -290.000],  # P[8]
#     [510.000, -180.000, -290.000],  # P[9]
#     [540.000, -180.000, -290.000],  # P[10]
#     [540.000, 180.000, -290.000],  # P[11]
#     [570.000, 180.000, -290.000],  # P[12]
#     [570.000, -180.000, -290.000],  # P[13]
#     [600.000, -180.000, -290.000],  # P[14]
#     [600.000, 180.000, -290.000],  # P[15]
#     [420.246, 0.000, 53.417]  # P[16](=P[1])
# ]

# 根據 path 生成和寫入 O002.LS 檔案
def writeLsFile(filename, path):
    with open(filename, 'w') as f:
        f.write("/PROG H001.LS\n")
        f.write("/ATTR\n")
        f.write("OWNER       = MNEDITOR;\n")
        f.write("COMMENT     = \"\";\n")
        f.write("PROG_SIZE   = 636;\n")
        f.write("CREATE      = DATE 24-07-17  TIME 11:59:14;\n")
        f.write("MODIFIED    = DATE 24-07-17  TIME 12:02:18;\n")
        f.write("FILE_NAME   = ;\n")
        f.write("VERSION     = 0;\n")
        f.write("LINE_COUNT  = 4;\n")
        f.write("MEMORY_SIZE = 992;\n")
        f.write("PROTECT     = READ_WRITE;\n")
        f.write("TCD:  STACK_SIZE    = 0,\n")
        f.write("      TASK_PRIORITY = 50,\n")
        f.write("      TIME_SLICE    = 0,\n")
        f.write("      BUSY_LAMP_OFF = 0,\n")
        f.write("      ABORT_REQUEST = 0,\n")
        f.write("      PAUSE_REQUEST = 0;\n")
        f.write("DEFAULT_GROUP    = 1,*,*,*,*;\n")
        f.write("CONTROL_CODE     = 00000000 00000000;\n")

        f.write("/MN\n")
        for i, point in enumerate(path, start=1):
            f.write("   {}:L P[{}] 100mm/sec CNT100    ;\n".format(i, i))
        f.write("/POS\n")

        for i, point in enumerate(path, start=1):
            f.write("P[{}]{{\n".format(i))  # Correct the string formatting issue
            f.write("   GP1:\n")
            f.write("    UF : 0, UT : 6,      CONFIG : 'N U T, 0, 0, 0',\n")
            f.write("    X =  {:.3f}  mm,    Y =   {:.3f}  mm,    Z =   {:.3f}  mm,\n".format(*point))
            f.write("    W =  -180.000 deg,    P =   0.000 deg,    R =   0.000 deg\n")
            f.write("};\n")
        f.write("/END\n")

def server_callback(req):
    if req.REQU_PP:
        path = get_path()
        res = GetPathResponse()
        res.RESP_PP = True if path else False
        return res
    else:
        return GetPathResponse(RESP_PP=False)

if __name__ == '__main__':
    read_parameters()
    outsole_path = get_path()
    writeLsFile("/home/honglang/PSP/files/H001.LS", outsole_path)

    # rospy.init_node('path_planning_service')
    # s = rospy.Service('get_path', GetPath, server_callback)
    # rospy.spin()