#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray, Int16MultiArray, Byte
from geometry_msgs.msg import Twist
import numpy as np
from roboticstoolbox import DHLink, DHRobot
import moveit_commander
import sys
import time
from serial.tools import list_ports

from pydobot import Dobot


def callback(data):
    global final_list,num_points
    final_list = []
    num_points = len(data.data) // 3
    for i in range(num_points):
        start_index = i * 3
        end_index = start_index + 3
        point_coords = list(data.data[start_index:end_index])
        final_list.append(point_coords)
        
    print("final_list:", final_list)
                
        

        
        
       

    

def motion_flag(data):
    global final_list,joint_msg, fr

    if data.data == 4 and fr == 0.0:
        print("data.data",data.data)
        for i in range(num_points):
            print("finalist[i]",final_list[i])
            print('x', final_list[i][0])
            print('y', final_list[i][1])
            print('z', final_list[i][2])

            port =  list_ports.comports()[0].device
            #print(port)
        
            device = Dobot(port=port)

            #device.go(final_list[i][0]*1000, final_list[i][1]*1000, final_list[i][2]*1000, 0.0)
            device.move_to(final_list[i][0], final_list[i][1], final_list[i][2], 0, wait=True)
            #device.move_to(pose[0], pose[1], pose[2], pose[3], wait=True)  # we wait until this movement is done before continuing

            device.close()
            
            
            time.sleep(3)
        final_list = []
        
        
def frame_flag(data):
    global fr
    fr = data.linear.x
           


def listener():
    rospy.init_node('ikine_subscriber_dobot', anonymous=True)
    rospy.Subscriber("/pointwrtbase", Float32MultiArray, callback)
   
    rospy.Subscriber("/frame_fb_topic", Twist, frame_flag)
    rospy.Subscriber("/pointer_triggers", Byte, motion_flag)

    

   
    
    rospy.spin()

if __name__ == '__main__':
    listener()