#!/usr/bin/env python3

import numpy as np
import cv2
import cv2.aruco as aruco
import rospy
import tf
from visualization_msgs.msg import Marker
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist
from std_msgs.msg import Byte, Float32MultiArray
from std_msgs.msg import MultiArrayLayout

import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np

def callback(data):
    global camwrtbase, flag,pub
    if flag ==0:
        
        # Extracting x, y, z coordinates from the received data
        x, y, z = data.data
        
        # Multiplying x and y coordinates by 2000, and z coordinate by 1000
        x *= 1000
        y *= 1000
        z *= 1000
        z= z - 260

        #print('x',x,'y',y,'z',z)
        # Creating the transformation matrix
        threeTcam = np.zeros((4, 4))
        threeTcam[:3, :3] = np.identity(3)
        
        
        # Applying transformation to the coordinates
        trans_threewrtcam = np.array([int(x), int(y), int(z)])
        trans_threewrtcam[0] -= 300  # Adding 100 to the y-coordinate
        trans_threewrtcam[2] -= 138
        
        # Printing intermediate results
        
        
        # Updating the transformation matrix
        threeTcam[:3, 3] = trans_threewrtcam
        threeTcam[3, 3] = 1
        
        
        # Printing the transformation matrix
        print("basewrtcam:\n", threeTcam)  # basewrtcam
        
        # Calculating the inverse transformation matrix
        camwrtbase = np.linalg.inv(threeTcam)
        camwrtbase_array = camwrtbase.flatten()
        msg = Float32MultiArray(data=camwrtbase_array)
            
            # Publish the transformation matrix
        pub.publish(msg)
        # Printing the inverse transformation matrix
        print("camwrtbase:\n", camwrtbase)
        flag= 1
def secondcallback(data):
    global flag, camwrtbase
    if flag == 0:
        rospy.loginfo("Camera transformation matrix is not yet initialized.")
        return
    
    xyz_final_list = data.data
    conwrtbase = []
    num_points = len(xyz_final_list) // 3  # Each point consists of x, y, z

    # Reshape the list of points into a 3D array (num_points x 3)
    points_array = np.array(xyz_final_list).reshape(num_points, 3)

    # Iterate over each point in the list
    for point in points_array:
        xconwrtcam, yconwrtcam, zconwrtcam = point
        xconwrtcam = xconwrtcam*1000
        yconwrtcam = yconwrtcam*1000
        zconwrtcam = zconwrtcam*1000
        #yconwrtcam = -yconwrtcam
        #zconwrtcam = -zconwrtcam


        conwrtcam = np.zeros((4, 4))
        conwrtcam[:3, :3] = np.identity(3)
        trans_conwrtcam = np.array([xconwrtcam, yconwrtcam, zconwrtcam])

        conwrtcam[:3, 3] = trans_conwrtcam
        conwrtcam[3, 3] = 1
        rospy.loginfo("conwrtcam:\n%s", conwrtcam)
        


        # Performing matrix multiplication
        conwrtbas = np.dot(conwrtcam, camwrtbase)
        print("conwrtbas",conwrtbas)
        x = conwrtbas[0,3]
        y = conwrtbas[1,3]
        z = conwrtbas[2,3]
        y = -y
        z = -z
        points = [x,y,z]
    
        conwrtbase.append(points)
    print("conwrtbase",conwrtbase)
    pointwrtbase_array =[item for sublist in conwrtbase for item in sublist]
    print("araay",pointwrtbase_array)
    msg = Float32MultiArray(data=pointwrtbase_array)
            
            # Publish the transformation matrix
    joint_pub.publish(msg)
    

            

def listener():
    global flag,pub,joint_pub
    flag = 0
    
    rospy.init_node('Transformation_Dobot', anonymous=True)
    rospy.Subscriber("/tvec", Float32MultiArray, callback)
    rospy.Subscriber("/xyz_list_topic", Float32MultiArray, secondcallback)
    pub = rospy.Publisher('/transformation_matrix', Float32MultiArray, queue_size=10)
    joint_pub = rospy.Publisher('/pointwrtbase', Float32MultiArray, queue_size=10)

    

    rospy.spin()

if __name__ == '__main__':
    listener()
