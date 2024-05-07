#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose,Twist
from std_msgs.msg import Byte, Float32MultiArray, String
from roboticstoolbox import DHRobot, DHLink
import numpy as np
import math
import tf

# Global variables to store the list of x, y, z values
global i 
i = 0
xyz_final = []
xyz_fake_list = []
xyz_list = []
def ws_checker(latest_pose):
    global camwrtbase
    pi = np.pi
    link1 = DHLink(d=0, a=0, alpha=pi/2, theta=0,sigma=0,mdh=0,offset=0,qlim = [np.deg2rad(-125),np.deg2rad(125)] )
    link2 = DHLink(d=0,a=135, alpha=0,theta=0,sigma=0,mdh = 0,offset=pi/2,qlim = [np.deg2rad(-90),np.deg2rad(10)])
    link3 = DHLink(d=0, a=147, alpha=0, theta=0,sigma=0,mdh=0,offset=-pi/2,qlim = [np.deg2rad(-75),np.deg2rad(60)]) 
    mask = [1, 1, 1, 0, 0, 0]
    links = [link1, link2, link3]

    robot = DHRobot(links, name="hri")

    link11 = DHLink(d=0, a=0, alpha=pi/2, theta=0,sigma=0,mdh=0,offset=0,qlim = [np.deg2rad(-125),np.deg2rad(125)] )
    link22 = DHLink(d=0,a=200, alpha=0,theta=0,sigma=0,mdh = 0,offset=pi/2,qlim = [np.deg2rad(-91),np.deg2rad(10)])
    link33 = DHLink(d=0, a=200, alpha=0, theta=0,sigma=0,mdh=0,offset=-pi/2,qlim = [np.deg2rad(-75),np.deg2rad(60)]) 
    mask = [1, 1, 1, 0, 0, 0]

    # Create a list of links
    linkss = [link11, link22, link33]

    # Create the robot
    robott = DHRobot(linkss, name="hri2")

    print("latest_pose",latest_pose)
    x = latest_pose[0]
    y = latest_pose[1]
    z = latest_pose[2]
    x = x*1000
    y = y*1000
    z = z*1000
    z = z - 20
    conwrtcam = np.zeros((4, 4))
    conwrtcam[:3, :3] = np.identity(3)
    trans_conwrtcam = np.array([x, y, z])

    conwrtcam[:3, 3] = trans_conwrtcam
    conwrtcam[3, 3] = 1
    print("conwrtcam",conwrtcam)
    conwrtbase = np.dot(conwrtcam, camwrtbase)
    print("conwrtbase:",conwrtbase)
    x = conwrtbase[0,3]
    y = conwrtbase[1,3]
    z = conwrtbase[2,3]
    y = -y
    z = -z
    print("X:",x)
    print("y:",y)
    print("z:",z)

    x = np.round(x,2)
    y = np.round(y,2)
    z = np.round(z,2)
    
    print("X_round:",x)
    print("y:",y)
    print("z:",z)

    

   
    T = np.array([[1.0, 0.0, 0.0, x],
                    [0.0, 1.0, 0.0, y],
                    [0.0, 0.0, 1.0, z],
                    [0.0, 0.0, 0.0, 1.0]])

    # Perform inverse kinematics to get joint angles in degrees
    joint_angles = robott.ik_LM(T, mask=mask,joint_limits = True)
   


    first = joint_angles[0][0]
    print("first",first)
    T[0, 3] -= 59 * np.cos(np.radians(first))
    T[1, 3] -= 59 * np.sin(np.radians(first))
   
    joint_angles = robot.ik_LM(T, mask=mask,joint_limits = True)
    print("pre edit",joint_angles)
    print("pre edit",np.rad2deg(joint_angles[0]))


    joint_angles[0][1] = -joint_angles[0][1]
    joint_angles[0][2] = (-joint_angles[0][2])+joint_angles[0][1]
    print("post edit joint a",np.rad2deg(joint_angles[0]))
    if joint_angles[1] == 1:
        return 1
    else:
        return 0

def points_callback(data):
    global xyz_list, xyz_fake_list
    xyz = data.data
    xyz_fake_list.append(xyz)
    if len(xyz_fake_list) > 10:
        xyz_fake_list.pop(0) 
    xyz_list = [list(point) for point in xyz_fake_list]
    
    

def data_formatting(pose):
    position = Point()
    position.x, position.y, position.z = pose[:3]
    orientation = Quaternion()
    # Assuming the input is in roll-pitch-yaw format
    # Convert degrees to radians
    roll, pitch, yaw = map(math.radians, [0,90,0])
    q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
    orientation.x, orientation.y, orientation.z, orientation.w = q
    pose = Pose(position=position, orientation=orientation)

    # Create PoseStamped
    pose_stamped = PoseStamped()
    pose_stamped.pose = pose
    return pose_stamped

def pointer_triggers_callback(data):
    global xyz_list, xyz_final, i,error,fb

    if data.data == 1:
        if fb != 0.0:
            if xyz_list and i < len(xyz_list):
                latest_pose = xyz_list[-1]
                print("latestpose",latest_pose)
                check = ws_checker(latest_pose)
                
                
                if check == 1:
                    xyz_final.append(latest_pose)
                    error_value = Twist()
                    error_value.linear.x = 2.0
                    error.publish(error_value)
                    
                    rospy.loginfo("Updated xyz_final: {}".format(xyz_final))
                else:   
                    error_value = Twist()

                    error_value.linear.x = 0.0
                    error.publish(error_value)
                    rospy.loginfo("Out of workspace")
        
        else:
            error_value = Twist()

            error_value.linear.x = 0.0
            error.publish(error_value)
                
            

    elif data.data == 2:
        if xyz_final:
            # Flatten the xyz_final list and publish it
            flattened_data = [item for sublist in xyz_final for item in sublist]
            pub.publish(Float32MultiArray(data=flattened_data))
            rospy.loginfo("Published xyz_final: {}".format(flattened_data))
            xyz_final.clear()  # Clear the xyz_final list after publishing
            i = 0

    elif data.data == 3:
        if xyz_final:
            xyz_final.pop(-1)
            i -= 1
            rospy.loginfo("Removed last pose from xyz_final: {}".format(xyz_final))
def third_callback(data):
    global camwrtbase
    camwrtbase_array = data.data
    
    # Reshape the 1D array back into a 4x4 transformation matrix
    camwrtbase = np.reshape(camwrtbase_array, (4, 4))
    print(camwrtbase)
def frame_callback(data):
    global fb
    fb = data.linear.x

def main():
    global pub, error

    rospy.init_node('Point_appender_dobot', anonymous=True)

    # Publisher to publish the xyz_final list
    pub = rospy.Publisher('/xyz_list_topic', Float32MultiArray, queue_size=10)
   

    # Subscribe to the pointer_triggers topic
    rospy.Subscriber("/pointer_triggers", Byte, pointer_triggers_callback)
    error = rospy.Publisher("ws_fb_topic",Twist,queue_size= 10)
    rospy.Subscriber("/tvec_pub", Float32MultiArray, points_callback)
    rospy.Subscriber("/transformation_matrix", Float32MultiArray,third_callback)
    rospy.Subscriber("/frame_fb_topic", Twist, frame_callback)


    # Spin to keep the node running
    rospy.spin()

if __name__ == '__main__':
    
    main()
    