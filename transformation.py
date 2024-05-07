import numpy as np
import cv2
import sys
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import time

class ArUcoDetector:
    def __init__(self):
        rospy.init_node('aruco_detector', anonymous=True)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.tvec_publisher = rospy.Publisher('/tvec', Float32MultiArray, queue_size=1)

        self.matrix_coefficients = None
        self.distortion_coefficients = None

        # Load calibration matrix and distortion coefficients
        self.load_calibration_data()

        # Flag to keep track of whether tvec has been published or not
        self.tvec_published = False

    def load_calibration_data(self):
        # Load calibration data from files
        calibration_matrix_path = "calibration_matrix.npy"
        distortion_coefficients_path = "distortion_coefficients.npy"

        self.matrix_coefficients = np.load(calibration_matrix_path)
        self.distortion_coefficients = np.load(distortion_coefficients_path)

    def image_callback(self, msg):
        global flag
        if flag == 0:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except CvBridgeError as e:
                print(e)
                return

            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            parameters = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

            corners, ids, _ = detector.detectMarkers(gray)

            # Check if marker with ID 10 is detected
            if ids is not None and 10 in ids:
                index = np.where(ids == 10)[0][0]  # Get the index of marker with ID 10
                corners = [corners[index]]  # Filter out the corners of the marker with ID 10
                ids = np.array([[10]])  # Set the ID to 10 as a numpy array

                for i in range(len(corners)):
                    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.035, self.matrix_coefficients, self.distortion_coefficients)
                    
                    
                    cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)  # Pass ids as numpy array
                    cv2.drawFrameAxes(cv_image, self.matrix_coefficients, self.distortion_coefficients, rvec, tvec, 0.01)

                    # Publish tvec on topic '/tvec'
                    tvec_msg = Float32MultiArray()
                    tvec_msg.data = tvec.flatten().tolist()
                    print(tvec_msg)
                    for i in range(1000):
                        self.tvec_publisher.publish(tvec_msg)
                        time.sleep(2)
                    flag = 1

            cv2.waitKey(1)
     
        

if __name__ == '__main__':
    global Flag
    flag = 0
    aruco_detector = ArUcoDetector()
    rospy.spin()
