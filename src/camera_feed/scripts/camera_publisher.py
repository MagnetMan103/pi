#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def camera_publisher():
    # Initialize the ROS node
    rospy.init_node('camera_publisher', anonymous=True)
    
    # Create a publisher for the camera images
    image_pub = rospy.Publisher('camera/image_raw', Image, queue_size=10)
    
    # Create a CvBridge to convert OpenCV images to ROS messages
    bridge = CvBridge()
    
    # Open the USB camera (0 is usually the first camera)
    cap = cv2.VideoCapture(0)
    
    # Check if camera opened successfully
    if not cap.isOpened():
        rospy.logerr("Failed to open camera")
        return
    
    # Set camera properties (optional)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    rospy.loginfo("Camera publisher started")
    
    # Set publishing rate (30 Hz)
    rate = rospy.Rate(30)
    
    while not rospy.is_shutdown():
        # Capture frame from camera
        ret, frame = cap.read()
        
        if ret:
            # Convert OpenCV image to ROS Image message
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            
            # Publish the image
            image_pub.publish(ros_image)
        else:
            rospy.logwarn("Failed to capture frame")
        
        rate.sleep()
    
    # Release the camera when done
    cap.release()
    rospy.loginfo("Camera publisher stopped")

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass
