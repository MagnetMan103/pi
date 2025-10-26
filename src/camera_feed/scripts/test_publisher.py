#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker():
    # Create a publisher that sends String messages to the 'chatter' topic
    pub = rospy.Publisher('chatter', String, queue_size=10)
    
    # Initialize the node
    rospy.init_node('test_publisher', anonymous=True)
    
    # Set the rate (1 Hz = once per second)
    rate = rospy.Rate(1)
    
    count = 0
    while not rospy.is_shutdown():
        message = f"Hello World from Pi! Count: {count}"
        rospy.loginfo(message)
        pub.publish(message)
        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
