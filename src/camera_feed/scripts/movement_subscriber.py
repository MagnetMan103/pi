#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class MovementController:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('movement_controller', anonymous=True)
        
        # Subscribe to movement commands
        self.cmd_sub = rospy.Subscriber(
            '/cmd_movement',
            String,
            self.command_callback
        )
        
        rospy.loginfo("Movement controller started and listening for commands...")
    
    def command_callback(self, msg):
        """
        Callback function that processes incoming movement commands
        """
        command = msg.data
        rospy.loginfo(f"Received command: {command}")
        
        # Execute the movement based on command
        if command == "forward":
            self.move_forward()
        elif command == "backward":
            self.move_backward()
        elif command == "left":
            self.turn_left()
        elif command == "right":
            self.turn_right()
        elif command == "stop":
            self.stop()
        else:
            rospy.logwarn(f"Unknown command received: {command}")
    
    def move_forward(self):
        """Move robot forward"""
        rospy.loginfo("ACTION: Moving forward")
        # TODO: Add your motor control code here
        # Example: GPIO.output(MOTOR_PIN, HIGH)
    
    def move_backward(self):
        """Move robot backward"""
        rospy.loginfo("ACTION: Moving backward")
        # TODO: Add your motor control code here
    
    def turn_left(self):
        """Turn robot left"""
        rospy.loginfo("ACTION: Turning left")
        # TODO: Add your motor control code here
    
    def turn_right(self):
        """Turn robot right"""
        rospy.loginfo("ACTION: Turning right")
        # TODO: Add your motor control code here
    
    def stop(self):
        """Stop all robot movement"""
        rospy.loginfo("ACTION: Stopping")
        # TODO: Add your motor control code here
    
    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = MovementController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
