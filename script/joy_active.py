#!/usr/bin/env python
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
import rospy

class JoyActive():
    """node to trigger adaptive control experiments"""
    def __init__(self):
        self.joy_sub = rospy.Subscriber('/joy',Joy,self.joy_callback,queue_size=1)
        self.active_pub = rospy.Publisher('/ac/active',Bool,queue_size=1)
        self.active_timer = rospy.Timer(rospy.Duration(1./10),self.active_callback)
        self.active = False
        self.last_button = 0

    def joy_callback(self,data):
        if data.buttons[0] == 1 and self.last_button == 0:
            self.active = not self.active
        self.last_button = data.buttons[0]

    def active_callback(self,event):
        self.active_pub.publish(Bool(self.active))

def main():
    rospy.init_node('joy_active')
    try:
        JoyActive()
        rospy.spin()
    except rospy.ROSException as e:
        print('closing joy active')
        raise e

if __name__ == "__main__":
    main()
