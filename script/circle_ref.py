import rospy
import numpy as np
from hamilton_ac.msg import Reference
from std_msgs.msg import Bool

class CircleRef():
    def __init__(self):
        self.active = False
        self.start_time = -1.
        self.R = rospy.get_param('/ref/R')
        self.T = rospy.get_param('/ref/T')
        self.c_x = rospy.get_param('/ref/c_x')
        self.c_y = rospy.get_param('/ref/c_y')
        self.c = np.array([self.c_x,self.c_y])
        self.rot = rospy.get_param('/ref/rot')
        self.freq = rospy.get_param('/ref/freq')

        self.ref_timer = rospy.Timer(1/self.freq,self.ref_callback)
        self.active_sub = rospy.Subscriber('/ac/active',self.active_callback,queue_size=1)
        self.ref_pub = rospy.Publisher('/ac/ref',Reference,queue_size=1)

    def ref_callback(self):
        ### TODO: finish reference callback
        pass

    def active_callback(self,data):
        ## TODO: finish active callback
        pass


def main():
    rospy.init_node("circle_ref")
    try:
        CircleRef()
        rospy.spin()
    except rospy.ROSException as e:
        rospy.logwarn('closing reference')
        raise e

if __name__ == "__main__":
    main()
