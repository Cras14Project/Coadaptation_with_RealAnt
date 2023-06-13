#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class Publisher():

""" a publisher class for testing when optitrack is not available.  Publishes a pose of zeros with time stamp to the same topic the camera_DAQ.py subscribes 
"""

    def __init__(self):
        self.node = rospy.init_node('writer', anonymous= True)
        self.pub = rospy.Publisher("/mocap_node/Robot_1/pose", PoseStamped, queue_size=100)
        self.rate = rospy.Rate(100)

    def talk(self):
        count = 0
        while not rospy.is_shutdown():

            pose = PoseStamped()
            now =  rospy.get_rostime()   
            pose.header.stamp.secs = now.secs
            pose.header.stamp.nsecs = now.nsecs

            
            self.pub.publish(pose)
            count += 1
            self.rate.sleep()


def main():
    pub = Publisher()
    pub.talk()

if __name__ == '__main__':
    main()
