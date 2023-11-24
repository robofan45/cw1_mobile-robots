   #!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose

# this is the callback function that will process incoming messages on the /pose topic
def pose_callback(data):
    rospy.loginfo(rospy.get_caller_id() + " received pose: %s" % data)

def listener():
    # initialize a ros node 'pose_listener'
    rospy.init_node('pose_listener', anonymous=True)

    # creating a subscriber object that subscribes to the /pose topic
    # and calls the 'pose_callback' function upon receiving a message
    rospy.Subscriber('/pose', Pose, pose_callback)

    # use rospy.spin() to keep the script for exiting until the node is stopped like a loop
    rospy.spin()

if __name__ == '__main__':
    listener()
