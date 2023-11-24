#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def move_turtle():
    # initializ a ROS node with the name 'turtle_mover'
    rospy.init_node('turtle_mover', anonymous=True)

    # create a publisher object that will publish messages of type Twist on the '/cmd_vel' topic
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    # set the loop rate (in this case, we choose 10 Hz)
    rate = rospy.Rate(10)

    # create a new Twist message instance
    vel_msg = Twist()

    # set linear velocity (move forward)
    vel_msg.linear.x = 1.0  # speed value in m/s
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0

    # set angular velocity (turn)
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 1.0  # rotation value in rad/s

    # keep publishing the velocity commands (move forward and turn) until the node is shutdown
    while not rospy.is_shutdown():
        pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_turtle()
    except rospy.ROSInterruptException:
        pass  # allow a clean shutdown if Ctrl+C is pressed