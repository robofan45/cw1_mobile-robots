#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Instructions for user interaction
msg = """
Reading from the keyboard and Publishing to /turtle1/cmd_vel!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

i/k: increase/decrease linear velocity
j/l: increase/decrease angular velocity
k: stop

CTRL-C to quit
"""

# Key bindings for movement commands
moveBindings = {
    'i': (1, 0),  # forward
    'o': (1, -1), # forward and right
    'j': (0, 1),  # turn left
    'l': (0, -1), # turn right
    'u': (1, 1),  # forward and left
    ',': (-1, 0), # backward
    '.': (-1, 1), # backward and left
    'm': (-1, -1),# backward and right
    'k': (0, 0),  # stop
}

# Function to capture keyboard input
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# Initial speed and turning rate
speed = 0.5
turn = 1

# Function to display current speed and turning rate
def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    # Initialize the ROS node
    rospy.init_node('turtle_teleop_key')
    # Publisher for Turtle's velocity
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=5)

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey()
            if key in moveBindings:
                x, th = moveBindings[key]  # Get linear and angular values from key binding
            else:
                x, th = 0, 0  # Default to stop
                if key == '\x03':  # Break loop on CTRL-C
                    break

            # Create and publish Twist message based on key input
            twist = Twist()
            twist.linear.x = x * speed
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)  # Print any exceptions

    finally:
        # Stop the turtle when exiting
        twist = Twist()
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)  # Reset terminal settings
