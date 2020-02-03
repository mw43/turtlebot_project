#! /usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Turtlebot Class
class Turtlebot:

    # Contructor Method - Initializes publishers and subscribers.
    def __init__(self):
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.callback_scan)

        self.MOVE_VEL = 0.1
        self.TURN_VEL = 0.5

        # Ranges for the real robot.
        #self.RIGHT_RANGE = range(355, 400)
        #self.LEFT_RANGE = range(400, 445)

        # Ranges for the simulation in Gazebo
        self.RIGHT_RANGE = range(0, 30)
        self.LEFT_RANGE = range(330, 360)

        # Ranges for the simulation in Robot Ignite Academy
        # self.RIGHT_RANGE = range(0, 30)
        # self.LEFT_RANGE = range(690, 720)


    # Callback function - takes left and right scan data and process it.
    def callback_scan(self, scan):
        sensor_data = scan.ranges

        # Checking if there is any scan data.
        if len(sensor_data) > 0:

            # Is true if a obstacle is detected.
            obstacle = False

            # Velocity command sent to the robot
            cmd = Twist()

            # Check right side.
            for i in self.RIGHT_RANGE:
                if sensor_data[i] < 0.3:
                    obstacle = True

            # Check left side
            for i in self.LEFT_RANGE:
                if sensor_data[i] < 0.3:
                    obstacle = True

            # Decide whether to turn or continue straight
            if obstacle:
                cmd.linear.x = 0
                cmd.angular.z = self.TURN_VEL
            else:
                cmd.linear.x = self.MOVE_VEL
                cmd.angular.z = 0

            # Send command to robot
            self.pub.publish(cmd)
        else:
            print("No scan data detected: has the subscriber received a message yet?")

# Main function creates the node and instantiates an object of class Turtlebot and spins.
if __name__ == '__main__':
    rospy.init_node('simple_avoidance')
    turtlebot = Turtlebot()
    rospy.spin()
