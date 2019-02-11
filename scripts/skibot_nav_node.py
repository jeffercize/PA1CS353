#!/usr/bin/env python
"""

Author: Jeffrey Banghart

"""

import numpy as np
import rospy
import roslib.packages

from skibot.msg import Pose

from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3
from skibot_nav.srv import SkibotToPoint

class SkibotNav(object):
    """ Sliding robot Navigator. """

    def __init__(self):
        rospy.init_node('approach')
        #init various objects for use later
        self.goal = None
        self.pose = None
        self.x_error = 0
        self.y_error = 0
        self.countdown = 0
        self.angle_error = 0
        #publishes to thrust which skibot is a subscriber too
        self.vel_pub = rospy.Publisher('thrust', Wrench, queue_size=10)
        #subscribe to the pose node to receive information from the skibot
        rospy.Subscriber('pose', Pose, self.pose_callback)
        #setup the service for moving the skibot
        rospy.Service('move_skibot_to_point', SkibotToPoint, self.navigate_service)
        #wait for service requests until told to shutdown
        rospy.spin()

    def pose_callback(self, pose_msg):
        self.pose = pose_msg

    def navigate(self, x, y):
        wrench = Wrench()
        #rate set to 100 to send messages to skibot at a specific interval
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            wrench.force = Vector3()
            wrench.torque = Vector3()
            wrench.force.x = 0
            wrench.torque.z = 0
            #Check if pose is None because pose_callback may not have been called yet
            if self.pose != None:
                self.x_error = x - self.pose.x
                self.y_error = y - self.pose.y
            # If the absolute combined error is greater than acceptable apply force to skibot
            if np.abs(self.x_error) + np.abs(self.y_error) > 0.01 :
                #Calculates the goalAngle using a tan inverse function built into numpy
                goalAngle = np.arctan2((y-self.pose.y), (x-self.pose.x)) * 180 / np.pi
                #Calculate the error between current theta and goalAngle
                self.angle_error = (goalAngle - (self.pose.theta * 180 / np.pi)) % 360
                if self.angle_error >= 180:
                    self.angle_error -= 360
                #The calculated angle is off by 45 degrees to this math corrects this and accounts for negative/positive flip
                if self.angle_error < -135 :
                    self.angle_error = 180 - ((180 + (self.angle_error - 45) + 1) * -1)
                if self.angle_error < -135 :
                    self.angle_error = 180 - ((180 + (self.angle_error - 45) + 1) * -1)

                if self.angle_error > 0.5 and self.pose != None:
                    if self.pose.theta_velocity < 1 :
                        #Apply torque based on angle error
                        wrench.torque.z = self.angle_error * 0.05
                if self.angle_error < -0.5 and self.pose != None:
                    if self.pose.theta_velocity > -1 :
                        #Apply torque based on angle error
                        wrench.torque.z = self.angle_error * 0.05

                # if the current total velocity is less than the total error
                if np.abs(self.pose.x_velocity) + np.abs(self.pose.y_velocity) < (np.abs(self.x_error) + np.abs(self.y_error)) :
                    #force is determined by the total absolute error and current absolute velocity
                    wrench.force.x = ((np.abs(self.x_error) + np.abs(self.y_error))) - np.abs(self.pose.x_velocity) + np.abs(self.pose.y_velocity)
                # if the current total velocity is greater than the total error
                if np.abs(self.pose.x_velocity) + np.abs(self.pose.y_velocity) > (np.abs(self.x_error) + np.abs(self.y_error)) :
                    #force is determined by the total absolute error and current absolute velocity
                    wrench.force.x = -0.3 * np.abs(self.pose.x_velocity) + np.abs(self.pose.y_velocity) - ((np.abs(self.x_error) + np.abs(self.y_error)))
                
                #clips the acceleration force to ensure movement even when there is minimal error
                if wrench.force.x < 0:
                    wrench.force.x = np.clip(wrench.force.x, -.5, -.35)
                if wrench.force.x > 0:
                    wrench.force.x = np.clip(wrench.force.x, .35, .5)
                countdown = 0
            else :
                #on the target so begin to countdown
                self.countdown += 1
            if self.countdown > 1500 : 
                self.countdown = 0
                #if the countdown has passed 1500 then the service is completed
                return True
            self.vel_pub.publish(wrench)


    def navigate_service(self, nav_srv):
        if (nav_srv.goal_x < 0 or nav_srv.goal_x > 4 or
            nav_srv.goal_y < 0 or nav_srv.goal_y > 4):
            rospy.loginfo("Invalid navigation request")
            return False
        return self.navigate(nav_srv.goal_x, nav_srv.goal_y)

if __name__ == "__main__":
    nav = SkibotNav()
