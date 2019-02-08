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
        self.goal = None
        self.pose = None
        self.x_error = 0
        self.y_error = 0
        self.countdown = 0
        self.angle_error = 0
        self.vel_pub = rospy.Publisher('thrust', Wrench, queue_size=10)
        rospy.Subscriber('pose', Pose, self.pose_callback)
        rospy.Service('move_skibot_to_point', SkibotToPoint, self.navigate_service)
        rospy.spin()

    def pose_callback(self, pose_msg):
        self.pose = pose_msg

    def navigate(self, x, y):
        wrench = Wrench()
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            wrench.force = Vector3()
            wrench.torque = Vector3()
            wrench.force.x = 0
            wrench.torque.z = 0
            if self.pose != None:
                self.x_error = x - self.pose.x
                self.y_error = y - self.pose.y

            if True and np.abs(self.x_error) + np.abs(self.y_error) > 0.01 :

                testAngle = np.arctan2((y-self.pose.y), (x-self.pose.x)) * 180 / np.pi
                self.angle_error = testAngle - (self.pose.theta * 180 / np.pi)
                if self.angle_error < -135 :
                    self.angle_error = 180 - ((180 + (self.angle_error - 45) + 1) * -1)
                if self.angle_error < -135 :
                    self.angle_error = 180 - ((180 + (self.angle_error - 45) + 1) * -1)

                if self.angle_error > 0.5 and self.pose != None:
                    if self.pose.theta_velocity < 1 :
                        wrench.torque.z = self.angle_error * 0.05
                if self.angle_error < -0.5 and self.pose != None:
                    if self.pose.theta_velocity > -1 :
                        wrench.torque.z = self.angle_error * 0.05

                if np.abs(self.pose.x_velocity) + np.abs(self.pose.y_velocity) < (np.abs(self.x_error) + np.abs(self.y_error)) * 1 :
                    wrench.force.x = ((np.abs(self.x_error) + np.abs(self.y_error))) - np.abs(self.pose.x_velocity) + np.abs(self.pose.y_velocity)

                if np.abs(self.pose.x_velocity) + np.abs(self.pose.y_velocity) > (np.abs(self.x_error) + np.abs(self.y_error)) * 1 :
                    wrench.force.x = -0.3 * np.abs(self.pose.x_velocity) + np.abs(self.pose.y_velocity) - ((np.abs(self.x_error) + np.abs(self.y_error)))
                
                if wrench.force.x < 0:
                    wrench.force.x = np.clip(wrench.force.x, -.5, -.35)
                if wrench.force.x > 0:
                    wrench.force.x = np.clip(wrench.force.x, .35, .5)
                
                countdown = 0
            elif False and np.abs(self.x_error) + np.abs(self.y_error) > 0.01 :
                testAngle = np.arctan2((y-self.pose.y), (x-self.pose.x)) * 180 / np.pi
                self.angle_error = testAngle - (self.pose.theta * 180 / np.pi)
                
                #adjust for the 45 degree mismatch in the math
                #if self.angle_error < -135 :
                #    self.angle_error = 180 - ((180 + (self.angle_error - 45) + 1) * -1)
                
                #turn to face the target
                if np.abs(self.angle_error) > 0.1:
                    rospy.loginfo(self.angle_error)
                    #rospy.loginfo(self.pose.theta_velocity)
                    if self.pose.theta_velocity > self.angle_error * 0.02 and self.angle_error * 0.02 > 0:
                        wrench.torque.z = -np.abs(self.angle_error)
                    elif self.pose.theta_velocity < np.abs(self.angle_error) * 0.02:
                        wrench.torque.z = np.abs(self.angle_error)
                    wrench.torque.z = np.clip(wrench.torque.z, -.3, .3)
                    
            else :
                self.countdown += 1
                rospy.loginfo(self.countdown)
            if self.countdown > 1500 : 
                self.countdown = 0
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
    #nav.run()
