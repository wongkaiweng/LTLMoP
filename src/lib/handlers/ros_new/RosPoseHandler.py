#!/usr/bin/env python
import rospy, math
from numpy import *
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion

from gazebo_msgs.srv import *
from gazebo_msgs.msg import *

import os, sys
# Climb the tree to find out where we are
p = os.path.abspath(__file__)
t = ""
while t != "src":
    (p, t) = os.path.split(p)
    if p == "":
        print "I have no idea where I am; this is ridiculous"
        sys.exit(1)

sys.path.append(os.path.join(p,"src","lib"))

# logger for ltlmop
import logging
ltlmop_logger = logging.getLogger('ltlmop_logger')

"""
=======================================
rosPose.py - ROS Interface Pose Handler
=======================================
"""

import lib.handlers.handlerTemplates as handlerTemplates

class RosPoseHandler(handlerTemplates.PoseHandler):
    def __init__(self, executor, shared_data, modelName="ego"):
        """
        Pose Handler for ROS and gazebo.

        modelName (str): The model name of the robot in gazebo to get the pose information from (default="pr2")
        """

        #GetModelState expects the arguments model_name and relative_entity_name
        #In this case it is pr2 and world respectively but can be changed for different robots and environments
        self.model_name = modelName
        self.relative_entity_name = 'world' #implies the gazebo global coordinates
        self.last_pose = None

        self.shared_data=shared_data['ROS_INIT_HANDLER']

    def getPose(self,cached = False):
        if (not cached) or self.last_pose is None:
            #Ros service call to get model state
            #This returns a GetModelStateResponse, which contains data on pose
            rospy.wait_for_service('/gazebo/get_model_state')
            try:
                gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
                resp = gms(self.model_name,self.relative_entity_name)
                #Cartesian Pose
                self.pos_x = resp.pose.position.x
                self.pos_y = resp.pose.position.y
                self.pos_z = resp.pose.position.z
                #Quaternion Orientation
                self.or_x = resp.pose.orientation.x
                self.or_y = resp.pose.orientation.y
                self.or_z = resp.pose.orientation.z
                self.or_w = resp.pose.orientation.w

            except rospy.ServiceException, e:
                ltlmop_logger.error("Service call failed: %s"%e)
            #  Use the tf module transforming quaternions to euler
            try:
                angles = euler_from_quaternion([self.or_x, self.or_y, self.or_z, self.or_w])
                self.theta = angles[2]
                shared=self.shared_data
                #The following accounts for the maps offset in gazebo for
                #initial region placement
                self.last_pose = array([self.pos_x, self.pos_y, self.theta, self.pos_z])

            except Exception:
                ltlmop_logger.error('Pose Broke', Exception)

        # map to pixels?
        return self.last_pose
