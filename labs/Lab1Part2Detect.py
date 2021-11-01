#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import os
import sys
import math
sys.path.insert(0, '../libraries')

import pprint
import rospy

from copy import deepcopy
from baxter_interface import (RobotEnable, Gripper, CameraController, Limb)
from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)

import cv2
import cv_bridge
from cv_bridge import (CvBridge, CvBridgeError)

from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from sensor_msgs.msg import (Image, Range)
from std_msgs.msg import Header

import requests
import cv2
import matplotlib.pyplot as plt
import numpy as np

from uogbaxter.detect import detect_toy

pp = pprint.PrettyPrinter(indent=4)
bridge = CvBridge()


# In[ ]:


rospy.init_node('Group7Baxter')


# In[ ]:


baxter = RobotEnable()
baxter.enable()


# In[ ]:


right_hand_camera = CameraController('right_hand_camera')


# In[ ]:


right_hand_camera.resolution = (960, 600)
right_hand_camera.gain = 0
right_hand_camera.open()


# In[ ]:


right_arm = Limb('right')


# In[ ]:


fingers = Gripper('right')
fingers.calibrate()


# ### Baxter Detect Class

# In[ ]:


class BaxterDetect(object):
    def __init__(self):
        self.distance = 0
        sensor_topic = "/robot/range/right_hand_range/state"
        right_hand_image_topic = '/cameras/right_hand_camera/image'
        display_topic = '/robot/xdisplay'
        
        self.__right_sensor = rospy.Subscriber(sensor_topic, Range, callback=self.__sensorCallback, queue_size=1)
        right_hand_image_subscriber = rospy.Subscriber(right_hand_image_topic, Image, callback=self.__processImageCallback, queue_size=1)
        self.__display_publisher = rospy.Publisher(display_topic, Image, queue_size=1)
        
    def __sensorCallback(self,msg,side):
       self.distance = msg.range

    def __processImageCallback(self, message):
        # convert ROS iamge to OpenCV image
        cv2_image = bridge.imgmsg_to_cv2(message)
        
        # this is where we do our image processing. 
        # We're just going to draw a red rectangle and the distance we previously got from our distance sensor
        # note, that in OpenCV, colors are represented as (blue, green, red), rather than RGB
        cv2.rectangle(
                      cv2_image,
                      pt1=(280,200),
                      pt2=(680,400),
                      color=(0,0,255),
                      thickness=5
                      )
        cv2.putText(
                    cv2_image,
                    text='%.2f' % self.distance,
                    org=(400,500),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=2,
                    color=(255,255,0)
                    )

        # convert OpenCV image back to ROS image
        ros_image = bridge.cv2_to_imgmsg(cv2_image)
        
        # publish our modified image to the display
        self.__display_publisher.publish(ros_image)

    def __findToy(self, img):
        try:
        
        except (TimeoutError, NewConnectionError, MaxRetryError, ConnectionError):
            print("Detection not possible, the SOE detection server could not be reached....")
        except:
            print("No animals found")


# In[ ]:


baxter_detect = BaxterDetect()


# In[ ]:


right_namespace = "ExternalTools/right/PositionKinematicsNode/IKService"

right_ik_service = rospy.ServiceProxy(right_namespace, SolvePositionIK)

right_ik_request = SolvePositionIKRequest()

ik_header = Header(stamp=rospy.Time.now(), frame_id='base')


# In[ ]:


def adjust_pose(target_pose):
    adj_p1 = deepcopy(target_pose)
    adj_p1.pose.position.x -= 0.15
    adj_p1.pose.position.z += 0.02
    return adj_p1


# In[ ]:


zero_pose = PoseStamped(
            header=ik_header,
            pose=Pose(
                position=Point(
                    x=0.5,
                    y=-0.5,
                    z=0.6,
                ),
                orientation=Quaternion(
                    x=0.0,
                    y=0.0,
                    z=0.0,
                    w=1.0,
                )
            )
        )

starting_pose = PoseStamped(
                header=ik_header,
                pose=Pose(
                    position=Point(
                        x=0.575814771825969, y=-0.6921240261798756, z=0.132303617877802
                    ),
                orientation=Quaternion(
                    x=-0.035401679659970944, y=0.7351025065602724, z=-0.011401826130588908, w=0.6769350222044543
                )
            )
        )


top_left = PoseStamped(
            header=ik_header,
            pose=Pose(
                position=Point(
                    x=0.7157861729277576, y=-0.5944180233461482, z=0.14473098504048154
                ),
                orientation=Quaternion(
                    x=-0.035401679659970944, y=0.7351025065602724, z=-0.011401826130588908, w=0.6769350222044543
                )
            )
        )

top_right = PoseStamped(
            header=ik_header,
            pose=Pose(
                position=Point(
                    x=0.7282098926508587, y=-0.8043508833787663, z=0.1451601697469714
                ),
                orientation=Quaternion(
                    x=-0.035401679659970944, y=0.7351025065602724, z=-0.011401826130588908, w=0.6769350222044543
                )
            )
        )

bottom_left = PoseStamped(
            header=ik_header,
            pose=Pose(
                position=Point(
                    x=0.7134527406891848, y=-0.5891747770147832, z=-0.05405738045318162
                ),
                orientation=Quaternion(
                    x=-0.051997633390138645, y=0.8097438412456411, z=-0.025234618335036367, w=0.5839301085952527
                )
            )
        )

bottom_right = PoseStamped(
            header=ik_header,
            pose=Pose(
                position=Point(
                    x=0.7249046758051918, y=-0.7983831869923568, z=-0.042637299029533254
                ),
                orientation=Quaternion(
                    x=-0.051997633390138645, y=0.8097438412456411, z=-0.025234618335036367, w=0.5839301085952527
                )
            )
        )

basket_pose = PoseStamped(
            header=ik_header,
            pose=Pose(
                position=Point(
                    x=-0.18960557257166022, y=-0.8013329235692273, z=0.07650624118442936
                ),
                orientation=Quaternion(
                    x=0.40813023278127375, y=1.9362437364362493, z=-0.2597175943373065, w=0.47320359766165804
                ),
            ),
        )

adj_pose = adjust_pose(top_left)
kangaroo = [starting_pose, adj_pose, top_left, 'reached', adj_pose, starting_pose, basket_pose, 'end']

adj_pose = adjust_pose(top_right)
hippo = [starting_pose, adj_pose, top_right, 'reached', adj_pose, starting_pose, basket_pose, 'end']

adj_pose = adjust_pose(bottom_left)
deer = [starting_pose, adj_pose, bottom_left, 'reached', adj_pose, starting_pose, basket_pose, 'end']

adj_pose = adjust_pose(bottom_right)
lion = [starting_pose, adj_pose, bottom_right, 'reached', adj_pose, starting_pose, basket_pose, 'end']

animals = {'kangaroo': kangaroo, 'hippo': hippo, 'deer': deer, 'lion': lion}


# In[ ]:


def find_solution(target_pose):
    right_ik_request.pose_stamp[:] = []
    right_ik_request.pose_stamp.append(target_pose)

    print right_ik_request.pose_stamp
    try:
        rospy.wait_for_service(right_namespace, 5.0)
        right_ik_response = right_ik_service(right_ik_request)
        if (right_ik_response.isValid[0]):
            right_limb_joints = dict(zip(right_ik_response.joints[0].name, right_ik_response.joints[0].position))
            pp.pprint(right_limb_joints)
            return right_limb_joints
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))


# In[ ]:


def execute_trajectory(target_joints):
    right_arm.move_to_joint_positions(target_joints)


# In[ ]:


user_choices = []
choices = animals.keys()

print("Please select two animals of your choice")
for i in range(2):
    user_input = raw_input('Animal ' + str(i+1) + ': ').lower()

    if(user_input in choices):
        user_choices.append(user_input)
        print(str(user_input) + " successfully selected!")
    else:
        print("Invalid choice, please try again.")


# In[ ]:


success = []
for choice in user_choices:
    
    selected = animals[choice]

    for pose in selected:
        if(pose == 'reached'):
            fingers.close()
            print("Picking up the " + str(choice) + " now!")
        elif(pose == 'end'):
            fingers.open()
            print("The retrieval of the " + str(choice) + " has been completed!")
        else:
            solution = find_solution(pose)
            if(solution):
                execute_trajectory(solution)
            else:
                print("No valid pose was found, terminating retrieval of ....")
                print("Try homing the arm and try the retrieval again!")
                break;
    success.append(choice)


# In[ ]:


failed = [choice for choice in user_choices if choice not in success]

print("============================\n      RETRIEVAL SUMMARY\n============================")

for animal in success:
    print("The retrieval of the " + animal + " was successful!")

for animal in failed:
    print("The retrieval of the " + animal + " failed....")
    


# In[ ]:




