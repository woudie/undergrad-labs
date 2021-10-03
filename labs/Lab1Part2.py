#!/usr/bin/env python
# coding: utf-8

# In[ ]:


import pprint
import math
import rospy

from copy import deepcopy
from baxter_interface import (RobotEnable, Gripper, Limb)

from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from std_msgs.msg import Header
from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)
pp = pprint.PrettyPrinter(indent=4)


# In[ ]:


rospy.init_node('Group7Baxter')


# In[ ]:


baxter = RobotEnable()
baxter.enable()


# In[ ]:


right_arm = Limb('right')


# In[ ]:


fingers = Gripper('right')
fingers.calibrate()


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




