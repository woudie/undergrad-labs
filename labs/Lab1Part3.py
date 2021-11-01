#!/usr/bin/env python
# coding: utf-8

# In[1]:


import pprint
import math
import rospy
import baxter_interface

from copy import copy
from baxter_interface import (RobotEnable, Gripper)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion)
from std_msgs.msg import Header
from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest)
pp = pprint.PrettyPrinter(indent=4)


# In[2]:


rospy.init_node('Group7Baxter')


# In[ ]:


baxter = RobotEnable()
baxter.enable()


# In[ ]:


right_arm = baxter_interface.Limb('right')


# In[ ]:


fingers = Gripper('right')
fingers.calibrate()


# In[ ]:




