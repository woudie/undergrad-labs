#!/usr/bin/env python
# coding: utf-8

# In[1]:


import pprint
import math
import rospy
import baxter_interface

from baxter_interface import RobotEnable
from baxter_interface import Gripper
pp = pprint.PrettyPrinter(indent=4)


# In[3]:


rospy.init_node('Group7Baxter')


# In[ ]:


baxter = RobotEnable()
baxter.enable()


# >> rosrun baxter_examples joint_recorder.py -f watch_me.rec

# In[11]:


fingers = Gripper('right')
fingers.calibrate()

vacuum = Gripper('left')
vacuum_sensor = baxter_interface.AnalogIO('left_vacuum_sensor_analog')


# In[26]:


fingers.close()


# In[25]:


vacuum.close(timeout=6)


# In[27]:


fingers.open()


# In[28]:


vacuum.open()


# >> rosrun baxter_interface joint_trajectory_action_server.py

# >> rosrun baxter_examples joint_trajectory_file_playback.py -f watch_me.rec
