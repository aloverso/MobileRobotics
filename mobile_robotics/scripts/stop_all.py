#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import sys, tty, termios

def teleop ():
    #publisher writes  - here, to cmd_vel
    pub1 = rospy.Publisher ('/robot1/cmd_vel', Twist, queue_size=10) #name of topic, typer of topic ("rostopic type /cmd_vel")
    pub2 = rospy.Publisher ('/robot2/cmd_vel', Twist, queue_size=10) #name of topic, typer of topic ("rostopic type /cmd_vel")
    pub3 = rospy.Publisher ('/robot3/cmd_vel', Twist, queue_size=10) #name of topic, typer of topic ("rostopic type /cmd_vel")

    rospy.init_node ('teleop', anonymous=True)
    r = rospy.Rate (10) # 10hz
    while not rospy.is_shutdown ():

        msg = Twist (Vector3 (0.0, 0.0, 0.0), Vector3 (0.0, 0.0, 0.0)) #first vector3 is linear, second is angular, find this syntax "rosmsg show geometry_msgs/Twist"
        #msg = Twist (angular=Vector3 (z=2.0)) #this works because python by default sets the other args to zero
        pub1.publish (msg)
        pub2.publish (msg)
        pub3.publish (msg)
        r.sleep ()
        
if __name__ == '__main__':
    try:
        teleop ()
    except rospy.ROSInterruptException: pass
