#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg

class Marco:
    def __init__(self, polo_names):
        self.robotname = "marco"
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.polos = 

    def run(self):
        self.broadcaster.sendTransform((0.0, 0.0, 0.0), (0, 0, 0, 1),
            rospy.Time.now(), "/"+self.robotname+"/odom","/world")
        try:
           (trans,rot) = self.listener.lookupTransform("/world", "/"+self.robotname+"/base_link", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        self.call_marco()

    def call_marco(self):
        for polo_name in self.polo_names:
            (trans,rot) = self.listener.lookupTransform("/"+self.robotname+"/odom", "/"+polo_name+"/base_link",) rospy.Time(0))
            radius = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            #if radius <
            angular = 4 * math.atan2(trans[1], trans[0])
            print angular

# Polo needs to start directly to Marco's right 
class Polo:
    def __init__(self, robotname):
        self.robotname = robotname
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

    def run(self):
        self.broadcaster.sendTransform((0.33655, 0.0, 0.0), (0, 0, 0, 1),
            rospy.Time.now(), "/"+self.robotname+"/odom","/world")
        try:
           (trans,rot) = self.listener.lookupTransform("/world", "/"+self.robotname+"/base_link", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

if __name__ == '__main__':
    rospy.init_node('marco_polo_robots')

    polo_names = ["polo1", "polo2"]
    marco = Marco(polo_names)
    polo = Polo("polo")

    while not rospy.is_shutdown():
        marco.run()
        polo.run()