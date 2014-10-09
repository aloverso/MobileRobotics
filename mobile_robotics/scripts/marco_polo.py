#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3

class Robot:
    def __init__(self,robotname):
        self.robotname = robotname
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.pub = rospy.Publisher("/%s/cmd_vel"%self.robotname, Twist, queue_size=10)

    def run(self):
        #print "running"
        self.broadcaster.sendTransform(
            (0.33655, 0.0, 0.0), 
            (0, 0, 0, 1),
            rospy.Time.now(), 
            "/%s/odom"%self.robotname,
            "/world")
        try:
           (trans,rot) = self.listener.lookupTransform(
            "/world", 
            "/%s/base_link"%self.robotname, 
            rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, 
            tf.ExtrapolationException):
            pass

    def publish_twist_velocity (self,x,a):
        msg = Twist (Vector3 (x, 0, 0), Vector3 (0, 0, a))
        self.pub.publish (msg)

class Marco(Robot):
    def __init__(self, polo_names):
        Robot.__init__(self, "marco")
        self.polo_names = polo_names
        self.last_call_time = rospy.get_time()
        self.startup_time = rospy.get_time()

    def run(self):
        Robot.run(self)
        if (rospy.get_time() - self.startup_time >= 5):
            angle_to_polo = 0
            #print rospy.get_time()
            #rospy.Timer(rospy.Duration(5), my_callback)
            #print now.secs
            if rospy.get_time() - self.last_call_time >= 5: #call every 5 seconds
                self.last_call_time = rospy.get_time()
                angle_to_polo = self.call_marco()
            #Robot.publish_twist_velocity(self, 0, angle_to_polo)
            
    def call_marco(self):
        closest_polo_dist = 1000000
        angle_to_polo = 0
        for polo_name in self.polo_names:
            try:
                (trans,rot) = self.listener.lookupTransform(
                    "/%s/base_link"%self.robotname, 
                    "/%s/base_link"%polo_name,
                     rospy.Time(0))
                radius = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
                angular = math.atan2(trans[1], trans[0])
                if radius < closest_polo_dist:
                    angle_to_polo = angular
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                pass
        return angle_to_polo

# Polo needs to start directly to Marco's right 
class Polo(Robot):
    def __init__(self, robotname):
        Robot.__init__(self,robotname)

    def get_force(self):
        # Boundry vector
        boundry_size = 10 #meters
        try:
            (trans,rot) = self.listener.lookupTransform(
                "/%s/base_link"%self.robotname, 
                "/world",
                 rospy.Time(0))
            dist = (trans[0]**2 + trans[1]^2)**0.5
            angl = math.atan2(trans[1],trans[0])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        b_weight = 1.0*(dist)**2 # gets higher towards boundry
        b_dir = angl

        # marco vector
        try:
            (trans, rot) = self.listener.lookupTransform(
                "/%s/base_link"%self.robotname,
                "/marco/base_link",
                rospy.Time(0))
            dist = (trans[0]**2 + trans[1]^2)**0.5
            angl = math.atan2(trans[1],trans[0])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        m_weight = 1.0*dist
        m_dir = angl 

        # add vectors
        t_x = b_weight*math.cos(b_dir)+m_weight*math.cos(m_dir)
        t_y = b_weight*math.sin(b_dir)+m_weight*math.sin(m_dir)


    def run(self):
        Robot.run(self)
        # find where marco is relative to self
        angular = 0
        radius = 1
        try:
            (trans,rot) = self.listener.lookupTransform(
                "/%s/base_link"%self.robotname, 
                "/marco/base_link",
                 rospy.Time(0))
            print tf.transformations.euler_from_quaternion(rot)
            #radius = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            angular = math.atan2(trans[1], trans[0])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        coef = (1 if angular > 0 else -1)
        angle_away_from_marco = angular + 3.14159*coef # 180 degrees away from where it sense marco
        speed = 0.5/radius # the closer marco is, the faster polo moves
        #Robot.publish_twist_velocity(self, speed, angle_away_from_marco) 

if __name__ == '__main__':
    rospy.init_node('marco_polo', anonymous=True)

    #polo_names = ["polo1", "polo2"]
    polo_names = ["polo"]
    marco = Marco(polo_names)
    polo = Polo("polo")

    while not rospy.is_shutdown():
        marco.run()
        polo.run()