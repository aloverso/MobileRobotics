#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Pose, Point, Quaternion

class Robot:
    def __init__(self,robotname,origin_transform):
        self.robotname = robotname
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.pub = rospy.Publisher("/%s/cmd_vel"%self.robotname, Twist, queue_size=10)
        self.pose_publisher = rospy.Publisher("/%s/posestamped"%self.robotname, PoseStamped, queue_size=10)
        self.origin_transform = origin_transform

    def get_transform(self, from_frame, to_frame):
        trans = (0,0,0)
        rot = (0,0,0,1)
        try:
           (trans,rot) = self.listener.lookupTransform(
            from_frame, 
            to_frame, 
            rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, 
            tf.ExtrapolationException):
            print ":( from %s to %s"%(from_frame,to_frame)
            pass
        return (trans, rot)

    def run(self):
        #print self.robotname + str(self.origin_transform)
        self.broadcaster.sendTransform(
            (0.0, self.origin_transform, 0.0), 
            (0, 0, 0, 1),
            rospy.Time.now(), 
            "/%s/odom"%self.robotname,
            "/world")
        (trans,rot) = self.get_transform("/world","/%s/base_link"%self.robotname)
        '''
        trans = (0,0,0)
        rot = (0,0,0,1)
        try:
           (trans,rot) = self.listener.lookupTransform(
            "/world", 
            "/%s/base_link"%self.robotname, 
            rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, 
            tf.ExtrapolationException):
            print ":( my world baselink tf"
            pass
            '''
        msg = PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id="world"), pose=Pose(position=Point(x=trans[0],y=trans[1],z=trans[2]), orientation=Quaternion(x=rot[0],y=rot[1],z=rot[2],w=rot[3])))
        self.pose_publisher.publish(msg)

    def publish_twist_velocity (self,x,a):
        msg = Twist (Vector3 (x, 0, 0), Vector3 (0, 0, a))
        self.pub.publish (msg)

    def __eq__(self, other):
        if self.robotname == other.robotname:
            return True
        else:
            return False

class Marco(Robot):
    def __init__(self, robotname, origin_transform):
        Robot.__init__(self, robotname, origin_transform)
        self.polos = []
        self.last_call_time = rospy.get_time()
        self.startup_time = rospy.get_time()
        self.switch_states = False
        self.switch_with = None

    def run(self):
        print self.robotname + " marco running"
        Robot.run(self)
        if (rospy.get_time() - self.startup_time >= 5):
            angle_to_polo = 0
            #print rospy.get_time()
            #rospy.Timer(rospy.Duration(5), my_callback)
            #print now.secs
            if rospy.get_time() - self.last_call_time >= 5: #call every 5 seconds
                self.last_call_time = rospy.get_time() 
                angle_to_polo = self.call_marco()
            print angle_to_polo
            Robot.publish_twist_velocity(self, 0.2, 0.2*angle_to_polo)
            #self.check_tagged_polo()

    def call_marco(self):
        closest_polo_dist = 1000000
        angle_to_polo = 0
        for polo in self.polos:
            polo_name = polo.robotname
            (trans,rot) = Robot.get_transform(self, "/%s/base_link"%self.robotname, "%s/base_link"%polo_name)
            radius = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            angular = math.atan2(trans[1], trans[0])
            if radius < closest_polo_dist:
                radius = closest_polo_dist
                angle_to_polo = angular
            '''
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
                print "D: tf b/w marco and polo call"
                pass
                '''
        return angle_to_polo

    def check_tagged_polo(self):
        tagging_radius = 0.5
        tagged_polo = None
        for polo in self.polos:
            polo_name = polo.robotname
            (trans,rot) = Robot.get_transform(self, "base_link"%self.robotname, "/%s/base_link"%polo_name)
            radius = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            if radius <  tagging_radius:
                tagged_polo = polo
                break
            '''
            try:
                (trans,rot) = self.listener.lookupTransform(
                    "/%s/base_link"%self.robotname, 
                    "/%s/base_link"%polo_name,
                     rospy.Time(0))
                radius = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
                if radius <  tagging_radius:
                    tagged_polo = polo
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print "-_- check tagged polo"
                pass
                '''
        if polo:
            self.switch_states = True
            self.switch_with = polo

# Polo needs to start directly to Marco's right 
class Polo(Robot):
    def __init__(self, robotname, origin_transform):
        Robot.__init__(self,robotname, origin_transform)
        self.marco = None

    def get_force(self):
        # Boundry vector
        boundry_size = 2 #meters
        dist = 0
        angl = 0
        (trans,rot) = Robot.get_transform(self, "/%s/base_link"%self.robotname, "/world")
        dist = (trans[0]**2 + trans[1]**2)**0.5
        angl = math.atan2(trans[1],trans[0])
        '''
        try:
            (trans,rot) = self.listener.lookupTransform(
                "/world",
                "/%s/base_link"%self.robotname, 
                 rospy.Time(0))
            dist = (trans[0]**2 + trans[1]**2)**0.5
            angl = math.atan2(trans[1],trans[0])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print ":S polo get force"
            pass
            '''
        b_weight = dist**4/boundry_size**4
        b_dir = angl
        dist = 0
        angl = 0
        # marco vector
        (trans,rot) = Robot.get_transform(self, "/%s/base_link"%self.robotname, "/%s/base_link"%self.marco.robotname)
        dist = (trans[0]**2 + trans[1]**2)**0.5
        angl = math.atan2(trans[1],trans[0])
        '''
        try:
            (trans, rot) = self.listener.lookupTransform(
                "/%s/base_link"%self.robotname,
                "/%s/base_link"%self.marco.robotname,
                rospy.Time(0))
            dist = (trans[0]**2 + trans[1]**2)**0.5
            angl = math.atan2(trans[1],trans[0])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "no marco :|"
            pass
            '''
        m_weight = (-dist + 2*boundry_size)/(4*boundry_size)
        m_dir = angl 

        # add vectors
        sum_x = b_weight*math.cos(b_dir)+m_weight*math.cos(m_dir)
        sum_y = b_weight*math.sin(b_dir)+m_weight*math.sin(m_dir)

        return sum_x,sum_y
        

    def run(self):
        print self.robotname + " polo running"
        Robot.run(self)       
        x,y = self.get_force()
        # Move based on force:
        tran = 0
        angl = 0
        if y > 0:
            tran = max(y*2,0.2)
        if x >=0:
            angl = -max(x*2,0.2)
        else:
            angl = max(x*2,0.2)
        Robot.publish_twist_velocity(self, tran, angl) 


robot_name_transforms = {"robot1":0, "robot2":0.34, "robot3":-0.34} #length at least 2
robots = []
polos = []


def switch_roles(ex_marco, ex_polo):
    print "switching"
    ex_marco_name = ex_marco.robotname
    ex_polo_name = ex_polo.robotname
    for robot in robots:
        if robot == ex_polo or robot == ex_marco:
            robots.remove(robot)
    for polo in polos:
        if polo == ex_polo:
            polos.remove(polo)
    new_polo = Polo(ex_marco_name)
    polos.append(polo)
    robots.append(polo)
    new_marco = Marco(ex_polo_name)
    new_marco.polos = polos
    robots.append(marco)
    for polo in polos:
        polo.marco = new_marco

if __name__ == '__main__':
    rospy.init_node('marco_polo', anonymous=True)
    marco = None
    #polo_names = ["polo1", "polo2"]
    for robot_name in robot_name_transforms:
        if robot_name=="robot1":
            marco = Marco(robot_name, robot_name_transforms[robot_name])
            robots.append(marco)
        else:
            polo = Polo(robot_name, robot_name_transforms[robot_name])
            robots.append(polo)
            polos.append(polo)

    marco.polos = polos
    for polo in polos:
        polo.marco = marco
    while not rospy.is_shutdown():
        for robot in robots:
            robot.run()
            if isinstance(robot,Marco) and robot.switch_states:
                switch_roles(robot, robot.switch_with)
