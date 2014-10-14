#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist, Vector3, PoseStamped, Pose, Point, Quaternion
from tf.transformations import euler_from_quaternion

class Robot:
    def __init__(self,robotname,origin_transform):
        self.robotname = robotname
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.pub = rospy.Publisher("/%s/cmd_vel"%self.robotname, Twist, queue_size=10)
        self.pose_publisher = rospy.Publisher("/%s/posestamped"%self.robotname, PoseStamped, queue_size=10)
        self.origin_transform = origin_transform
        self.startup_time = rospy.get_time()


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
        self.broadcaster.sendTransform(
            (0.0, self.origin_transform, 0.0), 
            (0, 0, 0, 1),
            rospy.Time.now(), 
            "/%s/odom"%self.robotname,
            "/world")
        (trans,rot) = self.get_transform("/world","/%s/base_link"%self.robotname)
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
        self.switch_states = False
        self.switch_with = None
        self.goal = (0,0,0)

    def run(self):
        Robot.run(self)
        if (rospy.get_time() - self.startup_time >= 10):
            '''
            if rospy.get_time() - self.last_call_time >= 5: #call every 5 seconds
                self.last_call_time = rospy.get_time() 
                self.call_marco()
            (trans,rot) = Robot.get_transform(self, 
                "/%s/odom"%self.robotname, 
                "/%s/base_link"%self.robotname)
            dx = self.goal[0] - trans[0]
            dy = self.goal[1] - trans[1]
            angles = euler_from_quaternion(rot)
            angle_to_goal = math.atan2(dy,dx) + angles[2]+2*math.pi
            if angle_to_goal>math.pi:
                angle_to_goal-=2*math.pi
            print angle_to_goal*180/3.14
            linear = 2
            '''
            polo = self.call_marco()
            (trans,rot) = Robot.get_transform(self, 
                "/%s/base_link"%self.robotname, 
                "/%s/base_link"%polo)
            angle = 0.5*math.atan2(trans[1],trans[0])
            linear = 0.5*math.sqrt(trans[0]**2 + trans[1]**2)
            
            Robot.publish_twist_velocity(self, linear, angle)
            #self.check_tagged_polo()

    def call_marco(self):
        closest_polo_dist = 1000000
        #angle_to_polo = 0
        closest_poloname = ""
        for polo in self.polos:
            polo_name = polo.robotname
            (trans,rot) = Robot.get_transform(self, 
                "/%s/base_link"%self.robotname, 
                "/%s/base_link"%polo_name)
            radius = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            if radius < closest_polo_dist:
                closest_polo_dist = radius
                closest_poloname = polo_name
            self.update_goal(closest_poloname)
        return closest_poloname

    def update_goal(self, closest_poloname):
        (trans,rot)= Robot.get_transform(self,
            "/%s/odom"%self.robotname, 
            "/%s/base_link"%closest_poloname)
        self.goal = trans

    def check_tagged_polo(self):
        tagging_radius = 0.5
        tagged_polo = None
        for polo in self.polos:
            polo_name = polo.robotname
            (trans,rot) = Robot.get_transform(self, 
                "base_link"%self.robotname, 
                "/%s/base_link"%polo_name)
            radius = math.sqrt(trans[0] ** 2 + trans[1] ** 2)
            if radius <  tagging_radius:
                tagged_polo = polo
                break
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
        
        b_weight = dist**4/boundry_size**4
        b_dir = -angl
        dist = 0
        angl = 0
        # marco vector
        (trans,rot) = Robot.get_transform(self, 
            "/%s/base_link"%self.robotname, 
            "/%s/base_link"%self.marco.robotname)
        dist = (trans[0]**2 + trans[1]**2)**0.5
        angl = math.atan2(trans[1],trans[0])
        
        m_weight = max((-dist + 2*boundry_size)/(4*boundry_size),0)
        m_dir = angl 

        # add vectors
        sum_x = b_weight*math.cos(b_dir)+m_weight*math.cos(m_dir)
        sum_y = b_weight*math.sin(b_dir)+m_weight*math.sin(m_dir)

        return sum_x,sum_y
        

    def run(self):
       # print self.robotname + " polo running"
        Robot.run(self)
        if (rospy.get_time() - self.startup_time >= 5):
            x,y = self.get_force()
           # print self.robotname, x,y
            # Move based on force:
            tran = 0
            angl = 0
            if y > 0:
                tran = max(y*2,0.2)
            if x >=0:
                angl = -max(x*3,1)
            else:
                angl = max(x*3,1)
            Robot.publish_twist_velocity(self, tran, angl) 


robot_name_transforms = {"robot1":0, "robot2":0.36, "robot3":-0.36} #length at least 2
robots = []
polos = []


def switch_roles(ex_marco, ex_polo):
    #print "switching"
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
