#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('robot_tf_listener')

    listener = tf.TransformListener()

    #turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        #print "running"
        print listener.getFrameStrings()
        (trans,rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
        try:
            (trans,rot) = listener.lookupTransform('/odom', '/neato', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "no"
            continue

        print "trans 0" + str(trans[0])
        print "trans 1" + str(trans[1])
        #angular = 4 * math.atan2(trans[1], trans[0])
        #linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        #cmd = geometry_msgs.msg.Twist()
        #cmd.linear.x = linear
        #cmd.angular.z = angular
        #turtle_vel.publish(cmd)

        rate.sleep()