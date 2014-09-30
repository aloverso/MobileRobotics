#!/usr/bin/env python  
import roslib
import rospy

import tf
"""
def handle_robot_pose(msg, robotname):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0),
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(),
                     robotname,
                     "world")

if __name__ == '__main__':
    rospy.init_node('robot_tf_broadcaster')
    robotname = rospy.get_param('~robot')
    rospy.Subscriber('/%s/pose' % robotname,
                     turtlesim.msg.Pose,
                     handle_robot_pose,
                     robotname)
    rospy.spin()
"""
if __name__ == '__main__':
    rospy.init_node('robot_tf_broadcaster')
    rate = rospy.Rate(10)

    broadcaster = tf.TransformBroadcaster

    while not rospy.is_shutdown():
        broadcaster.sendTransform(
            
                tf.Transform(tf.Quaternion(0, 0, 0, 1), tf.Vector3(0.1, 0.0, 0.2)),
            rospy.Time.now(),"base_link", "base_laser")
        r.sleep()