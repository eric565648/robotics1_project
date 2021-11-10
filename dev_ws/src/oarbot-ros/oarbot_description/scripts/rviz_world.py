#! /usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped,TransformStamped

def pose_cb(msg):
    
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header = msg.header
    t.header.frame_id = "world"
    t.child_frame_id = "summit_base_link"
    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = msg.pose.position.z
    t.transform.rotation = msg.pose.orientation

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('oarbot_base_pose')
    rospy.Subscriber("oarbot_base_pose",PoseStamped,pose_cb,queue_size=1)
    rospy.spin()