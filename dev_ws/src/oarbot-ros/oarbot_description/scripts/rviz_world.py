#! /usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped,TransformStamped
from tf2_ros.transform_listener import TransformListener

class baseState(object):
    def __init__(self) -> None:
        super().__init__()

        rospy.sleep(rospy.Duration(3))
        self.pose_sub = rospy.Subscriber("oarbot_base_pose",PoseStamped,self.pose_cb,queue_size=1)

        # tf
        self.br = tf2_ros.TransformBroadcaster()
        self.now_t = TransformStamped()
        self.now_t.header.frame_id = "world"
        self.now_t.child_frame_id = "summit_base_footprint"
        self.now_t.transform.rotation.w = 1

        # timer
        self.timer = rospy.Timer(rospy.Duration(0.01),self.tf_timer)

    def tf_timer(self, event):
        
        if self.now_t is None:
            return

        self.now_t.header.stamp = rospy.Time.now()
        self.br.sendTransform(self.now_t)

    def pose_cb(self, msg):

        t = TransformStamped()
        t.header = msg.header
        t.header.frame_id = "world"
        t.child_frame_id = "summit_base_footprint"
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.now_t = t

if __name__ == '__main__':
    rospy.init_node('oarbot_base_pose')
    p = baseState()
    rospy.spin()