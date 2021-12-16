#!/usr/bin/env python3
import rospy
import numpy as np
from kinova_msgs.msg import JointVelocity
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger, TriggerResponse
from math import pi
from copy import deepcopy as dp

def inpi(angles):
    angles_inpi = np.zeros(len(angles))
    i=0
    for ang in angles:
        while ang > pi:
            ang -= 2*pi
        while ang <= -pi:
            ang += 2*pi
        angles_inpi[i]=ang
        i += 1
    return angles_inpi

class JointControl:
    def __init__(self):
        
        # parameters
        self.vel_rate = 100
        self.vel_rate_ = rospy.Rate(self.vel_rate)
        self.file_folder = "/home/oarbot_silver/eric/robotics1_project/data/happy_motion_100/"
        
        self.waypoint_rate = 10
        self.Kp = 1./self.waypoint_rate

        self.j_zeros = np.array([pi,pi,pi/2,0,0,0])

        # variables
        self.joint_state = None

        # subscriber
        self.joint_sub = rospy.Subscriber("/j2n6s300_driver/out/joint_state",JointState,self.joint_state_cb,queue_size=1)

        # publisher
        self.vel_pub = rospy.Publisher("/j2n6s300_driver/in/joint_velocity",JointVelocity,queue_size=1)

        # trajectory service
        self.motion_srv = rospy.Service('motions',Trigger,self.motion_srv_cb)
    
    def joint_state_cb(self, msg):
        self.joint_state = msg
    
    def motion_srv_cb(self, req):

        if self.joint_state is None:
            return TriggerResponse(
                success=False,
                message="Current joint state not known")

        path=np.genfromtxt(self.file_folder+str(1)+'.csv',delimiter=',',dtype=float)

        print(len(path[1,:]))
        # move to starting pose
        start_q = path[4:10,0]
        start_q[0] *= -1
        start_q += self.j_zeros
        print(start_q)
        last_estimate = np.array(self.joint_state.position[0:6])
        print(last_estimate)
        print(np.rad2deg(inpi(start_q-last_estimate)))
        while True:
            last_estimate = np.array(self.joint_state.position[0:6])
            dx = np.rad2deg(inpi(start_q-last_estimate))
            if np.sum(np.abs(dx)) < 5:
                break
            #vel_d = self.Kp*(dx)
            vel_d = 10*dx
            vel_msg = JointVelocity()
            vel_msg.joint1 = vel_d[0]
            vel_msg.joint2 = vel_d[1]
            vel_msg.joint3 = vel_d[2]
            vel_msg.joint4 = vel_d[3]
            vel_msg.joint5 = vel_d[4]
            vel_msg.joint6 = vel_d[5]
            self.vel_pub.publish(vel_msg)

            self.vel_rate_.sleep()

        print("Ready to start")
        rospy.sleep(5)


        change_num = self.vel_rate/self.waypoint_rate
        change_count = 0
        path_i = 0
        last_vel = 0
        while True:
            this_target = dp(path[4:10,path_i])
            this_target[0] *= -1
            this_target += self.j_zeros
            last_estimate = np.array(self.joint_state.position[0:6])
            dx = np.rad2deg(inpi(this_target-last_estimate))
            vel_d = self.Kp*(dx)
            vel_msg = JointVelocity()
            vel_msg.joint1 = vel_d[0]
            vel_msg.joint2 = vel_d[1]
            vel_msg.joint3 = vel_d[2]
            vel_msg.joint4 = vel_d[3]
            vel_msg.joint5 = vel_d[4]
            vel_msg.joint6 = vel_d[5]
            self.vel_pub.publish(vel_msg)

            change_count += 1
            if change_count >= change_num:
                path_i += 1
                change_count = 0
            if path_i >= len(path[1,:]):
                break
            
            self.vel_rate_.sleep()
        
        print("Trajectory done")
        return TriggerResponse(
            success=True,
            message="Trajectory Following done")

def main():
    
    rospy.init_node('joint_trajectory_follower')
    jc = JointControl()
    rospy.spin()

if __name__ == '__main__':
    main()

            
