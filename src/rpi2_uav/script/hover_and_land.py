#!/usr/bin/env python

from __future__ import print_function

import rospy
from std_msgs.msg import String
import numpy as np
import time
import os
import sys
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, CommandTOL
from math import radians, degrees, pi, cos, sin
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
import time
import threading



class simple_uav():
    def __init__(self, uav_name):
        self.uav_name = uav_name
        self.current_state = None
        self.current_pos = None
        self.home_pos = None
        self.stopThread = False
        self.targetPos = None

        # all subscriber
        rospy.Subscriber('/'+self.uav_name+'/mavros/state', State, self.state_cb)
        rospy.Subscriber('/'+self.uav_name+'/mavros/local_position/pose', PoseStamped, self.local_position_cb)
        # rospy.Subscriber('/vicon/home/home', TransformStamped, self.home_cb)

        # all publisher
        self.targetPosPub = rospy.Publisher('/'+self.uav_name+'/mavros/setpoint_position/local', PoseStamped, queue_size=100)
        self.fakePosPub = rospy.Publisher('/'+self.uav_name+'/mavros/mocap/tf', TransformStamped, queue_size=100)


    def publishFakePos(self):
        seq = 0
        rate = rospy.Rate(60)
        fakepos = TransformStamped()
        fakepos.header.frame_id = 'world'
        fakepos.header.seq = seq
        fakepos.header.stamp = rospy.Time.now()
        fakepos.child_frame_id = ''
        fakepos.transform.translation.x = 0
        fakepos.transform.translation.y = 0
        fakepos.transform.translation.z = 0
        fakepos.transform.rotation.x = 0
        fakepos.transform.rotation.y = 0
        fakepos.transform.rotation.z = 0
        fakepos.transform.rotation.w = -1

        while not self.stopThread:
            seq += 1
            fakepos.header.seq = seq
            fakepos.header.stamp = rospy.Time.now()
            self.fakePosPub.publish(fakepos)
            rate.sleep()

    def state_cb(self, msg):
        self.current_state = msg


    def set_arm(self, arm, timeout):
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        rospy.wait_for_service('/'+self.uav_name+'/mavros/cmd/arming')
        arming_srv = rospy.ServiceProxy('/'+self.uav_name+'/mavros/cmd/arming', CommandBool)
        for i in xrange(timeout * loop_freq):
            if current_state.armed == arm:
                return True
            else:
                try:
                    res = arming_srv(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                    else:
                        return True
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            try:
                rate.sleep()
            except rospy.ROSException as e:
                rospy.logerr(e)
        return False            

    def land(self, timeout):
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        rospy.wait_for_service('/'+self.uav_name+'/mavros/cmd/land')
        land_set = rospy.ServiceProxy('/'+self.uav_name+'/mavros/cmd/land', CommandTOL)
        
        for i in xrange(timeout * loop_freq):
            if current_state.armed == False:
                return True
            else:
                try:
                    res = land_set(min_pitch=0,
                             yaw=0,
                             latitude=0,
                             longitude=0,
                             altitude=0)  # 0 is custom mode
                    # res = land_set()
                    print(res)
                    if not res.success:
                        rospy.logerr("failed to send land command")
                    else:
                        return True
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                print(e)

        return False
        
    def set_mode(self, mode, timeout):
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        rospy.wait_for_service('/'+self.uav_name+'/mavros/set_mode')
        mode_set = rospy.ServiceProxy('/'+self.uav_name+'/mavros/set_mode', SetMode)
        for i in xrange(timeout * loop_freq):
            if current_state.mode == mode:
                return True
            else:
                try:
                    res = mode_set(0,mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                    else:
                        return True
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        return False

    def local_position_cb(self, msg):
        self.current_pos = [msg.pose.position.x, 
                            msg.pose.position.y, 
                            msg.pose.position.z]

    def getquaternion(self, roll, pitch, yaw):
        # return tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        roll = radians(roll)
        pitch = radians(pitch)
        yaw = radians(yaw)
        cy = cos(yaw * 0.5);
        sy = sin(yaw * 0.5);
        cp = cos(pitch * 0.5);
        sp = sin(pitch * 0.5);
        cr = cos(roll * 0.5);
        sr = sin(roll * 0.5);

        q=[0,0,0,0]
        q[0] = -1*(cy * cp * sr - sy * sp * cr)
        q[1] = -1*(sy * cp * sr + cy * sp * cr)
        q[2] = -1*(sy * cp * cr - cy * sp * sr)
        q[3] = -1*(cy * cp * cr + sy * sp * sr)
        
        return q;

    def home_cb(self, msg):
        self.home_pos = [msg.transform.translation.x, 
                         msg.transform.translation.y,
                         msg.transform.translation.z]
        

    def setTargetPos(self,x,y,z):
        if self.targetPos is None:
            self.targetPos = PoseStamped()
        self.targetPos.pose.position.x = x
        self.targetPos.pose.position.y = y
        self.targetPos.pose.position.z = z

    def sendTargetPos(self):
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and not self.stopThread:
            if self.targetPos is not None:
                self.targetPosPub.publish(self.targetPos)
            rate.sleep()

    def takeoff(self):
        if self.set_mode('OFFBOARD', 5) and self.set_arm(True, 5):
            rospy.sleep(10)
            return True
        return False

    def land_and_disarm(self):
        try:
            self.land(5)
        except Exception as e:
            print(e)
        print('landed not disarm yet')
        rospy.sleep(3.5)
        try:
            self.set_arm(False, 5)
        except Exception as e:
            print(e)
                

if __name__ == '__main__':
    rospy.init_node('rpi2_test_scenario', anonymous=True)
    rate = rospy.Rate(5.0)
    uav = simple_uav("rpi2")
#################
    threading.Thread(target=uav.publishFakePos).start()
    while True:
        rate.sleep()
        cmd = raw_input("command: \"l\" for land, then \"q\" for exit the main loop: ")
        if cmd=='l':
            uav.stopThread = True
            rospy.sleep(3)
            break
###################
'''
    uav.setTarget(0.92, 4.75, 1.5)
    threading.Thread(target=uav.sendTargetPos).start()
    rospy.sleep(10)
    if uav.takeoff():
        print("takeoff success!!!")
        while not rospy.is_shutdown():
            rate.sleep()
            cmd = raw_input("command: \"l\" for land, then \"q\" for exit the main loop: ")
            if cmd=='l':
                uav.stopThread = True
                uav.land_and_disarm()
            elif cmd=='q':
                uav.set_arm(False, 5)
                break
    else:
        print("failed takeoff!!!")

'''

        
