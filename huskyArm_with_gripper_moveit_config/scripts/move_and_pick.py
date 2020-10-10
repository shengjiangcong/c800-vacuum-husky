#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 Wuhan PS-Micro Technology Co., Itd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Twist

class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        pub = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
        # 初始化ROS节点
        rospy.init_node('move_and_pick')
        rate = rospy.Rate(10) 
        msg = Twist()        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('manipulator')

        # 初始化需要使用move group控制的机械臂中的gripper group
        gripper = moveit_commander.MoveGroupCommander('gripper')
                
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)
        gripper.set_goal_joint_tolerance(0.001)
       
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.3)
        arm.set_max_velocity_scaling_factor(0.3)
        gripper.set_max_velocity_scaling_factor(0.08)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)

        #for i in range(1,2):
           # msg.linear.x = 0.3
           # msg.linear.y = 0
           # msg.linear.z = 0
            #msg.angular.x = 0
            #msg.angular.y = 0
            #msg.angular.z = 0
            #pub.publish(msg)
            #print i
            #rate.sleep()

               
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = 0.0124
        target_pose.pose.position.y = 0.6
        target_pose.pose.position.z = 0.35
        target_pose.pose.orientation.x = 0.707
        target_pose.pose.orientation.y = 0
        target_pose.pose.orientation.z = 0
        target_pose.pose.orientation.w = 0.707
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)

        # 规划运动路径
        traj = arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(1)

        target_pose.pose.position.z = 0.2
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)

        # 规划运动路径
        traj = arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(1)


        # 设置夹爪的目标位置，并控制夹爪运动
        gripper.set_joint_value_target([0.6])
        gripper.go()
        rospy.sleep(1)


        target_pose.pose.position.z = 0.3
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)

        # 规划运动路径
        traj = arm.plan()
        
        # 按照规划的运动路径控制机械臂运动
        arm.execute(traj)
        rospy.sleep(1)
        # 控制机械臂终端向右移动5cm
        # arm.shift_pose_target(1, 0.1, end_effector_link)
        # arm.go()
        # rospy.sleep(1)
  
        # 控制机械臂终端反向旋转90度
        # arm.shift_pose_target(3, -1.57, end_effector_link)
        # arm.go()
        # rospy.sleep(1)
           
        # 控制机械臂回到初始化位置
        #arm.set_named_target('home')
        #arm.go()
        for i in range(1,95):
            msg.linear.x = 0.3
            msg.linear.y = 0
            msg.linear.z = 0
            msg.angular.x = 0
            msg.angular.y = 0
            msg.angular.z = 0
            pub.publish(msg)
            rate.sleep()

        rospy.sleep(1)
        gripper.set_joint_value_target([0])
        gripper.go()
        rospy.sleep(1)
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()

    
    
