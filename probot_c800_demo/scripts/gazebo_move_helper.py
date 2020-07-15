#!/usr/bin/env python
import sys, os, rospy
import moveit_commander

print "Welcome to use gazebo move helper CLI..."

rospy.init_node('gazebo_move_helper')
arm = moveit_commander.MoveGroupCommander("manipulator")
grp = moveit_commander.MoveGroupCommander("gripper")
end_effector_link = arm.get_end_effector_link()

joint_tolerance_default = 0.001
arm.set_goal_joint_tolerance(joint_tolerance_default)
arm.set_max_acceleration_scaling_factor(0.3)
arm.set_max_velocity_scaling_factor(0.3)

pi = 3.14159
# ready to grasp
grasp_joint_pos = [2.4620431877943734, -0.39921600321909523, -0.423846277396712, 0.7160794148946339, -1.5661350448578082, 1.5484128398829382]
#ready to release
release_joint_pos = [1.4434078944765583, -0.20132406313248463, -0.42200197210922497, 0.9460313489593952, -1.5359659373918353, 0.5380832002536708]

small_step = 0.01
large_step = 0.05
small_degree = 1.0 / 180 * pi
large_degree = 5.0 / 180 * pi
degree = 0
while True:
	print "CMD> "
	line = sys.stdin.readline()
	pose = arm.get_current_pose().pose
	if line == "quit\n":
		print "quit!!!"
		break

	elif line == "joint\n":
		print "print joint positions!!!"
		os.system("rostopic echo -n 1 /joint_states | grep position")

	elif line == "home\n":
		print "goto home!!!"
		arm.set_named_target('home')
		arm.go(wait=True)
	elif line == "grasp\n":
		print "goto grasp!!!"
		arm.set_joint_value_target(grasp_joint_pos)
		arm.go(wait=True)
	elif line == "release\n":
		print "goto release!!!"
		arm.set_joint_value_target(release_joint_pos)
		arm.go(wait=True)

	elif line == "cc\n":
		print "close gripper!!!"
                degree +=0.1
		grp.set_joint_value_target([degree])
		grp.go(wait=True)
	elif line == "oo\n":
		print "open gripper!!!"
                degree +=0.005
                print degree
		grp.set_joint_value_target([degree])
		grp.go(wait=True)

	elif line == "x+\n":
		pose.position.x += small_step
		arm.set_pose_target(pose)
		arm.go(wait=True)
	elif line == "x++\n":
		pose.position.x += large_step
		arm.set_pose_target(pose)
		arm.go(wait=True)
	elif line == "y+\n":
		pose.position.y += small_step
		arm.set_pose_target(pose)
		arm.go(wait=True)
	elif line == "y++\n":
		pose.position.y += large_step
		arm.set_pose_target(pose)
		arm.go(wait=True)
	elif line == "z+\n":
		pose.position.z += small_step
		arm.set_pose_target(pose)
		arm.go(wait=True)
	elif line == "z++\n":
		pose.position.z += large_step
		arm.set_pose_target(pose)
		arm.go(wait=True)
	elif line == "x-\n":
		pose.position.x -= small_step
		arm.set_pose_target(pose)
		arm.go(wait=True)
	elif line == "x--\n":
		pose.position.x -= large_step
		arm.set_pose_target(pose)
		arm.go(wait=True)
	elif line == "y-\n":
		pose.position.y -= small_step
		arm.set_pose_target(pose)
		arm.go(wait=True)
	elif line == "y--\n":
		pose.position.y -= large_step
		arm.set_pose_target(pose)
		arm.go(wait=True)
	elif line == "z-\n":
		pose.position.z -= small_step
		arm.set_pose_target(pose)
		arm.go(wait=True)
	elif line == "z--\n":
		pose.position.z -= large_step
		arm.set_pose_target(pose)
		arm.go(wait=True)
