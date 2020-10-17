# c800-vacuum-husky
实现c800机械臂与husky小车联合移动抓取在gazebo内的仿真。
（需替换myworld.world文件内box的路径名称；
不同版本的box模型大小可能不一致，需调节甲爪的张合程度）
**操作步骤**
1.catkin_make
2.source devel/setup.bash
3.roslaunch huskyArm_with_gripper_moveit_config demo_gazebo.launch
4.rosrun huskyArm_with_gripper_moveit_config move_and_pick.py
