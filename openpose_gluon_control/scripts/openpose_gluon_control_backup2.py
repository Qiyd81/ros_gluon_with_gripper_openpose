#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys  # 添加此行
import rospy
import tf2_ros
import moveit_commander
import numpy as np
#from geometry_msgs.msg import PoseArray, PoseStamped
import threading
#from openpose import pyopenpose as op
from collections import deque
from tf2_geometry_msgs import do_transform_pose
from tf2_geometry_msgs import do_transform_point
from ros_openpose.msg import Frame  # 根据实际消息类型调整
from geometry_msgs.msg import PointStamped

class OpenPoseControl:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('openpose_gluon_control', anonymous=True)
        
	# MoveIt初始化
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 创建MoveGroup接口
        self.arm = moveit_commander.MoveGroupCommander("gluon_arm")
        self.arm.set_max_velocity_scaling_factor(0.2)  # 安全速度限制
        self.arm.set_planning_time(10.0)               # 增加规划时间
        self.arm.set_num_planning_attempts(20)        # 增加尝试次数

        # 初始化需要使用move group控制的机械臂中的gripper group
        self.gripper = moveit_commander.MoveGroupCommander('gluon_gripper')
        
        # 设置机械臂和夹爪的允许误差值
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)
        #gripper.set_goal_joint_tolerance(0.001)
        
        # 控制机械臂先回到初始化位置
        self.arm.set_named_target('home_zero')
        self.arm.go()
        rospy.sleep(2)
        # 注释 OpenPose 数据驱动的代码，手动设置目标
        self.arm.set_position_target([0.2, 0.2, 0.2])  # 示例坐标
        success = self.arm.go(wait=True)
        rospy.loginfo("手动目标规划结果: %s", success)

        # 设置夹爪的目标位置，并控制夹爪运动
        #gripper.set_joint_value_target([-0.2])
        #gripper.go()
        #rospy.sleep(1)

        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        #joint_positions = [-0.5, -0.5, 0.52832, 0.5820, -0.5, -0.5]
        #arm.set_joint_value_target(joint_positions)
                 
        # 控制机械臂完成运动
        #arm.go()
        #rospy.sleep(1)
        
        # 关闭并退出moveit
        #moveit_commander.roscpp_shutdown()
        #moveit_commander.os._exit(0)
        
        # TF监听器初始化
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # OpenPose订阅者
        self.pose_sub = rospy.Subscriber('/frame', Frame, self.pose_callback)
        
	#poseModel = op.PoseModel.BODY_25
	#print(op.getPoseBodyPartMapping(poseModel))
	#print(op.getPoseNumberBodyParts(poseModel))
	#print(op.getPosePartPairs(poseModel))
	#print(op.getPoseMapIndex(poseModel))
        # 安全参数
        self.joint_limits = {
            'min': [-3.14, -1.57, -3.14, -1.57, -3.14, -3.14],
            'max': [3.14, 1.57, 3.14, 1.57, 3.14, 3.14]
        }

        self.filtered_x = 0.0
        self.filtered_y = 0.0
        self.filtered_z = 0.0
        self.alpha = 0.2  # 滤波系数（0~1，越小越平滑）
        # 工作空间限制（单位：米）
        self.x_min, self.x_max = -0.5, 0.5
        self.y_min, self.y_max = -0.5, 0.5

        self.z_min, self.z_max = -0.1, 0.6

        # 目标队列与线程同步
        self.target_queue = deque()
        self.current_target = None
        self.lock = threading.Lock()
        
        # 启动控制线程
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.start()
	# 人体关节名称
	BODY_PARTS = {
	    0:  "Nose",
	    1:  "Neck",
	    2:  "RShoulder",
	    3:  "RElbow",
	    4:  "RWrist",
	    5:  "LShoulder",
	    6:  "LElbow",
	    7:  "LWrist",
	    8:  "MidHip",
	    9:  "RHip",
	    10: "RKnee",
	    11: "RAnkle",
	    12: "LHip",
	    13: "LKnee",
	    14: "LAnkle",
	    15: "REye",
	    16: "LEye",
	    17: "REar",
	    18: "LEar",
	    19: "LBigToe",
	    20: "LSmallToe",
	    21: "LHeel",
	    22: "RBigToe",
	    23: "RSmallToe",
	    24: "RHeel",
	    25: "Background"
	}

    def pose_callback(self, msg):
        if not msg.persons:
            rospy.logwarn("未检测到人体")
            return
        person = msg.persons[0]
        #for part in person.bodyParts:
	for i in range(0,len(person.bodyParts),1):
            # 获取摄像头坐标系下的坐标
	    part = person.bodyParts[i]
	    part_id = i
	    part_name = BODY_PARTS.get(part_id, "Unknown")
            x_cam = part.point.x
            y_cam = part.point.y
            z_cam = part.point.z

            # 转换到机械臂坐标系
            try:
                transform = self.tf_buffer.lookup_transform(
                    "dummy", # 目标坐标系：机械臂基坐标系
                    "camera_link",  # 源坐标系：摄像头坐标系
                    #msg.header.frame_id,
                    rospy.Time(0),
                    timeout=rospy.Duration(1.0)
                )
                point_cam = PointStamped()
                point_cam.header = msg.header
                point_cam.point.x = x_cam
                point_cam.point.y = y_cam
		if (z_cam <= self.z_max ):
                    point_cam.point.z = z_cam 
		else:
		    point_cam.point.z = self.z_max
                point_arm = do_transform_point(point_cam, transform)

                # 打印原始和变换后坐标
                rospy.loginfo("原始坐标: x=%.2f, y=%.2f, z=%.2f", part.point.x, part.point.y, part.point.z)
                rospy.loginfo("变换后坐标: x=%.2f, y=%.2f, z=%.2f", point_arm.point.x, point_arm.point.y, point_arm.point.z)

                # ...  滤波处理 ...
                self.filtered_x = self.alpha * point_arm.point.x + (1 - self.alpha) * self.filtered_x
                self.filtered_y = self.alpha * point_arm.point.y + (1 - self.alpha) * self.filtered_y
                self.filtered_z = self.alpha * point_arm.point.z + (1 - self.alpha) * self.filtered_z
                rospy.loginfo("滤波后坐标: x=%.2f, y=%.2f, z=%.2f", self.filtered_x, self.filtered_y, self.filtered_z)
		if (self.filtered_z <= self.z_max ):
                    pass 
		else:
		    self.filtered_z = self.z_max
                #self.arm.set_position_target([self.filtered_x, self.filtered_y, self.filtered_z])
                # 控制机械臂移动（示例：移动到右手腕）
                #if "RightWrist" in str(part):
                #self.arm.set_position_target([point_arm.point.x, point_arm.point.y, point_arm.point.z])
                # 在 pose_callback 中设置目标姿态
                # 检查坐标是否在合理范围内
                if (self.x_min <= self.filtered_x <= self.x_max) and \
                   (self.y_min <= self.filtered_y <= self.y_max) and \
                   (self.z_min <= self.filtered_z <= self.z_max):
                       # 设置目标位姿（继承当前姿态）                 
                    target_pose = self.arm.get_current_pose().pose  # 获取当前姿态
                    #target_pose.position.x = point_arm.point.x
                    #target_pose.position.y = point_arm.point.y
                    #target_pose.position.z = point_arm.point.z
                    target_pose.position.x = self.filtered_x
                    target_pose.position.y = self.filtered_y
                    target_pose.position.z = self.filtered_z


                    # 将坐标加入队列
                    with self.lock:
                        self.target_queue.append((
                            target_pose.position.x,
                            target_pose.position.y,
                            target_pose.position.z
                        ))
                        rospy.loginfo("新目标加入队列: (%.2f, %.2f, %.2f)", 
                                 target_pose.position.x, target_pose.position.y, target_pose.position.z)

                    #self.arm.set_pose_target(target_pose)  # 设置完整位姿

                    # 执行运动
                    #success = self.arm.go(wait=True)
                    #rospy.loginfo("运动规划结果: %s", success)
                    ##self.arm.go(wait=True)
                else:
                    rospy.logwarn("目标位置超出工作空间限制: x=%.2f, y=%.2f, z=%.2f", self.filtered_x, self.filtered_y, self.filtered_z)

            #except tf2_ros.TransformException as e:
            #    rospy.logwarn("TF 错误: {}".format(e))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr("坐标变换失败: {}".format(e))

    def control_loop(self):
        rate = rospy.Rate(10)  # 控制频率 10Hz
        while not rospy.is_shutdown():
            if self.current_target is None:
                with self.lock:
                    if len(self.target_queue) > 0:
                        self.current_target = self.target_queue.popleft()
                        rospy.loginfo("开始移动到目标: %s", self.current_target)
            
            if self.current_target is not None:
                success = self.move_to_target(self.current_target)
                if success:
                    rospy.loginfo("已到达目标: %s", self.current_target)
                    self.current_target = None
                else:
                    rospy.logwarn("运动规划失败，跳过目标: %s", self.current_target)
                    self.current_target = None
            rate.sleep()

    def move_to_target(self, target):
        try:
            target_pose = self.arm.get_current_pose().pose
            target_pose.position.x = target[0]
            target_pose.position.y = target[1]
            target_pose.position.z = target[2]
            self.arm.set_pose_target(target_pose)
            return self.arm.go(wait=True)
        except Exception as e:
            rospy.logerr("运动执行异常: {}".format(e))
            return False


#    def check_safety(self, pose):
#        """安全检查"""
#        # 碰撞检测（需根据实际配置扩展）
#        current_joints = self.arm.get_current_joint_values()
#        return True  # 此处应实现实际安全检查

#    def move_to_pose(self, target_pose):
        """执行运动规划"""
#        self.arm.set_pose_target(target_pose)
        
        # 运动规划
#        plan = self.arm.plan()
#        if not plan.joint_trajectory.points:
#            rospy.logerr("Planning failed for current pose")
#            return False
        
        # 执行运动
#        try:
#            self.arm.execute(plan, wait=True)
#            rospy.loginfo("Movement executed successfully")
#            return True
#        except moveit_commander.MoveItCommanderException as ex:
#            rospy.logerr("Execution failed: %s", ex)
#            return False
    def shutdown(self):
	self.control_thread.join()
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    #rospy.init_node('openpose_gluon_control_test')
    try:
        controller = OpenPoseControl()
        rospy.spin()
        controller.shutdown()
    except rospy.ROSInterruptException:
        pass

