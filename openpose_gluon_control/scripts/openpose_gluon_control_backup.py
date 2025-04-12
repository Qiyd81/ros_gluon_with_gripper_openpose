#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import moveit_commander
import numpy as np
from geometry_msgs.msg import PoseArray, PoseStamped
from tf2_geometry_msgs import do_transform_pose

class OpenPoseControl:
    def __init__(self):
        # MoveIt初始化
        moveit_commander.roscpp_initialize([])
        
        # 创建MoveGroup接口
        self.arm = moveit_commander.MoveGroupCommander("gluon_arm")
        self.arm.set_max_velocity_scaling_factor(0.2)  # 安全速度限制
        
        # TF监听器初始化
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # OpenPose订阅者
        self.pose_sub = rospy.Subscriber("/openpose/poses", PoseArray, self.pose_callback)
        
        # 安全参数
        self.joint_limits = {
            'min': [-3.14, -1.57, -3.14, -1.57, -3.14, -3.14],
            'max': [3.14, 1.57, 3.14, 1.57, 3.14, 3.14]
        }

    def pose_callback(self, msg):
        try:
            # 获取坐标变换
            transform = self.tf_buffer.lookup_transform(
                "base_link",   # 目标坐标系
                msg.header.frame_id,  # 源坐标系
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            
            # 遍历所有关节点（示例处理右手）
            for idx, pose in enumerate(msg.poses):
                if idx == 4:  # 假设第4个点是右手
                    target_pose = self.transform_pose(pose, transform)
                    if self.check_safety(target_pose):
                        self.move_to_pose(target_pose)

        except (tf2_ros.LookupException, 
                tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as ex:
            rospy.logwarn("TF Error: %s", ex)

    def transform_pose(self, pose, transform):
        """坐标变换到机械臂基座坐标系"""
        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.frame_id = transform.header.frame_id
        return do_transform_pose(pose_stamped, transform).pose

    def check_safety(self, pose):
        """安全检查"""
        # 碰撞检测（需根据实际配置扩展）
        current_joints = self.arm.get_current_joint_values()
        return True  # 此处应实现实际安全检查

    def move_to_pose(self, target_pose):
        """执行运动规划"""
        self.arm.set_pose_target(target_pose)
        
        # 运动规划
        plan = self.arm.plan()
        if not plan.joint_trajectory.points:
            rospy.logerr("Planning failed for current pose")
            return False
        
        # 执行运动
        try:
            self.arm.execute(plan, wait=True)
            rospy.loginfo("Movement executed successfully")
            return True
        except moveit_commander.MoveItCommanderException as ex:
            rospy.logerr("Execution failed: %s", ex)
            return False

if __name__ == '__main__':
    rospy.init_node('openpose_gluon_control')
    controller = OpenPoseControl()
    rospy.spin()
