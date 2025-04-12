#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import moveit_commander
import numpy as np
#from geometry_msgs.msg import PoseArray, PoseStamped
from tf2_geometry_msgs import do_transform_pose
from ros_openpose.msg import Frame  # 根据实际消息类型调整
from geometry_msgs.msg import PointStamped

class OpenPoseControl:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('openpose_gluon_control_test', anonymous=True)
        
	# MoveIt初始化
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 创建MoveGroup接口
        self.arm = moveit_commander.MoveGroupCommander("gluon_arm")
        self.arm.set_max_velocity_scaling_factor(0.2)  # 安全速度限制

        # 初始化需要使用move group控制的机械臂中的gripper group
        gripper = moveit_commander.MoveGroupCommander('gluon_gripper')
        
        # 设置机械臂和夹爪的允许误差值
        arm.set_goal_joint_tolerance(0.001)
        #gripper.set_goal_joint_tolerance(0.001)
        
        # 控制机械臂先回到初始化位置
        arm.set_named_target('home_zero')
        arm.go()
        rospy.sleep(2)

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
        self.pose_sub = rospy.Subscriber('/ros_openpose/frame', Frame, self.pose_callback)
        
        # 安全参数
        self.joint_limits = {
            'min': [-3.14, -1.57, -3.14, -1.57, -3.14, -3.14],
            'max': [3.14, 1.57, 3.14, 1.57, 3.14, 3.14]
        }

    def pose_callback(self, msg):
        if not msg.persons:
            return
        person = msg.persons[0]
        for part in person.bodyParts:
            # 获取摄像头坐标系下的坐标
            x_cam = part.point.x
            y_cam = part.point.y
            z_cam = part.point.z

            # 转换到机械臂坐标系
            try:
                transform = self.tf_buffer.lookup_transform(
                    "arm_base",
                    msg.header.frame_id,
                    rospy.Time(0),
                    timeout=rospy.Duration(1.0)
                )
                point_cam = PointStamped()
                point_cam.header = msg.header
                point_cam.point.x = x_cam
                point_cam.point.y = y_cam
                point_cam.point.z = z_cam
                point_arm = do_transform_point(point_cam, transform)

                # 控制机械臂移动（示例：移动到右手腕）
                #if "RightWrist" in str(part):
                self.arm.set_position_target([point_arm.point.x, point_arm.point.y, point_arm.point.z])
                self.arm.go(wait=True)

            except tf2_ros.TransformException as e:
                rospy.logwarn(f"TF 错误: {e}")


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
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    #rospy.init_node('openpose_gluon_control_test')
    controller = OpenPoseControl()
    rospy.spin()
    controller.shutdown()
