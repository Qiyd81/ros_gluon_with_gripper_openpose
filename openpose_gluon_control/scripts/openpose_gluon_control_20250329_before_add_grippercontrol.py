#!/usr/bin/env python
# -*- coding: utf-8 -*-

#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import tf2_ros
import threading
import moveit_commander
from tf2_geometry_msgs import do_transform_point
from ros_openpose.msg import Frame
from geometry_msgs.msg import PointStamped, Quaternion
import tf
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose
from moveit_msgs.msg import Constraints, OrientationConstraint
from std_msgs.msg import Header
import datetime

class OpenPoseControl:
    def __init__(self):
        rospy.init_node('openpose_gluon_control')
        
        # MoveIt 初始化
        moveit_commander.roscpp_initialize(sys.argv)
        self.arm = moveit_commander.MoveGroupCommander("gluon_arm")
        # 设置参考坐标系为 base_link
        self.arm.set_pose_reference_frame("base_link")
        #self.arm.set_position_target(target_pose, end_effector_link="gripper_base_link")
        self._set_orientation_constraint()  # 添加姿态约束

        # 定义补偿后的固定四元数（抵消 gripper_base_link 的初始旋转）
        self.fixed_orientation = quaternion_from_euler(0, 0, 0)  # 绕 Y 轴 -90 度
        rospy.loginfo("四元数坐标: x=%.2f, y=%.2f, z=%.2f, w=%.2f (rad)", self.fixed_orientation[0], self.fixed_orientation[1], self.fixed_orientation[2], self.fixed_orientation[3])


        self.arm.set_max_velocity_scaling_factor(0.2)
        self.arm.set_planning_time(15.0)
        self.arm.set_num_planning_attempts(10)
        self.arm.set_planner_id("RRTConnect")  # 使用 RRTConnect 规划器

        # 控制机械臂先回到初始化位置
        self.arm.set_named_target('home_zero')
        self.arm.go()
        rospy.sleep(2)
        # 获取当前位置
        home_pose = self.arm.get_current_pose().pose
        # 提取四元数 (顺序为 [x, y, z, w])
        quaternion = [
            home_pose.orientation.x,
            home_pose.orientation.y,
            home_pose.orientation.z,
            home_pose.orientation.w
        ]

        # 将四元数转换为欧拉角（弧度）
        roll, pitch, yaw = euler_from_quaternion(quaternion)        
        rospy.loginfo("零位坐标: x=%.2f, y=%.2f, z=%.2f | 欧拉角: roll=%.2f, pitch=%.2f, yaw=%.2f (rad)", home_pose.position.x, home_pose.position.y, home_pose.position.z, roll, pitch, yaw)

        # 注释 OpenPose 数据驱动的代码，手动设置目标
        self.arm.set_position_target([0.08, 0.1, 0.3, 0.0, 0.0, 0.0, 1])  # 示例坐标
        success = self.arm.go(wait=True)
        rospy.loginfo("手动目标规划结果1: %s", success)
        rospy.loginfo("Planning frame: %s", self.arm.get_planning_frame())
        print("get_end_effector_link:", self.arm.get_end_effector_link())

        pose_1 = self.arm.get_current_pose().pose
        # 提取四元数 (顺序为 [x, y, z, w])
        quaternion_1 = [
            pose_1.orientation.x,
            pose_1.orientation.y,
            pose_1.orientation.z,
            pose_1.orientation.w
        ]

        # 将四元数转换为欧拉角（弧度）
        roll_1, pitch_1, yaw_1 = euler_from_quaternion(quaternion_1)        
        rospy.loginfo("1位坐标: x_1=%.2f, y_1=%.2f, z_1=%.2f | 欧拉角: roll_1=%.2f, pitch_1=%.2f, yaw_1=%.2f (rad)", pose_1.position.x, pose_1.position.y, pose_1.position.z, roll_1, pitch_1, yaw_1)

        rospy.sleep(2)

        # self.arm.set_position_target([0.08, 0.2, 0.2, 0.0, 0.0, 0.0, 1])  # 示例坐标
        # success = self.arm.go(wait=True)
        # rospy.loginfo("手动目标规划结果2: %s", success) 

        # pose_2 = self.arm.get_current_pose().pose
        # # 提取四元数 (顺序为 [x, y, z, w])
        # quaternion_2 = [
        #     pose_2.orientation.x,
        #     pose_2.orientation.y,
        #     pose_2.orientation.z,
        #     pose_2.orientation.w
        # ]

        # # 将四元数转换为欧拉角（弧度）
        # roll_2, pitch_2, yaw_2 = euler_from_quaternion(quaternion_2)        
        # rospy.loginfo("2位坐标: x_2=%.2f, y_2=%.2f, z_2=%.2f | 欧拉角: roll_2=%.2f, pitch_2=%.2f, yaw_2=%.2f (rad)", pose_2.position.x, pose_2.position.y, pose_2.position.z, roll_2, pitch_2, yaw_2)

        # self.arm.set_position_target([0.08, 0.3, 0.1, 0.0, 0.0, 0.0, 1])  # 示例坐标
        # success = self.arm.go(wait=True)
        # rospy.loginfo("手动目标规划结果3: %s", success)  

        # pose_3 = self.arm.get_current_pose().pose
        # # 提取四元数 (顺序为 [x, y, z, w])
        # quaternion_3 = [
        #     pose_3.orientation.x,
        #     pose_3.orientation.y,
        #     pose_3.orientation.z,
        #     pose_3.orientation.w
        # ]

        # # 将四元数转换为欧拉角（弧度）
        # roll_3, pitch_3, yaw_3 = euler_from_quaternion(quaternion_3)        
        # rospy.loginfo("3位坐标: x_3=%.2f, y_3=%.2f, z_3=%.2f | 欧拉角: roll_3=%.2f, pitch_3=%.2f, yaw_3=%.2f (rad)", pose_3.position.x, pose_3.position.y, pose_3.position.z, roll_3, pitch_3, yaw_3)

        # self.arm.set_position_target([0.08, 0.0, 0.5, 0.0, 0.0, 0.0, 1])  # 示例坐标
        # success = self.arm.go(wait=True)
        # rospy.loginfo("手动目标规划结果4: %s", success) 

        # pose_4 = self.arm.get_current_pose().pose
        # # 提取四元数 (顺序为 [x, y, z, w])
        # quaternion_4 = [
        #     pose_4.orientation.x,
        #     pose_4.orientation.y,
        #     pose_4.orientation.z,
        #     pose_4.orientation.w
        # ]

        # # 将四元数转换为欧拉角（弧度）
        # roll_4, pitch_4, yaw_4 = euler_from_quaternion(quaternion_4)        
        # rospy.loginfo("4位坐标: x_4=%.2f, y_4=%.2f, z_4=%.2f | 欧拉角: roll_4=%.2f, pitch_4=%.2f, yaw_4=%.2f (rad)", pose_4.position.x, pose_4.position.y, pose_4.position.z, roll_4, pitch_4, yaw_4)

        # self.arm.set_position_target([0.08, -0.1, 0.3, 0.0, 0.0, 0.0, 1])  # 示例坐标
        # success = self.arm.go(wait=True)
        # rospy.loginfo("手动目标规划结果5: %s", success)

        # pose_5 = self.arm.get_current_pose().pose
        # # 提取四元数 (顺序为 [x, y, z, w])
        # quaternion_5 = [
        #     pose_5.orientation.x,
        #     pose_5.orientation.y,
        #     pose_5.orientation.z,
        #     pose_5.orientation.w
        # ]

        # # 将四元数转换为欧拉角（弧度）
        # roll_5, pitch_5, yaw_5 = euler_from_quaternion(quaternion_5)        
        # rospy.loginfo("5位坐标: x_5=%.2f, y_5=%.2f, z_5=%.2f | 欧拉角: roll_5=%.2f, pitch_5=%.2f, yaw_5=%.2f (rad)", pose_5.position.x, pose_5.position.y, pose_5.position.z, roll_5, pitch_5, yaw_5)

        # self.arm.set_position_target([0.08, -0.3, 0.1, 0.0, 0.0, 0.0, 1])  # 示例坐标
        # success = self.arm.go(wait=True)
        # rospy.loginfo("手动目标规划结果6: %s", success) 

        # pose_6 = self.arm.get_current_pose().pose
        # # 提取四元数 (顺序为 [x, y, z, w])
        # quaternion_6 = [
        #     pose_6.orientation.x,
        #     pose_6.orientation.y,
        #     pose_6.orientation.z,
        #     pose_6.orientation.w
        # ]

        # # 将四元数转换为欧拉角（弧度）
        # roll_6, pitch_6, yaw_6 = euler_from_quaternion(quaternion_6)        
        # rospy.loginfo("6位坐标: x_6=%.2f, y_6=%.2f, z_6=%.2f | 欧拉角: roll_6=%.2f, pitch_6=%.2f, yaw_6=%.2f (rad)", pose_6.position.x, pose_6.position.y, pose_6.position.z, roll_6, pitch_6, yaw_6)

        # 控制机械臂先回到初始化位置
        self.arm.set_named_target('home_zero')
        self.arm.go()
        rospy.sleep(2)             
        
        # TF 初始化
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 订阅 OpenPose 话题
        self.pose_sub = rospy.Subscriber('/frame', Frame, self.pose_callback, queue_size=1) #加入 queue_size=1,避免缓存过多消息。
        
        # # 初始化滤波参数和工作空间限制
        # self.filtered_x = 0.0
        # self.filtered_y = 0.0
        # self.filtered_z = 0.0
        # self.alpha = 0.2  # 滤波系数
        self.x_min, self.x_max = -0.2, 0.2
        self.y_min, self.y_max = -0.3, 0.3
        self.z_min, self.z_max = 0.1, 0.4
        
        # 线程安全控制
        self.is_processing = False  # 标志位
        self.lock = threading.Lock()  # 线程锁
        self.BODY_PARTS = {
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
        self.i = 0 #设置参数i，以循环遍历人体各关节点
        # 初始化最新消息变量
        self.latest_msg = None

        # 设置定时器（每0.25秒触发一次）
        self.timer = rospy.Timer(rospy.Duration(2), self.process_timer_callback)

    def _set_orientation_constraint(self):
        # 创建姿态约束
        orien_constraint = OrientationConstraint()
        orien_constraint.header.frame_id = "base_link"
        orien_constraint.link_name = "grasp_frame"
        orien_constraint.orientation.w = 1.0
        orien_constraint.absolute_x_axis_tolerance = 0.1
        orien_constraint.absolute_y_axis_tolerance = 0.1
        orien_constraint.absolute_z_axis_tolerance = 0.1
        orien_constraint.weight = 1.0

        constraints = Constraints()
        constraints.orientation_constraints.append(orien_constraint)
        self.arm.set_path_constraints(constraints)
    
    def pose_callback(self, msg):
        """接收消息回调函数：仅更新最新消息"""
        with self.lock:
            self.latest_msg = msg

    def process_timer_callback(self, event):
        """定时器回调：处理最新消息（非阻塞）"""
        # 取出当前最新消息
        current_msg = None        
        """回调函数：仅在非处理状态时触发"""
        # 如果正在处理消息，直接返回
        if self.is_processing:
            rospy.loginfo("正在处理上一消息，忽略新数据")
            return
        
        # 加锁并标记为处理中
        with self.lock:
            self.is_processing = True
            rospy.loginfo("is_processing置位true，开始处理msg")      
            if self.latest_msg is not None:
                current_msg = self.latest_msg
                self.latest_msg = None  # 清空以避免重复处理

        # 如果有新消息，启动线程处理
        if current_msg is not None:
            # self.start_processing_thread(current_msg)       
            try:
                # 处理单条消息
                self.process_message(current_msg)
                rospy.loginfo("msg处理完毕，运动规划完成")
                rospy.sleep(0.5)
            except Exception as e:
                rospy.logerr("处理消息时发生异常: %s", e)
            finally:
                # 处理完成，重置标志位
                with self.lock:
                    self.is_processing = False
                    rospy.loginfo("is_processing复位false，可以接收新消息msg") 
    
    def process_message(self, msg):
        # 打印消息的header信息
        try:
            print("Message Header:")
            # print(f"  Timestamp: secs={msg.header.stamp.secs}, nsecs={msg.header.stamp.nsecs}")
            # print(f"  Frame ID: {msg.header.frame_id}")
            print("  Timestamp: secs={}, nsecs={}".format(msg.header.stamp.secs, msg.header.stamp.nsecs))
            print("  Frame ID: {}".format(msg.header.frame_id))
            # 转换为 UTC 时间
            utc_time = datetime.datetime.utcfromtimestamp(msg.header.stamp.secs + msg.header.stamp.nsecs/1e9)
            # print(f"UTC Time: {utc_time}")  
            print("UTC Time: {}".format(utc_time)) 

        except AttributeError:
            print("Warning: This message type has no header!")

        # 打印接收到消息的当前时间
        receive_time = rospy.get_rostime()
        # print(f"Received Time: secs={receive_time.secs}, nsecs={receive_time.nsecs}\n") 
        print("  Received Time: secs={}, nsecs={}".format(receive_time.secs, receive_time.nsecs))
        # 转换为 UTC 时间
        utc_time_receive = datetime.datetime.utcfromtimestamp(receive_time.secs + receive_time.nsecs/1e9)
        # print(f"UTC Time receive: {utc_time_receive}")    
        print("UTC Time receive: {}".format(utc_time_receive))
        # 计算延迟（秒）
        delay = (receive_time.secs - msg.header.stamp.secs) + (receive_time.nsecs - msg.header.stamp.nsecs)/1e9
        # print(f"Delay: {delay} seconds")     
        print("Delay: {} seconds".format(delay))      

        """处理单条消息（包含坐标变换和运动规划）"""
        if not msg.persons:
            return
        
        person = msg.persons[0]

        # 每次处理当前关节点后，转到下一个关节点索引，如索引到达25，将回退到第一个关节点。    
        if (self.i < 8):
            part = person.bodyParts[self.i]  # 以第一个关节点为例
            part_name = self.BODY_PARTS.get(self.i, "Unknown")
            rospy.loginfo("现在开始规划移动到目标关节点是: %d(%s)", self.i, part_name)
            self.i +=1 
        else:
            self.i = 0
        # part = person.bodyParts[0]
        
        # # 获取摄像头坐标系下的坐标
        # x_cam = part.point.x
        # y_cam = part.point.y
        # z_cam = part.point.z
        
        try:
            # 坐标变换到机械臂基坐标系
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                "camera_link",
                rospy.Time(0),
                timeout=rospy.Duration(1.0)
            )
            point_cam = PointStamped()
            point_cam.header = msg.header
            point_cam.point.x = part.point.x
            point_cam.point.y = part.point.y
            point_cam.point.z = part.point.z  # 转换前坐标
            
            point_arm = do_transform_point(point_cam, transform)
            # x_arm = point_arm.point.x
            # y_arm = point_arm.point.y
            # z_arm = point_arm.point.z
            # 打印原始和变换后坐标
            rospy.loginfo("原始坐标: x=%.2f, y=%.2f, z=%.2f", part.point.x, part.point.y, part.point.z)     
            rospy.loginfo("变换后坐标: x=%.2f, y=%.2f, z=%.2f", point_arm.point.x, point_arm.point.y, point_arm.point.z)                       
            
            # # 滤波处理
            # self.filtered_x = self.alpha * x_arm + (1 - self.alpha) * self.filtered_x
            # self.filtered_y = self.alpha * y_arm + (1 - self.alpha) * self.filtered_y
            # self.filtered_z = self.alpha * z_arm + (1 - self.alpha) * self.filtered_z
            
            # # 强制限制坐标范围
            # self.filtered_x = max(min(self.filtered_x, self.x_max), self.x_min)
            # self.filtered_y = max(min(self.filtered_y, self.y_max), self.y_min)
            # self.filtered_z = max(min(self.filtered_z, self.z_max), self.z_min)
            # rospy.loginfo("滤波后坐标: x=%.2f, y=%.2f, z=%.2f", self.filtered_x, self.filtered_y, self.filtered_z)   

            # 获取当前位置
            current_pose = self.arm.get_current_pose().pose
            # current_x = current_pose.position.x
            # current_y = current_pose.position.y
            # current_z = current_pose.position.z

            target_pose = current_pose  # 继承当前姿态
            #target_pose.position.x = max(min(x_arm, x_max), self.x_min)
            target_pose.position.x = 0.08 #手动制定x值为固定值，减少规划难度
            target_pose.position.y = max(min(point_arm.point.y, self.y_max), self.y_min)
            # target_pose.position.z = max(min(z_arm, self.z_max), self.z_min)
            target_pose.position.z = 0.2 #手动制定z值为固定值，减少规划难度
            rospy.loginfo("限位后坐标: x=%.2f, y=%.2f, z=%.2f", target_pose.position.x, target_pose.position.y, target_pose.position.z) 
            # 执行运动
            self.arm.set_pose_target(target_pose)
            success = self.arm.go(wait=True)
            rospy.loginfo("运动规划结果: %s", success)
            rospy.sleep(2)                      
        
            # # 计算目标与当前位置的距离
            # distance = math.sqrt(
            #     (self.filtered_x - current_x)**2 +
            #     (self.filtered_y - current_y)**2 +
            #     (self.filtered_z - current_z)**2
            # )
            
            # if distance > 0.02:  # 2 厘米阈值
            #     # 设置目标位姿（固定垂直向下）
            #     target_pose = current_pose  # 继承当前姿态
            #     target_pose.position.x = self.filtered_x
            #     target_pose.position.y = self.filtered_y
            #     target_pose.position.z = self.filtered_z
            #     # q = tf.transformations.quaternion_from_euler(3.14159, 0, 0)
            #     # target_pose.orientation = Quaternion(*q)
                
            #     # 执行运动
            #     self.arm.set_pose_target(target_pose)
            #     success = self.arm.go(wait=True)
            #     rospy.loginfo("运动规划结果: %s (位移=%.3f米)", success, distance)
            # else:
            #     rospy.logwarn("目标位置未变化，跳过此次规划 (位移=%.3f米)", distance)
            # # 设置目标位姿（固定垂直向下）
            # target_pose = self.arm.get_current_pose().pose
            # target_pose.position.x = self.filtered_x
            # target_pose.position.y = self.filtered_y
            # target_pose.position.z = self.filtered_z
            # # q = tf.transformations.quaternion_from_euler(3.14159, 0, 0)
            # # target_pose.orientation = Quaternion(*q)
            
            # # 执行运动规划
            # self.arm.set_pose_target(target_pose)
            # success = self.arm.go(wait=True)
            # rospy.sleep(2)               
            # rospy.loginfo("运动规划结果: %s", success)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("坐标变换失败: %s", e)
        except Exception as e:
            rospy.logerr("运动规划失败: %s", e)
    
    def shutdown(self):
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        controller = OpenPoseControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        controller.shutdown()

# import sys  # 添加此行
# import rospy
# import tf2_ros
# import moveit_commander
# import numpy as np
# #from geometry_msgs.msg import PoseArray, PoseStamped
# import threading
# #from openpose import pyopenpose as op
# from collections import deque
# from tf2_geometry_msgs import do_transform_pose
# from tf2_geometry_msgs import do_transform_point
# from ros_openpose.msg import Frame  # 根据实际消息类型调整
# from geometry_msgs.msg import PointStamped
# from geometry_msgs.msg import Quaternion
# import tf
# from collections import deque

# class OpenPoseControl:
#     def __init__(self):
#         # 初始化ROS节点
#         rospy.init_node('openpose_gluon_control', anonymous=True)
        
#         # MoveIt初始化
#         moveit_commander.roscpp_initialize(sys.argv)
        
#         # 创建MoveGroup接口
#         self.arm = moveit_commander.MoveGroupCommander("gluon_arm")
#         self.arm.set_max_velocity_scaling_factor(0.2)  # 安全速度限制
#         self.arm.set_planning_time(20.0)               # 增加规划时间
#         self.arm.set_num_planning_attempts(5)        # 增加尝试次数

#         # 初始化需要使用move group控制的机械臂中的gripper group
#         #self.gripper = moveit_commander.MoveGroupCommander('gluon_gripper')
        
#         # 设置机械臂和夹爪的允许误差值
#         self.arm.set_goal_position_tolerance(0.01)
#         self.arm.set_goal_orientation_tolerance(0.1)
#         #gripper.set_goal_joint_tolerance(0.001)
        
#         # 控制机械臂先回到初始化位置
#         self.arm.set_named_target('home_zero')
#         self.arm.go()
#         rospy.sleep(2)
#         # 注释 OpenPose 数据驱动的代码，手动设置目标
#         self.arm.set_position_target([0.4, 0.0, 0.0])  # 示例坐标
#         success = self.arm.go(wait=True)
#         rospy.loginfo("手动目标规划结果1: %s", success)
#         self.arm.set_position_target([0.3, 0.2, 0.2])  # 示例坐标
#         success = self.arm.go(wait=True)
#         rospy.loginfo("手动目标规划结果2: %s", success) 
#         self.arm.set_position_target([0.2, 0.3, 0.2])  # 示例坐标
#         success = self.arm.go(wait=True)
#         rospy.loginfo("手动目标规划结果3: %s", success)  
#         self.arm.set_position_target([0.2, 0.2, 0.2])  # 示例坐标
#         success = self.arm.go(wait=True)
#         rospy.loginfo("手动目标规划结果4: %s", success)                     
#         self.arm.set_position_target([0.1, 0.2, 0.3])  # 示例坐标
#         success = self.arm.go(wait=True)
#         rospy.loginfo("手动目标规划结果5: %s", success)
#         self.arm.set_position_target([0.0, 0.1, 0.1])  # 示例坐标
#         success = self.arm.go(wait=True)
#         rospy.loginfo("手动目标规划结果6: %s", success)


# #         # 设置夹爪的目标位置，并控制夹爪运动
# #         #gripper.set_joint_value_target([-0.2])
# #         #gripper.go()
# #         #rospy.sleep(1)

# #         # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
# #         #joint_positions = [-0.5, -0.5, 0.52832, 0.5820, -0.5, -0.5]
# #         #arm.set_joint_value_target(joint_positions)
                 
# #         # 控制机械臂完成运动
# #         #arm.go()
# #         #rospy.sleep(1)
        
# #         # 关闭并退出moveit
# #         #moveit_commander.roscpp_shutdown()
# #         #moveit_commander.os._exit(0)
        
#     #     # TF监听器初始化
#     #     self.tf_buffer = tf2_ros.Buffer()
#     #     self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
#     #     # OpenPose订阅者
#     #     self.pose_sub = rospy.Subscriber('/frame', Frame, self.pose_callback)
        
#     #     #poseModel = op.PoseModel.BODY_25
#     #     #print(op.getPoseBodyPartMapping(poseModel))
#     #     #print(op.getPoseNumberBodyParts(poseModel))
#     #     #print(op.getPosePartPairs(poseModel))
#     #     #print(op.getPoseMapIndex(poseModel))
#     #     # 安全参数
#     #     # self.joint_limits = {
#     #     #     'min': [-3.14, -1.57, -3.14, -1.57, -3.14, -3.14],
#     #     #     'max': [3.14, 1.57, 3.14, 1.57, 3.14, 3.14]
#     #     # }

#     #     self.filtered_x = 0.0
#     #     self.filtered_y = 0.0
#     #     self.filtered_z = 0.0
#     #     self.alpha = 0.5  # 滤波系数（0~1，越小越平滑）
#     #     # 工作空间限制（单位：米）
#     #     self.x_min, self.x_max = -0.4, 0.4
#     #     self.y_min, self.y_max = -0.4, 0.4

#     #     self.z_min, self.z_max =  0.1, 0.4

#     #     # 目标队列与线程同步
#     #     #self.target_queue = deque()
#     #     #self.current_target = None
#     #     #self.lock = threading.Lock()
        
#     #     # 启动控制线程
#     #     #self.control_thread = threading.Thread(target=self.control_loop)
#     #     #self.control_thread.start()
#     #     self.BODY_PARTS = {
#     #         0:  "Nose",
#     #         1:  "Neck",
#     #         2:  "RShoulder",
#     #         3:  "RElbow",
#     #         4:  "RWrist",
#     #         5:  "LShoulder",
#     #         6:  "LElbow",
#     #         7:  "LWrist",
#     #         8:  "MidHip",
#     #         9:  "RHip",
#     #         10: "RKnee",
#     #         11: "RAnkle",
#     #         12: "LHip",
#     #         13: "LKnee",
#     #         14: "LAnkle",
#     #         15: "REye",
#     #         16: "LEye",
#     #         17: "REar",
#     #         18: "LEar",
#     #         19: "LBigToe",
#     #         20: "LSmallToe",
#     #         21: "LHeel",
#     #         22: "RBigToe",
#     #         23: "RSmallToe",
#     #         24: "RHeel",
#     #         25: "Background"
#     #     }
#     #     self.target_part_id = 0

#     #     # 初始化目标队列和锁
#     #     # self.target_queue = deque()
#     #     # self.lock = threading.Lock()
#     #     # self.control_thread = threading.Thread(target=self.control_loop)
#     #     # self.control_thread.start()
#     #     self.is_moving = False

#     # def pose_callback(self, msg):
#     #     if (self.is_moving == True):
#     #         rospy.loginfo("arm is moving")
#     #         pass
#     #     else:
#     #         if not msg.persons:
#     #             rospy.logwarn("未检测到人体")
#     #             return
#     #         person = msg.persons[0]
#     #         #for part in person.bodyParts:
#     #             #for i in range(0,len(person.bodyParts),1):
#     #             # 获取摄像头坐标系下的坐标
#     #         part = person.bodyParts[self.target_part_id]
#     #             #part_id = self.target_part_id
#     #         part_name = self.BODY_PARTS.get(self.target_part_id, "Unknown")
#     #         rospy.loginfo("目标关节点是: %d(%s)", self.target_part_id, part_name)
#     #         x_cam = part.point.x
#     #         y_cam = part.point.y
#     #         z_cam = part.point.z

#     #         # 转换到机械臂坐标系
#     #         try:
#     #             transform = self.tf_buffer.lookup_transform(
#     #                 "dummy", # 目标坐标系：机械臂基坐标系
#     #                 "camera_link",  # 源坐标系：摄像头坐标系
#     #                 #msg.header.frame_id,
#     #                 rospy.Time(0),
#     #                 timeout=rospy.Duration(1.0)
#     #             )
#     #             point_cam = PointStamped()
#     #             point_cam.header = msg.header
#     #             point_cam.point.x = x_cam
#     #             point_cam.point.y = y_cam
#     #             point_cam.point.z = z_cam 
#     #             #if (z_cam <= self.z_max ):
#     #             #    point_cam.point.z = z_cam 
#     #             # else:
#     #             #     point_cam.point.z = self.z_max
#     #             point_arm = do_transform_point(point_cam, transform)

#     #             # 打印原始和变换后坐标
#     #             rospy.loginfo("原始坐标: x=%.2f, y=%.2f, z=%.2f", part.point.x, part.point.y, part.point.z)
#     #             rospy.loginfo("变换后坐标: x=%.2f, y=%.2f, z=%.2f", point_arm.point.x, point_arm.point.y, point_arm.point.z)

#     #             # ...  滤波处理 ...
#     #             self.filtered_x = self.alpha * point_arm.point.x + (1 - self.alpha) * self.filtered_x
#     #             self.filtered_y = self.alpha * point_arm.point.y + (1 - self.alpha) * self.filtered_y
#     #             self.filtered_z = self.alpha * point_arm.point.z + (1 - self.alpha) * self.filtered_z
#     #             rospy.loginfo("滤波后坐标: x=%.2f, y=%.2f, z=%.2f", self.filtered_x, self.filtered_y, self.filtered_z)
#     #              # 滤波后坐标强制限制在工作空间内
#     #             self.filtered_x = max(min(self.filtered_x, self.x_max), self.x_min)
#     #             self.filtered_y = max(min(self.filtered_y, self.y_max), self.y_min)
#     #             self.filtered_z = max(min(self.filtered_z, self.z_max), self.z_min)
#     #             rospy.loginfo("硬限制后坐标: x=%.2f, y=%.2f, z=%.2f", self.filtered_x, self.filtered_y, self.filtered_z)           
#     #             # if (self.filtered_z <= self.z_max ):
#     #             #     pass
#     #             # else:
#     #             #     self.filtered_z = self.z_max
#     #             #self.arm.set_position_target([self.filtered_x, self.filtered_y, self.filtered_z])
#     #             # 控制机械臂移动（示例：移动到右手腕）
#     #             #if "RightWrist" in str(part):
#     #             #self.arm.set_position_target([point_arm.point.x, point_arm.point.y, point_arm.point.z])
#     #             # 在 pose_callback 中设置目标姿态
#     #             # 检查坐标是否在合理范围内
#     #             if (self.x_min <= self.filtered_x <= self.x_max) and \
#     #                (self.y_min <= self.filtered_y <= self.y_max) and \
#     #                (self.z_min <= self.filtered_z <= self.z_max):
#     #                    # 设置目标位姿（继承当前姿态）                 
#     #                 target_pose = self.arm.get_current_pose().pose  # 获取当前姿态
#     #                 #target_pose.position.x = point_arm.point.x
#     #                 #target_pose.position.y = point_arm.point.y
#     #                 #target_pose.position.z = point_arm.point.z
#     #                 target_pose.position.x = self.filtered_x
#     #                 target_pose.position.y = self.filtered_y
#     #                 target_pose.position.z = self.filtered_z
#     #                 # 设置末端垂直向下（绕 X 轴旋转 180 度）
#     #                 q = tf.transformations.quaternion_from_euler(3.14159, 0, 0)
#     #                 target_pose.orientation = Quaternion(*q)

#     #                 # # 将坐标加入队列
#     #                 # with self.lock:
#     #                 #     self.target_queue.append((
#     #                 #         target_pose.position.x,
#     #                 #         target_pose.position.y,
#     #                 #         target_pose.position.z
#     #                 #     ))
#     #                 # rospy.loginfo("新目标加入队列: (%.2f, %.2f, %.2f)", 
#     #                 #          target_pose.position.x, target_pose.position.y, target_pose.position.z)

#     #                 self.arm.set_pose_target(target_pose)  # 设置完整位姿

#     #                 # 执行运动
#     #                 arm_plan = self.arm.plan()
#     #                 if arm_plan:
#     #                     self.is_moving = True
#     #                     success = self.arm.execute(arm_plan)
#     #                     rospy.loginfo("运动规划结果: %s", success)\
#     #                 # else:
#     #                 #     rospy.loginfo("plan failed")
#     #                 self.is_moving = False
#     #                 ##self.arm.go(wait=True)
#     #                 #if (self.target_part_id < 25):
#     #                 #    self.target_part_id += 1
#     #                 #else:
#     #                 #    self.target_part_id = 0
#     #             else:
#     #                 rospy.logwarn("目标位置超出工作空间限制: x=%.2f, y=%.2f, z=%.2f", self.filtered_x, self.filtered_y, self.filtered_z)

#     #         #except tf2_ros.TransformException as e:
#     #         #    rospy.logwarn("TF 错误: {}".format(e))
#     #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
#     #             rospy.logerr("坐标变换失败: {}".format(e))

#     # def control_loop(self):
#     #     rate = rospy.Rate(10)  # 控制频率 10Hz
#     #     while not rospy.is_shutdown():
#     #         if self.current_target is None:
#     #             with self.lock:
#     #                 if len(self.target_queue) > 0:
#     #                     self.current_target = self.target_queue.popleft()
#     #                     rospy.loginfo("开始移动到目标: %s", self.current_target)
            
#     #         if self.current_target is not None:
#     #             success = self.move_to_target(self.current_target)
#     #             if success:
#     #                 rospy.loginfo("已到达目标: %s", self.current_target)
#     #                 self.current_target = None
#     #             else:
#     #                 rospy.logwarn("运动规划失败，跳过目标: %s", self.current_target)
#     #                 self.current_target = None
#     #         rate.sleep()

#     # def move_to_target(self, target):
#     #     try:
#     #         target_pose = self.arm.get_current_pose().pose
#     #         target_pose.position.x = target[0]
#     #         target_pose.position.y = target[1]
#     #         target_pose.position.z = target[2]
#     #         self.arm.set_pose_target(target_pose)
#     #         return self.arm.go(wait=True)
#     #     except Exception as e:
#     #         rospy.logerr("运动执行异常: {}".format(e))
#     #         return False


# #    def check_safety(self, pose):
# #        """安全检查"""
# #        # 碰撞检测（需根据实际配置扩展）
# #        current_joints = self.arm.get_current_joint_values()
# #        return True  # 此处应实现实际安全检查

# #    def move_to_pose(self, target_pose):
#         """执行运动规划"""
# #        self.arm.set_pose_target(target_pose)
        
#         # 运动规划
# #        plan = self.arm.plan()
# #        if not plan.joint_trajectory.points:
# #            rospy.logerr("Planning failed for current pose")
# #            return False
        
#         # 执行运动
# #        try:
# #            self.arm.execute(plan, wait=True)
# #            rospy.loginfo("Movement executed successfully")
# #            return True
# #        except moveit_commander.MoveItCommanderException as ex:
# #            rospy.logerr("Execution failed: %s", ex)
# #            return False
#     def shutdown(self):
#         #self.control_thread.join() 
#         moveit_commander.roscpp_shutdown()

# if __name__ == '__main__':
#     #rospy.init_node('openpose_gluon_control_test')
#     try:
#         controller = OpenPoseControl()
#         rospy.spin()
#         controller.shutdown()
#     except rospy.ROSInterruptException:
#         pass

