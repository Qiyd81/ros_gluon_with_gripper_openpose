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
from step_motor.msg import Motor  # 确保消息包已正确编译

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

        # 创建发布者
        self.gripper_pub = rospy.Publisher('/motor_control', Motor, queue_size=10)

        # 构造消息
        self.gripperopen_msg = Motor()
        self.gripperopen_msg.id = 1
        self.gripperopen_msg.speed = 100
        self.gripperopen_msg.dir = 0
        self.gripperopen_msg.mode = 2      # 注意验证实际字段名
        self.gripperopen_msg.angle = 18720
        self.gripperopen_msg.state = 0
        self.gripperopen_msg.sub_divide = 8

        self.gripperclose_msg = Motor()
        self.gripperclose_msg.id = 1
        self.gripperclose_msg.speed = 100
        self.gripperclose_msg.dir = 1
        self.gripperclose_msg.mode = 2      # 注意验证实际字段名
        self.gripperclose_msg.angle = 18720
        self.gripperclose_msg.state = 0
        self.gripperclose_msg.sub_divide = 8        

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
        if (self.i < 9):
            part = person.bodyParts[self.i]  # 以第i关节点为例
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
            if((self.i==5) or (self.i ==8)):
                # 设置目标点
                self.arm.set_pose_target(target_pose)
                rospy.loginfo("已设置目标坐标")

                # #设置目标点后即发布夹爪打开控制指令msg
                # self.gripper_pub.publish(self.gripperopen_msg)   
                # rospy.loginfo("Gripperopen Python control command sent!") 
                # rospy.sleep(0.5)  

                #开始运动
                success = self.arm.go(wait=True)
                rospy.loginfo("已运动到目标位置: %s", success)
                # # 等待连接建立（ROS1 Python没有直接获取订阅者数量的方法）
                # rate = rospy.Rate(10)  # 10Hz
                # while pub.get_num_connections() == 0 and not rospy.is_shutdown():
                #     rospy.loginfo("Waiting for subscribers...")
                #     rate.sleep()   
                # #发布夹爪夹紧控制指令msg
                # self.gripper_pub.publish(self.gripperclose_msg)   
                # rospy.loginfo("Gripperclose Python control command sent!")             
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

