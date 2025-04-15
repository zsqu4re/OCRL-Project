#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
测试LQR+VMC控制器在Gazebo仿真中的表现
这个脚本会发送一系列的目标状态给控制器，测试不同工作模式下的平衡性能
"""

import rospy
import numpy as np
import time
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

class GazeboControllerTest:
    def __init__(self):
        rospy.init_node('gazebo_controller_test')
        
        # 状态变量
        self.current_state = np.zeros(6)  # [theta, dtheta, l0, dl0, phi, dphi]
        self.initialized = False
        
        # 创建发布者
        self.desired_state_pub = rospy.Publisher('/desired_state', Float64MultiArray, queue_size=10)
        
        # 订阅关节状态
        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        
        # 等待控制器初始化
        rospy.loginfo("等待控制器初始化...")
        rospy.sleep(2.0)
        
        if not self.initialized:
            rospy.logwarn("没有收到关节状态，将继续测试但可能有问题")
    
    def joint_state_callback(self, msg):
        """处理关节状态信息"""
        try:
            # 简单处理：只是标记我们已经接收到关节状态
            self.initialized = True
        except Exception as e:
            rospy.logerr("处理关节状态时出错: %s", str(e))
    
    def send_desired_state(self, theta, theta_dot, l0, l0_dot, phi, phi_dot):
        """发送目标状态到控制器"""
        msg = Float64MultiArray()
        msg.data = [theta, theta_dot, l0, l0_dot, phi, phi_dot]
        
        self.desired_state_pub.publish(msg)
        rospy.loginfo("发送目标状态: [theta=%.3f, dtheta=%.3f, l0=%.3f, dl0=%.3f, phi=%.3f, dphi=%.3f]", 
                      theta, theta_dot, l0, l0_dot, phi, phi_dot)
    
    def test_sequence(self):
        """运行一系列测试序列"""
        # 等待ROS系统稳定
        rospy.sleep(1.0)
        
        # 测试1: 基本站立平衡 - 机器人应该保持垂直站立
        rospy.loginfo("测试1: 基本站立平衡")
        self.send_desired_state(0.0, 0.0, 0.08, 0.0, 0.0, 0.0)
        rospy.sleep(5.0)
        
        # 测试2: 改变高度 - 机器人应该稍微蹲下
        rospy.loginfo("测试2: 改变高度 (稍微蹲下)")
        self.send_desired_state(0.0, 0.0, 0.07, 0.0, 0.0, 0.0)
        rospy.sleep(5.0)
        
        # 测试3: 前倾 - 机器人应该向前倾斜
        rospy.loginfo("测试3: 前倾")
        self.send_desired_state(0.1, 0.0, 0.08, 0.0, 0.0, 0.0)
        rospy.sleep(5.0)
        
        # 测试4: 后倾 - 机器人应该向后倾斜
        rospy.loginfo("测试4: 后倾")
        self.send_desired_state(-0.1, 0.0, 0.08, 0.0, 0.0, 0.0)
        rospy.sleep(5.0)
        
        # 测试5: 扭转 - 机器人身体应该扭转
        rospy.loginfo("测试5: 扭转")
        self.send_desired_state(0.0, 0.0, 0.08, 0.0, 0.1, 0.0)
        rospy.sleep(5.0)
        
        # 回到原始状态
        rospy.loginfo("回到初始平衡状态")
        self.send_desired_state(0.0, 0.0, 0.08, 0.0, 0.0, 0.0)
        rospy.sleep(3.0)
        
        # 测试6: 小跳跃模拟 - 先蹲下然后站起
        rospy.loginfo("测试6: 小跳跃模拟")
        self.send_desired_state(0.0, 0.0, 0.06, 0.0, 0.0, 0.0)  # 蹲下
        rospy.sleep(2.0)
        self.send_desired_state(0.0, 0.0, 0.08, 0.3, 0.0, 0.0)  # 快速站起
        rospy.sleep(1.0)
        self.send_desired_state(0.0, 0.0, 0.08, 0.0, 0.0, 0.0)  # 稳定
        rospy.sleep(3.0)
        
        rospy.loginfo("测试序列完成")

def main():
    try:
        tester = GazeboControllerTest()
        tester.test_sequence()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()