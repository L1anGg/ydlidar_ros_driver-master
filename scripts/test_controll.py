#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wasd_teleop.py
WASD 键盘控制速度发布节点 (ROS1 Noetic)

W: 前进    S: 后退
A: 左转    D: 右转
空格: 急停  Q/Ctrl+C: 退出
"""

import rospy
from geometry_msgs.msg import Twist

# 如果是 Linux 且需要读取单键输入
import sys
import select
import termios
import tty


class WASDTeleop:
    def __init__(self):
        # 初始化节点
        rospy.init_node('wasd_teleop', anonymous=False)

        # 发布者：发布 Twist 到 /cmd_vel 话题
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 从参数服务器读取速度参数（可被 launch 文件覆盖）
        self.linear_speed = rospy.get_param('~linear_speed', 0.3)   # 前进/后退速度 m/s
        self.angular_speed = rospy.get_param('~angular_speed', 4.0) # 转向速度 rad/s
        self.rate = rospy.Rate(20)  # 发布频率 20Hz

        self.twist = Twist()
        self.settings = termios.tcgetattr(sys.stdin)  # 保存终端原始设置

    def get_key(self):
        """非阻塞读取单个按键，无按键时返回空字符串"""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def make_twist(self, linear, angular):
        """生成一条 Twist 消息"""
        t = Twist()
        t.linear.x = linear
        t.linear.y = 0.0
        t.linear.z = 0.0
        t.angular.x = 0.0
        t.angular.y = 0.0
        t.angular.z = angular
        return t

    def run(self):
        rospy.loginfo("""
==================================
  WASD 速度控制节点
----------------------------------
  W: 前进      S: 后退
  A: 左转      D: 右转
  空格: 急停    Q: 退出
  Ctrl+C: 退出
==================================
        """)

        current_key = ''
        try:
            while not rospy.is_shutdown():
                key = self.get_key()

                if key:
                    current_key = key

                # 根据按键设置速度
                if current_key in ('w', 'W'):
                    self.twist = self.make_twist(self.linear_speed, 0.0)     # 前进
                elif current_key in ('s', 'S'):
                    self.twist = self.make_twist(-self.linear_speed, 0.0)    # 后退
                elif current_key in ('a', 'A'):
                    self.twist = self.make_twist(0.0, self.angular_speed)    # 左转
                elif current_key in ('d', 'D'):
                    self.twist = self.make_twist(0.0, -self.angular_speed)   # 右转
                elif current_key == ' ':
                    self.twist = self.make_twist(0.0, 0.0)                   # 急停
                elif current_key in ('q', 'Q'):
                    rospy.loginfo("退出控制...")
                    break

                # 发布速度消息
                self.pub.publish(self.twist)

                # 打印当前状态
                rospy.loginfo_throttle(
                    1.0,
                    "线性速度: %.2f m/s | 角速度: %.2f rad/s",
                    self.twist.linear.x,
                    self.twist.angular.z,
                )
                self.rate.sleep()

        except rospy.ROSInterruptException:
            pass
        finally:
            # 停止前发送一次零速度，避免机器人继续运动
            self.pub.publish(self.make_twist(0.0, 0.0))
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            rospy.loginfo("节点已关闭，机器人已停止")


if __name__ == '__main__':
    try:
        WASDTeleop().run()
    except rospy.ROSInterruptException:
        pass
