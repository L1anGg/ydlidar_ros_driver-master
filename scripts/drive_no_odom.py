#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32 底盘桥接节点（仅 cmd_vel 速度下发）

串口协议（见电机驱动手册）:
    $pwm:V1,V2,V3,V4#    四路电机 PWM (-1000 ~ 1000)
                         前进 = (-right, left, -left, right)

话题:
    订阅 /cmd_vel   (geometry_msgs/Twist)
"""
import threading

import rospy
import serial

from geometry_msgs.msg import Twist


class Stm32ChassisBridge(object):

    def __init__(self):
        # ---------------- 参数 ----------------
        port          = rospy.get_param('~port', '/dev/ttyAMA0')
        baud          = rospy.get_param('~baud', 115200)
        # 差速解算用的轮距
        self.wheel_base    = float(rospy.get_param('~wheel_base', 0.175))
        # 前进增益：保持原值，最高速度不变
        self.speed_scale   = float(rospy.get_param('~speed_scale', 1000.0))
        self.max_speed     = float(rospy.get_param('~max_speed', 1.0))
        # 角速度增益：只放大转向的差速分量，不影响前进速度
        # 麦轮打滑导致实际转角偏小，需要 2~4 倍的补偿
        self.angular_gain  = float(rospy.get_param('~angular_gain', 1.0))
        # 驱动板 PWM 满量程 ±3600
        self.pwm_max       = int(rospy.get_param('~pwm_max', 3600))
        # 电机启动死区：|速度指令| 小于该值时抬到该值（实测值），<=0 关闭补偿
        self.cmd_deadband  = float(rospy.get_param('~cmd_deadband', 70.0))

        # 安全看门狗：超过该秒数没收到 /cmd_vel 就主动停车（<=0 关闭）
        self.cmd_timeout   = float(rospy.get_param('~cmd_timeout', 0.5))

        # ---------------- 状态 ----------------
        self._ser_lock = threading.Lock()
        # 看门狗状态
        self._last_cmd_time = None
        self._watchdog_stopped = False
        self._watchdog_ticks = 0

        # ---------------- 串口 ----------------
        self.ser = serial.Serial(port, baud, timeout=0.05)
        rospy.loginfo("串口已打开: %s @ %d", port, baud)

        # ---------------- ROS 接口 ----------------
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_cb)

        # ---------------- 安全看门狗 ----------------
        rospy.Timer(rospy.Duration(0.1), self._watchdog_cb)
        rospy.on_shutdown(self._shutdown_stop)

        rospy.loginfo("底盘桥接节点已启动，等待 /cmd_vel ...（超时 %.1fs 自动停车，死区补偿 %d）",
                      self.cmd_timeout, int(self.cmd_deadband))

    # ======================= 串口收发 =======================
    def send_raw(self, text):
        if isinstance(text, str):
            data = text.encode('ascii')
        else:
            data = text
        try:
            with self._ser_lock:
                self.ser.write(data)
        except (serial.SerialException, OSError) as e:
            rospy.logwarn_throttle(2.0, "串口写入异常: %s", e)

    def _watchdog_cb(self, _event):
        """看门狗：/cmd_vel 中断（导航包退出、遥控断线）后主动发零速度"""
        if self.cmd_timeout <= 0 or self._last_cmd_time is None:
            return

        elapsed = (rospy.Time.now() - self._last_cmd_time).to_sec()
        if elapsed <= self.cmd_timeout:
            if self._watchdog_stopped:
                self._watchdog_stopped = False
                self._watchdog_ticks = 0
                rospy.loginfo("/cmd_vel 已恢复，看门狗解除")
            return

        # 超时：立即停车一次，之后每秒补发一次（防丢帧）
        self._watchdog_ticks += 1
        if not self._watchdog_stopped:
            self._watchdog_stopped = True
            rospy.logwarn("/cmd_vel 已中断 %.1fs，主动停车", elapsed)
        if self._watchdog_ticks == 1 or self._watchdog_ticks % 10 == 0:
            self.send_speed(0, 0)

    def _shutdown_stop(self):
        """节点退出时确保底盘停止"""
        try:
            self.send_speed(0, 0)
            rospy.loginfo("节点退出，已发送停车指令")
        except Exception:
            pass

    def _deadband_comp(self, u):
        """单轮速度指令死区补偿：|u| 不足死区时抬到死区值，u=0 保持停车。

        左右各自独立补偿，不改变两轮差速关系（直行/弧线曲率不变）。
        """
        d = self.cmd_deadband
        if d <= 0 or u == 0:
            return u
        if -d < u < d:
            return d if u > 0 else -d
        return u

    def send_speed(self, left, right):
        left  = self._deadband_comp(left)
        right = self._deadband_comp(right)
        left  = max(-self.speed_scale, min(self.speed_scale, int(left)))
        right = max(-self.speed_scale, min(self.speed_scale, int(right)))
        frame = "$spd:%d,%d,%d,%d#" % (-right, left, -left, right)
        rospy.logdebug(frame)
        self.send_raw(frame)

    # ======================= 控制下发 =======================
    def cmd_vel_cb(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        # 差速底盘没有横向平移能力：若 DWA 发出 linear.y，这里无法执行
        if abs(msg.linear.y) > 1e-3:
            rospy.logwarn_throttle(
                2.0, "cmd_vel 含横向速度 linear.y=%.3f，差速底盘已忽略", msg.linear.y)

        # 差速解算（角速度乘增益补偿麦轮打滑）
        v_left  = v - w * self.wheel_base / 2.0 * self.angular_gain
        v_right = v + w * self.wheel_base / 2.0 * self.angular_gain

        # 限幅 + 换算成协议值
        v_left  = max(-self.max_speed, min(self.max_speed, v_left))
        v_right = max(-self.max_speed, min(self.max_speed, v_right))
        self.send_speed(v_left * self.speed_scale, v_right * self.speed_scale)


if __name__ == '__main__':
    rospy.init_node('cmd_vel_to_stm32')
    try:
        Stm32ChassisBridge()
    except (serial.SerialException, OSError) as e:
        rospy.logfatal("串口打开失败: %s", e)
        raise SystemExit(1)
    rospy.spin()