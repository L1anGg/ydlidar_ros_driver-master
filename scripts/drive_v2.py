#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32 底盘桥接节点 v2（cmd_vel -> $spd 速度指令）

drive_no_odom.py 下发的是 $pwm 开环占空比，重载底盘上必然二选一：
PWM 小了电机堵转，PWM 大了起步猛冲。v2 改为下发速度指令，
由驱动板用编码器做 PID 闭环，自动把 PWM 调到刚好能带动负载的值。

串口协议（4 路电机驱动板手册，115200 / 8N1）:
    $spd:M1,M2,M3,M4#   四路电机速度，范围 -1000 ~ 1000，板载 PID 自动调 PWM
                        该指令只对带编码器的电机有效，板子不返回任何数据
    M1~M4 的接线顺序沿用 v1 已验证过的：前进 = (-右, 左, -左, 右)

可选初始化指令（~init_cmd，断电保存的配置，一般只在首次上电时下发）:
    $mtype:1#             电机型号（1 = 520 电机）
    $MPID:0.8,0.06,0.5#   板载速度环 PID，默认值即可；带不动负载时只加大 P
    $upload:0,0,1#        上报轮速，用于调试观察

话题:
    订阅 /cmd_vel   (geometry_msgs/Twist)

标定 ~speed_scale:
    $spd 的 1000 对应多少 m/s 取决于轮径 / 减速比 / 电机，手册未给出，
    需要实测：让车直行一段，量出实际速度，再按"期望 / 实测"的比值修正。
    例：speed_scale=2000 时命令 0.5 m/s（$spd=1000），实测只有 0.35 m/s，
        说明给的值偏小，要放大：2000 * 0.5 / 0.35 ≈ 2857。
"""
import threading

import rospy
import serial

from geometry_msgs.msg import Twist


class Stm32ChassisBridge(object):

    def __init__(self):
        # ---------------- 参数 ----------------
        port = rospy.get_param('~port', '/dev/ttyAMA0')
        baud = int(rospy.get_param('~baud', 115200))
        # 差速解算用的轮距
        self.wheel_base = float(rospy.get_param('~wheel_base', 0.175))
        # 单轮最大线速度（m/s）
        self.max_speed = float(rospy.get_param('~max_speed', 1.0))
        # m/s -> $spd 数值的换算系数
        self.speed_scale = float(rospy.get_param('~speed_scale', 2000.0))
        # $spd 指令量程 ±1000，超出范围驱动板直接忽略
        self.spd_max = int(rospy.get_param('~spd_max', 1000))
        # 角速度增益：只放大转向的差速分量，不影响前进速度
        self.angular_gain = float(rospy.get_param('~angular_gain', 1.0))
        # 安全看门狗：超过该秒数没收到 /cmd_vel 就主动停车（<=0 关闭）
        self.cmd_timeout = float(rospy.get_param('~cmd_timeout', 0.5))
        # 启动时下发一次的配置指令（多条用空格分开）
        self.init_cmd = str(rospy.get_param('~init_cmd', ''))

        # ---------------- 状态 ----------------
        self._ser_lock = threading.Lock()
        self._last_cmd_time = None
        self._stopped = False
        self._watchdog_ticks = 0

        # ---------------- 串口 ----------------
        self.ser = serial.Serial(port, baud, timeout=0.05)
        rospy.loginfo("串口已打开: %s @ %d", port, baud)
        for cmd in self.init_cmd.split():
            self.send_raw(cmd)

        # ---------------- ROS 接口 ----------------
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_cb)
        rospy.Timer(rospy.Duration(0.1), self._watchdog_cb)
        rospy.on_shutdown(self._shutdown_stop)

        rospy.loginfo("底盘桥接 v2 已启动，等待 /cmd_vel ...（超时 %.1fs 自动停车）",
                      self.cmd_timeout)

    # ======================= 串口 =======================
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

    def send_speed(self, left, right):
        left = max(-self.spd_max, min(self.spd_max, int(round(left))))
        right = max(-self.spd_max, min(self.spd_max, int(round(right))))
        frame = "$spd:%d,%d,%d,%d#" % (-right, left, -left, right)
        rospy.logdebug(frame)
        self.send_raw(frame)

    # ======================= 安全 =======================
    def _watchdog_cb(self, _event):
        """/cmd_vel 中断（导航包退出、遥控断线）后主动停车"""
        if self.cmd_timeout <= 0 or self._last_cmd_time is None:
            return

        elapsed = (rospy.Time.now() - self._last_cmd_time).to_sec()
        if elapsed <= self.cmd_timeout:
            if self._stopped:
                self._stopped = False
                self._watchdog_ticks = 0
                rospy.loginfo("/cmd_vel 已恢复，看门狗解除")
            return

        # 超时：立即停车一次，之后每秒补发一次（防丢帧）
        self._watchdog_ticks += 1
        if not self._stopped:
            self._stopped = True
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

    # ======================= 控制下发 =======================
    def cmd_vel_cb(self, msg):
        # 差速底盘没有横向平移能力：若 DWA 发出 linear.y，这里无法执行
        if abs(msg.linear.y) > 1e-3:
            rospy.logwarn_throttle(
                2.0, "cmd_vel 含横向速度 linear.y=%.3f，差速底盘已忽略", msg.linear.y)

        v = msg.linear.x
        w = msg.angular.z

        # 差速解算（角速度乘增益补偿麦轮打滑）
        left = v - w * self.wheel_base / 2.0 * self.angular_gain
        right = v + w * self.wheel_base / 2.0 * self.angular_gain

        left = max(-self.max_speed, min(self.max_speed, left))
        right = max(-self.max_speed, min(self.max_speed, right))

        self._last_cmd_time = rospy.Time.now()
        self.send_speed(left * self.speed_scale, right * self.speed_scale)


if __name__ == '__main__':
    rospy.init_node('cmd_vel_to_stm32')
    try:
        Stm32ChassisBridge()
    except (serial.SerialException, OSError) as e:
        rospy.logfatal("串口打开失败: %s", e)
        raise SystemExit(1)
    rospy.spin()
