#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32 底盘桥接节点（cmd_vel 下发 + 编码器里程计）

串口协议（见电机驱动手册）:
    $mtype:N#            设置电机类型（2 = 四路电机，必须先设置，否则编码器恒为 0）
    $upload:A,B,C#   A:总编码器数据  B:10ms实时编码器数据  C:轮子速度    1=打开 0=不接收
    $MAll:M1,M2,M3,M4#   轮子转动的总编码器数据
    $MTEP:M1,M2,M3,M4#   10ms 实时增量编码器数据
    $MSPD:M1,M2,M3,M4#   轮子速度
    $pwm:V1,V2,V3,V4#    四路电机 PWM (-1000 ~ 1000)
                         前进 = (-right, left, -left, right)

话题:
    订阅 /cmd_vel   (geometry_msgs/Twist)
    发布 /odom      (nav_msgs/Odometry)
    发布 TF         odom -> base_footprint
"""
#deadzone = '$deadzone:2000#'
#ser.write(deadzone.encode())
#data = ser.read(15)
#print(f"{data.decode('utf-8', errors='ignore')}")
import math
import threading
import time

import rospy
import serial

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros


class Stm32ChassisBridge(object):

    def __init__(self):
        # ---------------- 参数 ----------------
        port          = rospy.get_param('~port', '/dev/ttyAMA0')
        baud          = rospy.get_param('~baud', 115200)
        self.wheel_base    = float(rospy.get_param('~wheel_base', 0.175))
        # 前进增益：保持原值，最高速度不变
        self.speed_scale   = float(rospy.get_param('~speed_scale', 1000.0))
        self.max_speed     = float(rospy.get_param('~max_speed', 1.0))
        # 角速度增益：只放大转向的差速分量，不影响前进速度
        # 麦轮打滑导致实际转角偏小，需要 2~4 倍的补偿
        self.angular_gain  = float(rospy.get_param('~angular_gain', 1.0))
        # 驱动板 PWM 满量程 ±3600
        self.pwm_max       = int(rospy.get_param('~pwm_max', 3600))
        wheel_diameter     = float(rospy.get_param('~wheel_diameter', 0.080))
        counts_per_rev     = float(rospy.get_param('~counts_per_rev', 906.0))
        odom_rate          = float(rospy.get_param('~odom_rate', 30.0))
        self.frame_id      = rospy.get_param('~frame_id', 'odom')
        self.child_frame_id = rospy.get_param('~child_frame_id', 'base_footprint')
        self.publish_tf    = bool(rospy.get_param('~publish_tf', True))
        self.left_sign     = float(rospy.get_param('~left_sign', 1.0))
        self.right_sign    = float(rospy.get_param('~right_sign', 1.0))
        self.mtype         = int(rospy.get_param('~mtype', 2))
        # 编码器通道 -> 左右轮 的组合配置
        # 格式 "M编号:符号,..."，编号对应 $MAll 的 M1~M4
        # 同侧镜像安装的两个电机前进时计数符号相反，所以必须一正一负
        self.enc_left      = self._parse_enc_cfg(
            rospy.get_param('~enc_left', '2:1,3:-1'))
        self.enc_right     = self._parse_enc_cfg(
            rospy.get_param('~enc_right', '4:1,1:-1'))
        self.upload_cmd    = rospy.get_param('~upload_cmd', '$upload:1,0,0#')
        self.rearm_period  = float(rospy.get_param('~rearm_period', 5.0))

        # 安全看门狗：超过该秒数没收到 /cmd_vel 就主动停车（<=0 关闭）
        self.cmd_timeout   = float(rospy.get_param('~cmd_timeout', 0.5))

        # 每个编码器计数对应的轮子行进距离（米）
        self.dist_per_count = math.pi * wheel_diameter / max(counts_per_rev, 1.0)

        # ---------------- 状态 ----------------
        self._enc = [0, 0, 0, 0]          # 最近一帧 $MAll
        self._enc_valid = False
        self._logged_raw = False
        self._prev_left = None
        self._prev_right = None
        self._prev_time = None
        self._x = 0.0
        self._y = 0.0
        self._th = 0.0
        self._vx = 0.0
        self._vth = 0.0
        self._enc_lock = threading.Lock()
        self._ser_lock = threading.Lock()
        # 看门狗状态
        self._last_cmd_time = None
        self._watchdog_stopped = False
        self._watchdog_ticks = 0

        # ---------------- 串口 ----------------
        self.ser = serial.Serial(port, baud, timeout=0.05)
        rospy.loginfo("串口已打开: %s @ %d", port, baud)

        # ---------------- ROS 接口 ----------------
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_cb)

        # 1) 先设置电机类型（板子不支持时用 _mtype:=0 关掉）
        # 2) 再打开编码器数据上传（只开总编码器数据，带宽最省）
        #if self.mtype > 0:
            #self.send_raw("$mtype:%d#" % self.mtype)
            #time.sleep(0.1)
        self.send_raw(self.upload_cmd)
        if self.rearm_period > 0:
            rospy.Timer(rospy.Duration(self.rearm_period), self.rearm_cb)
        #设置电机死区
        #deadzone = '$deadzone:2000#'
        #self.send_raw(deadzone)
        #ddata = self.ser.read(15)
        #print(f"{ddata.decode('ascii', errors='ignore')}")
        # ---------------- 线程 / 定时器 ----------------
        self.reader = threading.Thread(target=self.serial_loop, name='serial_reader')
        self.reader.daemon = True
        self.reader.start()

        rospy.Timer(rospy.Duration(1.0 / max(odom_rate, 1.0)), self.publish_odom)
        #rospy.loginfo("底盘桥接节点已启动，等待 /cmd_vel ...")

        # ---------------- 安全看门狗 ----------------
        rospy.Timer(rospy.Duration(0.1), self._watchdog_cb)
        rospy.on_shutdown(self._shutdown_stop)

        rospy.loginfo("底盘桥接节点已启动，等待 /cmd_vel ...（超时 %.1fs 自动停车）",
                      self.cmd_timeout)

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

    def rearm_cb(self, _event):
        """板子掉电/复位后可能丢掉 mtype / upload 设置，定期重发一次。"""
        if self.mtype > 0:
            self.send_raw("$mtype:%d#" % self.mtype)
        self.send_raw(self.upload_cmd)

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

    def send_speed(self, left, right):
        left  = max(-self.pwm_max, min(self.pwm_max, int(left)))
        right = max(-self.pwm_max, min(self.pwm_max, int(right)))
        frame = "$pwm:%d,%d,%d,%d#" % (-right, left, -left, right)
        rospy.logdebug(frame)
        self.send_raw(frame)

    def serial_loop(self):
        """读线程：收字节 -> 拆帧 -> 交给 handle_frame"""
        buf = b''
        while not rospy.is_shutdown():
            try:
                chunk = self.ser.read(64)
            except (serial.SerialException, OSError) as e:
                rospy.logwarn_throttle(2.0, "串口读取异常: %s", e)
                continue
            if not chunk:
                continue
            buf += chunk
            while True:
                start = buf.find(b'$')
                if start < 0:
                    buf = b''
                    break
                end = buf.find(b'#', start + 1)
                if end < 0:
                    buf = buf[start:]       # 半帧，留下次拼接
                    break
                raw = buf[start + 1:end]
                buf = buf[end + 1:]
                text = raw.decode('ascii', 'ignore').strip()
                if text:
                    self.handle_frame(text)

    def handle_frame(self, text):
        """解析 $TAG:body# 帧"""
        if ':' not in text:
            return
        tag, body = text.split(':', 1)
        tag = tag.strip().upper()
        if tag not in ('MALL', 'MTEP', 'MSPD'):
            return                       # $upload 回显等，直接忽略
        try:
            vals = [int(v) for v in body.replace(' ', '').split(',') if v != '']
        except ValueError:
            return
        if len(vals) < 2:
            return

        if tag == 'MALL':
            while len(vals) < 4:
                vals.append(vals[-1])
            with self._enc_lock:
                self._enc = vals[:4]
                self._enc_valid = True
            if not self._logged_raw:
                self._logged_raw = True
                rospy.loginfo("收到编码器原始数据: $%s#", text)
        elif tag == 'MSPD':
            rospy.logdebug("MSPD: %s", vals)

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

    # ======================= 里程计 =======================
    @staticmethod
    def _parse_enc_cfg(text):
        """"2:1,3:-1" -> [(1, 1), (2, -1)]，编号即 M1~M4"""
        cfg = []
        for item in str(text).split(','):
            item = item.strip()
            if not item:
                continue
            if ':' in item:
                idx_s, sign_s = item.split(':', 1)
            else:
                idx_s, sign_s = item, '1'
            idx = int(idx_s) - 1
            if 0 <= idx <= 3:
                cfg.append((idx, -1.0 if int(sign_s) < 0 else 1.0))
        return cfg or [(0, 1.0)]

    @staticmethod
    def _combine(cfg, enc):
        """按 [(索引, 符号), ...] 组合成一个计数值"""
        return sum(sign * enc[idx] for idx, sign in cfg) / float(len(cfg))

    @staticmethod
    def _unwrap(delta, span=65536):
        """处理编码器计数值的 16 位回绕"""
        if delta > span / 2.0:
            delta -= span
        elif delta < -span / 2.0:
            delta += span
        return delta

    @staticmethod
    def _normalize_angle(a):
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def publish_odom(self, _event):
        now = rospy.Time.now()

        with self._enc_lock:
            if not self._enc_valid:
                return
            enc = list(self._enc)

        # 左右轮计数：按 ~enc_left / ~enc_right 配置组合
        # 同侧镜像安装的两个电机计数符号相反，必须相减，不能相加
        left_raw  = self._combine(self.enc_left, enc)
        right_raw = self._combine(self.enc_right, enc)

        left = self.left_sign * left_raw
        right = self.right_sign * right_raw

        if self._prev_left is None or self._prev_right is None or self._prev_time is None:
            self._prev_left, self._prev_right, self._prev_time = left, right, now
            return

        dl = self._unwrap(left - self._prev_left)
        dr = self._unwrap(right - self._prev_right)
        self._prev_left, self._prev_right = left, right

        d_left  = dl * self.dist_per_count
        d_right = dr * self.dist_per_count

        ds  = 0.5 * (d_left + d_right)                    # 车体位移
        dth = (d_right - d_left) / self.wheel_base        # 车体转角

        self._x += ds * math.cos(self._th + 0.5 * dth)
        self._y += ds * math.sin(self._th + 0.5 * dth)
        self._th = self._normalize_angle(self._th + dth)

        dt = (now - self._prev_time).to_sec()
        self._prev_time = now
        if dt > 1e-6:
            self._vx = ds / dt
            self._vth = dth / dt

        qz = math.sin(self._th * 0.5)
        qw = math.cos(self._th * 0.5)

        # ---- /odom ----
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self._vx
        odom.twist.twist.angular.z = self._vth
        # 位姿协方差（x, y, yaw 对角）
        odom.pose.covariance[0]  = 0.02
        odom.pose.covariance[7]  = 0.02
        odom.pose.covariance[35] = 0.05
        # 速度协方差（vx, vth 对角）
        odom.twist.covariance[0]  = 0.02
        odom.twist.covariance[35] = 0.05
        self.odom_pub.publish(odom)

        # ---- TF odom -> base_footprint ----
        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now
            tf_msg.header.frame_id = self.frame_id
            tf_msg.child_frame_id = self.child_frame_id
            tf_msg.transform.translation.x = self._x
            tf_msg.transform.translation.y = self._y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)


if __name__ == '__main__':
    rospy.init_node('cmd_vel_to_stm32')
    try:
        Stm32ChassisBridge()
    except (serial.SerialException, OSError) as e:
        rospy.logfatal("串口打开失败: %s", e)
        raise SystemExit(1)
    rospy.spin()

#docker cp /home/l1ang/Desktop/ros/ros_to_stm32.py ros_noetic:/home/l1ang_ws/src/ydlidar_ros_driver-master/scripts

