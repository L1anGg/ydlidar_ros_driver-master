#!/usr/bin/env python3
import rospy
import tf
import math
import time
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
#from tf_conversions import transformations
from sensor_msgs.msg import LaserScan


THRESHOLD = 0.15  # 阈值 单位米

'''
该路径规划结合了道格拉斯-普克(Douglas-Peuker)抽稀算法，让平滑的路径变为直线，
使得底盘不太灵敏的差分小车能够正常导航,而不会因dwa等发布的平滑路径导致无法到达目标点
'''

class L1angPlanner:
    def __init__(self):
        rospy.init_node('L1angPlanner')
        self.pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        # 发布抽稀后的折线路径(供RViz查看)
        self.path_pub = rospy.Publisher('/simplified_path', Path, queue_size=1, latch=True)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)
        # 订阅AMCL实时位姿
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.next_goal = (0,0)
        self.sub_path = None
        self.threshold = THRESHOLD # 阈值
        self.start_point = []
        self.obstacle_dist = float('inf')          # 正前方最小距离
        self.stop_dist = 0.15                       # 急停阈值(米)
        rospy.Subscriber('/scan', LaserScan, self.scan_cb)

        # ---- 控制参数 ----
        self.yaw_kp   = 1.5     # 转向比例系数
        self.dist_kp  = 1.0     # 前进比例系数
        self.max_v    = 0.3     # 最大线速度 (m/s)
        self.max_w    = 1.5     # 最大角速度 (rad/s)
        self.yaw_tol  = 0.25    # 朝向容差 (rad)
        self.dist_tol = 0.1    # 到达拐点容差 (m)

        # ---- 路径状态 ----
        self.path = []           # 抽稀后的折线
        self.path_index = 0      # 当前跟踪的拐点索引
        self.new_goal_flag = False

    def scan_cb(self, msg):
        n = len(msg.ranges)
        c = n // 2                                  # 正前方索引(0°)
        span = int(20 / 360 * n)                    # 正前方 ±20° 的范围
        front = msg.ranges[c - span : c + span + 1]

        # 过滤 inf / nan / 超量程
        valid = [r for r in front
                if not math.isinf(r) and not math.isnan(r) and r > msg.range_min]
        self.obstacle_dist = min(valid) if valid else float('inf')


    # 实时获取小车当前坐标和朝向
    def amcl_pose(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        # 四元数转欧拉角，获取偏航角
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw
        


    #获取目标点和起点
    def goal_cb(self, msg):
        self.goal = [msg.pose.position.x, msg.pose.position.y]
        rospy.loginfo("收到目标: %.2f, %.2f" % tuple(self.goal))
        self.start_point = [self.current_x, self.current_y]
        rospy.loginfo("起点为: %.2f, %.2f" % tuple(self.start_point))
        self.new_goal_flag = True


    #速度发布函数
    def vel_publisher(self,x,z):
        msg = Twist()
        if self.obstacle_dist < self.stop_dist:
            x, z = 0.0, 0.0                          # 急停
            rospy.logwarn_throttle(1.0, "急停！前方障碍 %.3f m" % self.obstacle_dist)
        msg.linear.x = x
        msg.angular.z = z
        self.pub.publish(msg)

    # ---- 控制循环 (由 Timer 以 10Hz 调用) ----
    def control_cb(self, event):
        self.follow_path()

    # 根据当前位姿和下一个拐点发布速度
    def follow_path(self):
        # 没有路径 or 已走完 → 停车
        if not self.path or self.path_index >= len(self.path):
            self.vel_publisher(0.0, 0.0)
            return

        # 当前目标拐点
        tx, ty = self.path[self.path_index]
        dx = tx - self.current_x
        dy = ty - self.current_y
        dist = math.hypot(dx, dy)

        # 到达当前拐点 → 切换到下一个
        if dist < self.dist_tol:
            rospy.loginfo("到达拐点 %d" % self.path_index)
            self.path_index += 1
            return

        # 朝向误差
        target_yaw = math.atan2(dy, dx)
        yaw_err = self.normalize_angle(target_yaw - self.current_yaw)

        if abs(yaw_err) > self.yaw_tol:
            # 朝向没对 → 原地转
            w = self.clamp(self.yaw_kp * yaw_err, self.max_w)
            self.vel_publisher(0.0, w)
        else:
            # 朝向对 → 直走
            v = self.clamp(self.dist_kp * dist, self.max_v)
            self.vel_publisher(v, 0.0)

    @staticmethod
    def normalize_angle(a):
        """把角度归一化到 [-pi, pi]"""
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    @staticmethod
    def clamp(v, lim):
        return max(-lim, min(lim, v))

    # 发布抽稀后的折线路径(供RViz查看)
    def publish_path(self, points):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'map'
        for p in points:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = p[0]
            ps.pose.position.y = p[1]
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self.path_pub.publish(path_msg)

    #获取全局路径
    def get_globalpath(self):
        self.path_sub = rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.dp_path)
        
    #计算点到首尾所在直线的距离
    def get_point_dis(self,point,head,tail):
        x0, y0 = point
        x1, y1 = head
        x2, y2 = tail
        s = abs((x2 - x1) * (y0 - y1) - (y2 - y1) * (x0 - x1))
        h = math.hypot(x2 - x1, y2 - y1)
        if h == 0:
            return math.hypot(x0 - x1, y0 - y1)
        return s/h 

    def dp_path(self, msg):
        def dp(sp_point):
            if len(sp_point) < 3:
                return sp_point[:]                 # 点太少，直接返回

            head, tail = sp_point[0], sp_point[-1]
            #取最大距离
            max = 0
            p = 0
            for i in range(1,len(sp_point)-1):
                point_dis = self.get_point_dis(sp_point[i],head,tail)
                if point_dis > max:
                    max = point_dis
                    p = i
            if max > self.threshold:
                left = dp(sp_point[:p+1])
                right = dp(sp_point[p:])
                return left[:-1] + right 
            else:
                return [head, tail]
        
        point = []
        #遍历路径的所有点
        for ps in msg.poses:
            x = ps.pose.position.x
            y = ps.pose.position.y
            #计算距离
            point.append([x,y])
        
        simplified = dp(point)
        rospy.loginfo("原始 %d 点 → 抽稀 %d 点" % (len(point), len(simplified)))

        self.path = simplified
        if self.new_goal_flag:          # 只在新目标时重置索引
            self.path_index = 0
            self.new_goal_flag = False

        self.publish_path(simplified)   # 发布到 RViz
        return simplified

            

if __name__ == '__main__':
    planner = L1angPlanner()
    planner.get_globalpath()                                # 订阅全局路径
    rospy.Timer(rospy.Duration(0.1), planner.control_cb)    # 10Hz 控制循环
    rospy.spin()
