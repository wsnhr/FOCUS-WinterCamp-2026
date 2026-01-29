#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class VisualServoRed:
    """
    视觉伺服(Visual Servoing) - 红色目标追踪 + 搜索目标：

    核心流程（满足你提出的要求）：
    1) 使用 cv_bridge 将 ROS 图像消息转换为 OpenCV BGR 格式
    2) 将图像转换至 HSV 空间，过滤出红色区域（两段阈值）
    3) 计算红色色块的质心(Centroid) 和 像素面积(Area)
    4) 闭环控制：
       - 转向控制(Yaw)：质心相对图像中心的偏差，用 P 控制器输出角速度
       - 距离控制(Forward)：根据面积判断距离，面积小则前进，达到阈值则停止

    新增逻辑（你刚提的要求）：
    5) 当视野内没有目标时：不再原地停住，而是“带转弯半径地旋转搜索目标”
       - 通过同时给线速度 v 和角速度 w 实现圆弧运动
       - 转弯半径 R = v / |w|，避免“原地扭脚”
    """

    def __init__(self):
        # --------------------------
        # 1) ROS 输入输出话题
        # --------------------------
        self.image_topic = rospy.get_param("~image_topic", "/camera_face/color/image_raw")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")

        self.bridge = CvBridge()
        self.pub_cmd = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.sub_img = rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)

        # --------------------------
        # 2) 视觉伺服控制参数
        # --------------------------
        # P 控制器增益：越大转向越“激进”，容易抖；越小越稳但反应慢
        self.kp_yaw = rospy.get_param("~kp_yaw", 0.6)
        # 限制最大角速度，防止转太快导致不稳
        self.yaw_max = rospy.get_param("~yaw_max", 0.6)

        # 前进速度（看到目标且距离较远时）
        self.v_forward = rospy.get_param("~v_forward", 0.15)
        # 面积阈值：面积 >= area_stop 时认为“足够近”，停止前进
        self.area_stop = rospy.get_param("~area_stop", 12000)
        # 面积太小当噪声：小于该值不当成目标（避免误检小红点）
        self.area_min = rospy.get_param("~area_min", 50)

        # --------------------------
        # 3) 搜索目标（无目标时）的运动参数 —— 新增
        # --------------------------
        # 搜索时线速度（m/s）：建议 0.05~0.12，越大转弯半径越大
        self.search_v = rospy.get_param("~search_v", 0.10)
        # 搜索时角速度（rad/s）：建议 0.15~0.40，越小越“温柔”
        self.search_w = rospy.get_param("~search_w", 0.25)
        # 搜索方向：+1 左转；-1 右转（一直同方向转圈更简单更稳）
        self.search_dir = rospy.get_param("~search_dir", 1)

        # 丢失目标后延迟多久才进入搜索（秒）
        # 作用：避免偶尔一帧丢检就立刻转圈，导致抖动/抽动
        self.lost_delay = rospy.get_param("~lost_delay", 0.2)
        self.last_seen = rospy.Time(0)  # 上次看到目标的时间戳

        # --------------------------
        # 4) Debug 显示
        # --------------------------
        self.debug_view = rospy.get_param("~debug_view", True)

        rospy.loginfo("VisualServoRed started.")
        rospy.loginfo("  image_topic : %s", self.image_topic)
        rospy.loginfo("  cmd_topic   : %s", self.cmd_topic)
        rospy.loginfo("  search mode : v=%.3f m/s, w=%.3f rad/s, R=%.3f m, dir=%s",
                      self.search_v, self.search_w,
                      (self.search_v / abs(self.search_w)) if abs(self.search_w) > 1e-6 else 0.0,
                      "left" if self.search_dir >= 0 else "right")

    @staticmethod
    def _find_red_mask(hsv: np.ndarray) -> np.ndarray:
        """
        红色在 HSV 色环上跨越 0°，所以用两段阈值（低 hue 和高 hue）合并。
        注意：S/V 阈值用于抑制灰暗/白亮区域的误检。
        """
        lower1 = np.array([0, 120, 70])
        upper1 = np.array([10, 255, 255])
        lower2 = np.array([170, 120, 70])
        upper2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower1, upper1)
        mask2 = cv2.inRange(hsv, lower2, upper2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 去噪：开运算去小点，再膨胀让目标区域更连贯
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)

        return mask

    def _publish_search_twist(self):
        """
        无目标时的搜索策略：
        - 不原地转（避免扭脚）
        - 采用圆弧运动：同时给 linear.x 和 angular.z
        - 转弯半径 R = v / |w|
        """
        twist = Twist()

        # 若角速度为 0，会变成直线走（不符合“旋转找目标”）
        w = self.search_dir * abs(self.search_w)
        v = self.search_v

        twist.linear.x = v
        twist.angular.z = w

        self.pub_cmd.publish(twist)

    def image_cb(self, msg: Image):
        # --------------------------
        # 1) cv_bridge：ROS Image -> OpenCV BGR
        # --------------------------
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logwarn("cv_bridge convert failed: %s", e)
            return

        h, w = bgr.shape[:2]

        # --------------------------
        # 2) BGR -> HSV，红色分割
        # --------------------------
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = self._find_red_mask(hsv)

        # --------------------------
        # 3) 找红色区域：取最大轮廓作为目标
        # --------------------------
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 先准备一个 Twist，后面按不同逻辑填充
        twist = Twist()

        # --------------------------
        # 情况 A：没有检测到红色目标 -> 搜索模式
        # --------------------------
        if not contours:
            now = rospy.Time.now()
            # 目标丢失不立刻转圈，等 lost_delay 过后再进入搜索，避免抖动
            if (now - self.last_seen).to_sec() >= self.lost_delay:
                self._publish_search_twist()
            else:
                # 短暂丢检：保持停
                self.pub_cmd.publish(Twist())

            if self.debug_view:
                cv2.imshow("mask", mask)
                cv2.imshow("view", bgr)
                cv2.waitKey(1)
            return

        # 找最大红色块
        c = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(c))

        # --------------------------
        # 情况 B：面积太小 -> 当噪声处理 -> 也走“搜索/等待”逻辑
        # --------------------------
        if area < self.area_min:
            now = rospy.Time.now()
            if (now - self.last_seen).to_sec() >= self.lost_delay:
                self._publish_search_twist()
            else:
                self.pub_cmd.publish(Twist())

            if self.debug_view:
                cv2.imshow("mask", mask)
                cv2.imshow("view", bgr)
                cv2.waitKey(1)
            return

        # 走到这里表示：确实看到一个有效红色目标
        self.last_seen = rospy.Time.now()

        # 计算质心（Centroid）
        M = cv2.moments(c)
        cx = int(M["m10"] / (M["m00"] + 1e-6))
        cy = int(M["m01"] / (M["m00"] + 1e-6))

        # --------------------------
        # 4) 闭环控制
        # --------------------------
        # 4.1 Yaw：质心偏离图像中心的误差 -> P 控制
        # err_x 归一化到 [-1, 1]，便于调参不受分辨率影响
        err_x = (cx - (w / 2.0)) / (w / 2.0)

        # 角速度输出：若发现“转向方向反了”，把下面这行的负号去掉即可
        yaw = -self.kp_yaw * err_x
        yaw = max(-self.yaw_max, min(self.yaw_max, yaw))
        twist.angular.z = yaw

        # 4.2 Forward：通过面积判断距离
        # 面积小：目标远 -> 前进
        # 面积达到阈值：目标近 -> 停止
        if area < self.area_stop:
            twist.linear.x = self.v_forward
        else:
            twist.linear.x = 0.0

        # 发布速度命令
        self.pub_cmd.publish(twist)

        # --------------------------
        # Debug 可视化：mask + 原图叠字
        # --------------------------
        if self.debug_view:
            cv2.circle(bgr, (cx, cy), 6, (0, 255, 0), -1)  # 质心点
            cv2.line(bgr, (w // 2, 0), (w // 2, h), (255, 255, 255), 1)  # 中心线

            # 额外显示搜索半径信息，方便你讲解/答辩
            R = (self.search_v / abs(self.search_w)) if abs(self.search_w) > 1e-6 else 0.0

            cv2.putText(
                bgr,
                f"cx={cx} area={area:.0f} err={err_x:.2f} yaw={yaw:.2f} v={twist.linear.x:.2f}  (search R~{R:.2f}m)",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
            )
            cv2.imshow("mask", mask)
            cv2.imshow("view", bgr)
            cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("visual_servo_red")
    VisualServoRed()
    rospy.spin()

