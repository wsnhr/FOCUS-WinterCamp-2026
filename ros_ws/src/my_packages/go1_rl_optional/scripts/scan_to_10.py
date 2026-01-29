#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

class ScanToTen:
    def __init__(self):
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.out_topic  = rospy.get_param("~out_topic", "/scan_10")
        self.num_bins   = int(rospy.get_param("~num_bins", 10))
        self.print_hz   = float(rospy.get_param("~print_hz", 2.0))  # 0 = 不打印

        # 新增：只取前方FOV（默认180度），中心默认0 rad（正前方）
        self.front_fov_deg = float(rospy.get_param("~front_fov_deg", 180.0))
        self.front_center_rad = float(rospy.get_param("~front_center_rad", 0.0))

        self.latest = None
        self.last_print_t = rospy.Time(0)

        self.pub = rospy.Publisher(self.out_topic, Float32MultiArray, queue_size=10)
        self.sub = rospy.Subscriber(self.scan_topic, LaserScan, self.cb, queue_size=5)

        rospy.loginfo(
            f"[scan_to_10] Sub: {self.scan_topic}  Pub: {self.out_topic}  "
            f"bins={self.num_bins}  front_fov_deg={self.front_fov_deg}"
        )

    @staticmethod
    def _clamp(x, lo, hi):
        return lo if x < lo else hi if x > hi else x

    def cb(self, msg: LaserScan):
        ranges = list(msg.ranges)
        n = len(ranges)
        if n == 0:
            return

        angle_min = float(msg.angle_min)
        angle_max = float(msg.angle_max)
        inc = float(msg.angle_increment) if msg.angle_increment != 0 else None
        if inc is None:
            rospy.logwarn_throttle(2.0, "[scan_to_10] angle_increment is 0, cannot slice by angle.")
            return

        # 1) 计算前方180°对应的角度范围（默认 [-pi/2, +pi/2]）
        half_fov = math.radians(self.front_fov_deg) / 2.0
        want_min = self.front_center_rad - half_fov
        want_max = self.front_center_rad + half_fov

        # 与实际scan角度范围求交集，避免越界（比如雷达不是360也能用）
        use_min = max(want_min, angle_min)
        use_max = min(want_max, angle_max)
        if use_max <= use_min:
            rospy.logwarn_throttle(
                2.0,
                f"[scan_to_10] Front FOV [{want_min:.2f},{want_max:.2f}] not overlapping scan "
                f"[{angle_min:.2f},{angle_max:.2f}]"
            )
            return

        # 2) 角度 -> 索引（四舍五入更贴近实际射线）
        i0 = int(round((use_min - angle_min) / inc))
        i1 = int(round((use_max - angle_min) / inc))

        # clamp索引到合法区间
        i0 = self._clamp(i0, 0, n - 1)
        i1 = self._clamp(i1, 0, n - 1)
        if i1 <= i0:
            return

        front_ranges = ranges[i0:i1+1]

        # 3) 清洗：inf/nan/越界 -> clamp 到 [range_min, range_max]
        rmin = msg.range_min if msg.range_min > 0 else 0.0
        rmax = msg.range_max if msg.range_max > 0 else 10.0

        cleaned = []
        for r in front_ranges:
            if r is None or math.isnan(r) or math.isinf(r):
                r = rmax
            if r < rmin:
                r = rmin
            if r > rmax:
                r = rmax
            cleaned.append(r)

        m = len(cleaned)
        if m == 0:
            return

        # 4) 均分成 num_bins 桶：每桶取最小值
        bins = self.num_bins
        bin_size = max(1, m // bins)

        obs = []
        for i in range(bins):
            start = i * bin_size
            end = (i + 1) * bin_size if i < bins - 1 else m
            segment = cleaned[start:end]
            obs.append(min(segment) if segment else rmax)

        self.latest = obs

        # 5) 发布
        out = Float32MultiArray()
        out.data = obs
        self.pub.publish(out)

        # 6) 可选：限频打印
        if self.print_hz > 0:
            now = rospy.Time.now()
            if (now - self.last_print_t).to_sec() >= (1.0 / self.print_hz):
                self.last_print_t = now
                rospy.loginfo(f"[scan_to_10] front180 bins={bins} {['%.2f'%x for x in obs]}")

def main():
    rospy.init_node("scan_to_10")
    ScanToTen()
    rospy.spin()

if __name__ == "__main__":
    main()
