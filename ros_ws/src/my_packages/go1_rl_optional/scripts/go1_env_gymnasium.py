#!/usr/bin/env python3
import time
import numpy as np
import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

import gymnasium as gym
from gymnasium import spaces


class Go1LidarAvoidEnv(gym.Env):
    """
    Gymnasium Env (ROS1 + Gazebo)
    Obs:  /scan_10 (Float32MultiArray, len=10)  —— 你已改为前方180°版本
    Act:  [v_norm, w_norm] -> /cmd_vel           —— v_norm in [0,1], w_norm in [-1,1]
    Reset: /gazebo/reset_world or /gazebo/reset_simulation
    Done:  min(obs) < collision_dist (terminated) or step>=max_steps (truncated)
    """
    metadata = {"render_modes": []}

    def __init__(self):
        super().__init__()

        # Topics
        self.scan10_topic = rospy.get_param("~scan10_topic", "/scan_10")
        self.cmd_topic    = rospy.get_param("~cmd_topic", "/cmd_vel")

        # Timing
        self.dt = float(rospy.get_param("~dt", 0.10))
        self.max_steps = int(rospy.get_param("~max_steps", 400))

        # Action scaling
        self.v_max = float(rospy.get_param("~v_max", 0.6))
        self.w_max = float(rospy.get_param("~w_max", 1.2))

        # Collision
        self.collision_dist = float(rospy.get_param("~collision_dist", 0.35))

        # Reward weights (baseline)
        self.w_forward = float(rospy.get_param("~w_forward", 1.0))
        self.w_crash   = float(rospy.get_param("~w_crash", 5.0))
        self.w_smooth  = float(rospy.get_param("~w_smooth", 0.05))
self.w_turn = float(rospy.get_param("~w_turn", 0.15))      # 转动惩罚
self.w_spin = float(rospy.get_param("~w_spin", 0.20))      # 低速转圈额外惩罚
self.spin_v_thresh = float(rospy.get_param("~spin_v_thresh", 0.05))  # 低速阈值

self.safe_dist = float(rospy.get_param("~safe_dist", 0.60)) # 0.6m 内开始扣
self.w_clear = float(rospy.get_param("~w_clear", 0.30))     # 离障惩罚权重

        # Internal
        self._latest = None
        self._step_count = 0
        self._prev_action = np.zeros(2, dtype=np.float32)

        # ROS I/O
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber(self.scan10_topic, Float32MultiArray, self._scan_cb, queue_size=10)

        # Gazebo reset service (auto pick)
        self.reset_srv = None
        for name in ("/gazebo/reset_world", "/gazebo/reset_simulation"):
            try:
                rospy.wait_for_service(name, timeout=3.0)
                self.reset_srv = rospy.ServiceProxy(name, Empty)
                rospy.loginfo(f"[Go1Env] Using reset service: {name}")
                break
            except Exception:
                pass
        if self.reset_srv is None:
            rospy.logwarn("[Go1Env] No Gazebo reset service found. reset() will only stop the robot.")

        # Gymnasium spaces
        # 你的雷达 range_max=6.0，给 10.0 冗余足够
        self.observation_space = spaces.Box(low=0.0, high=10.0, shape=(10,), dtype=np.float32)
        # action: [v_norm (0..1), w_norm (-1..1)]
        self.action_space = spaces.Box(
            low=np.array([0.0, -1.0], dtype=np.float32),
            high=np.array([1.0,  1.0], dtype=np.float32),
            dtype=np.float32
        )

    def _scan_cb(self, msg: Float32MultiArray):
        arr = np.array(msg.data, dtype=np.float32)
        if arr.shape == (10,):
            self._latest = arr

    def _wait_obs(self, timeout=2.0):
        t0 = time.time()
        while self._latest is None and (time.time() - t0) < timeout and not rospy.is_shutdown():
            rospy.sleep(0.01)
        if self._latest is None:
            raise RuntimeError("Timeout waiting /scan_10. Make sure scan_to_10 is running.")

    def _publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._step_count = 0
        self._prev_action[:] = 0.0

        # stop
        self._publish_cmd(0.0, 0.0)
        rospy.sleep(0.05)

        # reset gazebo
        if self.reset_srv is not None:
            try:
                self.reset_srv()
            except Exception as e:
                rospy.logwarn(f"[Go1Env] reset service call failed: {e}")

        # wait fresh obs
        self._latest = None
        self._wait_obs(timeout=3.0)

        obs = self._latest.copy()
        info = {}
        return obs, info

    def step(self, action):
        self._step_count += 1

        a = np.array(action, dtype=np.float32).reshape(2,)
        v = float(np.clip(a[0], 0.0, 1.0) * self.v_max)
        w = float(np.clip(a[1], -1.0, 1.0) * self.w_max)

        self._publish_cmd(v, w)
        rospy.sleep(self.dt)

        self._wait_obs(timeout=1.0)
        obs = self._latest.copy()

        min_dist = float(np.min(obs))
        crashed = (min_dist < self.collision_dist)

        terminated = bool(crashed)
        truncated  = bool(self._step_count >= self.max_steps)

        # ---- reward terms ----
r_forward = self.w_forward * v

# 1) 转动惩罚：抑制持续大角速度转圈
r_turn = -self.w_turn * abs(w)

# 2) 原地转圈惩罚：v 很小还在转，额外扣
r_spin = 0.0
if v < self.spin_v_thresh:
    r_spin = -self.w_spin * abs(w)

# 3) 离障惩罚：离得越近越扣（在碰撞前就开始有压力）
# 安全距离以内开始扣分（线性），避免突然撞上才扣
r_clear = 0.0
if min_dist < self.safe_dist:
    r_clear = -self.w_clear * (self.safe_dist - min_dist)

# 4) 碰撞大罚
r_crash = -self.w_crash if crashed else 0.0

# 5) 平滑（保留）
r_smooth = -self.w_smooth * float(np.sum((a - self._prev_action) ** 2))

reward = float(r_forward + r_turn + r_spin + r_clear + r_crash + r_smooth)


        self._prev_action = a

        info = {"min_dist": min_dist, "crashed": crashed, "v": v, "w": w, "step": self._step_count}
        return obs, reward, terminated, truncated, info

    def close(self):
        self._publish_cmd(0.0, 0.0)
