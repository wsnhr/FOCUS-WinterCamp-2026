#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Twist

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def slew(prev, target, max_rate, dt):
    # max_rate: 每秒最大变化量（m/s^2 或 rad/s^2）
    delta = target - prev
    max_delta = max_rate * dt
    if delta > max_delta:  return prev + max_delta
    if delta < -max_delta: return prev - max_delta
    return target

def publish_ramp(pub, lx, az, duration, rate_hz=50, ramp_ratio=0.25,
                 max_lin_acc=0.25, max_ang_acc=0.45,
                 state=None):
    """
    简单0->1->0 ramp + 限加速度（重点）
    state: 用来保存上一时刻输出，跨段连续更稳
    """
    if state is None:
        state = {"vx": 0.0, "wz": 0.0}

    dt = 1.0 / float(rate_hz)
    total_steps = max(1, int(duration * rate_hz))

    ramp_steps = max(1, int(total_steps * ramp_ratio))
    const_steps = max(0, total_steps - 2 * ramp_steps)

    rate = rospy.Rate(rate_hz)

    for step in range(total_steps):
        if rospy.is_shutdown():
            break

        if step < ramp_steps:
            scale = step / float(ramp_steps)
        elif step < ramp_steps + const_steps:
            scale = 1.0
        else:
            scale = (total_steps - 1 - step) / float(ramp_steps)

        vx_t = lx * scale
        wz_t = az * scale

        # ✅ 限加速度/角加速度：抑制“转弯开始脚歪”和“转弯甩”
        vx = slew(state["vx"], vx_t, max_lin_acc, dt)
        wz = slew(state["wz"], wz_t, max_ang_acc, dt)
        state["vx"], state["wz"] = vx, wz

        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        pub.publish(msg)
        rate.sleep()

    return state

def stop(pub, seconds=0.05, rate_hz=50, state=None):
    # 短暂发0，不要太久
    return publish_ramp(pub, 0.0, 0.0, seconds, rate_hz=rate_hz,
                        ramp_ratio=0.8, max_lin_acc=0.25, max_ang_acc=0.45,
                        state=state)

def main():
    rospy.init_node("go1_square_cmdvel_smoother", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rospy.sleep(1.0)

    rate_hz = 50
    state = {"vx": 0.0, "wz": 0.0}

    rospy.on_shutdown(lambda: stop(pub, seconds=0.4, rate_hz=rate_hz, state=state))

    # 直行参数
    v = 0.15
    side = 6.0

    # ✅ 转弯参数：保持正常wz，但转弯给一点前进（走弧线，脚不拧）
    wz = 0.22
    lx_turn = 0.03

    # 预加载：解决“转弯开始脚歪”
    pre_t = 0.20
    pre_wz = 0.08

    # 注意：弧线转弯时，严格90°不再只由 turn=(pi/2)/wz 决定，但开环先这样够用
    turn = (math.pi/2.0) / wz

    rospy.loginfo("Publishing /cmd_vel to move Go1 in a square (stable turning)...")

    for i in range(4):
        rospy.loginfo(f"Edge {i+1}/4: forward")
        state = publish_ramp(pub, lx=v, az=0.0, duration=side,
                             rate_hz=rate_hz, ramp_ratio=0.25,
                             max_lin_acc=0.25, max_ang_acc=0.45, state=state)
        state = stop(pub, seconds=0.05, rate_hz=rate_hz, state=state)

        rospy.loginfo(f"Turn {i+1}/4: pre-turn then arc turn")
        # ✅ 预加载：小转向 + 小前进（很短）
        state = publish_ramp(pub, lx=lx_turn, az=pre_wz, duration=pre_t,
                             rate_hz=rate_hz, ramp_ratio=0.6,
                             max_lin_acc=0.25, max_ang_acc=0.45, state=state)

        # ✅ 正式转弯：ramp更大，收尾更软
        state = publish_ramp(pub, lx=lx_turn, az=wz, duration=turn,
                             rate_hz=rate_hz, ramp_ratio=0.45,
                             max_lin_acc=0.25, max_ang_acc=0.45, state=state)
        state = stop(pub, seconds=0.04, rate_hz=rate_hz, state=state)

    stop(pub, seconds=0.6, rate_hz=rate_hz, state=state)
    rospy.loginfo("Done.")

if __name__ == "__main__":
    main()

