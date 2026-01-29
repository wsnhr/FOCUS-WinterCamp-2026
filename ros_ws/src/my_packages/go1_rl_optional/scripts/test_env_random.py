#!/usr/bin/env python3
import numpy as np
import rospy
from go1_env_gymnasium import Go1LidarAvoidEnv

def main():
    rospy.init_node("test_env_random", anonymous=True)
    env = Go1LidarAvoidEnv()

    for ep in range(3):
        obs, _ = env.reset()
        ep_ret = 0.0
        for t in range(300):
            # random action: v_norm in [0,1], w_norm in [-1,1]
            a = np.array([np.random.rand(), np.random.uniform(-1, 1)], dtype=np.float32)
            obs, r, terminated, truncated, info = env.step(a)
            ep_ret += r
            if terminated or truncated:
                print(f"[EP{ep}] done t={t} return={ep_ret:.2f} info={info}")
                break
        if not (terminated or truncated):
            print(f"[EP{ep}] finished return={ep_ret:.2f}")

    env.close()

if __name__ == "__main__":
    main()
