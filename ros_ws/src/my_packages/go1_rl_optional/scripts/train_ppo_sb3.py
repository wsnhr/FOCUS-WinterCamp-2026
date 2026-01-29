#!/usr/bin/env python3
import os
import rospy
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from go1_env_gymnasium import Go1LidarAvoidEnv

def main():
    rospy.init_node("train_ppo_sb3", anonymous=True)

    logdir = rospy.get_param("~logdir", "runs/go1_ppo")
    total_timesteps = int(rospy.get_param("~total_timesteps", 200_000))
    model_path = rospy.get_param("~model_path", "models/go1_ppo.zip")

    os.makedirs(logdir, exist_ok=True)
    os.makedirs(os.path.dirname(model_path) or ".", exist_ok=True)

    env = DummyVecEnv([lambda: Go1LidarAvoidEnv()])

    model = PPO(
        "MlpPolicy",
        env,
        verbose=1,
        tensorboard_log=logdir,
        n_steps=2048,
        batch_size=64,
        gamma=0.99,
        learning_rate=3e-4,
    )

    model.learn(total_timesteps=total_timesteps)
    model.save(model_path)
    print(f"Saved model to: {model_path}")

if __name__ == "__main__":
    main()
