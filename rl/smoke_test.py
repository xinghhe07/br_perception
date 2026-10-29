#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rl/smoke_test.py — 强化学习环境冒烟测试(不训练正式策略)

验证三件事:
  1. 依赖版本可打印(便于锁定 / 复现)
  2. SB3 Contrib 的 MaskablePPO 能真的跑起来(不是只有 import 通过)
  3. **动作掩码真的生效** —— 只允许 action 0 时, 训练采样与 predict 都不得选别的

用 `D:\\Anaconda3\\envs\\rl\\python.exe rl/smoke_test.py` 运行。
本测试不依赖本仓库的场地/MuJoCo 模型, 只验证 RL 工具链可用。
"""

import sys

import numpy as np
import gymnasium as gym
from gymnasium import spaces


def versions():
    import importlib
    out = {"python": sys.version.split()[0]}
    for m in ("torch", "gymnasium", "mujoco", "stable_baselines3", "sb3_contrib", "numpy"):
        try:
            mod = importlib.import_module(m)
            out[m] = getattr(mod, "__version__", "?")
        except Exception:
            out[m] = "MISSING"
    return out


class MaskOnlyZero(gym.Env):
    """4 个离散动作, 但只有 action 0 合法; 记录实际被选中的动作。"""

    observation_space = spaces.Box(-1.0, 1.0, shape=(4,), dtype=np.float32)
    action_space = spaces.Discrete(4)

    def __init__(self):
        self.selected = []
        self.steps = 0
        self.max_steps = 8

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.steps = 0
        return np.zeros(4, dtype=np.float32), {}

    def step(self, action):
        self.selected.append(int(action))
        self.steps += 1
        reward = 1.0 if int(action) == 0 else -10.0
        terminated = self.steps >= self.max_steps
        return np.zeros(4, dtype=np.float32), reward, terminated, False, {}

    def action_masks(self):
        mask = np.zeros(4, dtype=bool)
        mask[0] = True          # 只允许 action 0
        return mask


def main() -> int:
    print("== versions ==")
    for k, v in versions().items():
        print("  %-18s %s" % (k, v))

    from sb3_contrib import MaskablePPO
    print("\n== MaskablePPO import OK ==")

    env = MaskOnlyZero()
    model = MaskablePPO(
        "MlpPolicy", env, n_steps=8, batch_size=8, verbose=0, device="cpu",
        policy_kwargs=dict(net_arch=[16, 16]),
    )
    model.learn(total_timesteps=64)

    bad = [a for a in env.selected if a != 0]
    print("训练采样动作: %s" % sorted(set(env.selected)))
    print("非法动作数  : %d" % len(bad))
    assert not bad, "训练采样违反了 action mask!"

    # predict 也要遵守传入的 mask
    acts = set()
    for _ in range(20):
        a, _ = model.predict(np.zeros(4, dtype=np.float32),
                             action_masks=np.array([True, False, False, False]))
        acts.add(int(a))
    print("predict 动作: %s" % sorted(acts))
    assert acts == {0}, "predict 违反了 action mask!"

    print("\nPASS: mask 在采样与推理两侧都生效")
    return 0


if __name__ == "__main__":
    sys.exit(main())
