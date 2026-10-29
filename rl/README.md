# rl/ — BR 决胜策略训练(强化学习)

本目录是 `doc/强化学习决策与导航方案.md` 的**训练侧代码**。与 ROS 侧(`src/`)完全分离:
ROS 只加载导出的 ONNX actor 并做动作掩码/安全过滤, 不在此目录运行。

## 现状

| 项 | 状态 |
|---|---|
| 依赖环境 `D:\Anaconda3\envs\rl`(Python 3.11) | ✅ torch 2.11.0+cu128 / gymnasium 1.4.0 / mujoco 3.15.0 |
| SB3 / SB3 Contrib | ✅ 已装 **2.9.2** 并锁定,见 `requirements.txt` |
| Maskable PPO 可用性 | ✅ `smoke_test.py` 通过(mask 在**采样与推理两侧**都生效) |
| 快速比赛环境(Gymnasium) | ⬜ 未实现(方案 §6.1) |
| MuJoCo MJCF 场地 + 机器人 | ⬜ 未实现(方案 §6.3, 由 `config/field_geometry.yaml` 生成) |
| ONNX 导出与 ROS 策略节点 | ⬜ 未实现(方案 §9 Phase 4) |

## 用法

```bash
# 1. 环境(仅首次)
D:\Anaconda3\envs\rl\python.exe -m pip install -r rl/requirements.txt \
    --extra-index-url https://download.pytorch.org/whl/cu128

# 2. 冒烟测试: 版本 + MaskablePPO + mask 有效性
D:\Anaconda3\envs\rl\python.exe rl/smoke_test.py
```

## 约定

- **不要**在 `src/` 里直接调用 Python 训练栈;两者通过 ONNX + 消息接口解耦。
- **规则版本化**:记分器/动作合法性必须带 V3 规则版本标识(见方案 §3.1);
  V3 草案里"五色石 250 vs 255""L1-L2 有无阶梯"等冲突项**未裁定前不得写死常数**。
- 动作掩码是训练与部署的**共同契约**:训练用 SB3 的 `action_masks`,
  部署侧在 C++ 安全过滤里再独立校验一遍,不能只靠模型。
