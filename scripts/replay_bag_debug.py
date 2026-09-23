#!/usr/bin/env python3
"""
replay_bag_debug.py — 测试数据回放（任务_v1.7 §15.4 第 3 层：无实物联调的**唯一手段**）

配对文件：`launch/record_test_bag.launch.py`（录制端）。
**录了不能回放 = 没录** —— 本脚本补的就是那一半。

═══════════════════════════════════════════════════════════════════════════════
它到底做什么：**先体检，再回放**
═══════════════════════════════════════════════════════════════════════════════
  ① **体检**（`--check-only` 可只做这一步）
     读 bag 的 `metadata.yaml`，逐话题列出消息数与类型，并与"**应当有的根数据**"
     清单比对。缺哪一路**明确点名**。

     ⚠️⚠️ 为什么体检比回放本身还重要：
         一个**少录了一路相机**的 bag，回放时那一路**静默地什么都没有** ——
         感知链照跑、话题照出、诊断里那一槽报 NEVER，
         而人很容易把它读成"检测器有问题"。**先花 3 秒体检，省半小时排错。**

  ② **回放**：`ros2 bag play`，带倍速 / 循环 / 起始偏移。

  ⚠️ 读 `metadata.yaml` 而不是 `rosbag2_py`：那是个普通 YAML，
     与仓库其它 Python 脚本的依赖口径一致（只依赖 PyYAML），不需要额外的 ROS Python 包。

═══════════════════════════════════════════════════════════════════════════════
用法
═══════════════════════════════════════════════════════════════════════════════
  # 只体检，不播放（拿不准这个 bag 里有什么时先跑这个）
  python3 scripts/replay_bag_debug.py ~/br_perception_bags/20260923_1430_lab --check-only

  # 回放（1 倍速）
  python3 scripts/replay_bag_debug.py ~/br_perception_bags/20260923_1430_lab

  # 慢放，便于肉眼看 RViz
  python3 scripts/replay_bag_debug.py <bag> --rate 0.5

  # 循环放（反复调参数时省得手动重开）
  python3 scripts/replay_bag_debug.py <bag> --loop

  # ⚠️ 回放时**另开终端**起感知链（本脚本**只放 bag，不起任何节点**）：
  #   ros2 launch br_perception bringup_full.launch.py perception_rviz:=true comm:=false
  #   （⚠️ comm:=false —— 回放时没有真下位机，开着只会刷 device_unavailable）

  # 回放期间该看什么
  ros2 topic hz   /perception/output
  ros2 topic echo /system/degradation_state --once     # 哪几路是 NEVER / STALE
"""

import argparse
import io
import os
import subprocess
import sys

try:
    import yaml
except ImportError:
    # ⚠️ 退出码分层（同 `scripts/check_param_consumers.py`）：
    #    0=正常 / 1=**bag 有问题**（体检不通过）/ 2=**脚本自身跑不起来**
    #    两者撞车 = 信号不可信 = 人开始绕过它。
    print('需要 PyYAML: sudo apt install -y python3-yaml   (不是 python-yaml)',
          file=sys.stderr)
    sys.exit(2)


# ═══════════════════════════════════════════════════════════════════════════════
# 输出编码：**绝不能因为打印不出来就崩**（同 check_param_consumers.py 的教训）
# ═══════════════════════════════════════════════════════════════════════════════
# ⚠️ Windows 默认控制台是 cp936，打不出 `⚠️` 之类的符号会抛 UnicodeEncodeError。
#    处置：**只改 errors、不改 encoding** —— 改 encoding 会让 cp936 控制台
#    收到 UTF-8 字节而按 GBK 解 ⇒ 中文全变乱码（那比崩更糟）。
#    机器可读的标记一律用 ASCII（`[OK]` / `[X]` / `[!]`），不依赖任何符号能否打印。
for _s in (sys.stdout, sys.stderr):
    try:
        _s.reconfigure(errors='replace')
    except (AttributeError, ValueError):
        pass


# ═══════════════════════════════════════════════════════════════════════════════
# "应当有的根数据" —— **必须与 `launch/record_test_bag.launch.py` 的清单一致**
# ═══════════════════════════════════════════════════════════════════════════════
# ⚠️ 两份清单**分居两个文件**，改一边忘一边就会让体检谎报。改录制端时**同步这里**。
REQUIRED = [
    ('/livox/lidar/pointcloud',      '雷达 → 感知链输入'),
    ('/livox/imu',                   'IMU（Point-LIO 用）'),
    ('/camera_wide/image_raw',       '工业相机 宽'),
    ('/camera_tele/image_raw',       '工业相机 长焦'),
    ('/camera/depth/image_rect_raw', 'D435i 深度'),
    ('/tf',                          '坐标系（回放时 TF 也要在）'),
    ('/tf_static',                   '静态外参 TF'),
]
# 录制端开了 `raw_livox:=true` / `perception_out:=true` 时才有 —— **不算缺**，
# 但有就给出来（`/livox/lidar` 在时要能重跑 Point-LIO；输出话题在时能对照两次运行）。
OPTIONAL = [
    ('/livox/lidar',                'CustomMsg（重跑 Point-LIO 用）'),
    ('/aft_mapped_to_init',         '里程计（Point-LIO 输出）'),
    ('/perception/lidar/result',    '雷达结果（对照用）'),
    ('/perception/camera/frame',    '相机聚合帧（对照用）'),
    ('/perception/output',          '最终输出（对照用）'),
    ('/system/degradation_state',   '降级诊断（对照用）'),
]


def find_bag(path):
    """
    定到真正的 bag 目录。

    ⚠️ 用户常犯的两种错，都要能自动纠正并**说一声**（不静默）：
      · 给了 bag 目录里的某个文件（如 `metadata.yaml`）→ 取它的父目录
      · 给了 bag 的**上一级**（如 `~/br_perception_bags`）→ 取它下面唯一的那个子目录
    """
    # ⚠️ **先判存在** —— 路径根本不存在时，下面的 `os.listdir` 会抛
    #    **未捕获的 FileNotFoundError**，而 Python 默认退出码是 **1**，
    #    正好撞上"1 = bag 有问题"的含义（2026-09-23 实测撞了）。
    #    这是同一类错：**两种完全不同的失败长得一样**。
    if not os.path.exists(path):
        print('[X] 路径不存在: {}'.format(path), file=sys.stderr)
        return None

    if os.path.isfile(path):
        path = os.path.dirname(os.path.abspath(path))
    # ⚠️ 是 **`isfile`** 不是 `isdir` —— `metadata.yaml` 是个**文件**。
    #    第一版写成 `isdir` ⇒ 恒 False ⇒ **永远找不到 bag**（写得挺像，错得很静）。
    #    这也是为什么本脚本要拿假 bag 真跑一遍：光看代码看不出来。
    if os.path.isfile(os.path.join(path, 'metadata.yaml')):
        return path

    # 给了上一级：若下面只有一个含 metadata.yaml 的子目录，就用它
    subs = [d for d in sorted(os.listdir(path))
            if os.path.isdir(os.path.join(path, d))
            and os.path.isfile(os.path.join(path, d, 'metadata.yaml'))]
    if len(subs) == 1:
        print('[!] {} 本身不是 bag 目录，自动取它下面唯一的那个: {}'.format(path, subs[0]))
        return os.path.join(path, subs[0])
    if len(subs) > 1:
        print('[X] {} 下有 {} 个 bag，**说清是哪一个**:\n    {}'.format(
            path, len(subs), '\n    '.join(subs)))
        sys.exit(2)
    return None


def read_metadata(bag_dir):
    """读 `metadata.yaml` → (话题→消息数, 总时长秒, 总消息数)。"""
    meta_path = os.path.join(bag_dir, 'metadata.yaml')
    with io.open(meta_path, encoding='utf-8') as f:
        doc = yaml.safe_load(f)
    info = doc.get('rosbag2_bagfile_information', {})
    counts = {}
    for t in info.get('topics_with_message_count', []):
        name = t.get('topic_metadata', {}).get('name')
        if name:
            counts[name] = t.get('message_count', 0)
    dur_ns = info.get('duration', {}).get('nanoseconds', 0)
    return counts, dur_ns / 1e9, info.get('message_count', 0)


def report(bag_dir, counts, dur_s, total):
    """体检报告：逐话题 + 与应有清单比对。返回是否有问题。"""
    print('=' * 78)
    print('bag: {}'.format(bag_dir))
    print('     {} 条消息 / {:.1f} 秒 / {} 个话题'.format(total, dur_s, len(counts)))
    print('=' * 78)

    print('\n① 话题清单（bag 里实际有的）:')
    for name in sorted(counts):
        n = counts[name]
        # ⚠️ 消息数为 0 的话题**要标出来** —— 它和"没有这个话题"在回放时表现一样，
        #    但排查方向不同（一个是录制时话题在但没数据，一个是压根没录）
        mark = '  [!] 0 条' if n == 0 else ''
        print('   {:<34} {:>7} 条{}'.format(name, n, mark))

    problems = 0
    print('\n② 应有根数据:')
    for name, why in REQUIRED:
        if name in counts and counts[name] > 0:
            print('   [OK] {:<32} {:>7} 条   {}'.format(name, counts[name], why))
        elif name in counts:
            print('   [X]  {:<32} **0 条**   {} —— 录制时它在了但没数据'.format(name, why))
            problems += 1
        else:
            print('   [X]  {:<32} **没有**   {}'.format(name, why))
            problems += 1

    present_opt = [(n, w) for n, w in OPTIONAL if n in counts and counts[n] > 0]
    if present_opt:
        print('\n③ 可选（录的时候开了 raw_livox / perception_out 才有）:')
        for name, why in present_opt:
            print('   [OK] {:<32} {:>7} 条   {}'.format(name, counts[name], why))

    if problems:
        print('\n[X] **体检不通过**：{} 路应有根数据缺失/为空。'.format(problems))
        print('    ⚠️ 回放时那几路会**静默地什么都没有**（链照跑、话题照出、'
              '对应槽报 NEVER），\n       很容易被误读成"检测器坏了"。'
              '**先补录，别拿这个 bag 下结论。**')
    else:
        print('\n[OK] 体检通过：应有根数据齐全。')
    return problems


def main():
    ap = argparse.ArgumentParser(
        description='测试数据回放（体检 + ros2 bag play）',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag', help='bag 目录（或它下面的某个文件 / 它的上一级）')
    ap.add_argument('--check-only', action='store_true',
                    help='只体检，不播放。**拿不准 bag 里有什么时先跑这个**')
    ap.add_argument('--rate', type=float, default=1.0,
                    help='播放倍速（默认 1.0；0.5 = 慢放，便于肉眼看 RViz）')
    ap.add_argument('--loop', action='store_true', help='循环播放')
    ap.add_argument('--start-offset', type=float, default=0.0,
                    help='从第 N 秒开始播（跳过开头静止段用）')
    ap.add_argument('--allow-incomplete', action='store_true',
                    help='体检不通过时**仍然播放**（默认不播 —— 见下面那条理由）')
    args = ap.parse_args()

    bag_dir = find_bag(args.bag)
    if not bag_dir:
        print('[X] 在 {} 下找不到 bag（没有 metadata.yaml）'.format(args.bag),
              file=sys.stderr)
        sys.exit(2)

    try:
        counts, dur_s, total = read_metadata(bag_dir)
    except Exception as exc:                       # noqa: BLE001
        print('[X] 读不了 {}: {}'.format(os.path.join(bag_dir, 'metadata.yaml'), exc),
              file=sys.stderr)
        sys.exit(2)

    problems = report(bag_dir, counts, dur_s, total)

    if args.check_only:
        print('\n（--check-only：只体检，不播放）')
        return 1 if problems else 0

    # ⚠️⚠️ **体检不通过时默认不播** —— 理由不是"洁癖"，是**避免拿一个注定误判的
    #    环境去下结论**：少一路的 bag 回放出来，人会去查那一路检测器，
    #    而真正的错在录制。要看症状就显式加 `--allow-incomplete`。
    if problems and not args.allow_incomplete:
        print('\n[X] 体检不通过 → **不播放**。')
        print('    · 想补录: ros2 launch br_perception record_test_bag.launch.py ...')
        print('    · 就想看看: 加 `--allow-incomplete`（但别拿这次的结果下结论）')
        return 1

    if args.rate <= 0.0:
        print('[X] --rate 必须 > 0', file=sys.stderr)
        sys.exit(2)

    cmd = ['ros2', 'bag', 'play', bag_dir, '--rate', str(args.rate)]
    if args.loop:
        cmd.append('--loop')
    if args.start_offset > 0.0:
        cmd += ['--start-offset', str(args.start_offset)]

    print('\n' + '=' * 78)
    print('开始回放: {}'.format(' '.join(cmd)))
    print('⚠️ 本脚本**只放 bag，不起任何节点** —— 另开一个终端起感知链:')
    print('    ros2 launch br_perception bringup_full.launch.py '
          'perception_rviz:=true comm:=false')
    print('⚠️ comm:=false —— 回放时没有真下位机，开着只会刷 device_unavailable')
    print('=' * 78 + '\n')

    try:
        return subprocess.call(cmd)
    except FileNotFoundError:
        # ⚠️ 这是"脚本跑不起来"（环境缺 ros2），不是"bag 有问题" → 退出码 2
        print('[X] 找不到 `ros2` 命令 —— 先 source /opt/ros/humble/setup.bash',
              file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print('\n用户中断')
        return 130


if __name__ == '__main__':
    # ⚠️⚠️ **退出码分层**（同 `scripts/check_param_consumers.py`，理由也一样）：
    #     0 = 正常
    #     1 = **bag 有问题**（体检不通过）
    #     2 = **脚本自身跑不起来**（环境缺依赖 / 参数用法错 / 意外异常）
    #
    #   为什么非要分出 2：1 的含义是"这个 bag 不能用"，而"Python 自己抛了个异常"
    #   默认也是 1 ⇒ **崩了与"bag 坏了"长得一模一样**。对门禁/自动化来说，
    #   被混淆的信号会让人往错的方向修（去补录 bag，而不是去修环境）。
    try:
        sys.exit(main())
    except SystemExit:
        raise
    except BaseException:                      # noqa: BLE001 —— 这里就是要抓全部
        import traceback
        traceback.print_exc()
        print('\n[X] **脚本自身故障**（不是"bag 有问题"）—— 退出码 2。'
              '请连同上面的 traceback 一起反馈', file=sys.stderr)
        sys.exit(2)
