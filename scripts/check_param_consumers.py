#!/usr/bin/env python3
"""
check_param_consumers.py — 找出"改了没效果还不报错"的 ROS 2 参数

═══════════════════════════════════════════════════════════════════════════════
为什么需要它（这个坑本仓库已经踩过多次）
═══════════════════════════════════════════════════════════════════════════════
ROS 2 的两条静默行为合起来很危险：
  1. **yaml 里未声明的参数被静默忽略** —— 改了不生效，不报错、不打日志；
  2. **同名键出现在多个 yaml 时，谁生效取决于 launch 的 parameters 顺序**
     —— 调个顺序、或新写个 launch 忘了排最后，"本应是总闸的那个文件"就失效了。

两者的症状都是"看起来配好了，实际没接上"，到现场才暴露。

三类报告：
  A. **无消费者的键** —— config 里有，但 `src/` 里没有任何 `declare_parameter` 读它。
     改它**没有任何效果**。应白名单化（见下方 WHITELIST），白名单外的直接报错。
  B. **同名键出现在多个 yaml** —— "两处真源"。**修法是删掉重复键**，
     让它只依赖内容、不依赖 launch 顺序。
  C. **声明了但没有任何 yaml 提供** —— 只能吃代码里的默认值。
     多半是正常的（有默认值），但列出来便于发现"以为配了其实没配"。

⚠️ 本脚本是**启发式**（正则扫源码），不是编译器。它能抓的是主要形态：
     `declare_parameter<T>("k", ...)` / `declare_parameter("k", ...)` /
     `get_or_declare(node, "k", ...)`。将来若有人用宏或循环声明参数，这里会漏 ——
     所以**报告要人看**，不要无声地当 CI 通过。

用法:
  python3 scripts/check_param_consumers.py            # 只报告
  python3 scripts/check_param_consumers.py --strict   # 有 A/B 类问题 → 退出码 1
"""

import argparse
import io
import os
import re
import sys

try:
    import yaml
except ImportError:
    sys.exit('需要 PyYAML: sudo apt install -y python3-yaml   (不是 python-yaml)')


# ═══════════════════════════════════════════════════════════════════════════════
# 白名单：**已知的、无消费者**的键
# ═══════════════════════════════════════════════════════════════════════════════
# ⚠️ 每条都必须带"接线到期条件" —— 没有到期条件的占位键会**永久挂账**，
#    那正是本仓库最想避免的（"看起来有人在管"）。
# ⚠️ 往这里加条目前先问：它**什么时候**会被接上？答不上来就别加，直接删键。
WHITELIST = {
    'use_sim_time':            'ROS 2 内建参数，由 rclcpp 自己读，不在 src/ 里 declare',
    'publish_debug_image':     '相机层"画框回图"未实现。到期：联调期需要时',
    'save_crash_dump':         '策略未定（存什么/存哪/多大）。到期：P1，且必须先定容量上限',
    'lock_parameters':         '比赛期禁止改参数的闸。到期：health_monitor(模块26) 落地时',
    'thermal_management_enabled':  '到期：health_monitor(模块26) 落地时',
    'cpu_temp_warn_threshold':     '到期：health_monitor(模块26) 落地时',
    'cpu_temp_recover_threshold':  '到期：health_monitor(模块26) 落地时',
    'thermal_degrade_camera':      '到期：health_monitor(模块26) 落地时；取值策略本身也待决策',
    'thermal_preview_hz':          '到期：health_monitor(模块26) 落地时',
    'thermal_preview_enabled':     '到期：health_monitor(模块26) 落地时',
}

# ROS 2 用来给节点传 QoS 覆盖等的保留键前缀（rclcpp 自己处理）
RESERVED_PREFIXES = ('qos_overrides.',)

# ⚠️ **整块子系统尚未编写** → 那一族的键"无消费者"是预期的。
#    同 WHITELIST：每条必须带**接线到期条件**。
WHITELIST_PREFIXES = {
    'kalman.':
        '模块 18 kalman_tracker ⬜ 未写。到期：融合层落地时',
    'association.':
        '模块 19 target_associator ⬜ 未写（且 §四 已定"范围收窄为查表，不做匈牙利"）。到期：融合层落地时',
    'tracking.':
        '模块 18/19 的下游参数 ⬜。到期：融合层落地时',
    'temporal_sync.':
        '模块 20/21 融合层 ⬜ 未写。到期：融合层落地时',
    'shared_zone.':
        '模块 20 lidar_camera_fusion ⬜ 未写。到期：融合层落地时',
    'lidar_weight':
        '模块 20 融合权重 ⬜。到期：融合层落地时',
    'camera_weight':
        '模块 20 融合权重 ⬜。到期：融合层落地时',
    'inference':
        '模块 8/9（earth/sky_block_detector）⬜ 未写，且卡模型。到期：模型到位时',
    'extrinsic_monitor.':
        '模块 28 ⛔ 已降级为调试工具（`enabled` 默认 false，不占主线）。到期：'
        '真实相机标定完成后，再决定做不做',
    'confirm_frame_count':
        '模块 18 kalman_tracker 的确认帧数 ⬜。到期：融合层落地时',
    'delete_frame_count':
        '模块 18 kalman_tracker 的删除帧数 ⬜。到期：融合层落地时',
    # ⚠️ 下面这几个**不是"未写"，是"死配置"** —— 差别很重要：死配置该删，不该留。
    'static_tf.':
        '⚠️ **死配置**：launch 用 static_transform_publisher 写死数值，没有任何节点读这些键'
        '（台账 §七 已记）。到期：**重新决定 TF 数值从哪来**时 —— 二选一：要么删键、'
        '要么改成由 yaml 驱动 launch。',
}

# ⚠️ **整个文件不由本包的节点消费**的 yaml —— 扫描时跳过。
#    不跳的话它们会刷出成百条假"无消费者"，把真问题淹掉（工具喊狼来了等于没用）。
EXTERNAL_YAML = {
    'pointlio_gazebo.yaml':
        'Point-LIO 外部包的参数（消费方是 ~/lio_ws 的 point_lio 节点，不在本仓库）',
}

# ⚠️ **一个文件同时是"运行期 ROS 参数"和"脚本的数据源"** 的情况。
#
#    `field_geometry.yaml` 被 `bringup_full` 传给节点（节点只 declare 它要的那几十个键），
#    但**整份文件**同时是地图生成链的输入（`scripts/field_model_generator.py` /
#    `make_field_from_config.py` / `render_map.py` 直接按 dict 读，**不走 declare_parameter**）。
#    所以那些 zone_colors / stairs / ramp / building_spots.* 之类的几何常量
#    "没有 C++ 消费者"是**正常的**，不是漏配。
#    ⚠️ 不把这类分开的话，一份 field_geometry 就能刷出两百多条假警告。
TOOLING_FILES = {
    'field_geometry.yaml':
        '地图几何常量：节点只读其中几十个键，其余由 scripts/*.py 地图生成链按 dict 直接读',
}

# 声明参数的写法（三种，逐个匹配）
# ⚠️ 模板参数**必须用非贪婪 `.*?` 而不是 `[^>]*`** —— 本仓库大量出现嵌套模板：
#      `declare_parameter<std::vector<double>>("building_spot_x")`
#    用 `[^>]*` 会在第一个 `>`（`<double` 的那个）就停，匹配失败 →
#    把**有消费者**的键误报成"无消费者"。第一版就是这么错的，一跑就刷了几百条假阳性。
#    `.*?>` + 紧跟 `(` 的组合能正确吃到最后一个 `>`。
DECLARE_PATTERNS = [
    re.compile(r'declare_parameter\s*<.*?>\s*\(\s*"([^"]+)"'),
    re.compile(r'declare_parameter\s*\(\s*"([^"]+)"'),
    # 本仓库 coordinate_transformer 用的辅助函数
    re.compile(r'get_or_declare\s*\(\s*\w+\s*,\s*"([^"]+)"'),
]


def flatten(d, prefix=''):
    """把嵌套 dict 拍平成 ROS 2 的点号键（`odometry: {topic: x}` → `odometry.topic`）"""
    for k, v in d.items():
        key = '{}{}'.format(prefix, k)
        if isinstance(v, dict):
            yield from flatten(v, key + '.')
        else:
            yield key


def load_param_yaml(path):
    """
    读一个 ROS 2 参数文件 → `{键: [作用域, ...]}`；不是参数文件则返回 None。

    ⚠️ **必须区分参数文件与普通 YAML**：`config/camera_hardware.yaml` 是给脚本读的
       纯数据（没有 `ros__parameters`），把它当参数文件会报一堆假"无消费者"。
    """
    with io.open(path, encoding='utf-8') as f:
        doc = yaml.safe_load(f)
    if not isinstance(doc, dict):
        return None

    out = {}
    for scope, body in doc.items():
        if not isinstance(body, dict) or 'ros__parameters' not in body:
            continue           # 不是 ROS 参数文件（或该作用域不合法）→ 跳过
        for key in flatten(body['ros__parameters']):
            out.setdefault(key, []).append(scope)
    return out or None


def scan_declares(src_dirs):
    # ⚠️ 下面是 **raw 字符串**（前缀 r），不是多余的：本说明里有 `\s` 这类正则片段，
    #    普通字符串会触发 `SyntaxWarning: invalid escape sequence`（每次运行刷一条）。
    #    ⚠️ 也别在说明里写三引号本身 —— 那会**提前终止 docstring**，后面的文字变成代码。
    r"""
    扫源码里的 declare_parameter，返回 {键: [文件:行, ...]}

    ⚠️⚠️ **必须整文件扫，不能逐行扫** —— 本仓库大量出现**跨行**的调用：
        `this->declare_parameter<std::vector<double>>(`
        `    "height_threshold_z_keys", {0.0, 1.0, 3.0});`
        逐行扫时 `(` 与 `"key"` 不在同一行 → 匹配失败 →
        把**活的键**误报成"无消费者"。那比假阳性更危险：
        照报告去"删掉没人读的键"，删掉的是**真在用的**。
        （第二版就是这么错的：`ground_segmenter` 的两个阈值键被误报成孤儿。）
        现在按**整段文本**匹配，行号由字符偏移反算。
        ⚠️ 正则里的 `\s` 天然包含换行，所以 `>` 与 `(` 之间、`(` 与 `"key"` 之间
           跨行都吃得下；不需要 DOTALL。
    """
    found = {}
    for root_dir in src_dirs:
        for dirpath, _dirnames, filenames in os.walk(root_dir):
            for name in filenames:
                if not name.endswith(('.cpp', '.hpp')):
                    continue
                path = os.path.join(dirpath, name)
                text = io.open(path, encoding='utf-8', errors='replace').read()
                rel = os.path.relpath(path).replace('\\', '/')
                for pat in DECLARE_PATTERNS:
                    for m in pat.finditer(text):
                        lineno = text.count('\n', 0, m.start()) + 1
                        found.setdefault(m.group(1), []).append('{}:{}'.format(rel, lineno))
    return found


def scan_declares_by_file(src_dirs):
    """{源文件相对路径: {键, ...}} —— 按文件分组，供 launch 可达性检查用。"""
    per_file = {}
    for root_dir in src_dirs:
        for dirpath, _dirnames, filenames in os.walk(root_dir):
            for name in filenames:
                if not name.endswith(('.cpp', '.hpp')):
                    continue
                path = os.path.join(dirpath, name)
                text = io.open(path, encoding='utf-8', errors='replace').read()
                keys = set()
                for pat in DECLARE_PATTERNS:
                    keys.update(pat.findall(text))
                if keys:
                    per_file[os.path.relpath(path).replace('\\', '/')] = keys
    return per_file


def scan_launch(launch_path, config_names):
    """
    从一个 launch 文件里粗略抽出：它引用了哪些 config yaml、起了哪些节点。

    ⚠️ **这是正则近似，不是执行 launch** —— 本机没有 ROS，`import launch` 跑不起来。
       够用的原因：本仓库的 launch 就那么两种写法（`os.path.join(..., 'config', 'x.yaml')`
       与 `name='...'`），没上元编程。**但要人看报告，不能无声当 CI 通过。**
    """
    text = io.open(launch_path, encoding='utf-8', errors='replace').read()
    yamls = set()
    for m in re.finditer(r"'config'\s*,\s*'([A-Za-z0-9_]+\.yaml)'", text):
        if m.group(1) in config_names:
            yamls.add(m.group(1))
    nodes = set(re.findall(r"name\s*=\s*'([a-z][a-z0-9_]+)'", text))
    # 组装式 launch（bringup_debug 用 NODE_SPECS 元组）里节点名不在 name= 上 —— 补一种
    nodes.update(re.findall(r"'([a-z][a-z0-9_]+)',\s*\n\s*'br_perception::", text))

    # ⚠️ **内联参数字典也要算"已提供"**：桥接节点（`livox_custom_to_pcl2` 等）的
    #    参数是 `parameters=[{'input_topic': ..., 'frame_id': ...}]` 直接写在 launch 里的，
    #    不经过任何 yaml。只认文件名的话，这类节点会被误报成"缺一堆键"。
    #    ⚠️ 这里刻意**宁可多收**（`launch_arguments={'rviz': ...}` 之类也会被收进来）——
    #       多收只会减少假阳性；真缺的键仍然会因为"全 launch 里都没出现"而被报出来。
    inline = set(re.findall(r"'([a-z][a-z0-9_]*)'\s*:", text))

    return yamls, nodes, inline


def main():
    ap = argparse.ArgumentParser(description='找出"改了没效果"的 ROS 2 参数')
    ap.add_argument('--strict', action='store_true',
                    help='有 A/B 类问题则退出码 1（给 CI / 预提交用）')
    ap.add_argument('--root', default=os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    help='仓库根（默认取本脚本的上层目录）')
    args = ap.parse_args()

    root = args.root
    config_dir = os.path.join(root, 'config')
    src_dir = os.path.join(root, 'src')

    if not os.path.isdir(config_dir) or not os.path.isdir(src_dir):
        sys.exit('在 {} 下找不到 config/ 与 src/ —— 用 --root 指仓库根'.format(root))

    # ── 收集 ──
    yaml_keys = {}           # 键 → [文件名, ...]
    scope_is_wildcard = {}   # 键 → [该文件是不是 '/**' 通配, ...]，与 yaml_keys 同序
    for name in sorted(os.listdir(config_dir)):
        if not name.endswith(('.yaml', '.yml')):
            continue
        if name in EXTERNAL_YAML:
            print('[跳过] {} —— {}'.format(name, EXTERNAL_YAML[name]))
            continue
        keys = load_param_yaml(os.path.join(config_dir, name))
        if keys is None:
            continue
        for k, sc in keys.items():
            yaml_keys.setdefault(k, []).append(name)
            scope_is_wildcard.setdefault(k, []).append('/**' in sc)

    declares = scan_declares([src_dir])

    # ── A. 无消费者的键 ──
    #    ⚠️ 拆成两类：A1 真问题（未获准）；A2 由脚本消费（TOOLING_FILES）——后者是信息
    tooling_only = {k for k, files in yaml_keys.items()
                    if k not in declares and all(f in TOOLING_FILES for f in files)}
    no_consumer = sorted(k for k in yaml_keys
                         if k not in declares and k not in tooling_only)

    # ── B. 同名键出现在多个 yaml ──
    #    ⚠️⚠️ 必须分两类，否则会淹没真问题：
    #      B1 **两个都是 `/**` 通配** → **真风险**。通配文件对任何节点都生效，
    #         而只要同一个节点同时收到这两份，**谁生效就取决于 launch 的 parameters 顺序**。
    #         `debug_enabled` 那次就是这么翻的（lidar_params.yaml + debug_flags.yaml）。
    #      B2 节点名作用域（`comm_node:` / `perception_pipeline:` …）→ 一般是**合法**的：
    #         各节点读各自的文件，互不覆盖（例如 `output_topic` 是 comm_node 订阅的那头、
    #         perception_pipeline 发布的那头 —— **配对的两个键，同名但不同消费者**）。
    #         它仍然值得看一眼（可能只是命名撞车），但**不是**"两处真源"。
        # ⚠️ 每个文件**各自**的作用域要分开记，不能用 scopes[k] 的扁平列表：
        #    那个列表是所有文件的并集，判不出"这一份是不是通配"。
    dup_wild = []      # 真风险：≥2 份文件里都是 '/**' 通配
    dup_other = []     # 一般是合法的：各读各的节点作用域文件
    for k, files in sorted((k, v) for k, v in yaml_keys.items() if len(v) > 1):
        if scope_is_wildcard[k].count(True) >= 2:
            dup_wild.append((k, files))
        else:
            dup_other.append((k, files))
    dup = dup_wild

    # ── C. 声明了但没有任何 yaml 提供 ──
    no_provider = sorted(k for k in declares if k not in yaml_keys)

    # ═══════════════════════════════════════════════════════════════════════════
    # 报告
    # ═══════════════════════════════════════════════════════════════════════════
    problems = 0

    print('=' * 78)
    print('A. 无消费者的键（改了没有任何效果）')
    print('=' * 78)
    def whitelist_reason(key):
        """精确键优先；否则查前缀白名单（整块子系统未写的那种）。"""
        if key in WHITELIST:
            return WHITELIST[key]
        for prefix, reason in WHITELIST_PREFIXES.items():
            if key.startswith(prefix):
                return reason
        return None

    unexpected = []
    for k in no_consumer:
        if k.startswith(RESERVED_PREFIXES):
            continue
        reason = whitelist_reason(k)
        if reason:
            print('  [白名单] {:<34} {}'.format(k, reason))
        else:
            unexpected.append(k)
    for k in unexpected:
        print('  ✗ [未获准] {:<32} 出现于 {}（没有任何 declare_parameter 读它）'.format(
            k, ', '.join(yaml_keys[k])))
    problems += len(unexpected)
    print('  -> 白名单 {} 项 / 未获准 {} 项'.format(
        len([k for k in no_consumer if whitelist_reason(k)]), len(unexpected)))

    print()
    print('=' * 78)
    print('B1. ⚠️⚠️ 真风险：同名键在**两份通配文件**里 —— 谁生效取决于 launch 顺序')
    print('=' * 78)
    if not dup_wild:
        print('  (无)')
    for k, files in dup_wild:
        print('  ✗ {:<30} {}'.format(k, ' + '.join(files)))
    print('  -> 修法：**删掉其中一份**（保留语义上更该管的那个文件），'
          '让它只依赖内容、不依赖顺序。')
    problems += len(dup_wild)

    print()
    print('=' * 78)
    print('B2. 同名键出现在多个**节点作用域**文件（一般是合法的，仍值得看一眼）')
    print('=' * 78)
    print('  说明：各节点读各自的文件、互不覆盖 —— 例如 `output_topic` 是 comm_node')
    print('        订阅的那头 + perception_pipeline 发布的那头，**配对的两个键**。')
    print('        ⚠️ 但同名容易让人以为"改一处就够"，值得考虑是否该起不同的名字。')
    for k, files in dup_other:
        print('  · {:<30} {}'.format(k, ' + '.join(files)))

    print()
    print('=' * 78)
    print('C. 声明了但没有任何 yaml 提供（运行期只能吃代码默认值）')
    print('=' * 78)
    print('  ⚠️ 多数是正常的（参数有默认值就行）。列出来是为了发现"以为配了其实没配"。')
    for k in no_provider:
        print('  · {:<34} {}'.format(k, ', '.join(declares[k][:2]) +
                                     (' 等' if len(declares[k]) > 2 else '')))
    print('  -> 共 {} 个'.format(len(no_provider)))

    # ═══════════════════════════════════════════════════════════════════════════
    # D. **按 launch 校验参数可达性**
    # ═══════════════════════════════════════════════════════════════════════════
    # ⚠️⚠️ 为什么必须有这一节：B1 的修法是"删掉重复键"，可**删完并不会自动保证**
    #    剩下的那一份会被传给这个节点 —— "两份都传"是**三条 launch 的巧合一致**，
    #    不是被强制的。只要将来有人写个只传其中一份的 launch，那些键就**静默回落
    #    到代码默认值**（例如 `platform_z_{0.0}`），**同一个坑换个外壳**。
    #     这一节就是那道强制。
    print()
    print('=' * 78)
    print('D. 按 launch 校验参数可达性（"+ 缺" = 该节点声明了、但这份 launch 没给它值）')
    print('=' * 78)
    print('  ⚠️ 正则近似解析 launch（本机无 ROS，import launch 跑不起来）——报告要人看。')
    print('  ⚠️ "+ 缺"**不等于出错**（参数多半有代码默认值），它 = "以为配了其实没配"。')

    per_file_declares = scan_declares_by_file([src_dir])
    launch_dir = os.path.join(root, 'launch')
    config_names = {n for n in os.listdir(config_dir) if n.endswith('.yaml')}

    # yaml 文件名 → 它提供的键
    file_keys = {}
    for fname in sorted(config_names):
        ks = load_param_yaml(os.path.join(config_dir, fname))
        if ks:
            file_keys[fname] = set(ks)

    def declared_keys_of(node_name):
        """节点名 → 它声明的键。约定：`src/**/<node_name>.cpp` 即该节点的源文件。"""
        keys = set()
        for rel, ks in per_file_declares.items():
            if os.path.basename(rel) in ('{}.cpp'.format(node_name),
                                         '{}.hpp'.format(node_name)):
                keys.update(ks)
        return keys

    d_problems = 0
    if not os.path.isdir(launch_dir):
        print('  (没有 launch/ 目录，跳过)')
    else:
        for fname in sorted(os.listdir(launch_dir)):
            if not fname.endswith('.launch.py'):
                continue
            yamls, nodes, inline = scan_launch(os.path.join(launch_dir, fname), config_names)
            # 缺文件：引用了 config/ 里不存在的 yaml（改了名忘了改 launch）
            missing_files = sorted(y for y in yamls if y not in config_names)
            provided = set()
            for y in yamls:
                provided |= file_keys.get(y, set())
            provided |= inline          # launch 内联 dict 里的键也算"已提供"
            missing_keys = {}
            for node in sorted(nodes):
                dk = declared_keys_of(node)
                gap = sorted(k for k in dk if k not in provided)
                if gap:
                    missing_keys[node] = gap

            if not yamls and not nodes:
                continue
            print('  · {}'.format(fname))
            print('      yaml: {}'.format(', '.join(sorted(yamls)) or '(无)'))
            for y in missing_files:
                print('      ✗ 引用了不存在的 config/{}'.format(y))
            for node, gap in missing_keys.items():
                shown = gap[:6]
                print('      + {}: 缺 {} 个键  {}{}'.format(
                    node, len(gap), ', '.join(shown),
                    ' …' if len(gap) > len(shown) else ''))
            d_problems += len(missing_files)

    print()
    print('=' * 78)
    if problems:
        print('结论: 发现 {} 处需要处理（A 类未获准 {} + B 类重复 {}）'.format(
            problems, len(unexpected), len(dup)))
    else:
        print('结论: A/B 两类都没有问题')
    print('=' * 78)

    if args.strict and problems:
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
