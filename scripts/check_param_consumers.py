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

═══════════════════════════════════════════════════════════════════════════════
⚠️⚠️ **退出码契约 —— 调用方（pre-commit / CI / 人）必须逐条区分**
═══════════════════════════════════════════════════════════════════════════════
    0 = 干净
    1 = **查出问题**（A 类未获准 / B1 重复键）→ 去改 config 或代码
    2 = **检查器自身故障** → **去修工具，别去删白名单**

  ⚠️ **3 与 4 的处置完全相反，所以绝不能合并。** 合并的后果不是"少个数字"，
     而是**门禁撒谎**：某天脚本因为环境问题（编码、缺依赖、路径）跑不动，
     给出 1，于是有人照着"去改 config"的方向白忙 —— 或者更糟，
     干脆绕过这个检查。**信号一旦不可信，真出事时就没人看了。**
     （本脚本 2026-09-23 就真的以 1 崩过：GBK 控制台打不出 `⬜` 抛
      UnicodeEncodeError，未捕获 ⇒ 退出码 1。见下方"输出编码"那段。）

  CI 里建议显式区分，例如：
      python3 scripts/check_param_consumers.py --strict
      case $? in
        0) echo "参数接线 OK" ;;
        1) echo "::error::参数接线有问题，看上面报告" ; exit 1 ;;
        *) echo "::error::检查器自身故障（退出码 $?），**不是**配置问题" ; exit 2 ;;
      esac

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
    # ⚠️ **退出码 2**（检查器自身跑不起来），不是 1 —— 1 的含义是"查出问题"。
    #    两者撞车 = 门禁撒谎 = 人开始绕过它（见下方"输出编码"那段的同一条道理）。
    print('需要 PyYAML: sudo apt install -y python3-yaml   (不是 python-yaml)',
          file=sys.stderr)
    sys.exit(2)


# ═══════════════════════════════════════════════════════════════════════════════
# 输出编码：**绝不能因为打印不出来就崩**
# ═══════════════════════════════════════════════════════════════════════════════
# ⚠️⚠️ 2026-09-23 修的真 bug：本脚本报告里含 `⬜ ✗ ✅ ⚠️` 等**非 GBK 字符**，
#    在 Windows 默认控制台（cp936）上 `print` 会抛 UnicodeEncodeError ——
#    **未捕获 → 退出码 1**。而 `--strict` 正是用 **1 表示"发现问题"** ⇒
#    **"检查器自己崩了"与"查出问题了"完全无法区分**。
#    后果比"崩"更糟：这是个要进 pre-commit 的**门禁**，
#    **门禁谎报失败，人的第一反应就是绕过它** —— 信号一旦不可信，
#    真出事时就没人看了。与仓库里其它"假信息"是同一类病。
#
# 处置（三道）：
#   ① 把 stdout/stderr 的错误处理改成 **`errors='replace'`** —— 兜底，任何字符都不再崩。
#      ⚠️⚠️ **刻意只改 errors、不改 encoding**：改 encoding 会让 cp936 控制台收到
#         UTF-8 字节而按 GBK 解 ⇒ **中文全变乱码**（那比崩更糟：崩了至少知道要看哪儿）。
#         中文本来就是 GBK 原生的，真正编不出的只有那几个符号 → 让它们变 `?` 即可。
#   ② **机器可读的标记改成 ASCII**（`[X]` / `[ ]` / `[!]` / `[OK]`）——
#      这样哪怕控制台是 cp936，报告的结构仍然**看得懂**（不依赖任何符号能否打印）。
#   ③ 退出码分层：0=干净 / 1=查出问题 / **2=检查器自身故障**（见文件末 `__main__`）
def _setup_stdio():
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors='replace')   # ⚠️ 只动 errors，别动 encoding
        except (AttributeError, ValueError):
            pass      # 老 Python 或已被重定向的流 —— 无所谓，② 已经保证结构能看


_setup_stdio()


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
        '模块 18 kalman_tracker [ ] 未写。到期：融合层落地时',
    'association.':
        '模块 19 target_associator [ ] 未写（且 §四 已定"范围收窄为查表，不做匈牙利"）。到期：融合层落地时',
    'tracking.':
        '模块 18/19 的下游参数 [ ]。到期：融合层落地时',
    'temporal_sync.':
        '模块 20/21 融合层 [ ] 未写。到期：融合层落地时',
    'shared_zone.':
        '模块 20 lidar_camera_fusion [ ] 未写。到期：融合层落地时',
    'lidar_weight':
        '模块 20 融合权重 [ ]。到期：融合层落地时',
    'camera_weight':
        '模块 20 融合权重 [ ]。到期：融合层落地时',
    'inference':
        '模块 8/9（earth/sky_block_detector）[ ] 未写，且卡模型。到期：模型到位时',
    'extrinsic_monitor.':
        '模块 28 [!] 已降级为调试工具（`enabled` 默认 false，不占主线）。到期：'
        '真实相机标定完成后，再决定做不做',
    'confirm_frame_count':
        '模块 18 kalman_tracker 的确认帧数 [ ]。到期：融合层落地时',
    'delete_frame_count':
        '模块 18 kalman_tracker 的删除帧数 [ ]。到期：融合层落地时',
    # ⚠️ **本工具看不到的消费者：被 launch 直接读的 yaml**（**不是**"死配置"）。
    #
    #    这些键由 `launch/bringup_full.launch.py` 用 `yaml.safe_load` **直接读**
    #    （拿去生成 static_transform_publisher 的实参），**不走 `declare_parameter`**
    #    —— 而本工具 A 类的判据只认 declare，所以看不见这个消费者。
    #
    #    ⚠️⚠️ 2026-09-23 更正：此处原先写的是
    #        "**死配置**：launch 写死数值，没有任何节点读这些键"。
    #        **那半句"没人读"是对的，结论"死配置、该删"是错的。**
    #        正确处置是**让 launch 去读它**（已做）—— 删掉才是真把外参事实源丢了。
    #    ⚠️ 这个标签错得有普遍意义，值得记住：
    #        **"没人读"不等于"该删"，先问一句"它本该由谁读"。**
    #        外参这种东西本来就该由 launch/标定产物流到运行期，不该写死在 launch 里。
    'static_tf.':
        '[OK] 活的：由 `bringup_full.launch.py` 直接 `yaml.safe_load` 读（不走 declare，'
        '故本工具看不见）—— 见上方那段说明',
}

# ⚠️ **配对键**：一条契约的**两头**，各自节点读各自的文件 —— **不是"两处真源"**。
#    它们**必须同名**（同名本身就是那条契约），所以列进 B2 只会制造噪音。
#    ⚠️ 但配对键有个**真风险**：改一头忘一头 ⇒ 链路静默断（与"两处真源"同类但不同因）。
#       所以这里**只是从 B2 里挪走**，不是免检 —— 见 D 段（按 launch 校验可达性）。
PAIRED_KEYS = {
    'output_topic':
        '配对：`perception_pipeline` 发 / `comm_node` 订（一条契约的两头，必须同名）',
    'emergency_stop_topic':
        '配对：`perception_pipeline` 发 / `comm_node` 订（一条契约的两头，必须同名）',
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
    r"""
    从一个 launch 文件里粗略抽出：**每个节点**各拿了哪些 config yaml。

    ⚠️⚠️ **必须按节点算，不能按 launch 汇总**（2026-09-23 反向验证抓到的假阴性）：
       原先返回的是"这份 launch 引用到的 yaml 全集"，于是
       **A 节点少传了、但 B 节点传了同一份** ⇒ 报告照样说"没缺"。
       而"某个节点漏传了自己的参数文件"**正是 D 段要抓的那件事** —— 汇总口径
       对它要防的失效模式**基本无效**（那次反向验证：抽掉
       `mustika_pillar_check` 的 `fusion_params.yaml`，报告一点反应都没有）。

    做法：按 `Node(` 切成块，**每块**里各自抽 `name=` 与它自己的 yaml / 内联 dict。

    ⚠️ 仍是**正则近似，不是执行 launch**（本机没 ROS，`import launch` 跑不起来）。
       "块"的边界靠 `Node(` 切分 —— 本仓库的 launch 没上元编程，够用；
       但**报告要人看**，别无声当 CI 通过。
    ⚠️ 组装式 launch（`bringup_debug` 用 NODE_SPECS 元组 + 列表推导）**切不出块** ——
       那种情况退回"整份 launch 一份集合"，并在报告里标出来（见返回值第 2 项）。

    @return (per_node, unassigned_or_None)
            per_node: {节点名: {'yaml': set, 'inline': set}}
            第 2 项为 True 表示这份 launch **是按块切不开的**（结果只能当"整份"看）
    """
    text = io.open(launch_path, encoding='utf-8', errors='replace').read()

    # ── 模块级变量 → yaml 文件的映射 ──
    # ⚠️ **必须解析这个**：`lidar_perception.launch.py` 那种写法是
    #      `lidar_params = os.path.join(pkg_dir, 'config', 'lidar_params.yaml')`
    #    定义在**文件顶部**，然后在 `Node(parameters=[lidar_params, ...])` 里引用变量。
    #    ⚠️ 第一版按 `Node(` 切块后**看不到块外的变量定义** ⇒ 那个 launch 被误报成
    #       "缺 77 个键"（假阳性）。**假阳性会让人去修一个不存在的问题。**
    #    ⚠️⚠️ **标识符前面不能是 `.`**（负向后顾）—— 否则 `**os.path**.join(...)`
    #       里的 `path` 会被当成变量名。第一版就是这么错的：
    #       `_load_static_tfs()` **函数体内**有一句 `path = os.path.join(..., 'fusion_params.yaml')`，
    #       于是 `var_to_yaml = {'path': 'fusion_params.yaml'}`；
    #       而每个 chunk 的 `parameters=[os.path.join(...)]` 里都含单词 `path`
    #       ⇒ **每个节点都被凭空加上 `fusion_params.yaml`** ⇒ per-node 检查退化成并集。
    #       （这正是用户指出的那个"退化成按整份算"，只是原因不在"放弃 per-node"。）
    var_to_yaml = {}
    for m in re.finditer(
            r"^[ \t]*(?<![.\w])([A-Za-z_][A-Za-z0-9_]*)\s*=\s*os\.path\.join\("
            r"[^)]*'config'\s*,\s*'([A-Za-z0-9_]+\.yaml)'",
            text, re.M):
        var_to_yaml[m.group(1)] = m.group(2)

    # 按 `Node(` 切块。用 `str.split` 而不是正则 —— 不需要配平括号，
    # 因为本仓库的每个 Node(...) 都写在同一段连续文本里，切到下一个 Node( 为止即可。
    # ⚠️ `ComposableNode(` 里也含 `Node(`，所以容器式 launch 同样会被切到 ✓
    chunks = text.split('Node(')[1:]
    if not chunks:
        return {}, True

    def _yaml_refs(snippet):
        """
        从一段代码里抽出它引用的 config yaml —— 直接写的 + 经变量引用的。

        ⚠️⚠️ **只看 `parameters=[...]` 列表里的内容，不看整段代码**
        （2026-09-23 修）：按 `Node(` 切块时，块**会跨进两个 Node 之间的其它代码**
        （函数定义、注释…）。原先在整块上找 `'config', 'x.yaml'`，于是
        `_load_static_tfs()` 里那句 `os.path.join(..., 'fusion_params.yaml')`
        被算到了**它前面那个节点**头上（`livox_custom_to_pcl2` 的块长 1826 字符就撞上了）。
        ⇒ 只在参数列表里找，块划粗了也不会误收。
        """
        found = set()
        # 两种写法都要认：
        #   · `parameters=[os.path.join(..., 'config', 'x.yaml'), ...]`（直接写路径）
        #   · `parameters=[lidar_params, field_geometry]`（引用模块级变量）
        for lst in re.findall(r"parameters\s*=\s*\[([^\]]*)\]", snippet, re.S):
            found |= {y for y in re.findall(r"'config'\s*,\s*'([A-Za-z0-9_]+\.yaml)'", lst)
                      if y in config_names}
            for ident in re.findall(r"(?<![.\w])([A-Za-z_][A-Za-z0-9_]*)", lst):
                if ident in var_to_yaml and var_to_yaml[ident] in config_names:
                    found.add(var_to_yaml[ident])
        return found

    per_node = {}
    for chunk in chunks:
        # 节点名（`name='xxx'` 在参数里）
        m_name = re.search(r"name\s*=\s*'([a-z][a-z0-9_]+)'", chunk)
        if not m_name:
            continue
        name = m_name.group(1)

        # 这个节点拿到的 yaml（直接写路径的 + 经变量引用的）
        ys = _yaml_refs(chunk)
        # 这个节点内联 dict 里给的键
        # ⚠️ 刻意**宁可多收**（`launch_arguments={'rviz': ...}` 之类也会被收进来）——
        #    多收只会减少假阳性；真缺的键仍会因为"这个节点这块里一次都没出现"被报出来。
        inl = set(re.findall(r"'([a-z][a-z0-9_]*)'\s*:", chunk))

        # ⚠️ 记下 `package=` —— `[--] 找不到源码` 那条提示**只对本包的节点有意义**
        #    （`rviz2` / `static_tf_*` 是外部包的可执行，永远找不到源码，
        #      每次都报就成了噪音，而噪音会让人连真提示一起无视）。
        m_pkg = re.search(r"package\s*=\s*'([a-z_0-9]+)'", chunk)
        own = (m_pkg is not None and m_pkg.group(1) == 'br_perception')

        slot = per_node.setdefault(name, {'yaml': set(), 'inline': set(), 'own': own})
        slot['yaml'] |= ys
        slot['inline'] |= inl
        slot['own'] = slot.get('own', own) or own

    # 组装式 launch：名字在 NODE_SPECS 元组里，`Node(` 块切不出来 →
    # 退回"整份 launch 一份集合"，并告知调用方这是**降级结果**
    # ── 组装式 launch（`bringup_debug` 的 NODE_SPECS 元组）──
    #
    # ⚠️⚠️ **必须无条件扫，不能"per_node 为空才扫"**（2026-09-23 修的一个静默漏检）：
    #    `bringup_debug` 里既有组装式节点、又有字面量 `Node(name='rviz2')`
    #    ⇒ `per_node` **非空** ⇒ "空了才降级"的分支**永远不触发** ⇒
    #    **相机层那 6 个节点被静默跳过**（一个都没进报告，而报告看起来是"全 OK"）。
    #    这比漏报更糟：**它让人以为检查过了**。
    #
    # ⚠️ 写法：`('image_preprocessor', 'br_perception::camera::...', ...)` —— **整行**，
    #    所以要用 `,\s+` 而不是跨行的 `,\s*\n\s*`。
    spec_names = set(re.findall(r"'([a-z][a-z0-9_]+)',\s+'br_perception::", text))
    degraded = False
    if spec_names:
        # ⚠️ 这些节点**按块切不出来**（名字在元组里、`parameters=make_params(...)` 是函数调用），
        #    所以只能给"整份 launch"的集合 —— 那种口径**抓不到"单个节点漏传"**
        #    （两个节点传了同一份 yaml 就会掩盖另一个）。已在报告里标出。
        degraded = True
        # ⚠️ **降级就老老实实按"整份"扫** —— 组装式 launch 的参数来自**函数调用**
        #    （`parameters=make_params(uses_color)`），既不在 `parameters=[...]` 列表里，
        #    也不在 Node 块里。用 `_yaml_refs`（只认列表）会**一个 yaml 都收不到**，
        #    于是那些节点全被误报成"缺 N 个键"（2026-09-23 修这个时真踩了）。
        #    ⇒ 整份文本上找"直接写的路径" + "经模块级变量引用的"，这才是"按整份算"的本义。
        all_y = {y for y in re.findall(r"'config'\s*,\s*'([A-Za-z0-9_]+\.yaml)'", text)
                 if y in config_names}
        for ident, y in var_to_yaml.items():
            # 变量名在文件里出现过就算（组装式无法精确定位到节点，只能这么粗）
            if y in config_names and re.search(r"(?<![.\w])" + re.escape(ident) + r"\b", text):
                all_y.add(y)
        all_i = set(re.findall(r"'([a-z][a-z0-9_]*)'\s*:", text))
        for n in spec_names:
            per_node.setdefault(n, {'yaml': set(), 'inline': set()})
            per_node[n]['yaml'] |= all_y
            per_node[n]['inline'] |= all_i
    if not per_node:
        return {}, True

    return per_node, degraded


def selftest():
    """
    **门禁自证** —— 验证"该响的时候它真的会响"。

    ⚠️⚠️ 为什么必须有这个（2026-09-23 用户要求，起因是一次真事故）：
        D 段一度**退化成了"按整份算集合"**，于是"某节点漏传了自己的参数文件"
        根本报不出来 —— 而报告照旧打 `[OK]`、退出码照旧 0。
       **一个永远通过的门禁比没有门禁更糟**：它会让人以为检查过了。
       这与本工具从 `fake_master --selftest` 学来的做法一致：
       **验证工具本身，而不是验证某个具体配置。**

    做法：在**临时目录**里造一个最小仓库（不碰真仓库），
          故意制造三类问题，逐条断言必须被报出来。
    """
    import shutil
    import tempfile

    ok = True
    def check(name, good):
        nonlocal ok
        ok &= bool(good)
        print('  {}  {}'.format('[OK]' if good else '[X] ', name))

    root = tempfile.mkdtemp(prefix='chkparam_selftest_')
    try:
        os.makedirs(os.path.join(root, 'config'))
        os.makedirs(os.path.join(root, 'launch'))
        os.makedirs(os.path.join(root, 'src', 'demo'))

        def w(rel, text):
            with io.open(os.path.join(root, rel), 'w', encoding='utf-8') as f:
                f.write(text)

        # 两个**通配**文件同名键（B1）+ 一个无消费者的键（A）
        w('config/a.yaml', '/**:\n  ros__parameters:\n    dup_key: 1\n    orphan_key: 2\n')
        w('config/b.yaml', '/**:\n  ros__parameters:\n    dup_key: 3\n')
        # 一个节点作用域的 yaml（给下面那个节点提供 feed_key）
        w('config/n.yaml', 'demo_node:\n  ros__parameters:\n    feed_key: 4\n')
        # 源码：demo_node 声明 feed_key
        w('src/demo/demo_node.cpp',
          'void f(rclcpp::Node* n) {\n  n->declare_parameter<int>("feed_key", 0);\n}\n')

        # ⚠️ 关键：这份 launch 里**先有一个别的节点也传 n.yaml**，
        #    再让 demo_node **漏传** —— 这正是"并集口径"会漏报的形态。
        launch_good = (
            "from launch_ros.actions import Node\n"
            "n = Node(package='br_perception', executable='demo_node', name='demo_node',\n"
            "         parameters=[os.path.join(pkg, 'config', 'n.yaml')])\n"
        )
        w('launch/good.launch.py', launch_good)
        # 漏传版：demo_node 的 parameters 空了
        w('launch/missing.launch.py', launch_good.replace(
            "parameters=[os.path.join(pkg, 'config', 'n.yaml')]", "parameters=[]"))

        def run(target_launch):
            """在当前临时仓库上跑一遍分析，返回 (缺键节点, B1 键, A 未获准键)。"""
            cfg_dir = os.path.join(root, 'config')
            cfg_names = {n for n in os.listdir(cfg_dir) if n.endswith('.yaml')}
            per_node, _deg = scan_launch(os.path.join(root, 'launch', target_launch),
                                         cfg_names)
            file_keys = {}
            for fn in sorted(cfg_names):
                ks = load_param_yaml(os.path.join(cfg_dir, fn))
                if ks:
                    file_keys[fn] = set(ks)
            per_file = scan_declares_by_file([os.path.join(root, 'src')])

            def declared(node):
                keys = set()
                want = {'{}.cpp'.format(node), '{}_node.cpp'.format(node)}
                for rel, ks in per_file.items():
                    if os.path.basename(rel) in want:
                        keys |= ks
                return keys

            gaps = set()
            for node, slot in per_node.items():
                provided = set(slot['inline'])
                for y in slot['yaml']:
                    provided |= file_keys.get(y, set())
                for k in declared(node):
                    if k not in provided:
                        gaps.add(k)

            # A/B1：直接复用 main 里那套判据的等价逻辑
            scopes, yk = {}, {}
            for fn in sorted(cfg_names):
                ks = load_param_yaml(os.path.join(cfg_dir, fn))
                if not ks:
                    continue
                for k, sc in ks.items():
                    yk.setdefault(k, []).append(fn)
                    scopes.setdefault(k, []).append('/**' in sc)
            wild = sorted(k for k, v in yk.items()
                          if len(v) > 1 and scopes.get(k, []).count(True) >= 2)
            a_unlisted = sorted(k for k in yk
                                if k not in per_file_declares_all(per_file)
                                and k not in ('use_sim_time',))
            return gaps, wild, a_unlisted

        def per_file_declares_all(per_file):
            allk = set()
            for ks in per_file.values():
                allk |= ks
            return allk

        print('门禁自证（在临时目录里造数据，不碰真仓库）:')

        gaps, wild, a_un = run('good.launch.py')
        check('① 正常情况：demo_node 的 feed_key **不该**报缺（实得缺 %s）' % (sorted(gaps) or '无'),
              'feed_key' not in gaps)

        gaps2, _, _ = run('missing.launch.py')
        check('② ⭐ 漏传必需文件：**必须**报出 feed_key 缺（实得 %s）' % (sorted(gaps2) or '无'),
              'feed_key' in gaps2)

        check('③ 两份通配文件同名键 → 必须进 B1（实得 %s）' % (wild or '无'),
              'dup_key' in wild)

        check('④ 无消费者的键 → 必须进 A 未获准（实得 %s）' % (a_un or '无'),
              'orphan_key' in a_un)

        print('  ——自证{}——'.format('通过' if ok else '**失败：门禁不会响，别信它的 [OK]**'))
    finally:
        shutil.rmtree(root, ignore_errors=True)
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description='找出"改了没效果"的 ROS 2 参数')
    ap.add_argument('--strict', action='store_true',
                    help='有 A/B 类问题则退出码 1（给 CI / 预提交用）')
    ap.add_argument('--selftest', action='store_true',
                    help='门禁自证：造最小仓库验证"该响时会响"（零副作用）')
    ap.add_argument('--root', default=os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                    help='仓库根（默认取本脚本的上层目录）')
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    root = args.root
    config_dir = os.path.join(root, 'config')
    src_dir = os.path.join(root, 'src')

    if not os.path.isdir(config_dir) or not os.path.isdir(src_dir):
        # ⚠️ 同 42 行：**这是"检查器跑不起来"，不是"查出问题"** → 退出码 2
        print('在 {} 下找不到 config/ 与 src/ —— 用 --root 指仓库根'.format(root),
              file=sys.stderr)
        sys.exit(2)

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

    # ── 先把 launch 里的**内联参数字典**扫出来 ──
    # ⚠️ 2026-09-23 复核指出：C 段原先**只查 yaml**，于是桥接节点那 5 个参数
    #    （`input_topic` / `frame_id` / `scan_lines` / `elev_min_deg` / `elev_max_deg`）
    #    被判成"没提供" —— 而它们**在 launch 里以 `parameters=[{...}]` 形式给了**
    #    （`bringup_full` / `bringup_gazebo`）。**假阳性会让人去修一个不存在的问题**。
    # ⚠️ `config_names` **必须在这里就算**（原先它在 D 段才算）——
    #    上面那段用了它，晚定义会 NameError，而被 try/except 吞掉后**静默失效**：
    #    报告照样出，只是内联 dict 一个都没收进来（第一版就是这么错的）。
    launch_dir = os.path.join(root, 'launch')
    config_names = {n for n in os.listdir(config_dir) if n.endswith('.yaml')}
    launch_inline_keys = set()
    launch_scan = {}          # {launch 文件名: (per_node, degraded)} —— D 段要用
    if os.path.isdir(launch_dir):
        for fname in sorted(os.listdir(launch_dir)):
            if fname.endswith('.launch.py'):
                try:
                    per_node, degraded = scan_launch(os.path.join(launch_dir, fname),
                                                     config_names)
                    launch_scan[fname] = (per_node, degraded)
                    for slot in per_node.values():
                        launch_inline_keys |= slot['inline']
                except Exception as exc:      # noqa: BLE001 —— 单个 launch 扫不动不该拖垮全表
                    # ⚠️ **要打出来** —— 上面那版靠静默吞异常，结果"改了没生效"都看不出来
                    print('  [!] 扫不了 {}: {}'.format(fname, exc))

    # ── C. 声明了但**没有任何 yaml 提供** ──
    #    ⚠️ 分两档、**并标出来源**（2026-09-23 复核建议）：
    #      `[none]`        —— 哪都没提供，**只能吃代码默认值**（这才是要看的）
    #      `[launch-dict]` —— yaml 里没有，但 **launch 的内联 dict 给了**
    #    ⚠️ 不分档的话，"真缺口"会和"其实给了只是不在 yaml"糊在一起 ——
    #       前者要处理，后者不用；糊在一起就会**将来真缺口反而看不出来**。
    c_none = sorted(k for k in declares
                    if k not in yaml_keys and k not in launch_inline_keys)
    c_inline = sorted(k for k in declares
                      if k not in yaml_keys and k in launch_inline_keys)

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
        print('  [X] [未获准] {:<32} 出现于 {}（没有任何 declare_parameter 读它）'.format(
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
        print('  [X] {:<30} {}'.format(k, ' + '.join(files)))
    print('  -> 修法：**删掉其中一份**（保留语义上更该管的那个文件），'
          '让它只依赖内容、不依赖顺序。')
    problems += len(dup_wild)

    print()
    print('=' * 78)
    print('B2. 同名键出现在多个**节点作用域**文件（一般是合法的，仍值得看一眼）')
    print('=' * 78)
    print('  说明：各节点读各自的文件、互不覆盖。分两种，**别混为一谈**：')
    print('    · **配对键**（一条契约的两头，必须同名）→ 已单独归类，不算问题')
    print('    · **同名不同义**（两个节点各自命名撞车）→ 值得看一眼：')
    print('      会让人以为"改一处就够"，其实两处毫无关系。')
    for k, files in dup_other:
        if k in PAIRED_KEYS:
            print('  [配对] {:<28} {}\n         {}'.format(k, ' + '.join(files), PAIRED_KEYS[k]))
        else:
            print('  [同名] {:<28} {}'.format(k, ' + '.join(files)))

    print()
    print('=' * 78)
    print('C. 声明了但**没有任何 yaml 提供** —— 按来源分两档')
    print('=' * 78)
    print('  · "没提供"不是"出错" —— 参数多半有代码默认值。分档是为了让人一眼分清')
    print('    "**以为配了其实没配**"（要看的）与"其实给了、只是不在 yaml"（不用管的）。')

    print()
    print('  [none]  哪都没提供 —— 只能吃代码默认值：')
    if not c_none:
        print('          （无）')
    for k in c_none:
        print('     {:<36} {}'.format(k, ', '.join(declares[k][:2]) +
                                      (' 等' if len(declares[k]) > 2 else '')))

    print()
    print('  [launch-dict]  yaml 里没有，但 **launch 的 `parameters=[{...}]` 给了**'
          '（不是缺口）：')
    if not c_inline:
        print('          （无）')
    for k in c_inline:
        print('     {:<36} {}'.format(k, ', '.join(declares[k][:2]) +
                                      (' 等' if len(declares[k]) > 2 else '')))
    if launch_inline_keys:
        # ⚠️ **说清它是"宁可多收"** —— 这条例外可能**掩盖**真问题：
        #    正则把所有 `'xxx':` 都算成"提供了"，**连 launch 参数（rviz/lio/comm…）
        #    也一并收进来**。所以 `[none]` 为空**不等于**"每个参数都被配过"，
        #    只等于"没有任何一个参数名在整套 launch 里一次都没出现"。
        print()
        print('  ⚠️ 全套 launch 内联 dict 共提供 {} 个键（**宁可多收**：连 rviz/lio/comm '
              '这类 launch 参数也收进来了）。'.format(len(launch_inline_keys)))
        print('     故 `[none]` 为空 ≠ "全都配过"，只 ≠ "参数名在 launch 里一次都没出现"。')

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
    # ⚠️ `launch_dir` / `config_names` 已在上面（C 段前）算过，这里不再重复赋值

    # yaml 文件名 → 它提供的键
    file_keys = {}
    for fname in sorted(config_names):
        ks = load_param_yaml(os.path.join(config_dir, fname))
        if ks:
            file_keys[fname] = set(ks)

    def declared_keys_of(node_name):
        """
        节点名 → 它声明的键。

        ⚠️ 文件名**不一定等于节点名**（2026-09-23 修的一个静默漏检）：
           · `camera_depth`  这个**可执行**名，源文件是 `camera_depth_node.cpp`
           · `camera_perception` 同理 → `camera_perception_node.cpp`
           只按 `<name>.cpp` 找的话，这两个节点**一个参数都查不到**，
           而调用方看到的是"这一行没打印"——**什么都不说**，
           于是报告看起来"全 OK"，实际上那两个节点**根本没被检查**。
           ⇒ 候选名里加 `_node` 变体。
        """
        keys = set()
        wanted = {'{}.cpp'.format(node_name),     '{}.hpp'.format(node_name),
                  '{}_node.cpp'.format(node_name), '{}_node.hpp'.format(node_name)}
        for rel, ks in per_file_declares.items():
            if os.path.basename(rel) in wanted:
                keys.update(ks)
        return keys

    d_problems = 0
    if not os.path.isdir(launch_dir):
        print('  (没有 launch/ 目录，跳过)')
    else:
        for fname, (per_node, degraded) in sorted(launch_scan.items()):
            print('  · {}'.format(fname))
            # ⚠️⚠️ **口径必须显式打出来，两种都要**（2026-09-23 用户要求）：
            #     · `[!] 按整份算` → 这份 launch **没真正按节点检查**，抓不到
            #        "A 节点漏传了自己的文件（而 B 节点传了同一份）"
            #     · `[OK] 按节点算` → **确实逐节点查过**，下面每条才可信
            #    ⚠️ 只标降级的那种是不够的：**"没有 [!]" 与 "逐节点查过" 在读者
            #       眼里长得一样**，而前者可能意味着"检查退化了但没打标记"。
            #       两种状态各自**明说**，才不会有"看起来通过了"的中间态。
            if degraded:
                print('      [!] 口径=**按整份算**（组装式 launch / NODE_SPECS 之类）'
                      '→ 抓不到"单个节点漏传自己的文件"，**别把下面的 [OK] 当成逐节点验过**')
            else:
                print('      [OK] 口径=**按节点算**（{} 个节点各自核过）'.format(len(per_node)))

            missing_files = set()
            worst = 0
            for node, slot in sorted(per_node.items()):
                # 该节点**自己**拿到的 yaml 提供的键（+ 它自己的内联 dict）
                provided = set(slot['inline'])
                for y in sorted(slot['yaml']):
                    provided |= file_keys.get(y, set())
                    if y not in config_names:
                        missing_files.add(y)      # 改名忘了改 launch

                dk = declared_keys_of(node)
                gap = sorted(k for k in dk if k not in provided)
                label = '{} ← {}'.format(node, ', '.join(sorted(slot['yaml'])) or '(无 yaml)')
                if gap:
                    shown = gap[:6]
                    print('      + {}: 缺 {} 个键  {}{}'.format(
                        label, len(gap), ', '.join(shown),
                        ' …' if len(gap) > len(shown) else ''))
                    worst += len(gap)
                elif dk:
                    print('      [OK] {}: {} 个声明参数全部可达'.format(label, len(dk)))
                elif slot.get('own'):
                    # ⚠️⚠️ **"找不到本包节点的源码"必须打出来**（2026-09-23）：
                    #    否则它和"检查过、没问题"在报告里**长得一样**（都是什么都不打印），
                    #    而前者意味着**这个节点根本没被检查**。
                    #    ⚠️ 这正是本工具一路在修的那类失效：**沉默看起来像通过**。
                    #    ⚠️ 只对本包的节点报 —— 外部包（rviz2 / tf2_ros）永远找不到源码，
                    #       每次都报就成了噪音，噪音会让人连真提示一起无视。
                    # ⚠️ 字面量里的 `{` `}` 要写双份 —— `str.format` 会把 `{,_node}`
                    #    当成替换字段（2026-09-23 真崩过一次：KeyError: ',_node'）。
                    print('      [--] {}: ⚠️ 找不到对应源码（`src/**/<name>.cpp` 或 '
                          '`<name>_node.cpp`）→ **本节点未被检查**，别当成已通过'.format(label))

            for y in sorted(missing_files):
                print('      [X] 引用了不存在的 config/{}'.format(y))
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
    # ⚠️⚠️ **退出码必须分层**（2026-09-23）：
    #     0 = 干净
    #     1 = **查出问题**（`--strict` 下）
    #     2 = **检查器自身故障**
    #
    #   为什么非要分出 2：此前一个 UnicodeEncodeError 让脚本**以 1 退出**，
    #   而 1 的含义是"发现了 A/B 类问题" ⇒ **崩溃伪装成了"查出问题"**。
    #   对门禁来说这是最坏的一种错：**它撒谎**。而门禁一撒谎，
    #   人的第一反应就是绕过它 —— 再往后真有问题也没人看了。
    #   ⇒ 宁可多一个码，也不让两种完全不同的失败长得一样。
    try:
        sys.exit(main())
    except SystemExit:
        raise
    except BaseException:                      # noqa: BLE001 —— 这里就是要抓全部
        import traceback
        traceback.print_exc()
        print('\n[X] **检查器自身故障**（不是"发现了配置问题"）—— 退出码 2。')
        print('    请连同上面的 traceback 一起反馈；在此之前，本结果**不可采信**。')
        sys.exit(2)
