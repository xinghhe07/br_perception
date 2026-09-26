#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_topic_consumers.py — 「把一个话题挂到任何开关下之前，先列出它的订阅者」

═══════════════════════════════════════════════════════════════════════════════
它为什么存在（一次真实的咬伤，2026-09-23 → 2026-09-24）
═══════════════════════════════════════════════════════════════════════════════
2026-09-23 把 `lidar_perception_node` 的 `publish_intermediate(...)`（10 条话题）
挂到了 `debug_enabled` 下，依据是一张写"**代码订阅者 0**"的表。

那张表**对了一半**（2026-09-24 逐条实测后更正）：那 10 条里 **2 条是数据接口**
（`filtered` 1 个订阅者、`non_ground` 3 个订阅者），另 8 条确实零订阅者。
⚠️ 实测方法也有教训：中间一版用 `grep -l "<话题>" src/` 数出"10 条里 9 条有"——
**错的**，那数的是"**提到**这个话题的文件"，把**发布者也数了进去**。
⇒ 数订阅者**必须看 `create_subscription` 那个调用块**，这就是本脚本用它做归因的原因。
而 `debug_enabled` 在比赛模式是 **false** ⇒ **组件那条链直接饿死**，
症状是"组件全都不出结果"——与"代码坏了"长得一样。
⚠️ 而**跑单体时完全看不出来**（单体的数据走 `LidarResult`，不靠这些话题）。

⇒ 拟出的纪律：**数据话题挂"拓扑开关"，纯可视化才挂"调试开关"**；
  这条动作不该靠人记 —— 本脚本就是它。

═══════════════════════════════════════════════════════════════════════════════
它做什么 / 不做什么
═══════════════════════════════════════════════════════════════════════════════
做：① 从 `src/**/*.cpp|hpp` 里把所有话题名字面量扫出来（`"/xxx/yyy"` 形态）
    ② 统计每个话题的**发布者/订阅者**文件（按文件去重，跨文件的重复实现不算两份）
    ③ 打印成表，并**点名**两类要人看的组合：
         [数据被挂成调试] 有订阅者、且它只出现在 `debug_*` 命名的函数里
         [孤儿话题]       零订阅者（⇒ 适合挂 `debug_enabled`，但每帧发它要花 CPU）
不做：**不做编译级分析**。话题名若是变量拼接、或经过 remap/launch 改名，本脚本看不到
      （命中时会在表里标 `?`）。它是个**探针**，不是判据 —— 结论仍要人核。

用法: python3 scripts/check_topic_consumers.py            # 全部话题
      python3 scripts/check_topic_consumers.py non_ground # 只看匹配的
"""
import io
import os
import re
import sys

# ⚠️ 本机控制台是 GBK —— 不换编码，⚠️ 与表格线会直接抛 UnicodeEncodeError
#    （同 `make_field_from_config.py` 的前置写法）
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.path.join(ROOT, 'src')

# 话题名字面量：双引号里以 / 开头的路径（够覆盖本仓库的实际写法）
TOPIC_RE = re.compile(r'"\s*(/[A-Za-z0-9_/]+)\s*"')
# 判定"这个调用落在哪个函数里"用：函数定义行（行尾是 `(` 或 `) {` 之类，不含 `;`）
FUNC_RE = re.compile(r'^\s*(?:[\w:<>,\s\*&]+)\s+(\w+)\s*\([^;]*$')
NEWLINE = chr(10)


def iter_sources():
    for base, _dirs, files in os.walk(SRC):
        for f in files:
            if f.endswith(('.cpp', '.hpp', '.h')):
                yield os.path.join(base, f)


def call_blocks(text, key):
    """取出所有 `key(...)` 整块（**括号配平**）—— 话题名常写在下一行，逐行判断会漏。
    返回 [(字符偏移, 块文本)]（偏移用来判断"这个调用落在哪个函数里"）。"""
    out = []
    for m in re.finditer(re.escape(key) + r'[^;]*?\(', text):
        i = m.end() - 1
        depth = 0
        for j in range(i, len(text)):
            if text[j] == '(':
                depth += 1
            elif text[j] == ')':
                depth -= 1
                if depth == 0:
                    out.append((i, text[i:j + 1]))
                    break
    return out


def enclosing_func(text, lines, pos):
    """`pos` 之前**最近**的那个函数定义名（近似：按行找最近的函数定义行；够本脚本用）"""
    upto = text.count(NEWLINE, 0, pos)          # pos 所在行号（0 基）
    name = ''
    for i in range(upto, -1, -1):
        m = FUNC_RE.match(lines[i]) if i < len(lines) else None
        if m and '=' not in lines[i].split('(')[0]:
            name = m.group(1)
            break
    return name


def is_debug_func(name):
    n = name.lower()
    # ⚠️ `publish_intermediate_data` **不是**调试类 —— 它发的是 `filtered` / `non_ground`
    #    两条**数据接口**，挂的是拓扑开关（`publish_intermediate_topics`）。
    #    不排掉它，本脚本会对这两条**误报**（那正是"探针要会看人话"的地方）。
    if 'publish_intermediate_data' in n:
        return False
    return 'debug' in n or 'publish_intermediate' in n


# 话题名走**变量**的写法（如 `lidar_topic_`）—— 字面量在头文件的默认值里。
# ⚠️ 不解析它，本脚本会**漏报**：2026-09-24 首跑就把 `/perception/lidar/result`
#    报成"零订阅者"，而 `sky_block_detector` 正订着它（通过 `lidar_topic_`）。
VAR_DEF_RE = re.compile(r'(\w+)\s*[{=]\s*"\s*(/[A-Za-z0-9_/]+)\s*"')


def var_defaults():
    """{变量名: 话题字面量} —— 从整个 src 里捞 `std::string x{"/topic"}` / `x = "/topic"`"""
    out = {}
    for path in iter_sources():
        try:
            txt = io.open(path, encoding='utf-8', errors='replace').read()
        except OSError:
            continue
        for name, topic in VAR_DEF_RE.findall(txt):
            out.setdefault(name, topic)
    return out


def scan():
    """返回 topic -> {'pub': set(文件), 'sub': set(文件), 'debug_ctx': set(文件)}"""
    info = {}
    vd = var_defaults()

    def slot(t):
        return info.setdefault(t, {'pub': set(), 'sub': set(), 'debug_ctx': set()})

    for path in iter_sources():
        rel = os.path.relpath(path, ROOT).replace('\\', '/')
        try:
            lines = io.open(path, encoding='utf-8', errors='replace').read().split('\n')
        except OSError:
            continue
        text = NEWLINE.join(lines)

        # ⚠️ 必须按**括号配平的整块**看，不能逐行 —— 本仓库的写法是
        #    create_subscription<Msg>( 换行后才写话题名, ...)，逐行判断会漏。
        for key, role in (('create_subscription', 'sub'), ('create_publisher', 'pub')):
            for pos, blk in call_blocks(text, key):
                names = TOPIC_RE.findall(blk)
                if not names:
                    # 话题名走变量（`lidar_topic_` 之类）⇒ 用它的字面量默认值补上；
                    # 补不上就不猜（盲区已在文件头声明）
                    for ident in re.findall(r'\(\s*(\w+)\s*,', blk):
                        if ident in vd:
                            names.append(vd[ident])
                dbg = is_debug_func(enclosing_func(text, lines, pos))
                for t in names:
                    s = slot(t)
                    s[role].add(rel)
                    if dbg:
                        s['debug_ctx'].add(rel)

        # 其它位置（注释 / RViz / 手写列表）出现过的名字也登记，免得整条话题看不见
        for t in TOPIC_RE.findall(text):
            slot(t)
    return info


def main():
    filt = sys.argv[1] if len(sys.argv) > 1 else ''
    info = scan()

    rows = sorted((t, v) for t, v in info.items() if filt in t)
    if not rows:
        print('没有匹配 %r 的话题' % filt)
        return 0

    flagged = 0
    print('%-46s %4s %4s  %s' % ('话题', '发', '订', '状态'))
    print('-' * 100)
    for t, v in rows:
        n_pub, n_sub = len(v['pub']), len(v['sub'])
        note = ''
        if n_sub == 0 and n_pub > 0:
            note = '孤儿（零订阅者）⇒ 适合挂 debug_enabled，但每帧发它要花 CPU'
        if n_sub > 0 and v['debug_ctx'] and not v['debug_ctx'] - v['sub']:
            # 它出现在 debug 类上下文里，而**有**订阅者 ⇒ 就是 9-23 那个坑的形状
            note = ('⚠️ **有 %d 个订阅者，却出现在 debug 类上下文里** ⇒ '
                    '开关一关就饿死它们（2026-09-23 的坑）' % n_sub)
            flagged += 1
        print('%-46s %4d %4d  %s' % (t, n_pub, n_sub, note))

    print('-' * 100)
    if flagged:
        print('⚠️ %d 条话题需要人核（形状与 2026-09-23 的坑相同）。' % flagged)
        print('   处置：数据话题（有订阅者）挂 `publish_intermediate_topics` 这类**拓扑开关**；')
        print('         纯可视化（零订阅者）才挂 `debug_enabled`。见台账 §八。')
    else:
        print('没有"数据话题被挂进调试开关"的形状。')
    print()
    print('⚠️ 本脚本是**探针**不是判据：话题名经变量拼接 / launch remap 改名的，它看不到。')
    print('   订阅者列出来之后，仍要人判断"这条话题是不是数据接口"。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
