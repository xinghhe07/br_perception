#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""check_cpp_strings.py — 抓「**中文串里嵌了 ASCII 双引号**」这个坑

═══════════════════════════════════════════════════════════════════════════════
它为什么存在（2026-09-24 一天里踩了三次）
═══════════════════════════════════════════════════════════════════════════════
本仓库的注释与日志几乎全用中文，而中文里"引用一个词"的习惯写法是 `"..."` ——
在注释里无害，**在 C++ 字符串字面量里就是提前闭合**：

    RCLCPP_WARN(log, "这不是 bug，是"相机不做检测"这条切分的代价");   // ← 编译错

⚠️ **编译器报到的地方往往不是这里**（它会把后面那半句当成标识符，报到几行之后），
   症状是"一堆莫名其妙的语法错"，而人第一眼会去怀疑别处。
⚠️ 中文引号「」『』是全角、**不是问题**；要抓的是**半角** `"` 夹在汉字之间。

⇒ 判据：**先剥掉注释与字符串，再在"代码区"里找 `汉字 + " + 汉字`** ——
   出现在代码区的，只可能是"字符串被提前闭合后剩下的碎片"或"字符串里真夹了 `"`"。
   两种都是编译错。

═══════════════════════════════════════════════════════════════════════════════
它做什么 / 不做什么
═══════════════════════════════════════════════════════════════════════════════
做：① 逐个文件扫描 `src/**` + `test/**` + `include/**` 的 `.cpp/.hpp/.h/.cc`
    ② **按 C++ 词法剥注释与字符串**（行注释 / 块注释 / 字符串字面量 / 字符字面量，
       处理 `\\` 转义），只在"代码区"里找上面那个指纹
    ③ 报出**文件 + 行号 + 那一小段上下文**
不做：**不是编译器**。它只抓这一类错（外加"字符串跨行未闭合"这个同族症状）。
     别的语法/类型错它一律不管 —— 那是 `colcon build` 的事。

用法: python3 scripts/check_cpp_strings.py            # 扫 src/ test/（默认）
      python3 scripts/check_cpp_strings.py <路径...>   # 只看这些
返回码: 0 = 干净；1 = 有命中（可直接挂进 pre-commit / CI）
"""
import io
import os
import re
import sys

# ⚠️ 本机控制台是 GBK —— 不换编码，⚠️ 与表格线会直接抛 UnicodeEncodeError
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
EXTS = ('.cpp', '.hpp', '.h', '.cc')

BS   = chr(92)   # 反斜杠（写成 chr() 免得被各层转义搅乱）
DQ   = chr(34)   # 双引号
NL   = chr(10)
# 指纹：汉字/中文标点 紧邻 ASCII 双引号 再紧邻汉字
FINGERPRINT = re.compile('[一-鿿，。：；（）　]' + DQ + '[一-鿿]')


def strip_comments_and_strings(src):
    """把注释整体抹成等长空白、字符串字面量**整体跳过**；返回"代码区"。
    返回 (code_only, string_hits, unterminated) ——
      · `string_hits`：字符串**内部**就夹着指纹的行号（提前闭合的另一种写法）
      · `unterminated`：字符串/注释没闭合就撞到行尾或文件尾
    """
    out = []
    hits = []
    unterminated = []
    i, n = 0, len(src)
    while i < n:
        c = src[i]

        # ── 行注释：抹成等长空白（保行号）──
        if c == '/' and i + 1 < n and src[i + 1] == '/':
            j = src.find(NL, i)
            j = n if j < 0 else j
            out.append(' ' * (j - i))
            i = j
            continue

        # ── 块注释：抹成等长空白（换行保留）──
        if c == '/' and i + 1 < n and src[i + 1] == '*':
            j = src.find('*/', i + 2)
            if j < 0:
                unterminated.append((src.count(NL, 0, i) + 1, '块注释未闭合'))
                j = n
            else:
                j += 2
            out.append(''.join(ch if ch == NL else ' ' for ch in src[i:j]))
            i = j
            continue

        # ── 字符串字面量：整体跳过，但检查**里面**是否夹了裸引号 ──
        if c == DQ:
            j, esc = i + 1, False
            while j < n:
                if src[j] == NL:
                    unterminated.append((src.count(NL, 0, i) + 1, '字符串未闭合（撞到行尾）'))
                    break
                if esc:
                    esc = False
                elif src[j] == BS:
                    esc = True
                elif src[j] == DQ:
                    break
                j += 1
            body = src[i + 1:j]
            if FINGERPRINT.search(body):
                hits.append(src.count(NL, 0, i) + 1)
            out.append(src[i:min(j + 1, n)])
            i = j + 1
            continue

        # ── 字符字面量：跳过（`'{'` 这种别把括号配平搞乱）──
        if c == "'":
            j, esc = i + 1, False
            while j < n and src[j] != NL:
                if esc:
                    esc = False
                elif src[j] == BS:
                    esc = True
                elif src[j] == "'":
                    break
                j += 1
            out.append(src[i:min(j + 1, n)])
            i = j + 1
            continue

        out.append(c)
        i += 1
    return ''.join(out), hits, unterminated


def iter_files(paths):
    for p in paths:
        if os.path.isfile(p):
            yield p
            continue
        for base, _dirs, files in os.walk(p):
            if any(seg in base.replace('\\', '/') for seg in ('/build', '/install', '/.git')):
                continue
            for f in files:
                if f.endswith(EXTS):
                    yield os.path.join(base, f)


def main():
    args = sys.argv[1:] or ['src', 'test']
    targets = [a if os.path.isabs(a) else os.path.join(ROOT, a) for a in args]

    n_files, bad = 0, 0
    for path in iter_files(targets):
        n_files += 1
        rel = os.path.relpath(path, ROOT).replace('\\', '/')
        try:
            src = io.open(path, encoding='utf-8', errors='replace').read()
        except OSError as e:
            print('跳过（读不了）: %s (%s)' % (rel, e))
            continue

        code, str_hits, unterminated = strip_comments_and_strings(src)
        # 代码区里的指纹 ⇒ 字符串被提前闭合（最典型的那种）
        code_lines = [code.count(NL, 0, m.start()) + 1 for m in FINGERPRINT.finditer(code)]

        problems = []
        for ln in sorted(set(code_lines)):
            line = src.split('\n')[ln - 1].strip()[:96]
            problems.append((ln, '**代码区**里出现「汉字+ASCII引号+汉字」⇒ 字符串提前闭合', line))
        for ln in sorted(set(str_hits)):
            line = src.split('\n')[ln - 1].strip()[:96]
            problems.append((ln, '**字符串内部**夹了 ASCII 引号', line))
        for ln, why in unterminated:
            line = src.split('\n')[ln - 1].strip()[:96] if ln - 1 < len(src.split('\n')) else ''
            problems.append((ln, why, line))

        if problems:
            bad += 1
            print('FAIL ' + rel)
            for ln, why, line in problems:
                print('   line %-5d %s' % (ln, why))
                print('      %s' % line)
        else:
            print('OK   ' + rel)

    print('-' * 96)
    if bad:
        print('%d/%d 个文件有命中。⚠️ 中文引用请用「」『』（全角），**别用半角双引号**。' % (bad, n_files))
        return 1
    print('%d 个文件全清（代码区无"汉字+半角引号+汉字"）。' % n_files)
    return 0


if __name__ == '__main__':
    sys.exit(main())
