#!/usr/bin/env python3
"""
parse_field_assembly.py — 解析官方/第三方 2027RC 场地总装配 STEP, 按零件输出世界坐标包围盒

用途: 与 config/field_geometry.yaml 交叉核对场地几何 (净区尺寸 / 各层高度 / 建筑位 /
      共享区 / 台阶坡道位置 等)。

为什么需要它: STEP 里每个零件的 CARTESIAN_POINT 是**零件局部坐标**, 要按
NEXT_ASSEMBLY_USAGE_OCCURRENCE + ITEM_DEFINED_TRANSFORMATION 组成的装配树
逐层变换才能还原成场地坐标。散点统计会被局部坐标误导。

实现要点 (踩过的坑, 改代码前先看):
    · CDSR 的 PRODUCT_DEFINITION_SHAPE 指向的是 **NAUO**, 不是 PRODUCT_DEFINITION,
      要再解一层 NAUO 的 child 才是零件
    · REPRESENTATION_RELATIONSHIP 的 **rep1 恒为装配体总表示 (#5931)**, 零件在 rep2
    · rep2 只是个带零件名的空壳 (items 仅含放置轴), 几何在它配对的
      **ADVANCED_BREP_SHAPE_REPRESENTATION** 里 (靠 SHAPE_REPRESENTATION_RELATIONSHIP 关联)
    · 变换直接取 T = M(轴1) 即可 —— 轴2 恒为单位阵, 不必走装配树
    · 同一零件被多次实例化 (如建造点 ×10), **必须按实例存**, 按 PD 索引会被覆盖
    · ABSR 的 items 里含共享的放置轴 (#7415, 原点 (0,0,0)), 取点时要排除, 否则污染包围盒

实测核对结果 (2026-09-14, 与 config/field_geometry.yaml 对比):
    ✅ 吻合: L1 6×6 高600 / L2 3×3 +300 顶925 / 核心柱 Ø270×800 / 穆斯蒂卡柱 Ø270×500 /
             楼梯坡道带 x[1.5,2.5] y[2.8,8.2] / L2 建造位 4.25·6.75 /
             共享区 1200×1200@(5.5,1.25) 与 1000×1000@(5.5,9.75) / 储存区 / 启动区
    ⚠ 已按官方更正: L1 建造位 2.77·8.23→2.80·8.20, L1→L2 阶梯 y[4.88,5.88]→[5.0,6.0],
                     L1 重试区 3.5→3.70
    ❓ 未决: 围栏/屏障高度 —— 模型量出 100mm, config 是 50mm (可能含底座凸缘)

用法:
    python3 scripts/parse_field_assembly.py <装配.step>            # 包围盒表 + 装配实例表

坐标系: 输出为模型原始坐标, 单位 mm。
        SolidWorks 默认 Y-up → 高度轴是 Y, X/Z 是场地平面。
        ⚠ 原点通常在半场中心, 与 field_geometry.yaml 的"西南角原点"不同, 需自行换算。
"""

import argparse
import math
import re
import sys
from collections import Counter

# ── STEP 语句切分 ─────────────────────────────────────────────────────────
# 实体形如  #123 = TYPE ( args ) ;   也有复合实体 (多个括号组)
STMT_RE = re.compile(r'#(\d+)\s*=\s*(.*?);\s*(?=#\d+\s*=|ENDSEC|$)', re.S)
REF_RE = re.compile(r'#(\d+)')
NUM3_RE = re.compile(
    r'\(\s*([-+0-9.Ee]+)\s*,\s*([-+0-9.Ee]+)\s*,\s*([-+0-9.Ee]+)\s*\)')


def load(path):
    """返回 (statements, refs) —— refs[id] = 该实体里出现的所有 #引用"""
    txt = open(path, encoding='latin-1').read()
    # 只取 DATA; 段
    m = re.search(r'DATA;\s*(.*?)\s*ENDSEC;', txt, re.S)
    body = m.group(1) if m else txt
    stmts, refs = {}, {}
    for sid, sbody in STMT_RE.findall(body):
        i = int(sid)
        stmts[i] = ' '.join(sbody.split())
        refs[i] = [int(r) for r in REF_RE.findall(sbody)]
    return stmts, refs


def first_type(body):
    m = re.match(r'([A-Z_0-9]+)', body)
    return m.group(1) if m else ''


_UNI_RE = re.compile(r'\\X2\\([0-9A-Fa-f]+)\\X0\\')


def dec_name(s):
    """STEP 的 \\X2\\<utf16be-hex>\\X0\\ 转义 → 可读中文"""
    def rep(m):
        try:
            return bytes.fromhex(m.group(1)).decode('utf-16-be', 'ignore')
        except Exception:
            return m.group(0)
    return _UNI_RE.sub(rep, s)


# ── 几何取数 ──────────────────────────────────────────────────────────────
def point_of(stmts, pid):
    m = NUM3_RE.search(stmts.get(pid, ''))
    return tuple(float(g) for g in m.groups()) if m else None


def dir_of(stmts, did):
    m = NUM3_RE.search(stmts.get(did, ''))
    return tuple(float(g) for g in m.groups()) if m else None


def axis_of(stmts, aid):
    """AXIS2_PLACEMENT_3D('', #origin, #axis_z, #axis_x) → (origin, z, x)"""
    b = stmts.get(aid, '')
    if not b.startswith('AXIS2_PLACEMENT_3D'):
        return None
    r = REF_RE.findall(b)
    if len(r) < 3:
        return None
    o = point_of(stmts, int(r[0]))
    z = dir_of(stmts, int(r[1]))
    x = dir_of(stmts, int(r[2]))
    if o is None or z is None or x is None:
        return None
    return o, z, x


def normalize(v):
    n = math.sqrt(sum(c * c for c in v))
    return tuple(c / n for c in v) if n > 1e-12 else (0.0, 0.0, 1.0)


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def mat_of_axis(ax):
    """AXIS2_PLACEMENT_3D → 4x4 齐次矩阵 (列 = 基向量, 末列 = 原点)"""
    o, z, x = ax
    z = normalize(z)
    x = normalize(x)
    y = normalize(cross(z, x))
    x = normalize(cross(y, z))          # 重新正交化
    return [[x[0], y[0], z[0], o[0]],
            [x[1], y[1], z[1], o[1]],
            [x[2], y[2], z[2], o[2]],
            [0.0, 0.0, 0.0, 1.0]]


def mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def mat_inv_rigid(M):
    """刚体矩阵求逆: R^T, -R^T t"""
    R = [row[:3] for row in M[:3]]
    t = [M[i][3] for i in range(3)]
    Rt = [[R[j][i] for j in range(3)] for i in range(3)]
    tt = [-sum(Rt[i][j] * t[j] for j in range(3)) for i in range(3)]
    return [Rt[0] + [tt[0]], Rt[1] + [tt[1]], Rt[2] + [tt[2]], [0, 0, 0, 1]]


def apply(M, p):
    return tuple(sum(M[i][j] * p[j] for j in range(3)) + M[i][3] for i in range(3))


# ── 装配树 ────────────────────────────────────────────────────────────────
def build(path):
    stmts, refs = load(path)

    # 名称: PRODUCT_DEFINITION → formation → PRODUCT
    formation_of = {}
    for i, b in stmts.items():
        if b.startswith('PRODUCT_DEFINITION ('):
            r = REF_RE.findall(b)
            if r:
                formation_of[i] = int(r[0])
    prod_of_formation = {}
    for i, b in stmts.items():
        if b.startswith('PRODUCT_DEFINITION_FORMATION ') or \
           b.startswith('PRODUCT_DEFINITION_FORMATION('):
            r = REF_RE.findall(b)
            if r:
                prod_of_formation[i] = int(r[0])
    name_of_product = {}
    for i, b in stmts.items():
        if b.startswith('PRODUCT ('):
            mm = re.search(r"PRODUCT\s*\(\s*'([^']*)'", b)
            if mm:
                name_of_product[i] = dec_name(mm.group(1))

    def pd_name(pd):
        f = formation_of.get(pd)
        p = prod_of_formation.get(f) if f else None
        return name_of_product.get(p, f'<pd{pd}>') if p else f'<pd{pd}>'

    # NAUO: 父 #a, 子 #b
    nauo_child = {}
    for i, b in stmts.items():
        if b.startswith('NEXT_ASSEMBLY_USAGE_OCCURRENCE'):
            r = REF_RE.findall(b)
            if len(r) >= 2:
                nauo_child[i] = int(r[1])

    pd_of_pdshape = {}
    for i, b in stmts.items():
        if b.startswith('PRODUCT_DEFINITION_SHAPE'):
            r = REF_RE.findall(b)
            if r:
                raw = int(r[-1])
                pd_of_pdshape[i] = nauo_child.get(raw, raw)

    # 几何体在哪: SHAPE_REPRESENTATION_RELATIONSHIP(命名SR, ADVANCED_BREP_SR)
    #   命名 SR 只装放置轴 (items = (#7415)), 真正的几何在 ABSR 里
    geom_rep_of = {}
    for i, b in stmts.items():
        if b.startswith('SHAPE_REPRESENTATION_RELATIONSHIP'):
            r = REF_RE.findall(b)
            if len(r) >= 2:
                geom_rep_of[int(r[0])] = int(r[1])

    # 父子关系
    edges = {}
    child_set = set()
    for i, b in stmts.items():
        if b.startswith('NEXT_ASSEMBLY_USAGE_OCCURRENCE'):
            r = REF_RE.findall(b)
            if len(r) >= 2:
                par, ch = int(r[0]), int(r[1])
                edges.setdefault(par, []).append(ch)
                child_set.add(ch)

    # 每个 child PD 的变换: CDSR → REPRESENTATION_RELATIONSHIP(rep1,rep2,idt)
    # ⚠ 实测: rep1 在所有 CDSR 里都是同一个 (#5931 = 装配体总表示),
    #   零件自己的几何在 rep2; 变换把 rep2 的局部坐标放到装配坐标系。
    insts = []          # 每个装配实例: {name, T, rep} —— 必须按实例存, 同零件多次实例化
    for i, b in stmts.items():
        if not b.startswith('CONTEXT_DEPENDENT_SHAPE_REPRESENTATION'):
            continue
        r = REF_RE.findall(b)
        if len(r) < 2:
            continue
        rel_id = int(r[0])
        rb = stmts.get(rel_id, '')
        m1 = re.search(r"REPRESENTATION_RELATIONSHIP\s*\(\s*'[^']*'\s*,\s*'[^']*'\s*,"
                       r"\s*#(\d+)\s*,\s*#(\d+)\s*\)", rb)
        m2 = re.search(r"REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION\s*\(\s*#(\d+)\s*\)", rb)
        if not (m1 and m2):
            continue
        rep1, rep2 = int(m1.group(1)), int(m1.group(2))
        idt = int(m2.group(1))
        ir = REF_RE.findall(stmts.get(idt, ''))
        if len(ir) < 2:
            continue
        a1, a2 = axis_of(stmts, int(ir[0])), axis_of(stmts, int(ir[1]))
        if a1 is None or a2 is None:
            continue
        # 零件局部 → 装配: 把 a1 (零件在装配中的位置) 与 a2 (父参考, 通常单位阵) 对齐
        T = mat_mul(mat_of_axis(a1), mat_inv_rigid(mat_of_axis(a2)))
        # rep2 带零件名 (SHAPE_REPRESENTATION 的 name), 但只有放置轴; 几何在它配对的 ABSR 里
        nm = re.search(r"SHAPE_REPRESENTATION\s*\(\s*'([^']*)'", stmts.get(rep2, ''))
        insts.append({
            'name': dec_name(nm.group(1)) if nm else f'<rep{rep2}>',
            'T': T,
            'rep': geom_rep_of.get(rep2, rep2),
        })

    # 不需要走装配树: 实测每个 CDSR 的轴1 就是该实例在装配体坐标里的放置
    # (rep1 恒为装配体总表示 #5931, 轴2 恒为单位阵), 所以 T = M(轴1) 直接可用。

    # 收集每个 rep 的所有点 (BFS 引用图, 简单且不依赖具体实体语义)
    def rep_points(rep):
        seen, pts = set(), []
        stk = [rep]
        while stk:
            i = stk.pop()
            if i in seen:
                continue
            seen.add(i)
            b = stmts.get(i, '')
            if b.startswith('CARTESIAN_POINT'):
                p = point_of(stmts, i)
                if p:
                    pts.append(p)
                    continue
            # ⚠ AXIS2_PLACEMENT_3D 的原点不是几何点 —— 不排掉它, 零件包围盒会被
            #   放置轴原点污染 (实测: 天空块的 bbox 被拉到轴原点 Z=4250, 比实体高 367mm)
            if b.startswith('AXIS2_PLACEMENT_3D'):
                continue
            if len(seen) > 200000:
                break
            for r in refs.get(i, []):
                if r not in seen:
                    stk.append(r)
        return pts

    def solid_roots(rep):
        """ABSR 的 items 里除放置轴以外的实体 (轴的原点 (0,0,0) 会污染包围盒)"""
        b = stmts.get(rep, '')
        items = re.search(r'\(\s*((?:#\d+[^()]*)+)\)', b)
        if not items:
            return [rep]
        roots = [int(r) for r in REF_RE.findall(items.group(1))
                 if not stmts.get(int(r), '').startswith('AXIS2_PLACEMENT_3D')]
        return roots or [rep]

    out = []
    for inst in insts:
        # ⚠ 多实体零件 (如栅栏 = 3~4 段墙) 的 ABSR items 里有多个实体, 必须**全部**收,
        #   早前 `break` 在第一个有点的实体上 → 只量到零件的一部分 (实测 2区栅栏 少读了南墙)
        pts, per_root = [], []
        for root in solid_roots(inst['rep']):
            rp = rep_points(root)
            if not rp:
                continue
            pts.extend(rp)
            wp_ = [apply(inst['T'], q) for q in rp]
            per_root.append(tuple(round(f([q[i] for q in wp_]), 1)
                                  for i in range(3) for f in (min, max)) + (len(rp),))
        if not pts:
            continue
        wp = [apply(inst['T'], p) for p in pts]
        xs = [p[0] for p in wp]
        ys = [p[1] for p in wp]
        zs = [p[2] for p in wp]
        out.append({
            'name': inst['name'],
            'n': len(pts),
            'x': (min(xs), max(xs)),
            'y': (min(ys), max(ys)),
            'z': (min(zs), max(zs)),
            # 世界坐标去重后保留一位小数 —— 供 --dump 看"厚度/内外面"这类包围盒答不了的问题
            # (矩形环的 bbox 只给外沿, 环厚要看顶点的 X/Z 各出现哪几个值)
            'u': sorted({round(v, 1) for v in xs}),
            'v': sorted({round(v, 1) for v in ys}),
            'w': sorted({round(v, 1) for v in zs}),
            # 原始点集 (去重) —— 多实体零件 (如 天空块-红 = 全部红件) 靠它还原每块矩形
            'p': sorted({(round(a, 1), round(b, 1), round(c, 1)) for a, b, c in wp}),
            # 每个实体 (solid) 各自的包围盒 —— 多实体零件按实体拆开
            #
            # ⚠️⚠️ **本工具正是"天空方块构型"那个未决问题的裁判**（2026-09-24 记，见台账 §八 9.4）：
            #   裁定已定"顶面纯色"，但**块由哪些件构成**仍有两解，而两解对判色链的**风险不同**：
            #     (C) **上下两个半高件**（本仓库仿真模型采用的）—— 绕竖轴旋转不变，
            #         于是 `field_geometry.yaml` 里那个 `code`（红南/红北/红西/红东）
            #         **没有几何解释**（不存在"这个块朝南"这个事实）;
            #     (B) 相对的两个**竖直面**一红一蓝、顶面本来无色 —— `code` 恰是它的自然产物。
            #   ⇒ **量法**：拿 `天空块-红` / `天空块-蓝` 两个零件的**每个实体**包围盒，
            #      看它们的 **z 区间与平面跨度**：
            #        · 红 z[0.10,0.20] + 蓝 z[0,0.10]（平面各跨满 200×200）⇒ **(C) 成立**;
            #        · 红/蓝 各跨满一个 200×200 的**竖直面**（z 各跨满 0~0.20）⇒ **(B)**。
            #   ⚠️ 为什么值得量：若 (B) 为真，**掠射拍到侧面会给出一个自信的错颜色**
            #      ⇒ 直接判反塔的归属（比"判不出"贵得多）⇒ 门控更不能放松。
            'roots': sorted(per_root, key=lambda t: (t[1], t[5])),
        })
    return out


def dump(parts, needle):
    """按零件名子串过滤, 打印去重后的世界坐标分量 (mm) —— 用来量厚度/内外沿"""
    hit = [p for p in parts if needle in p['name']]
    if not hit:
        print(f'没有匹配 {needle!r} 的零件')
        return
    for p in hit:
        print(f"{p['name']}  ({p['n']} 点)")
        print(f"    X: {p['x'][0]:9.1f}~{p['x'][1]:9.1f}  去重 {len(p['u'])} 个: {p['u'][:24]}")
        print(f"    Y: {p['y'][0]:9.1f}~{p['y'][1]:9.1f}  去重 {len(p['v'])} 个: {p['v'][:24]}")
        print(f"    Z: {p['z'][0]:9.1f}~{p['z'][1]:9.1f}  去重 {len(p['w'])} 个: {p['w'][:24]}")
        print(f"    实体 {len(p['roots'])} 个 (各自的包围盒, 已折算成场地坐标 m):")
        for x0, x1, y0, y1, z0, z1, n in p['roots']:
            print(f"        x[{5.5 + x0 / 1000:7.4f},{5.5 + x1 / 1000:7.4f}] "
                  f"y[{5.5 - z1 / 1000:7.4f},{5.5 - z0 / 1000:7.4f}] "
                  f"z[{(y0 - 25) / 1000:5.2f},{(y1 - 25) / 1000:5.2f}]  {n} 点")


def placements(path):
    """每个装配实例的 (零件名, 放置位置 mm) —— 直接取 ITEM_DEFINED_TRANSFORMATION 的轴原点。
    比走完整包围盒稳健: 不依赖 CDSR/NAUO 的 PD 编号对得上。"""
    stmts, _ = load(path)

    formation_of, prod_of_formation, name_of_product = {}, {}, {}
    for i, b in stmts.items():
        if b.startswith('PRODUCT_DEFINITION ('):
            r = REF_RE.findall(b)
            if r:
                formation_of[i] = int(r[0])
        elif b.startswith('PRODUCT_DEFINITION_FORMATION'):
            r = REF_RE.findall(b)
            if r:
                prod_of_formation[i] = int(r[0])
        elif b.startswith('PRODUCT ('):
            mm = re.search(r"PRODUCT\s*\(\s*'([^']*)'", b)
            if mm:
                name_of_product[i] = dec_name(mm.group(1))

    def pd_name(pd):
        f = formation_of.get(pd)
        p = prod_of_formation.get(f) if f else None
        return name_of_product.get(p, f'<pd{pd}>')

    # NAUO: 父 #a, 子 #b —— 装配实例的形状由子的 PD 决定
    nauo_child = {}
    for i, b in stmts.items():
        if b.startswith('NEXT_ASSEMBLY_USAGE_OCCURRENCE'):
            r = REF_RE.findall(b)
            if len(r) >= 2:
                nauo_child[i] = int(r[1])

    pd_of_pdshape = {}
    for i, b in stmts.items():
        if b.startswith('PRODUCT_DEFINITION_SHAPE'):
            r = REF_RE.findall(b)
            if r:
                raw = int(r[-1])
                # 这个引用可能直接是 PD, 也可能是 NAUO (SW 导出常见) → 解到子的 PD
                pd_of_pdshape[i] = nauo_child.get(raw, raw)

    out = []
    for i, b in stmts.items():
        if not b.startswith('CONTEXT_DEPENDENT_SHAPE_REPRESENTATION'):
            continue
        r = REF_RE.findall(b)
        if len(r) < 2:
            continue
        rel_id, pds_id = int(r[0]), int(r[1])
        pd = pd_of_pdshape.get(pds_id)
        rb = stmts.get(rel_id, '')
        m2 = re.search(r"REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION\s*\(\s*#(\d+)\s*\)", rb)
        if not m2:
            continue
        ir = REF_RE.findall(stmts.get(int(m2.group(1)), ''))
        if len(ir) < 2:
            continue
        a1 = axis_of(stmts, int(ir[0]))
        a2 = axis_of(stmts, int(ir[1]))
        out.append({
            'name': pd_name(pd) if pd else f'<pds{pds_id}>',
            'pd': pd,
            'axis1': a1[0] if a1 else None,
            'axis2': a2[0] if a2 else None,
            'z1': a1[1] if a1 else None,
        })
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('step')
    ap.add_argument('--dump', metavar='零件名子串',
                    help='只打印匹配零件的去重坐标分量 (mm) —— 量厚度/内外面用')
    ap.add_argument('--check', action='store_true',
                    help='与 config/field_geometry.yaml 对表')
    args = ap.parse_args()

    parts = build(args.step)

    if args.dump is not None:          # 传空串 = 全部零件 (空串是 falsy, 不能用 if args.dump)
        dump(parts, args.dump)
        return

    if parts:
        print(f"装配体零件 {len(parts)} 个 —— 世界坐标包围盒 (mm, 模型原始坐标)")
        print()
        print(f"{'零件名':<26} {'点数':>5}  {'X':>19} {'Y 高度':>17} {'Z':>19}")
        print('-' * 96)
        for p in sorted(parts, key=lambda q: (q['name'], q['y'][0])):
            print(f"{p['name']:<26} {p['n']:>5}  "
                  f"{p['x'][0]:>8.1f}~{p['x'][1]:>8.1f} "
                  f"{p['y'][0]:>7.1f}~{p['y'][1]:>7.1f} "
                  f"{p['z'][0]:>8.1f}~{p['z'][1]:>8.1f}")
        print()

    ps = placements(args.step)
    print(f"装配实例 {len(ps)} 个 —— 放置点取自 ITEM_DEFINED_TRANSFORMATION 的轴原点 (mm)")
    print()
    print(f"{'零件名':<26} {'轴1 原点 (零件原点, 非中心)':>40}")
    print('-' * 96)
    for p in sorted(ps, key=lambda q: (q['name'] or '')):
        a1 = p['axis1']
        s1 = f"({a1[0]:9.1f},{a1[1]:9.1f},{a1[2]:9.1f})" if a1 else 'None'
        print(f"{p['name']:<26} {s1:>40}")


if __name__ == '__main__':
    main()
