# -*- coding: utf-8 -*-
"""render_map.py — 从 config/field_geometry.yaml 渲染最终场地地图 (2026-09-11)

与 field_model_generator.py 的区别:
  · 那个用 field_model_spec.py 里的一套 DRAFT 占位坐标 (L1 位 3.9,3.2 / 坡道 3.0,0.2 ...), 已过期
  · 本脚本**只读 config/field_geometry.yaml** —— 该文件是 2026-09-09/11 按 SW 实测 + 用户确认
    回填后的唯一事实源, 因此画出来就是"当前认为正确的场地"

用法:  python scripts/render_map.py
输出:  scripts/out/map_final.png  (总览)  scripts/out/map_final_zoom.png (南端/西侧细节)
"""
import io
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon
import yaml

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(HERE, 'out')

for f in ('Microsoft YaHei', 'SimHei', 'SimSun', 'DengXian'):
    try:
        matplotlib.rcParams['font.sans-serif'] = [f]
        matplotlib.rcParams['axes.unicode_minus'] = False
        break
    except Exception:
        continue

with open(os.path.join(ROOT, 'config', 'field_geometry.yaml'), encoding='utf-8') as fh:
    P = yaml.safe_load(fh)['/**']['ros__parameters']
Z = P['zone_colors']


def rgb(key, fallback=(200, 200, 200)):
    v = Z.get(key, fallback)
    return tuple(c / 255.0 for c in v)


def rect(ax, x0, y0, x1, y1, color, alpha=1.0, ec='none', lw=0.8, z=1, ls='-'):
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, facecolor=color, alpha=alpha,
                           edgecolor=ec, linewidth=lw, zorder=z, linestyle=ls))


def draw_field(ax, detail=True):
    """全场 (0..11)²"""
    ax.set_xlim(-0.35, 11.35)
    ax.set_ylim(-0.35, 11.35)
    ax.set_aspect('equal')
    ax.grid(True, lw=0.3, alpha=0.25)
    ax.set_xticks(range(0, 12))
    ax.set_yticks(range(0, 12))

    # ── 地面 ──
    rect(ax, 0, 0, 5.5, 11, rgb('ground_red'), z=0)
    rect(ax, 5.5, 0, 11, 11, rgb('ground_blue'), z=0)

    # ── 地面共享区 (北, 穆斯蒂卡柱周围 1000×1000) ──
    gs = P['ground_shared_zone']
    gx, gy = gs['center']
    gh = gs['half_size']
    rect(ax, gx - gh[0], gy - gh[1], gx + gh[0], gy + gh[1], rgb('ground_shared_cream'),
         alpha=0.95, ec='#888', lw=0.8, z=2)
    ax.plot([gx - gh[0], gx + gh[0]], [gy - gh[1], gy + gh[1]], color='#999', lw=0.6, zorder=3)
    ax.plot([gx - gh[0], gx + gh[0]], [gy + gh[1], gy - gh[1]], color='#999', lw=0.6, zorder=3)

    # ── 天空棋盘 (南) ──
    sb = P['sky_block_initial']
    oxr, oyr = sb['outer_x_range'], sb['outer_y_range']
    ixr, iyr = sb['inner_x_range'], sb['inner_y_range']
    rect(ax, oxr[0], oyr[0], oxr[1], oyr[1], rgb('ground_shared_cream'),
         alpha=0.95, ec='#888', lw=0.9, z=2)          # 外框 1200
    rect(ax, ixr[0], iyr[0], ixr[1], iyr[1], 'none',
         ec='#aaa', lw=0.7, z=3, ls='--')             # 内框 1000
    step = sb['grid_step']
    ox, oy = sb['grid_origin']
    pos = sb['positions']
    # ⚠️⚠️ 2026-09-24 裁定: 天空方块**顶面是纯色**（"每块红蓝各半"是误读）——
    #    依据见 `config/field_geometry.yaml` 的 `sky_block_initial` 与台账 §八 9.4。
    #    ⇒ 俯视图（本图就是俯视图）看到的是**顶面** ⇒ 每格画**一个整色块**，
    #      而不是原来那样把它切成红半/蓝半（那是误读的产物，图上颜色对不上世界）。
    #    ⚠️ 画哪个色 = "**正放**时朝上的颜色" ⇒ 当前模型是**红上**（`make_field_from_config.py`
    #       建的是"下蓝上红"两个半高件）⇒ 这里画红。
    #    ⚠️ **待确认(FAQ)**: 规则书 Figure 4 里各格朝上的颜色（红/蓝交错图案）本仓库
    #       **没有可解析依据** ⇒ 现在全画红。若 Figure 4 确为交错，这里要按那份数据改。
    #    ⚠️ `code`（红南/红北/红西/红东）是**块在棋盘里的方向记号**，**不影响顶面颜色**
    #       ⇒ 本图不再按它切块。（仍在 `positions` 里保留，那是官方总装配的实测数据。）
    #    ⚠️ 本脚本是旧渲染器，**现行的是 `render_map_from_sdf.py`**（直接读生成的 model.sdf，
    #       天然与模型一致）—— 两处都要对，别只改一处。
    for i in range(0, len(pos), 3):
        x, y, _code = pos[i], pos[i + 1], pos[i + 2]
        x0, x1 = x - step / 2, x + step / 2
        y0, y1 = y - step / 2, y + step / 2
        rect(ax, x0, y0, x1, y1, rgb('sky_block_red_face'), ec='#333', lw=0.5, z=4)

    # ── 存储区 ──
    for side, col_key in (('ours', 'storage_red'), ('theirs', 'storage_blue')):
        c = P['storage_zone'][side]['corners']
        xs, ys = c[0::2], c[1::2]
        rect(ax, min(xs), min(ys), max(xs), max(ys), rgb(col_key), alpha=0.85, ec='#444', lw=0.8, z=3)

    # ── 启动区 (红 TR/BR, 蓝镜像) ──
    sz = P['start_zones']['box_size'] / 2
    for side, col_key in (('ours', 'start_retry_red'), ('theirs', 'start_retry_blue')):
        for tag in ('tr', 'br'):
            cx, cy = P['start_zones'][side][tag]
            rect(ax, cx - sz, cy - sz, cx + sz, cy + sz, rgb(col_key), ec='#222', lw=0.7, z=4)
            ax.text(cx, cy, tag.upper(), ha='center', va='center', fontsize=5.5,
                    color='w', zorder=5, fontweight='bold')

    # ── L1 平台 ──
    l1 = P['l1_platform']
    x0, x1 = l1['x_range']
    y0, y1 = l1['y_range']
    rect(ax, x0, y0, x1, y1, rgb('l1_area_red'), alpha=0.30, ec='#555', lw=1.2, z=5)
    ax.text(x0 + 0.15, y1 - 0.3, 'L1  h=0.6', fontsize=8, color='#333', zorder=9)

    # ── L1 共享带 (外框 ⊃ 内框, 两端) ──
    sh = P['l1_shared_zone']
    bw = sh['band_width'] / 2
    for end in ('south', 'north'):
        oy0, oy1 = sh['ends'][end]['outer_y_range']
        iy0, iy1 = sh['ends'][end]['inner_y_range']
        rect(ax, 5.5 - bw, oy0, 5.5 + bw, oy1, rgb('l1_shared_cream'), alpha=0.95,
             ec='#555', lw=0.9, z=6)                       # 外框 1000×750
        rect(ax, 5.25, iy0, 5.75, iy1, 'none', ec='#c60',
             lw=1.1, z=8, ls='--')                         # 内框 500×500 = 共享建筑位

    # ── 楼梯 / 坡道 / 转运区 (红西 / 蓝东 双侧对称, 2026-09-11 用户确认"右边也有左边的全套") ──
    for side, stair_key, ramp_key, tc_key in (
            ('ours', 'l1_stairs_red', 'ramp_red', 'transfer_red'),
            ('theirs', 'l1_stairs_blue', 'ramp_blue', 'transfer_blue')):
        st = P['stairs'][side]
        bx0, bx1 = st['band_x']
        y_a, y_b = st['from_xy'][1], st['to_xy'][1]
        rect(ax, bx0, min(y_a, y_b), bx1, max(y_a, y_b), rgb(stair_key),
             alpha=0.9, ec='#333', lw=0.7, z=6)
        n = st.get('step_count', 3)
        for k in range(1, n):                      # 画踏面分隔线, 直观看出几级
            yy = min(y_a, y_b) + k * (max(y_a, y_b) - min(y_a, y_b)) / n
            ax.plot([bx0, bx1], [yy, yy], color='#333', lw=0.4, zorder=7)
        ax.text((bx0 + bx1) / 2, (y_a + y_b) / 2, '梯%d' % n,
                ha='center', va='center', fontsize=5.5, zorder=8)

        rp = P['ramp'][side]
        rb0, rb1 = rp['band_x']
        ylo, yhi = rp['low_xy'][1], rp['high_xy'][1]
        rect(ax, rb0, min(ylo, yhi), rb1, max(ylo, yhi), rgb(ramp_key), alpha=0.9,
             ec='#333', lw=0.7, z=6)
        mid = (rb0 + rb1) / 2
        ax.annotate('', xy=(mid, yhi - 0.05), xytext=(mid, ylo + 0.05),
                    arrowprops=dict(arrowstyle='->', lw=0.9, color='#333'), zorder=7)
        ax.text(mid + 0.62, (ylo + yhi) / 2, '坡道\n3.5m', fontsize=5,
                ha='left', va='center', zorder=7)

        tc = P['transfer_zone'][side]['corners']
        tcx, tcy = P['transfer_zone'][side]['center']
        rect(ax, min(tc[0::2]), min(tc[1::2]), max(tc[0::2]), max(tc[1::2]),
             rgb(tc_key), ec='#222', lw=0.8, z=7)
        ax.text(tcx, tcy, '转', ha='center', va='center', fontsize=6, zorder=8)

    # ── L1 重试区 ──
    rz = P['retry_zones']['l1']
    for side, col_key in (('ours', 'l1_retry_red'), ('theirs', 'l1_retry_blue')):
        cx, cy = rz[side]
        rect(ax, cx - 0.35, cy - 0.35, cx + 0.35, cy + 0.35, rgb(col_key), ec='#222', lw=0.7, z=7)

    # ── L1→L2 阶梯 (红西条 / 蓝东条) —— V3 撤销, 跟随 config.enabled ──
    s2 = P['l1_l2_stairs']
    if s2.get('enabled', True):
        for side, col_key in (('ours', 'l2_stairs_red'), ('theirs', 'l2_stairs_blue')):
            sx0, sx1 = s2[side]['x_range']
            sy0, sy1 = s2['y_range']
            rect(ax, sx0, sy0, sx1, sy1, rgb(col_key), alpha=0.95, ec='#333', lw=0.7, z=8)

    # ── L2 平台 ──
    l2 = P['l2_platform']
    x0, x1 = l2['x_range']
    y0, y1 = l2['y_range']
    rect(ax, x0, y0, x1, y1, rgb('l2_area'), alpha=0.55, ec='#444', lw=1.0, z=8)
    ax.text(x0 + 0.1, y1 - 0.22, 'L2  h=0.9', fontsize=7, color='#222', zorder=10)

    # ── 建筑位 (10) ──
    sx, sy, sz_ = P['building_spot_x'], P['building_spot_y'], P['building_spot_z']
    hs = P['building_spots']['spot_half_size']
    for i, (x, y, z) in enumerate(zip(sx, sy, sz_)):
        is_shared = abs(x - 5.5) < 1e-6
        col = rgb('building_spot_l1') if z < 0.75 else rgb('building_spot_l2')
        ec = '#c00' if (x < 5.5 and z < 0.75) else ('#00c' if (x > 5.5 and z < 0.75) else '#000')
        rect(ax, x - hs, y - hs, x + hs, y + hs, col, ec=ec, lw=1.0, z=9)
        tag = 'L1' if z < 0.75 else 'L2'
        ax.text(x, y, tag, ha='center', va='center', fontsize=5,
                color='w', zorder=10, fontweight='bold')

    # ── 柱子 ──
    for key in ('mustika_pillar', 'core_pillar'):
        px, py = P[key]['position']
        r = P[key]['radius']
        ax.add_patch(Circle((px, py), r, facecolor=rgb('central_pillar_brown'),
                            edgecolor='#000', lw=0.8, zorder=11))

    # ── 中轴隔墙 (带南北开口) ──
    cd = P['center_divider']
    ax.plot([5.5, 5.5], [cd['segments'][1], cd['segments'][3]], color='#5a3a00',
            lw=2.2, zorder=12, solid_capstyle='butt')

    # ── 外圈围栏 ──
    rect(ax, 0, 0, 11, 11, 'none', ec='#5a3a00', lw=1.6, z=13)

    ax.set_title('ROBOCON 2027 BR 场地 — 数据源 config/field_geometry.yaml', fontsize=10)


def main():
    os.makedirs(OUT, exist_ok=True)

    fig, ax = plt.subplots(figsize=(11, 11))
    draw_field(ax)
    ax.set_xlabel('X 东 (m)   ·   红=西半场(我方)  蓝=东半场')
    ax.set_ylabel('Y 北 (m)')
    fig.tight_layout()
    p1 = os.path.join(OUT, 'map_final.png')
    fig.savefig(p1, dpi=150)
    print('->', p1)

    # ── 细节图: 南端天空棋盘 + L1 西侧通道 ──
    fig2, axs = plt.subplots(1, 2, figsize=(11, 6.5),
                             gridspec_kw={'width_ratios': [1.37, 0.333]})
    draw_field(axs[0]); axs[0].set_xlim(4.2, 6.8); axs[0].set_ylim(0.3, 2.2)
    axs[0].set_title('南端 · 天空棋盘 (1200 外框 / 1000 内框 / 25 格 × 200)', fontsize=9)
    draw_field(axs[1]); axs[1].set_xlim(1.0, 3.2); axs[1].set_ylim(2.2, 8.8)
    axs[1].set_title('西侧通道 · 楼梯(40/41/43) → 转运区(44) → 坡道(45)', fontsize=9)
    fig2.tight_layout()
    p2 = os.path.join(OUT, 'map_final_zoom.png')
    fig2.savefig(p2, dpi=150)
    print('->', p2)

    # ── 汇总 ──
    sx, sy, sz_ = P['building_spot_x'], P['building_spot_y'], P['building_spot_z']
    print()
    print('建筑位 %d 个:' % len(sx))
    for i, (x, y, z) in enumerate(zip(sx, sy, sz_)):
        who = '共享' if abs(x - 5.5) < 1e-6 else ('红(我方)' if x < 5.5 else '蓝(对方)')
        print('   [%2d] (%.2f, %.2f)  z=%.1f  %s' % (i, x, y, z, who))
    print('天空方块 %d 块 @ 棋盘南端' % (len(P['sky_block_initial']['positions']) // 3))


if __name__ == '__main__':
    main()
