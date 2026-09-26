# -*- coding: utf-8 -*-
"""make_field_from_config.py — 从 config/field_geometry.yaml 生成 Gazebo 场地 (2026-09-11)

取代旧的 make_gazebo_world.py + rc1_field_zup.stl 路线:
  · 旧路线读 field_model_spec.py 的 DRAFT 占位坐标, 且柱子坐标写死 S.CORE_PILLAR_POS
    (两根柱都放在 5.5,5.5 —— 穆斯蒂卡柱实际应在 5.5,9.75), 场地 STL 本身也是旧 spec 造的。
  · 本脚本**只读 config/field_geometry.yaml**, 用 SDF 基本体重建整个场地, 不用外部 STL。

做法: 所有轴对齐结构用 <box>, 柱子用 <cylinder>, 坡道用生成的楔形 .obj。
      这样 config 一改, 重跑本脚本即可同步。

输出 (scripts/out/gazebo/):
  worlds/br_field.world
  models/br_field/model.sdf + model.config
  models/br_field/meshes/ramp_red.obj, ramp_blue.obj
  models/sensor_rig/*        (沿用 make_gazebo_world.py 的传感器平台, 数据与场地无关)

用法:  python scripts/make_field_from_config.py
"""
import io
import os
import sys

import yaml

sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
DB = os.path.join(HERE, 'out', 'gazebo')
WORLD_DIR = os.path.join(DB, 'worlds')
FIELD_DIR = os.path.join(DB, 'models', 'br_field')
MESH_DIR = os.path.join(FIELD_DIR, 'meshes')
for d in (WORLD_DIR, FIELD_DIR, MESH_DIR):
    os.makedirs(d, exist_ok=True)

with open(os.path.join(ROOT, 'config', 'field_geometry.yaml'), encoding='utf-8') as fh:
    P = yaml.safe_load(fh)['/**']['ros__parameters']
Z = P['zone_colors']


def f(v):
    return ('%.4f' % v).rstrip('0').rstrip('.')


def col(key, alpha=1.0):
    r, g, b = Z[key]
    return '%.3f %.3f %.3f %.2f' % (r / 255.0, g / 255.0, b / 255.0, alpha)


# 视觉板用的薄厚度 (纯装饰, 不参与碰撞)
PLATE = 0.004


class Sdf(object):
    def __init__(self):
        self.L = []
        self.n = 0

    def a(self, s):
        self.L.append(s)

    def box(self, name, x0, y0, x1, y1, z0, z1, color, collide=True, alpha=1.0):
        cx, cy, cz = (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2
        sx, sy, sz = abs(x1 - x0), abs(y1 - y0), abs(z1 - z0)
        self.n += 1
        self.a('    <link name="%s">' % name)
        self.a('      <pose>%s %s %s 0 0 0</pose>' % (f(cx), f(cy), f(cz)))
        self.a('      <visual name="v"><geometry><box><size>%s %s %s</size></box></geometry>'
               '<material><ambient>%s</ambient><diffuse>%s</diffuse></material></visual>'
               % (f(sx), f(sy), f(sz), color, color))
        if collide:
            self.a('      <collision name="c"><geometry><box><size>%s %s %s</size></box></geometry></collision>'
                   % (f(sx), f(sy), f(sz)))
        self.a('    </link>')

    def cyl(self, name, x, y, z0, z1, r, color):
        self.n += 1
        self.a('    <link name="%s">' % name)
        self.a('      <pose>%s %s %s 0 0 0</pose>' % (f(x), f(y), f((z0 + z1) / 2)))
        self.a('      <visual name="v"><geometry><cylinder><radius>%s</radius><height>%s</height></cylinder></geometry>'
               '<material><ambient>%s</ambient><diffuse>%s</diffuse></material></visual>'
               % (f(r), f(abs(z1 - z0)), color, color))
        self.a('      <collision name="c"><geometry><cylinder><radius>%s</radius><height>%s</height></cylinder></geometry></collision>'
               % (f(r), f(abs(z1 - z0))))
        self.a('    </link>')

    def mesh(self, name, uri, x, y, z, color):
        self.n += 1
        self.a('    <link name="%s">' % name)
        self.a('      <pose>%s %s %s 0 0 0</pose>' % (f(x), f(y), f(z)))
        self.a('      <visual name="v"><geometry><mesh><uri>%s</uri></mesh></geometry>'
               '<material><ambient>%s</ambient><diffuse>%s</diffuse></material></visual>' % (uri, color, color))
        self.a('      <collision name="c"><geometry><mesh><uri>%s</uri></mesh></geometry></collision>' % uri)
        self.a('    </link>')


def write_wedge_obj(path, x0, x1, y_lo, y_hi, z_lo, z_hi, mtl_rgb=None, mtl_name='ramp'):
    """右三角楔: 在 y_lo 处高 z_lo, 线性升到 y_hi 处高 z_hi; x 方向等宽。
    导出局部坐标 (原点在楔体包围盒中心), 便于用 pose 摆放。
    同时写一个 .mtl 并在 OBJ 里 usemtl —— 否则 Gazebo 报
    "Missing material for shape[...] in OBJ file" (斜坡显示成默认灰)。"""
    mtl_base = None
    if mtl_rgb:
        mtl_path = os.path.splitext(path)[0] + '.mtl'
        mtl_base = os.path.basename(mtl_path)
        with open(mtl_path, 'w', encoding='utf-8') as md:
            md.write('newmtl %s\nKa 1 1 1\nKd %s %s %s\nKs 0 0 0\nd 1\n' % (
                mtl_name, f(mtl_rgb[0]), f(mtl_rgb[1]), f(mtl_rgb[2])))
    cx, cy = (x0 + x1) / 2.0, (y_lo + y_hi) / 2.0
    hx = (x1 - x0) / 2.0

    def P3(x, y, z):
        return (x - cx, y - cy, z)

    v = [P3(x0, y_lo, z_lo), P3(x1, y_lo, z_lo), P3(x1, y_hi, z_hi), P3(x0, y_hi, z_hi),
         P3(x0, y_lo, 0.0), P3(x1, y_lo, 0.0), P3(x1, y_hi, 0.0), P3(x0, y_hi, 0.0)]
    faces = [(0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1), (3, 2, 6, 7), (1, 5, 6, 2), (0, 3, 7, 4)]
    with open(path, 'w', encoding='utf-8') as fd:
        fd.write('# ramp wedge  x[%s,%s] y[%s,%s] z[%s,%s]  (局部坐标, 原点=包围盒中心)\n' % tuple(
            f(t) for t in (x0, x1, y_lo, y_hi, z_lo, z_hi)))
        if mtl_base:
            fd.write('mtllib %s\n' % mtl_base)
            fd.write('usemtl %s\n' % mtl_name)
        fd.write('o ramp\n')
        for p in v:
            fd.write('v %s %s %s\n' % (f(p[0]), f(p[1]), f(p[2])))
        for fa in faces:
            fd.write('f ' + ' '.join(str(i + 1) for i in fa) + '\n')
    return cx, cy          # 网格/碰撞体要用这个 pose 摆回去


def gen_field():
    s = Sdf()
    l1 = P['l1_platform']
    l2 = P['l2_platform']
    L1H, L2H = l1['height'], l2['height']
    l1x0, l1x1 = l1['x_range']
    l1y0, l1y1 = l1['y_range']
    l2x0, l2x1 = l2['x_range']
    l2y0, l2y1 = l2['y_range']
    ff = P['fences']
    fw, fh = ff['width'], ff['height']
    cd = P['center_divider']
    cw, chh = cd['wall_width'], cd['wall_height']

    # ── 地面底板: 结构板比 11×11 大一个墙厚, 好让围栏有支撑 ──
    #    11×11 是**净场地**(不算围栏), 围栏立在它外面 —— 用户 2026-09-11 明确
    s.box('ground_red', -fw, -fw, 5.5, 11 + fw, -0.05, 0.0, col('ground_red'))
    s.box('ground_blue', 5.5, -fw, 11 + fw, 11 + fw, -0.05, 0.0, col('ground_blue'))

    # ── 地面共享区 (北) / 天空棋盘底板 (南) ──
    gs = P['ground_shared_zone']
    gx, gy = gs['center']
    gh = gs['half_size']
    s.box('ground_shared_area', gx - gh[0], gy - gh[1], gx + gh[0], gy + gh[1],
          0, PLATE, col('ground_shared_cream'), collide=False)
    sb = P['sky_block_initial']
    oxr, oyr = sb['outer_x_range'], sb['outer_y_range']
    s.box('sky_board', oxr[0], oyr[0], oxr[1], oyr[1], 0, PLATE,
          col('ground_shared_cream'), collide=False)

    # ── 存储区 (红蓝) ──
    for side, key in (('ours', 'storage_red'), ('theirs', 'storage_blue')):
        c = P['storage_zone'][side]['corners']
        s.box('storage_' + side, min(c[0::2]), min(c[1::2]), max(c[0::2]), max(c[1::2]),
              0, PLATE, col(key), collide=False)

    # ── 启动区 (TR/BR) ──
    hs = P['start_zones']['box_size'] / 2
    for side, key in (('ours', 'start_retry_red'), ('theirs', 'start_retry_blue')):
        for tag in ('tr', 'br'):
            cx, cy = P['start_zones'][side][tag]
            s.box('start_%s_%s' % (side, tag), cx - hs, cy - hs, cx + hs, cy + hs,
                  0, PLATE, col(key), collide=False)

    # ── L1 平台: 结构体**恰为 6×6** (2026-09-17 按官方 `二层基座` X/Z 均恰 ±3000 更正;
    #    旧代码向外多长了一个墙厚 fw —— 官方无此外扩) ──
    #    ★ L1 台面按半场分色: 西(红) l1_area_red / 东(蓝) l1_area_blue (§14 两色都有, 早前整块刷红是错的)
    s.box('l1_structure_w', l1x0, l1y0, 5.5, l1y1, 0, L1H, col('l1_area_red'))
    s.box('l1_structure_e', 5.5, l1y0, l1x1, l1y1, 0, L1H, col('l1_area_blue'))
    s.box('l1_play_w', l1x0, l1y0, 5.5, l1y1, L1H, L1H + PLATE, col('l1_area_red'), collide=False)
    s.box('l1_play_e', 5.5, l1y0, l1x1, l1y1, L1H, L1H + PLATE, col('l1_area_blue'), collide=False)

    # ── L1 共享带 (南北两端外框) ──
    sh = P['l1_shared_zone']
    bw = sh['band_width'] / 2
    for end in ('south', 'north'):
        oy0, oy1 = sh['ends'][end]['outer_y_range']
        s.box('l1_shared_%s' % end, 5.5 - bw, oy0, 5.5 + bw, oy1, L1H, L1H + PLATE,
              col('l1_shared_cream'), collide=False)

    # ── 楼梯 / 坡道 / 转运区 (红西 / 蓝东) ──
    for side, stair_key, ramp_key, tc_key in (
            ('ours', 'l1_stairs_red', 'ramp_red', 'transfer_red'),
            ('theirs', 'l1_stairs_blue', 'ramp_blue', 'transfer_blue')):
        st = P['stairs'][side]
        bx0, bx1 = st['band_x']
        ya, yb = st['from_xy'][1], st['to_xy'][1]
        n = st['step_count']
        hstep = st['step_height']
        y0, y1 = min(ya, yb), max(ya, yb)
        run = (y1 - y0) / n
        for k in range(n):                     # 3 级踏面, 每级升 0.15
            s.box('stairs_%s_%d' % (side, k), bx0, y0 + k * run, bx1, y0 + (k + 1) * run,
                  0, hstep * (k + 1), col(stair_key))

        rp = P['ramp'][side]
        rb0, rb1 = rp['band_x']
        lo_y, hi_y = rp['low_xy'][1], rp['high_xy'][1]     # low=接地端, high=顶部
        ya, yb = min(lo_y, hi_y), max(lo_y, hi_y)
        za = 0.0 if lo_y < hi_y else rp['rise']            # y=ya 处的高度
        zb = rp['rise'] if lo_y < hi_y else 0.0            # y=yb 处的高度
        name = 'ramp_%s.obj' % ('red' if side == 'ours' else 'blue')
        rcx, rcy = write_wedge_obj(os.path.join(MESH_DIR, name), rb0, rb1, ya, yb, za, zb,
                                   mtl_rgb=tuple(c / 255.0 for c in Z[ramp_key]), mtl_name='ramp')
        s.mesh('ramp_%s' % side, 'model://br_field/meshes/' + name, rcx, rcy, 0.0, col(ramp_key))

        tc = P['transfer_zone'][side]['corners']
        s.box('transfer_%s' % side, min(tc[0::2]), min(tc[1::2]), max(tc[0::2]), max(tc[1::2]),
              0, L1H, col(tc_key))       # 与 L1 同高的实心台

    # ── L1→L2 阶梯 (红西条 / 蓝东条), 2 级 ──
    s2 = P['l1_l2_stairs']
    sy0, sy1 = s2['y_range']
    hstep2 = s2['step_height']
    for side, key in (('ours', 'l2_stairs_red'), ('theirs', 'l2_stairs_blue')):
        sxr = s2[side]['x_range']         # 红 x[3.7,4.0] 贴 L2 西边 / 蓝 x[7.0,7.3] 贴 L2 东边
        s.box('l2stair_%s' % side, sxr[0], sy0, sxr[1], sy1,
              L1H, L1H + hstep2, col(key))     # 中间踏面; 再上一级即 L2 平台本身

    # ── L2 平台 (无侧墙) ──
    #   ⚠ 2026-09-11: 规则书 §14 色表里虽列了 "L2 Side Wall", 但**实物/团队模型没有** ——
    #   用户明确 "L2 没有侧墙, 删掉"。早前按色表加过 6 段 l2sw_*, 已移除。
    #   SW 亦印证: 草图10 只有 L2 外框轮廓, 无任何双线(墙)特征。
    s.box('l2_platform', l2x0, l2y0, l2x1, l2y1, L1H, L2H, col('l2_area'))

    # ── 建筑位 (10 个薄板, 用 per-spot z) ──
    sx_, sy_, sz_ = P['building_spot_x'], P['building_spot_y'], P['building_spot_z']
    hsz = P['building_spots']['spot_half_size']
    for i, (x, y, z) in enumerate(zip(sx_, sy_, sz_)):
        s.box('spot_%02d' % i, x - hsz, y - hsz, x + hsz, y + hsz, z, z + PLATE,
              col('building_spot_l1' if z < 0.75 else 'building_spot_l2'), collide=False)

    # ── 柱 (含顶部凹槽: 用外径圆柱近似, 凹槽对仿真影响小) ──
    mp = P['mustika_pillar']
    s.cyl('mustika_pillar', mp['position'][0], mp['position'][1], 0, mp['height'],
          mp['radius'], col('mustika_pillar_brown'))
    cp = P['core_pillar']
    s.cyl('core_pillar', cp['position'][0], cp['position'][1], L2H, L2H + cp['height'],
          cp['radius'], col('central_pillar_brown'))

    # ── 天空方块 12 块 (200×200×200, 落在地面) ──
    # ⚠️⚠️ 2026-09-24 **裁定: 顶面是纯色**（"每块红蓝各半"是**误读**）—— 依据见
    #   `config/field_geometry.yaml` 的 sky_block_initial 与台账 §八:
    #     ① 官方原文 "colored red on one side and blue on the other" = **相对的两面**
    #        一红一蓝, 不是"每一面各半";
    #     ② 规则 3.5.6 "scored based on the color of their **top surface**" +
    #        3.5.8 "flip … to ensure their **color faces upward**" ——
    #        这套判据**只有在"顶面纯色、翻转即换色"时才可执行**;
    #        若顶面恒半红半蓝, 裁判自己都判不出分, 规则不可能这样写;
    #     ③ 官方总装配的"每块 = 整块 + 半块"是**沿竖直方向**拼的两个半高件
    #        （所以才有"红蓝各半"这种说法），**不是把顶面切成两半**。
    #   ⇒ 块建成**上下两半**: 下半蓝、上半红 ⇒
    #        正放: 顶面**纯红**      倒放(翻转 180°): 顶面**纯蓝**
    #
    #   ⚠️ `positions` 第三个数 code（红南/红北/红西/红东）现在读作**该块在棋盘里的
    #     朝向**（与 4.1.4 的镜像对称一致），**不是单块顶面的分区方向**:
    #      它**不影响顶面判色**（块绕竖轴旋转是不变的量），所以本生成器**不再按它切面**。
    #      数据**保留**（那是官方总装配的实测方向），但不再当"分色方向"用。
    #
    #   ⚠️ **待确认（FAQ）**: 规则书 Figure 4 里各格**朝上的颜色**（红/蓝交错图案）
    #      本仓库**没有可解析依据** ⇒ 现在 12 块**全部红上**。若 Figure 4 确为红蓝交错,
    #      要改成"其中若干块倒放"（蓝上红下）——**那件事的数据来源待组织方确认**。
    #
    #   ⚠️⚠️ 给判色链的话: 仿真里若跑出 `RATIO_AMBIGUOUS`（红蓝各半）⇒ **是模型错了,
    #      不是规则模糊** —— **不许据此去放宽 `decisive_ratio`**（那会把判错的代价
    #      从"判不出"变成"判反", 而判反 = 把塔的归属送给对手）。
    step = sb['grid_step']
    pos = sb['positions']
    hblock = 0.20
    for i in range(0, len(pos), 3):
        x, y, _code = pos[i], pos[i + 1], pos[i + 2]
        x0, x1 = x - step / 2, x + step / 2
        y0, y1 = y - step / 2, y + step / 2
        # 两个半高件（竖直拼）—— 名字直说哪半在上、是什么色，改错了一眼能看出来
        s.box('sky_%02d_bottom_blue' % (i // 3), x0, y0, x1, y1,
              0, hblock / 2, col('sky_block_blue_face'))
        s.box('sky_%02d_top_red' % (i // 3), x0, y0, x1, y1,
              hblock / 2, hblock, col('sky_block_red_face'))

    # ── 场地外圈围栏 (20mm × 50mm) —— 立在 11×11 净区**外面** ──
    s.box('fence_s', -fw, -fw, 11 + fw, 0.0, 0, fh, col('game_field_boundary'))
    s.box('fence_n', -fw, 11.0, 11 + fw, 11 + fw, 0, fh, col('game_field_boundary'))
    s.box('fence_w', -fw, 0.0, 0.0, 11.0, 0, fh, col('game_field_boundary'))
    s.box('fence_e', 11.0, 0.0, 11 + fw, 11.0, 0, fh, col('game_field_boundary'))

    # ── 中轴隔墙 (x=5.5, 50mm 宽 × 100mm 高) ──
    #   ✅ 2026-09-17 二次更正 (用户发现"地面中轴上靠围栏少了墙") —— 地面段**跑满 y[0,11]**,
    #   减去 L1 台体 + 两处共享区 (config 的 ground_breaks_y)。L1 段 y[3.3,7.7]。
    #   ⚠ 上一版给的是 y[1.85,9.25] —— 那是只读包围盒的错, 漏掉了藏在 `1区栅栏` 实体**内部**
    #     的南北两截贴围栏短墙 (详见 config 里那段二次更正 + .tmp_pdf/probe_ring_entities.py)。
    #   埋在 L1 台体 / L2 台体里的段落不建: 官方建模是连续跑过去被实体吞掉, 看不见, 等价
    def minus_span(a, b, cuts):
        """[a,b] 减去若干区间后剩下的段 (丢掉 20mm 级碎片)"""
        out, cur = [], a
        for c0, c1 in sorted(cuts):
            if c1 <= cur or c0 >= b:
                continue
            if c0 > cur:
                out.append((cur, min(c0, b)))
            cur = max(cur, c1)
        if cur < b:
            out.append((cur, b))
        return [(p, q) for p, q in out if q - p > 0.05]

    xa, xb = 5.5 - cw / 2, 5.5 + cw / 2
    # 地面段要减掉: L1 台体 (埋掉看不见) + 两处共享区 (天空棋盘 / 圣所, 官方本来就没墙)
    # ⚠️ `ground_breaks_y` 是**拍平**的 `[y0,y1, y0,y1, ...]`（2026-09-25 改，
    #    原为嵌套数组 `[[..],[..]]` —— 那会让 bringup_debug.launch.py 整份起不来，
    #    理由见 config/field_geometry.yaml 该键上方的 ⚠️⚠️）。这里两两配对还原成区间。
    _gby = cd.get('ground_breaks_y', [])
    gcut = [(l1y0, l1y1)] + list(zip(_gby[0::2], _gby[1::2]))
    for k, (p0, p1) in enumerate(
            minus_span(cd['ground_y_range'][0], cd['ground_y_range'][1], gcut)):
        s.box('divider_g_%d' % k, xa, p0, xb, p1, 0.0, chh, col('center_divider_fence'))
    for k, (p0, p1) in enumerate(
            minus_span(cd['l1_y_range'][0], cd['l1_y_range'][1], [(l2y0, l2y1)])):
        s.box('divider_l1_%d' % k, xa, p0, xb, p1, L1H, L1H + chh, col('center_divider_fence'))

    # ── L1 周界屏障 (50×100) —— **立在 6×6 台面里侧**, 外沿恰与台面边齐 ──
    #    ✅ 2026-09-17 按官方 `2区栅栏` 更正: 顶点去重 X = {±3000, ±2950} → 厚 50mm,
    #    外沿在 ±3000 (= L1 边), 即站在台面**内**, 不是外侧 (旧代码画在 x[2.48,2.50] 外面)。
    #    西/东边在转运区处留口 (官方 Z 去重含 ±800 → y[3.7,4.7] 确有开口)。
    pbi = fw
    gap_y0 = P['transfer_zone']['ours']['corners'][1]
    gap_y1 = P['transfer_zone']['ours']['corners'][5]
    s.box('l1pb_s', l1x0, l1y0, l1x1, l1y0 + pbi, L1H, L1H + fh, col('l1_perimeter_barrier'))
    s.box('l1pb_n', l1x0, l1y1 - pbi, l1x1, l1y1, L1H, L1H + fh, col('l1_perimeter_barrier'))
    s.box('l1pb_w_s', l1x0, l1y0 + pbi, l1x0 + pbi, gap_y0, L1H, L1H + fh, col('l1_perimeter_barrier'))
    s.box('l1pb_w_n', l1x0, gap_y1, l1x0 + pbi, l1y1 - pbi, L1H, L1H + fh, col('l1_perimeter_barrier'))
    s.box('l1pb_e_s', l1x1 - pbi, l1y0 + pbi, l1x1, gap_y0, L1H, L1H + fh, col('l1_perimeter_barrier'))
    s.box('l1pb_e_n', l1x1 - pbi, gap_y1, l1x1, l1y1 - pbi, L1H, L1H + fh, col('l1_perimeter_barrier'))

    # ── L1 重试区 (薄板) ──
    rz = P['retry_zones']['l1']
    for side, key in (('ours', 'l1_retry_red'), ('theirs', 'l1_retry_blue')):
        cx, cy = rz[side]
        s.box('retry_%s' % side, cx - 0.35, cy - 0.35, cx + 0.35, cy + 0.35,
              L1H, L1H + PLATE, col(key), collide=False)

    out = ['<?xml version="1.0"?>', '<sdf version="1.6">', '  <model name="br_field">',
           '    <static>true</static>'] + s.L + ['  </model>', '</sdf>']
    text = '\n'.join(out)
    with open(os.path.join(FIELD_DIR, 'model.sdf'), 'w', encoding='utf-8') as fd:
        fd.write(text + '\n')
    with open(os.path.join(FIELD_DIR, 'model.config'), 'w', encoding='utf-8') as fd:
        fd.write('<?xml version="1.0"?>\n<model>\n  <name>br_field</name>\n  <version>1.1</version>\n'
                 '  <sdf version="1.6">model.sdf</sdf>\n  <author><name>br_perception</name></author>\n'
                 '  <description>ROBOCON 2027 field, generated from config/field_geometry.yaml</description>\n</model>\n')
    return len(s.L), text


# 世界里要不要放传感器平台 (仿真里唯一的雷达/相机/IMU 来源)。
# ⚠ 2026-09-11 修: 这个开关原来只在 make_gazebo_world 里, 而 world 已改由本文件生成,
#   导致 sed 改 True 也不生效 —— 现在放在这里, make_gazebo_world 会把它的设置同步过来。
WITH_SENSOR_RIG = False
SENSOR_RIG_POSE = (5.5, 2.0, 1.3, 0.0)      # x y z yaw

# 世界里要不要放**仿真载具** (fishbot, 顶替未完工的实车) —— 仿真里雷达/相机/IMU 的唯一来源。
# ⚠ 模型本身是脚本外的**手工资产** (scripts/out/gazebo/models/fishbot/, 2026-09-17 VM 加的),
#   本文件只负责写 <include> —— 模型不由本生成器重建。
#   开关必须放这里: 2026-09-17 那版是直接手改 world 文件的, 一跑生成器就没了。
WITH_VEHICLE = True
VEHICLE_POSE = (0.35, 0.35, 0.005, 0.0)      # x y z yaw = 红方 TR 启动格中心 (start_zones.ours.tr)


def gen_world():
    include_rig = ''
    if WITH_SENSOR_RIG:
        include_rig = ('    <include>\n      <uri>model://sensor_rig</uri>\n'
                       '      <pose>%s %s %s 0 0 %s</pose>\n    </include>\n' % SENSOR_RIG_POSE)
    include_veh = ''
    if WITH_VEHICLE:
        include_veh = ('    <!-- 仿真载具 (顶替未完工的实车); 出生位姿 = 红区 TR 启动位中心 -->\n'
                       '    <include>\n      <uri>model://fishbot</uri>\n'
                       '      <pose>%s %s %s 0 0 %s</pose>\n    </include>\n' % VEHICLE_POSE)
    txt = '''<?xml version="1.0"?>
<sdf version="1.6">
  <world name="br_field">
    <physics type="ode"><max_step_size>0.005</max_step_size></physics>
    <light type="directional" name="sun"><pose>0 0 6 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.2 -1</direction></light>
    <model name="ground">
      <static>true</static>
      <link name="ground_link"><visual name="v"><geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry>
      <material><ambient>0.4 0.4 0.4 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material></visual>
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry></collision></link>
    </model>
    <include><uri>model://br_field</uri></include>
''' + include_rig + include_veh + '''  </world>
</sdf>
'''
    with open(os.path.join(WORLD_DIR, 'br_field.world'), 'w', encoding='utf-8') as fd:
        fd.write(txt)


def main():
    n, _ = gen_field()
    gen_world()
    print('br_field/model.sdf 已重新生成 (%d 行)' % n)
    print('  ->', os.path.join(FIELD_DIR, 'model.sdf'))
    print('  ->', os.path.join(WORLD_DIR, 'br_field.world'))
    print('  坡道楔形网格:', MESH_DIR)
    print()
    print('数据源: config/field_geometry.yaml')
    print('  L1 %.2f  L2 %.2f  穆斯蒂卡柱 %s h=%.2f  核心柱 %s h=%.2f' % (
        P['l1_platform']['height'], P['l2_platform']['height'],
        P['mustika_pillar']['position'], P['mustika_pillar']['height'],
        P['core_pillar']['position'], P['core_pillar']['height']))
    print('  建筑位 %d 个 (含 L2 的 4 个, per-spot z)  天空方块 %d 块' % (
        len(P['building_spot_x']), len(P['sky_block_initial']['positions']) // 3))


if __name__ == '__main__':
    main()
