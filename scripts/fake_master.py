#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fake_master.py — 伪下位机 (STM32 主控模拟器), 任务书 §5.1/§5.2 的协议对端

为什么要有它:
    通信层的另一端是 STM32 主控, 而**车还没出**。没有它, protocol_encoder 写完了也只能
    靠单测证明"自己和自己一致", 没法回答真正的问题: 发出去的字节流, 对面读得出吗?
    本脚本站在主控的位置, 收感知帧、校验 CRC、把 payload 解成可读的表格,
    并能按指令回发 BR_RESET —— 于是整条通信链在没有硬件的情况下就能跑通。

⚠ **刻意用独立的 Python 重写**, 不调用 C++ 那套:
    收发两端共享同一份实现的话, 两边一起错会互相抵消, 联调时才发现 ——
    那不是测试, 那是照镜子。

⚠ **默认行为: 只收、只打印, 不回发任何字节**。所有回发必须由
   --reset-after / --heartbeat-every 显式开启 —— 不会悄悄往总线上塞东西。

═══════════════════════════════════════════════════════════════════════════
用法
═══════════════════════════════════════════════════════════════════════════
0) 先自检 (不需要任何设备):
       python3 scripts/fake_master.py --selftest
   四条 CRC 标准验证向量 + 编解码往返 + **独立 golden 帧**(验证字段布局与 C++ 一致)。
   CRC 或布局不对的话整个工具都在骗人, 所以这一步不是可选的。

1) 用 socat 造一对虚拟串口 (两条命令各开一个终端):
       socat -d -d pty,raw,echo=0,link=/tmp/ttyROS   pty,raw,echo=0,link=/tmp/ttyFAKE
   然后:
       终端 A: ros2 run br_perception comm_node --ros-args -p serial_port:=/tmp/ttyROS
       终端 B: python3 scripts/fake_master.py /tmp/ttyFAKE

2) 没有 socat 也可以走管道:
       python3 scripts/fake_master.py --stdio < 来自感知的字节流 > 回给主控的字节流

常用开关:
       --reset-after N      收到第 N 个感知帧后回发一次 BR_RESET (测复位链路)
       --reset-zone {0,1}   BR_RESET 的 zone
       --heartbeat-every N  每收 N 帧回一个心跳
       --max-frames N       收满 N 帧后正常退出 (脚本化测试用)
       --quiet              只打统计, 不打逐帧明细
"""

import argparse
import os
import struct
import sys
import time

try:
    import select as _select          # POSIX; 用来做"卡住"检测
except ImportError:                   # pragma: no cover
    _select = None


# ═══════════════════════════════════════════════════════════════════════════
# 协议常量 —— 必须与 src/communication/protocol_encoder.hpp 逐项一致
# ═══════════════════════════════════════════════════════════════════════════

HEADER = b'\xAA\x55'
OVERHEAD = 6            # 帧头(2) + 序号(1) + 类型(1) + 长度(2)
CRC_BYTES = 2

TYPE_PERCEPTION = 0x01
TYPE_HEARTBEAT = 0x02
# 紧急停止 (原 0x03 ERROR, 2026-09-22 收窄语义并实现)。
# ⚠️ 台架联调**必须认得它** —— 否则"紧急停止到底有没有上线"这件事
#    在伪下位机这一侧根本验证不了, 而那正是这个包存在的唯一理由。
#    payload = uint32 timestamp_ms(小端) + uint8 estop(0=解除 1=要求停车)
TYPE_ESTOP = 0x03
TYPE_BR_RESET = 0x10

ESTOP_CLEAR = 0
ESTOP_ACTIVE = 1


def decode_estop(payload):
    """解 0x03 payload → dict; 长度不符返回 None (**拒绝而不是猜**)。"""
    if len(payload) != 5:
        return None
    ts = int.from_bytes(payload[0:4], 'little')
    return {'timestamp_ms': ts, 'estop': payload[4]}

# BR_RESET 的 zone (任务书 §5.1 v1.6)。扩展新 zone 时改这里 +
# ProtocolDecoder::kMaxKnownResetZone + 协议表 —— 三处。
RESET_ZONE_GROUND = 0
RESET_ZONE_L1_RETRY = 1
KNOWN_RESET_ZONES = (RESET_ZONE_GROUND, RESET_ZONE_L1_RETRY)

# 默认 CRC 参数: CRC-16/CCITT-FALSE。协议冻结后若改, 这里与 C++ 的
# ProtocolConfig 必须同步改 —— 两边不一致的症状是"帧全被丢弃, 且不报错"。
CRC_POLY = 0x1021
CRC_INIT = 0xFFFF
CRC_REFIN = False
CRC_REFOUT = False
CRC_XOROUT = 0x0000

# 四条标准验证向量 (输入 ASCII "123456789"), 见 ProtocolConfig 注释
CHECK_INPUT = b'123456789'
CHECK_VECTORS = [
    # (变体名, poly, init, refin, refout, xorout, 期望)
    ('CCITT-FALSE', 0x1021, 0xFFFF, False, False, 0x0000, 0x29B1),
    ('XMODEM     ', 0x1021, 0x0000, False, False, 0x0000, 0x31C3),
    ('MODBUS     ', 0x8005, 0xFFFF, True,  True,  0x0000, 0x4B37),
    ('KERMIT     ', 0x1021, 0x0000, True,  True,  0x0000, 0x2189),
]

# 独立 golden 帧 —— 由 C++ 侧 test_protocol_encoder.cpp 顶部注释里的脚本生成,
# **不是**本文件 build_frame 的输出。它同时钉死了 CRC 与**字段布局**:
# 若 HDR_FMT / MUSTIKA_FMT 等的列顺序与 protocol_encoder.cpp 的 put_* 顺序不一致,
# 本文件内部自洽却与 C++ 不通 —— 只有这条能发现。
# 再生成方式见 test_protocol_encoder.cpp (跑那段 Python 脚本, 贴回这里与测试里)。
# ⚠️ 2026-09-22: 与 test/test_protocol_encoder.cpp 的 kGoldenFrameHex **必须逐字符一致**。
#    本次改动: payload 第 19 字节 (索引 18) 由 `01`(odometry_reliable) 改为 `0F`
#    (状态标志字节, 四位全置) —— 帧长仍是 28, 只有 CRC 跟着变 (5599 → A1A0)。
#    ⚠️ **改了协议布局, 这份常量与 C++ 那份要一起改** —— 两边不同步的话,
#       联调时表现是"伪下位机自检通过、但对端发来的帧全解不动", 很难往这上面想。
GOLDEN_FRAME_HEX = ('AA5500011C003930000096001FFF000084036401000001000F2602CF033601000000'
                    'A1A0')

# 单帧 payload 上界 —— **必须与 ProtocolConfig::max_payload_bytes 一致**。
# 由字段构成算出, 不写死 272: 26 (头16+穆8+柱2) + (1 + 4×16 建筑位) + (1 + 18×10 障碍物)
MAX_BUILDING_SPOTS = 16
MAX_OBSTACLES = 10
MAX_PAYLOAD = 26 + (1 + 4 * MAX_BUILDING_SPOTS) + (1 + 18 * MAX_OBSTACLES)   # = 272

# 缓冲里有数据但 N 秒没有新进展 → 提示 (纯调试特性, 不是协议功能)
STUCK_WARN_SEC = 5.0

MUSTIKA_STATE = {
    0: '未知', 1: '柱上', 2: 'TR转运中', 3: 'BR持有',
    4: '已供奉', 5: '掉落', 6: '重试归还中',
}
SPOT_STATE = {0: '空', 1: '1地球', 2: '2地球', 3: '完整塔'}
SPOT_COLOR = {0: '无', 1: '红', 2: '蓝', 3: '未知'}
SPOT_OWNER = {0: '无人', 1: '我方', 2: '对方'}
OBSTACLE_TYPE = {0: '未知', 1: '敌方机器人', 2: '掉落方块', 3: '其他'}


def log(msg):
    """统一写 stderr —— stdout 永远留给协议字节 (--stdio 模式下尤其重要)"""
    print(msg, file=sys.stderr)


# ═══════════════════════════════════════════════════════════════════════════
# CRC16 —— 位逐位实现 (与 protocol_encoder.cpp 的查表版本是两套代码)
# ═══════════════════════════════════════════════════════════════════════════

def _reflect(value, bits):
    out = 0
    for i in range(bits):
        out = (out << 1) | ((value >> i) & 1)
    return out


def crc16(data, poly=CRC_POLY, init=CRC_INIT,
          refin=CRC_REFIN, refout=CRC_REFOUT, xorout=CRC_XOROUT):
    """Rocksoft/Williams 模型的位逐位 CRC16。

    ⚠ 输入反射施加在**数据字节**上 (不是建表阶段) —— 这一点写错过一次:
      反射挪到建表会让 MODBUS/KERMIT 整条算错, 而 CCITT-FALSE/XMODEM 照样对,
      只测非反射向量发现不了。
    """
    crc = init
    for byte in data:
        if refin:
            byte = _reflect(byte, 8)
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    if refout:
        crc = _reflect(crc, 16)
    return (crc ^ xorout) & 0xFFFF


# ═══════════════════════════════════════════════════════════════════════════
# 帧序号
# ═══════════════════════════════════════════════════════════════════════════

class SeqCounter:
    """自增帧序号。"取-用-显示"用**同一个值**, 这类错位从结构上不可能再犯。

    (曾经在心跳分支写成先自增再 `seq - 1` 显示, 回绕时打出 seq=-1。)

    ⚠ start 默认 0 = "主控刚上电"。这对**首次**联调是对的, 但重测时会坑:
      C++ 侧丢弃"序号不新于上次"的帧, 每次重启本脚本都从 0 开始, 于是第二次
      以后发的帧全被判重复、**静默丢弃** (bytes 在涨, frames_received 不动)。
      重测要么重启 comm_node, 要么把 --seq-start 调到上一次之后。
    """

    def __init__(self, start=0):
        self._n = start & 0xFF

    def take(self):
        """取下一个序号并自增 —— 返回**取到的那个值**, 不是自增后的"""
        v = self._n
        self._n = (self._n + 1) & 0xFF
        return v


# ═══════════════════════════════════════════════════════════════════════════
# 封帧 / 解析
# ═══════════════════════════════════════════════════════════════════════════

def build_frame(seq, ptype, payload):
    body = HEADER + bytes([seq & 0xFF, ptype]) + struct.pack('<H', len(payload)) + payload
    return body + struct.pack('<H', crc16(body))


def parse_stream(buf):
    """从字节流里解出所有完整帧, 返回 (帧列表, 剩余缓冲, 垃圾字节数, CRC 错误数)。

    与 protocol_decoder.cpp 同样的策略: 找不到帧头时保留末尾孤立的 0xAA,
    CRC 失败只跳 1 字节 (无法证明帧头是真的)。

    ⚠ 每轮对入参做一次 O(n) 拷贝 (要 del 前缀, 不能改调用方的 bytes)。
      对调试场景足够; 若将来发现跟不上, 改成持有 bytearray 状态的对象式 parser。
    """
    frames, garbage, crc_errors = [], 0, 0
    buf = bytearray(buf)

    while True:
        idx = buf.find(HEADER)
        if idx < 0:
            keep = 1 if buf.endswith(b'\xAA') else 0
            garbage += len(buf) - keep
            buf = buf[len(buf) - keep:]
            break
        if idx > 0:
            garbage += idx
            del buf[:idx]

        if len(buf) < OVERHEAD:
            break
        plen = buf[4] | (buf[5] << 8)

        # ⚠ 长度闸门: 超过上限即判为垃圾里的假帧头, 跳 1 字节重新同步。
        #   没有这一道, 一个坏长度字段 (如 0xFFFF) 会让缓冲区**无限增长** ——
        #   主循环一直在 `len(buf) < total` 处等, 永远等不到也永远不报错。
        #   (C++ 侧的 protocol_decoder 有同样的闸门, 这里必须对齐)
        if plen > MAX_PAYLOAD:
            garbage += 1
            del buf[:1]
            continue

        total = OVERHEAD + plen + CRC_BYTES
        if len(buf) < total:
            break

        body = bytes(buf[:total - CRC_BYTES])
        got = buf[total - 2] | (buf[total - 1] << 8)
        if crc16(body) != got:
            crc_errors += 1
            garbage += 1
            del buf[:1]
            continue

        frames.append({'seq': buf[2], 'type': buf[3], 'payload': bytes(buf[6:6 + plen])})
        del buf[:total]

    return frames, bytes(buf), garbage, crc_errors


# ═══════════════════════════════════════════════════════════════════════════
# 感知帧 payload 解码 —— 逐字段对照 protocol_encoder.cpp
# ═══════════════════════════════════════════════════════════════════════════

HDR_FMT = '<IhhhhBBBB'        # 16B: ts_ms, x/y/z_cm, yaw_deg10, 比赛态×4
# ⚠️ 第 3 个 B 原叫 `odometry_reliable`(0/1), 2026-09-22 起是**状态标志字节**:
#    bit0=里程计可靠 bit1=位置有效 bit2=圣所字段有效 bit3=持握数字段有效
#    (见 protocol_encoder.hpp 的 StatusFlag)。**别再按 0/1 读它。**
MUSTIKA_FMT = '<BBBhhb'       #  8B: status, sanctuary, flags, x/y/z_cm
PILLAR_FMT = '<BB'            #  2B: 穆斯蒂卡柱/核心柱 occupied
SPOT_FMT = '<BBBB'            #  4B: spot_id, state, top_color, ownership
OBS_FMT = '<BhhbBBBBBHBBbbB'  # 18B: id,x,y,z,w,d,h,type,ally,dwell,push,held,vx,vy,threat

FIXED_PREFIX = 16 + 8 + 2     # 头部 + 穆斯蒂卡 + 柱子


def decode_perception(payload):
    """把 0x01 感知帧的 payload 解成 dict。

    ⚠ 长度不对或**有字节没被消费** → 返回 None (不猜)。
      后半条是关键: 若字段布局与编码器不一致, 解出来的值会"看起来合理但全错位",
      明确失败比静默错值强得多 —— 这是个调试工具。
    """
    if len(payload) < FIXED_PREFIX + 2:
        return None
    off = 0

    ts, px, py, pz, yaw10, remain, held, over, endwarn = \
        struct.unpack_from(HDR_FMT, payload, off)
    off += struct.calcsize(HDR_FMT)

    m_status, m_sanct, m_odom, mx, my, mz = struct.unpack_from(MUSTIKA_FMT, payload, off)
    off += struct.calcsize(MUSTIKA_FMT)

    pillar_m, pillar_c = struct.unpack_from(PILLAR_FMT, payload, off)
    off += struct.calcsize(PILLAR_FMT)

    spot_count = payload[off]
    off += 1
    spots = []
    for _ in range(spot_count):
        if off + 4 > len(payload):
            return None
        sid, state, color, owner = struct.unpack_from(SPOT_FMT, payload, off)
        off += 4
        spots.append({'id': sid, 'state': state, 'color': color, 'owner': owner})

    if off >= len(payload):
        return None
    obs_count = payload[off]
    off += 1
    obstacles = []
    for _ in range(obs_count):
        if off + struct.calcsize(OBS_FMT) > len(payload):
            return None
        (oid, ox, oy, oz, ow, od, oh, otype,
         ally_tr, dwell, pushing, held_obs, vx, vy, threat) = \
            struct.unpack_from(OBS_FMT, payload, off)
        off += struct.calcsize(OBS_FMT)
        obstacles.append({'id': oid, 'x': ox, 'y': oy, 'z': oz, 'type': otype,
                          'size': (ow, od, oh), 'ally_tr': ally_tr, 'dwell_ms': dwell,
                          'pushing': pushing, 'held': held_obs, 'v': (vx, vy),
                          'threat': threat})

    # ── 总长回算: 有字节没被消费 = 布局与本解码器不一致, 拒绝而不是忽略 ──
    if off != len(payload):
        return None

    return {
        'timestamp_ms': ts,
        'ego': (px, py, pz, yaw10 / 10.0),
        'match': {'remaining_sec': remain, 'held': held,
                  'over_limit': bool(over), 'end_release_warn': bool(endwarn)},
        'mustika': {'state': m_status, 'sanctuary': bool(m_sanct),
                    'odom_reliable': bool(m_odom), 'pos_cm': (mx, my, mz)},
        'pillars': {'mustika_occupied': bool(pillar_m), 'core_occupied': bool(pillar_c)},
        'spots': spots,
        'obstacles': obstacles,
    }


def format_perception(d):
    """人类可读的一帧摘要"""
    lines = []
    ex, ey, ez, eyaw = d['ego']
    lines.append("  t={}ms  剩余={}s  自身=({},{},{})cm yaw={:.1f}°".format(
        d['timestamp_ms'], d['match']['remaining_sec'], ex, ey, ez, eyaw))
    m = d['mustika']
    lines.append("  穆斯蒂卡: {}  ({}cm)  圣所={} 里程计={}".format(
        MUSTIKA_STATE.get(m['state'], '?'), m['pos_cm'],
        '达成' if m['sanctuary'] else '未达成',
        '可靠' if m['odom_reliable'] else '降权'))
    lines.append("  柱: 穆斯蒂卡柱={} 核心柱={}  持握={} 超限={} 终场告警={}".format(
        '有物' if d['pillars']['mustika_occupied'] else '空',
        '有物' if d['pillars']['core_occupied'] else '空',
        d['match']['held'],
        '是' if d['match']['over_limit'] else '否',
        '是' if d['match']['end_release_warn'] else '否'))
    if d['spots']:
        for s in d['spots']:
            lines.append("  建筑位#{}: {} 顶色={} 归属={}".format(
                s['id'], SPOT_STATE.get(s['state'], '?'),
                SPOT_COLOR.get(s['color'], '?'), SPOT_OWNER.get(s['owner'], '?')))
    else:
        lines.append("  建筑位: (无)")
    if d['obstacles']:
        for o in d['obstacles']:
            lines.append("  障碍#{}: {} @({},{},{})cm 尺寸={}cm 速度={} 威胁={}".format(
                o['id'], OBSTACLE_TYPE.get(o['type'], '?'), o['x'], o['y'], o['z'],
                o['size'], o['v'], o['threat']))
    else:
        lines.append("  障碍物: (无)")
    return '\n'.join(lines)


# ═══════════════════════════════════════════════════════════════════════════
# 自检
# ═══════════════════════════════════════════════════════════════════════════

def selftest():
    ok = True

    def check(name, good):
        nonlocal ok
        ok &= bool(good)
        log('  {}  {}'.format('OK  ' if good else '失败', name))

    log('CRC16 标准验证向量 (输入 "123456789"):')
    for name, poly, init, refin, refout, xorout, want in CHECK_VECTORS:
        got = crc16(CHECK_INPUT, poly, init, refin, refout, xorout)
        check('{}  期望 {:04X} 实得 {:04X}'.format(name, want, got), got == want)

    log('\n独立 golden 帧 (外部来源, 验证字段布局与 C++ 一致):')
    golden = bytes.fromhex(GOLDEN_FRAME_HEX)
    gframes, grest, ggarbage, gcrc = parse_stream(golden)
    check('解出 1 帧', len(gframes) == 1)
    check('无残余/垃圾/CRC 错', grest == b'' and ggarbage == 0 and gcrc == 0)
    if gframes:
        gd = decode_perception(gframes[0]['payload'])
        check('seq == 0', gframes[0]['seq'] == 0)
        check('type == 0x01', gframes[0]['type'] == TYPE_PERCEPTION)
        check('payload 28 字节', len(gframes[0]['payload']) == 28)
        check('字段可解且长度吻合', gd is not None)
        if gd:
            check('timestamp == 12345', gd['timestamp_ms'] == 12345)
            check('自身位姿 == (150,-225,0,90.0)', gd['ego'] == (150, -225, 0, 90.0))
            check('穆斯蒂卡状态 == 柱上', gd['mustika']['state'] == 1)
            check('穆斯蒂卡位置 == (550,975,54)', gd['mustika']['pos_cm'] == (550, 975, 54))
            check('建筑位/障碍物均为空', gd['spots'] == [] and gd['obstacles'] == [])

    log('\n自发自收 (验证本文件内部一致):')
    payload = struct.pack(HDR_FMT, 12345, 150, -225, 0, 900, 100, 1, 0, 0)
    payload += struct.pack(MUSTIKA_FMT, 1, 0, 1, 550, 975, 54)
    payload += struct.pack(PILLAR_FMT, 1, 0)
    payload += bytes([2]) + struct.pack(SPOT_FMT, 0, 3, 1, 1) \
                          + struct.pack(SPOT_FMT, 1, 0, 0, 0)
    payload += bytes([1]) + struct.pack(OBS_FMT, 7, 100, 200, 0, 35, 35, 35,
                                        2, 0, 0, 0, 0, 5, -10, 0)
    frames, rest, garbage, crc_errors = parse_stream(build_frame(0, TYPE_PERCEPTION, payload))
    check('解出 1 帧', len(frames) == 1 and rest == b'' and garbage == 0 and crc_errors == 0)
    if frames:
        d = decode_perception(frames[0]['payload'])
        check('建筑位数 == 2', d is not None and len(d['spots']) == 2)
        check('障碍数 == 1 且 id == 7', d is not None and d['obstacles'][0]['id'] == 7)
        check('障碍速度 == (5,-10)', d is not None and d['obstacles'][0]['v'] == (5, -10))

    log('\n紧急停止 (0x03) 往返 —— ⚠️ 这条是"停车指令到底能不能上线"的台架判据:')
    # 位逐位构造, 与 C++ 的 encode_estop 无关 (不能拿被测实现证明被测实现)
    ts_ms = 12345
    estop_payload_on = struct.pack('<IB', ts_ms, ESTOP_ACTIVE)
    ef, erest, eg, ecrc = parse_stream(build_frame(0, TYPE_ESTOP, estop_payload_on))
    check('触发帧: 解出 1 帧且无残余/垃圾/CRC 错',
          len(ef) == 1 and erest == b'' and eg == 0 and ecrc == 0)
    if ef:
        check('type == 0x03', ef[0]['type'] == TYPE_ESTOP)
        check('payload 5 字节', len(ef[0]['payload']) == 5)
        d = decode_estop(ef[0]['payload'])
        check('可解', d is not None)
        if d:
            check('timestamp_ms == {}'.format(ts_ms), d['timestamp_ms'] == ts_ms)
            check('estop == 1 (要求停车)', d['estop'] == ESTOP_ACTIVE)

    estop_payload_off = struct.pack('<IB', ts_ms, ESTOP_CLEAR)
    ef2, _, _, _ = parse_stream(build_frame(1, TYPE_ESTOP, estop_payload_off))
    check('解除帧: 解出 1 帧', len(ef2) == 1)
    if ef2:
        d2 = decode_estop(ef2[0]['payload'])
        # ⚠️ **解除方向必须也能解** —— 只发停车不发解除, 主控会永远停在那个状态
        check('estop == 0 (解除)', d2 is not None and d2['estop'] == ESTOP_CLEAR)

    check('长度不符时拒绝而不是猜', decode_estop(b'\x00\x01\x02') is None)

    log('\n流式边界:')
    junk = b'\x00\xFF' + build_frame(0, TYPE_PERCEPTION, payload)[:5]
    f2, _, g2, _ = parse_stream(junk)
    check('半截帧不解出, 垃圾计数 >= 2', len(f2) == 0 and g2 >= 2)

    big = build_frame(1, TYPE_PERCEPTION, bytes([0xA5]) * (MAX_PAYLOAD + 1))
    f3, _, g3, _ = parse_stream(big)
    check('超上限被判假帧头', len(f3) == 0 and g3 >= 1)

    exact = build_frame(1, TYPE_PERCEPTION, bytes([0xA5]) * MAX_PAYLOAD)
    f4, _, g4, _ = parse_stream(exact)
    check('恰好上限被接受', len(f4) == 1 and g4 == 0)

    log('\n结果: ' + ('全部通过' if ok else '**有失败项, 先修 CRC/布局再联调**'))
    return 0 if ok else 1


def check_config(path):
    """读 comm_params.yaml 里的 crc16_* 参数, 对照四条标准变体验证。

    ⚠ 与 --selftest 是**两件事**, 别混:
        --selftest     验证**本脚本**的 CRC 实现对不对 (与你的配置无关)
        --check-config 验证**你 yaml 里配的** CRC 参数对不对
    改了 yaml 的 CRC 参数之后, 要跑的是后者。
    """
    try:
        import yaml
    except ImportError:
        log('需要 PyYAML: sudo apt install -y python3-yaml')
        return 3

    try:
        with open(path, encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
    except OSError as e:
        log('读不到 {}: {}'.format(path, e))
        return 3

    params = ((cfg or {}).get('comm_node') or {}).get('ros__parameters')
    if not params:
        log('{} 里找不到 comm_node.ros__parameters —— 是不是传错文件了?'.format(path))
        return 3

    poly = int(params.get('crc16_polynomial', CRC_POLY))
    init = int(params.get('crc16_initial', CRC_INIT))
    refin = bool(params.get('crc16_reflect_in', CRC_REFIN))
    refout = bool(params.get('crc16_reflect_out', CRC_REFOUT))
    xorout = int(params.get('crc16_xor_out', CRC_XOROUT))

    log('{} 里的 CRC16 参数:'.format(path))
    log('  poly=0x{:04X}  init=0x{:04X}  refin={}  refout={}  xorout=0x{:04X}'.format(
        poly, init, refin, refout, xorout))

    for name, ep, ei, eri, ero, ex, want in CHECK_VECTORS:
        if (poly, init, refin, refout, xorout) == (ep, ei, eri, ero, ex):
            got = crc16(CHECK_INPUT, poly, init, refin, refout, xorout)
            good = (got == want)
            log('匹配标准变体 {} —— check 值应为 0x{:04X}'.format(name, want))
            log('  实算 0x{:04X}  {}'.format(got, 'OK' if good else '**失败**'))
            return 0 if good else 1

    log('⚠ 这组参数**不对应任何标准变体**。若下位机确实用的自定义算法, 请人工确认;')
    log('  否则多半是抄错了 —— 对照下面四条:')
    for name, ep, ei, eri, ero, ex, want in CHECK_VECTORS:
        log('    {} poly=0x{:04X} init=0x{:04X} refin={} refout={} xorout=0x{:04X}'.format(
            name, ep, ei, eri, ero, ex))
    log('⚠ 另外别忘了: **两端都要用同一组参数** —— 改了 yaml 而 encoder 仍用默认,')
    log('   症状是"串口正常打开却一帧都收不到"。')
    return 2


# ═══════════════════════════════════════════════════════════════════════════
# 主循环
# ═══════════════════════════════════════════════════════════════════════════

def open_channel(args):
    """返回 (infile, outfile)。--stdio 下 stdout 必须**关缓冲**, 否则管道场景会卡死"""
    if args.stdio:
        # ⚠ 关键: sys.stdout.buffer 连到 pipe/文件时是**块缓冲**(8KiB), write() 的
        #   字节会留在缓冲区里不出去, 对面永远等不到 —— 看起来像协议不通。
        outfile = os.fdopen(sys.stdout.fileno(), 'wb', buffering=0)
        return sys.stdin.buffer, outfile

    try:
        fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY)
    except OSError as e:
        log('打不开 {}: {}'.format(args.port, e))
        log('提示: 用 socat 造一对虚拟串口 —— 见本文件头部说明')
        return None, None

    if args.baud:
        try:
            import termios
            attrs = termios.tcgetattr(fd)
            baud = getattr(termios, 'B{}'.format(args.baud), None)
            if baud is None:
                log('不支持的波特率 {}, 忽略'.format(args.baud))
            else:
                attrs[4] = attrs[5] = baud
                termios.tcsetattr(fd, termios.TCSANOW, attrs)
        except Exception as e:        # noqa: BLE001 — PTY 上多半不需要, 不该因此退出
            log('设置波特率失败 (忽略): {}'.format(e))

    return os.fdopen(os.dup(fd), 'rb', buffering=0), os.fdopen(fd, 'wb', buffering=0)


def main():
    ap = argparse.ArgumentParser(description='伪下位机 (协议对端模拟器)')
    ap.add_argument('port', nargs='?', help='串口设备, 例如 /tmp/ttyFAKE (socat 造出来的)')
    ap.add_argument('--stdio', action='store_true', help='从 stdin 读、往 stdout 写 (管道模式)')
    ap.add_argument('--baud', type=int, default=0, help='设置波特率 (仅真实串口需要; PTY 可省)')
    ap.add_argument('--selftest', action='store_true',
                    help='只跑自检 (验证本脚本的 CRC/布局实现) 然后退出')
    ap.add_argument('--check-config', metavar='YAML',
                    help='读 comm_params.yaml 的 crc16_* 参数并对照标准变体验证 (改了协议参数后跑这个)')
    ap.add_argument('--reset-after', type=int, default=0, metavar='N',
                    help='收到第 N 个感知帧后回发一次 BR_RESET')
    ap.add_argument('--seq-start', type=int, default=0, metavar='N',
                    help='本进程发出的第一帧的 seq (默认 0 = 主控刚上电)。'
                         'C++ 侧会丢弃 seq 不新于上次的帧, 所以重启本脚本重测时, '
                         '若仍从 0 开始, 复位帧会被当重复帧静默丢掉 —— '
                         '要么重启 comm_node, 要么用这个参数跳到上一次之后')
    ap.add_argument('--reset-zone', type=int, default=RESET_ZONE_GROUND,
                    choices=KNOWN_RESET_ZONES,
                    help='BR_RESET 的 zone: 0=地面启动区 1=L1 重试区')
    ap.add_argument('--heartbeat-every', type=int, default=0, metavar='N',
                    help='每收 N 帧回一个心跳')
    ap.add_argument('--max-frames', type=int, default=0, metavar='N',
                    help='收满 N 帧后正常退出 (0 = 不收工)')
    ap.add_argument('--quiet', action='store_true', help='只打统计, 不打逐帧明细')
    args = ap.parse_args()

    if args.selftest:
        return selftest()
    if args.check_config:
        return check_config(args.check_config)

    if not args.port and not args.stdio:
        ap.error('要么给一个串口设备, 要么用 --stdio; 只想自检就加 --selftest')

    infile, outfile = open_channel(args)
    if infile is None:
        return 2

    log('伪下位机已启动 ({}), Ctrl-C 退出'.format(args.port or '--stdio'))
    log('默认只收不发; 回发需 --reset-after / --heartbeat-every 显式开启')

    counter = SeqCounter(args.seq_start)
    buf = b''
    n_frames = n_crc_err = n_garbage = n_reset_sent = n_hb_sent = 0
    n_estop = n_estop_active = 0
    reset_sent = False

    def send(ptype, payload, note):
        sent = counter.take()          # 取-用-显示同一个值
        outfile.write(build_frame(sent, ptype, payload))
        log('  [发→] {} seq={}'.format(note, sent))

    running = True
    last_progress = time.monotonic()
    # ⚠ select 并非到处可用: Windows 上它**只支持 socket**, 对管道/文件会抛
    #   OSError(WinError 10093)。Linux (本工具的目标平台) 上管道与 pty 都支持。
    #   这里做成"不可用就永久退回阻塞读取", 而不是让它把整个工具带崩 ——
    #   卡住提示是锦上添花, 不该成为能不能用的前提。
    use_select = _select is not None
    try:
        while running:
            if use_select:
                try:
                    ready, _, _ = _select.select([infile], [], [], 1.0)
                except (OSError, ValueError):
                    use_select = False
                    log('提示: 本平台不支持对该通道用 select —— '
                        '退回阻塞读取, "卡住"提示关闭 (Linux 上可正常使用)')
                    ready = True        # 落到下面的阻塞 read

                if not ready:
                    # 超时: 缓冲区里有货却迟迟没进展 → 提示
                    # (只在调试工具里做, 协议层不管这件事)
                    if buf and time.monotonic() - last_progress > STUCK_WARN_SEC:
                        log('  ⚠ 缓冲区里积着 {} 字节, 已 {}s 无新数据 —— '
                            '对面卡住或链路断了?'.format(len(buf), STUCK_WARN_SEC))
                        last_progress = time.monotonic()   # 免得刷屏
                    continue

            chunk = infile.read(4096)
            if not chunk:
                break                                   # EOF
            buf += chunk
            frames, buf, garbage, crc_errors = parse_stream(buf)
            n_garbage += garbage
            n_crc_err += crc_errors
            if frames:
                last_progress = time.monotonic()

            for fr in frames:
                n_frames += 1
                kind = {TYPE_PERCEPTION: '感知帧', TYPE_HEARTBEAT: '心跳',
                        TYPE_ESTOP: '**紧急停止**'}.get(
                    fr['type'], '未知类型 0x{:02X}'.format(fr['type']))
                log('\n[收←] #{:<4} seq={:<3} 类型={} payload={}字节'.format(
                    n_frames, fr['seq'], kind, len(fr['payload'])))

                if fr['type'] == TYPE_PERCEPTION and not args.quiet:
                    d = decode_perception(fr['payload'])
                    log(format_perception(d) if d else
                        '  payload 解不动 (长度或布局不符) —— 拒绝而不是猜')

                if fr['type'] == TYPE_ESTOP:
                    d = decode_estop(fr['payload'])
                    if d is None:
                        log('  payload 解不动 (长度应为 5) —— 拒绝而不是猜')
                    else:
                        n_estop += 1
                        if d['estop'] == ESTOP_ACTIVE:
                            n_estop_active += 1
                        log('  timestamp_ms={} estop={} ({})'.format(
                            d['timestamp_ms'], d['estop'],
                            '要求停车' if d['estop'] == ESTOP_ACTIVE else '解除'))

                if args.heartbeat_every and n_frames % args.heartbeat_every == 0:
                    send(TYPE_HEARTBEAT, b'', '心跳')
                    n_hb_sent += 1

                if args.reset_after and not reset_sent and n_frames >= args.reset_after:
                    send(TYPE_BR_RESET, bytes([args.reset_zone, 0x00, 0x00]),
                         'BR_RESET zone={}'.format(args.reset_zone))
                    n_reset_sent += 1
                    reset_sent = True

                if args.max_frames and n_frames >= args.max_frames:
                    running = False
                    break
    except KeyboardInterrupt:
        log('\n用户中断')               # 只留给真的 Ctrl-C, 不再兼职"正常收工"

    log('\n──── 统计 ────')
    log('收到帧        : {}'.format(n_frames))
    log('CRC 错误      : {}'.format(n_crc_err))
    log('同步丢弃字节  : {}'.format(n_garbage))
    log('发出 心跳/复位: {} / {}'.format(n_hb_sent, n_reset_sent))
    # ⚠️ 单列紧急停止 —— 这一行就是"停车指令到底上线没有"的**台架判据**。
    #    只在真有 0x03 时才多打一行, 免得正常联调时噪音。
    if n_estop:
        log('收到 紧急停止  : {} 帧 (其中"要求停车" {} 帧)'.format(n_estop, n_estop_active))
    log('未解析残余    : {} 字节'.format(len(buf)))
    if n_crc_err:
        log('⚠ 有 CRC 错误 —— 先核对两端的 CRC 参数 (多项式/初值/反射/异或出口)')
    if buf:
        log('⚠ 退出时缓冲里还有数据 —— 帧没收全就断链了?')
    return 0


if __name__ == '__main__':
    sys.exit(main())
