#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""115 安全闭环（提速版）：backend仅初始化一次，闭环内多轮复用。

默认只读取基线，不发送动作；--execute-align 执行生产XY闭环；
--execute-test 先注入指定XY偏差，再闭环返回参考位置。
坏帧重采、不放宽1px/2px阈值、GDK与Minth串行；失败时撤销累计OFFSET。
2026-10-20：--one-step 时改为 SL1 开环一步到位（一次纠偏+一次验证，动作后无条件判 aligned），
默认仍为多轮闭环；仅 SL1 路径（vision_plc_helper_115._run_pick_alignment）传入该开关。
"""
import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

PROGRAMS_ROOT = Path('/data/wxf0721/runtime/programs')
PKG_ROOT = Path('/home/agi/G2_AprilTag_Placement_0.2.0')
RUNTIME_ROOT = Path('/data/wxf0721/runtime')
sys.path.insert(0, str(PROGRAMS_ROOT))
sys.path.insert(0, str(PKG_ROOT))
sys.path.insert(0, str(RUNTIME_ROOT))

from vision_115 import apply_115_runtime_profile

# 2026-09-20 提速：5→3帧。测量耗时约省40%。Z噪声会回升(之前实测5帧±2~7、
# 3帧预计±3~10mm)，靠 kept_best(≤10mm)兜底停最优点；若实测产品变歪请回退5帧。
# 2026-10-21 提速(方案2)：3→2帧。历史 2026-09-17 实测 4.4→3.5s 精度不变；
# 若Z残差变大（WQ已zero-z不纠Z，受影响小），回退 3 即可。
VALID_FRAMES = 2
MAX_ATTEMPTS = 20
GAIN = 1.0  # [qjxopt]
TOL_MM = 4.0  # [qjxopt]
# 2026-09-20 提速：XY达标即视为对准，不再追单目Z噪声。Z是视觉深度测量误差，
# 非真实位置偏差(放料精度由XY决定)；XY≤2.0时产品已验证准确。
XY_TOL_MM = 4.0  # [qjxopt]
# 2026-09-21 提速：12→20mm。大基线(如SL1 Y偏51mm)粗调每步只能12mm导致磨4~6次，
# 加大到20mm一次吃掉更多；FINE_ZONE=10mm以下仍按细调步距4mm，不受此影响。
MAX_STEP_XY_MM = 20.0
# 2026-09-21：15→20。XL真实Z偏差~15mm(gain后~17mm)被15 clamp残留~1.3mm，
# 叠加XY达标即停后Z残差没机会纠、产品放不到位；加大步距让真实Z偏差一轮纠完。
MAX_STEP_Z_MM = 20.0
FINE_ZONE_MM = 10.0
FINE_STEP_MM = 4.0
MAX_INIT_MM = 100.0
# 2026-10-20 SL1 开环一步到位（--one-step）：单步幅度门放宽到300，允许"复位→目标"大跨度。
# 2026-10-21 应需求放宽到600：允许工位不做挂载直接大跨度一步到示教点。
# ⚠ 注意：放开门限后大跨度 OFFSET 会真实执行（末端直线运动），有扫掠碰撞风险，
# 旧值备份于 .bak_20261021_wq_gain 之前的版本；回退改回 300 即可。
MAX_ONE_STEP_INIT_MM = 600.0
MAX_ITERS = 12
# max_iters/diverged 时若历史最优优于该值，回到最优位置而非全量回退
# 10.0：单目Z噪声今天实测WQ一帧摆18mm(+7.4→-10.6)，物理XY已到2~4mm时
# 测得XYZ最优可达8~10mm。且 best≤baseline 恒成立(首测即基线)，保最优
# 按测量值永不劣于回退起点；10仍远小于典型基线13~30mm。
KEEP_BEST_MM = 10.0
# 2026-10-21 提速后复调：0.3→0.1 曾导致验证帧 MOVED 报错（大跨度动作余振未停稳），
# 且关节速度已提至0.6，故取 0.5 确保回稳；验证稳定后可再试 0.3~0.4。
SETTLE_S = 0.5
# 2026-10-21 应需求放宽到25：允许工位不做挂载直接从远处位形一步（旋转可达25°）。
# ⚠ 旋转门同时作用于一步到位与多轮闭环；放开后大角度旋转补偿会一次性下发，
# 有扫掠碰撞风险。回退改回 8.0 即可。
MAX_ROTATION_DEG = 25.0
# XL 工位需要末端姿态补偿：实测 XL 加旋转+纠 Z 才放得准。
# 仅 XL（rot_comp=True）时启用；姿态偏差 ≤ ROT_TOL_DEG 才判 aligned。
ROT_TOL_DEG = 1.0
# 单步旋转上限（度），限制单次姿态补偿幅度，避免猛转。
MAX_STEP_ROT_DEG = 4.0


def measure(backend, vision, relation):
    """在已打开的backend上采集合格帧；只跳过视觉坏帧，其余异常立即失败。"""
    from g2_tag_placement.controller import G2AprilTagPlacement, PlacementError

    boards, ends, rejected = [], [], []
    # [2026-10-26] 底盘导航任务收尾竞态：GO 的回执可能先于底盘任务终止返回，
    # 此时 snapshot 报 CHASSIS_ACTIVE(state=2)。最多等 10s（0.5s×20），稳态后再测。
    start = None
    for _wait in range(20):
        try:
            start = backend.snapshot()
            break
        except Exception as exc:
            if getattr(exc, 'code', '') != 'CHASSIS_ACTIVE':
                raise
            time.sleep(0.5)
    if start is None:
        raise RuntimeError('CHASSIS_ACTIVE: 底盘任务未在10s内终止')
    for attempt in range(1, MAX_ATTEMPTS + 1):
        frame = backend.capture()
        G2AprilTagPlacement._unchanged(start, frame['snapshot'])
        stamp = frame['timestamp_ns']
        observation = vision.observe(
            frame['image'], timestamp_ns=stamp,
            base_T_camera=frame['base_T_camera'],
            base_T_camera_timestamp_ns=stamp,
        )
        if not observation['diagnostic_pass']:
            reasons = observation.get('rejection_reasons') or []
            detected = observation.get('detected_ids') or []
            rejected.append({'attempt': attempt, 'reasons': reasons, 'detected_ids': detected})
            print(f'[safe-align115] 丢弃坏帧 {attempt}: {reasons} detected_ids={detected}', file=sys.stderr)
            continue
        boards.append(np.array(observation['base_T_board_nominal']))
        ends.append(np.array(frame['base_T_right_end']))
        if len(boards) >= VALID_FRAMES:
            break
    if len(boards) < VALID_FRAMES:
        raise PlacementError('VISION', f'{MAX_ATTEMPTS}次采样仅{len(boards)}个合格帧，拒绝执行动作')

    import vision_pick_correction as vpc
    board_now = vpc._median_pose(boards)
    end_now = vpc._median_pose(ends)
    goal = board_now @ relation
    delta = (goal[:3, 3] - end_now[:3, 3]) * 1000.0
    rotation = goal[:3, :3] @ end_now[:3, :3].T
    rotation_deg = float(np.degrees(np.linalg.norm(cv2.Rodrigues(rotation)[0])))
    rot_rx, rot_ry, rot_rz = _rotation_matrix_to_euler_zyx(rotation)
    return {
        'delta_mm': [float(value) for value in delta],
        'xy_distance_mm': float(np.linalg.norm(delta[:2])),
        'xyz_distance_mm': float(np.linalg.norm(delta)),
        'rotation_deg': rotation_deg,
        'rotation_euler_deg': [round(rot_rx, 3), round(rot_ry, 3), round(rot_rz, 3)],
        'current_pose_mm': [float(value) for value in end_now[:3, 3] * 1000.0],
        'valid_frames': len(boards),
        'rejected_frames': rejected,
    }


def clamp_xyz(x, y, z, step_xy, step_z):
    import math
    xy = math.hypot(x, y)
    if xy > step_xy:
        scale = step_xy / xy
        x, y = x * scale, y * scale
    if abs(z) > step_z:
        z = step_z if z > 0 else -step_z
    return x, y, z


def _rotation_matrix_to_euler_zyx(R):
    """base 坐标系旋转矩阵 → ZYX 欧拉角（度）。

    与 EndEffectorController.euler_to_quaternion（ZYX）互逆：
    R = Rz(rz)·Ry(ry)·Rx(rx)，rx/ry/rz 为绕 base X/Y/Z 的固定轴旋转角。
    仅用于小角度（<8°），无万向锁风险。
    """
    import math
    r00, r10, r20 = R[0, 0], R[1, 0], R[2, 0]
    r21, r22 = R[2, 1], R[2, 2]
    ry = math.atan2(-r20, math.sqrt(r00 * r00 + r10 * r10))
    if abs(ry) < math.pi / 2:
        rx = math.atan2(r21, r22)
        rz = math.atan2(r10, r00)
    else:
        rx = math.atan2(-R[1, 2], R[1, 1])
        rz = 0.0
    return (math.degrees(rx), math.degrees(ry), math.degrees(rz))


def clamp_rot(rx, ry, rz, step_deg):
    """旋转分量各轴独立限幅到 ±step_deg（度）。"""
    def _c(v):
        return max(-step_deg, min(step_deg, v))
    return _c(rx), _c(ry), _c(rz)


def send_offset(robot, command):
    return bool(robot.OFFSET(command))


def rollback(robot, net_command):
    command = {'rx': round(-net_command.get('rx', 0.0), 3),
               'ry': round(-net_command.get('ry', 0.0), 3),
               'rz': round(-net_command.get('rz', 0.0), 3),
               'rrx': round(-net_command.get('rrx', 0.0), 3),
               'rry': round(-net_command.get('rry', 0.0), 3),
               'rrz': round(-net_command.get('rrz', 0.0), 3)}
    if all(abs(command[k]) < 0.001 for k in ('rx', 'ry', 'rz', 'rrx', 'rry', 'rrz')):
        return True, command
    print(f'[safe-align115] 失败回退累计命令 {command}', file=sys.stderr)
    return send_offset(robot, command), command


def rollback_to(robot, net, target_net):
    """回退到历史最优位置：发送 target_net 与当前累计 net 的差值OFFSET。"""
    command = {'rx': round(target_net.get('rx', 0.0) - net.get('rx', 0.0), 3),
               'ry': round(target_net.get('ry', 0.0) - net.get('ry', 0.0), 3),
               'rz': round(target_net.get('rz', 0.0) - net.get('rz', 0.0), 3),
               'rrx': round(target_net.get('rrx', 0.0) - net.get('rrx', 0.0), 3),
               'rry': round(target_net.get('rry', 0.0) - net.get('rry', 0.0), 3),
               'rrz': round(target_net.get('rrz', 0.0) - net.get('rrz', 0.0), 3)}
    if all(abs(command[k]) < 0.001 for k in ('rx', 'ry', 'rz', 'rrx', 'rry', 'rrz')):
        return True, command
    print(f'[safe-align115] 回退到历史最优位置 {command}', file=sys.stderr)
    return send_offset(robot, command), command


def emit(result):
    # flush=True 关键：若随后 GDK 清理阶段 C++ abort（SIGABRT），
    # 已写出的 RESULT 仍能即时落盘到 stdout，guard 才能正确判 aligned，
    # 否则块缓冲未 flush 会丢结果、误判为"无可解析结果=危险"。
    print('VISION_ALIGN_SAFE_115_RESULT ' + json.dumps(result, ensure_ascii=False),
          flush=True)


def align_loop(backend, vision, relation, robot, initial, baseline, net, mode, rot_comp=False):
    """在已打开的backend/robot上执行闭环；失败时撤销本次累计OFFSET。

    rot_comp=True（仅 XL 工位）时额外做末端姿态补偿：实测 XL 靠旋转+纠 Z 才能放准，
    故把姿态差转成 rrx/rry/rrz 与 XY 同轮下发，并纳入 aligned 判定。
    """
    measurements = []
    previous = None
    current = initial
    best_distance = initial['xyz_distance_mm']
    net.setdefault('rrx', 0.0)
    net.setdefault('rry', 0.0)
    net.setdefault('rrz', 0.0)
    best_net = dict(net)
    try:
        for observation_index in range(1, MAX_ITERS + 2):
            measurements.append(current)
            distance = current['xyz_distance_mm']
            print(
                f"[safe-align115] 第{observation_index}次 delta={current['delta_mm']} "
                f"XYZ={distance:.3f}mm",
                file=sys.stderr,
            )
            if distance < best_distance:
                best_distance = distance
                best_net = dict(net)
            if current['rotation_deg'] > MAX_ROTATION_DEG:
                ok, command = rollback(robot, net)
                emit({'status': 'rotation_exceeded', 'mode': mode,
                      'measurements': measurements, 'rollback_ok': ok,
                      'rollback_command': command})
                return 2
            if observation_index == 1 and distance > MAX_INIT_MM:
                ok, command = rollback(robot, net)
                emit({'status': 'initial_exceeded', 'mode': mode,
                      'measurements': measurements, 'rollback_ok': ok,
                      'rollback_command': command})
                return 2
            if distance <= TOL_MM or current['xy_distance_mm'] <= XY_TOL_MM:
                if rot_comp and current['rotation_deg'] > ROT_TOL_DEG:
                    # XL：位置已达标但姿态还超阈值，不能停，继续补旋转
                    pass
                else:
                    emit({'status': 'aligned', 'mode': mode,
                          'iterations': observation_index - 1, 'baseline': baseline,
                          'measurements': measurements, 'net_command_mm': net})
                    return 0
            if observation_index > MAX_ITERS:
                if best_distance <= KEEP_BEST_MM:
                    ok, command = rollback_to(robot, net, best_net)
                    emit({'status': 'kept_best', 'mode': mode, 'reason': 'max_iters',
                          'best_distance_mm': best_distance,
                          'measurements': measurements, 'rollback_ok': ok,
                          'rollback_command': command})
                    return 2
                ok, command = rollback(robot, net)
                emit({'status': 'max_iters', 'mode': mode,
                      'measurements': measurements, 'rollback_ok': ok,
                      'rollback_command': command})
                return 2
            if previous is not None and distance > previous + 2.0:
                if best_distance <= KEEP_BEST_MM:
                    ok, command = rollback_to(robot, net, best_net)
                    emit({'status': 'kept_best', 'mode': mode, 'reason': 'diverged',
                          'best_distance_mm': best_distance,
                          'measurements': measurements, 'rollback_ok': ok,
                          'rollback_command': command})
                    return 2
                ok, command = rollback(robot, net)
                emit({'status': 'diverged', 'mode': mode,
                      'measurements': measurements, 'rollback_ok': ok,
                      'rollback_command': command})
                return 2
            previous = distance
            delta = current['delta_mm']
            if distance <= FINE_ZONE_MM:
                step_xy = step_z = FINE_STEP_MM
            else:
                step_xy, step_z = MAX_STEP_XY_MM, MAX_STEP_Z_MM
            rx, ry, rz = clamp_xyz(delta[0] * GAIN, delta[1] * GAIN, delta[2] * GAIN, step_xy, step_z)
            correction = {'rx': round(rx, 3), 'ry': round(ry, 3), 'rz': round(rz, 3)}
            if rot_comp:
                erx, ery, erz = current['rotation_euler_deg']
                rrx, rry, rrz = clamp_rot(erx, ery, erz, MAX_STEP_ROT_DEG)
                correction['rrx'] = round(rrx, 3)
                correction['rry'] = round(rry, 3)
                correction['rrz'] = round(rrz, 3)
            print(f'[safe-align115] 纠偏 {correction}', file=sys.stderr)
            if not send_offset(robot, correction):
                emit({'status': 'correction_not_confirmed', 'mode': mode,
                      'measurements': measurements, 'motion_sent': 'unknown',
                      'manual_check_required': True})
                return 3
            net['rx'] += correction['rx']
            net['ry'] += correction['ry']
            net['rz'] += correction['rz']
            if rot_comp:
                net['rrx'] += correction['rrx']
                net['rry'] += correction['rry']
                net['rrz'] += correction['rrz']
            time.sleep(SETTLE_S)
            current = measure(backend, vision, relation)
    except Exception as exc:
        ok, command = rollback(robot, net)
        emit({'status': 'failed', 'mode': mode, 'error': str(exc),
              'code': getattr(exc, 'code', 'ERROR'), 'rollback_ok': ok,
              'rollback_command': command})
        return 2


def align_loop_one_step(backend, vision, relation, robot, initial, baseline, net, mode,
                        rot_comp=False, verify=False, one_step_gain=None, zero_z=False,
                        xy_tol=None):
    """SL1 专用：一步到位（一次测量 → 一次下发完整偏差 → 一次验证）。

    verify=False（生产路径）：纯开环，验证测量仅写日志，动作后无条件判 aligned。
    verify=True（对比测试）：验证残差 ≤ TOL_MM/XY_TOL_MM 才判 aligned；超差则
    撤销本次 OFFSET（rollback）返回 rc=2；验证测量失败同样回退（rollback_ok 表达）。
    one_step_gain：步进增益覆盖（None=用全局 GAIN），用于补偿单步执行效率。
    zero_z：只纠 XY、不动 Z（Z 向单目噪声大的工位，如 WQ）。
    与 align_loop 的区别：
      * 动作前只做安全门：姿态超限 / 偏差超 MAX_ONE_STEP_INIT_MM → 回退 rc=2；
      * 单步下发完整偏差*增益（不限幅），从起始姿态直接走到目标点。
    """
    measurements = [initial]
    net.setdefault('rrx', 0.0)
    net.setdefault('rry', 0.0)
    net.setdefault('rrz', 0.0)
    distance = initial['xyz_distance_mm']
    print(f"[safe-align115] 第1次 delta={initial['delta_mm']} XYZ={distance:.3f}mm",
          file=sys.stderr)
    try:
        if initial['rotation_deg'] > MAX_ROTATION_DEG:
            ok, command = rollback(robot, net)
            emit({'status': 'rotation_exceeded', 'mode': mode,
                  'measurements': measurements, 'rollback_ok': ok,
                  'rollback_command': command})
            return 2
        if distance > MAX_ONE_STEP_INIT_MM:
            ok, command = rollback(robot, net)
            emit({'status': 'initial_exceeded', 'mode': mode,
                  'measurements': measurements, 'rollback_ok': ok,
                  'rollback_command': command})
            return 2
        tol_xy = XY_TOL_MM if xy_tol is None else xy_tol
        if distance <= TOL_MM or initial['xy_distance_mm'] <= tol_xy:
            emit({'status': 'aligned', 'mode': mode, 'iterations': 0,
                  'baseline': baseline, 'measurements': measurements,
                  'net_command_mm': net})
            return 0
        delta = initial['delta_mm']
        gain = GAIN if one_step_gain is None else one_step_gain
        correction = {'rx': round(delta[0] * gain, 3),
                      'ry': round(delta[1] * gain, 3),
                      'rz': 0.0 if zero_z else round(delta[2] * gain, 3)}
        if rot_comp:
            erx, ery, erz = initial['rotation_euler_deg']
            correction['rrx'] = round(erx, 3)
            correction['rry'] = round(ery, 3)
            correction['rrz'] = round(erz, 3)
        print(f'[safe-align115] 一步纠偏 {correction}', file=sys.stderr)
        if not send_offset(robot, correction):
            emit({'status': 'correction_not_confirmed', 'mode': mode,
                  'measurements': measurements, 'motion_sent': 'unknown',
                  'manual_check_required': True})
            return 3
        for key, value in correction.items():
            net[key] = net.get(key, 0.0) + value
        # 动作后做一次验证测量；verify=True 时参与判定，超差/测量失败则撤销 OFFSET。
        try:
            time.sleep(SETTLE_S)
            current = measure(backend, vision, relation)
            measurements.append(current)
            print(f"[safe-align115] 一步后验证 delta={current['delta_mm']} "
                  f"XYZ={current['xyz_distance_mm']:.3f}mm", file=sys.stderr)
        except Exception as exc:  # noqa: BLE001
            if verify:
                ok, command = rollback(robot, net)
                emit({'status': 'verify_failed', 'mode': mode, 'error': str(exc),
                      'code': getattr(exc, 'code', 'ERROR'),
                      'measurements': measurements, 'rollback_ok': ok,
                      'rollback_command': command})
                return 2
            print(f'[safe-align115] 验证测量失败（纯开环不影响判定）：{exc}',
                  file=sys.stderr)
            emit({'status': 'aligned', 'mode': mode, 'iterations': 1, 'baseline': baseline,
                  'measurements': measurements, 'net_command_mm': net})
            return 0
        if verify:
            dist = current['xyz_distance_mm']
            xy = current['xy_distance_mm']
            if dist <= TOL_MM or xy <= tol_xy:
                emit({'status': 'aligned', 'mode': mode, 'iterations': 1, 'baseline': baseline,
                      'measurements': measurements, 'net_command_mm': net})
                return 0
            ok, command = rollback(robot, net)
            emit({'status': 'one_step_exceeded', 'mode': mode,
                  'verify_xyz_mm': dist, 'verify_xy_mm': xy,
                  'measurements': measurements, 'rollback_ok': ok,
                  'rollback_command': command})
            return 2
        emit({'status': 'aligned', 'mode': mode, 'iterations': 1, 'baseline': baseline,
              'measurements': measurements, 'net_command_mm': net})
        return 0
    except Exception as exc:
        ok, command = rollback(robot, net)
        emit({'status': 'failed', 'mode': mode, 'error': str(exc),
              'code': getattr(exc, 'code', 'ERROR'), 'rollback_ok': ok,
              'rollback_command': command})
        return 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, required=True)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--execute-align', action='store_true')
    group.add_argument('--execute-test', action='store_true')
    parser.add_argument('--inject-rx', type=float, default=5.0)
    parser.add_argument('--inject-ry', type=float, default=-5.0)
    step_group = parser.add_mutually_exclusive_group()
    step_group.add_argument('--one-step', action='store_true',
                            help='SL1 开环一步到位：一次测量→一次下发完整偏差→无条件判 aligned')
    step_group.add_argument('--one-step-verify', action='store_true',
                            help='SL1 一步到位+验证判定（对比测试）：验证残差超阈值则撤销OFFSET返回rc=2')
    parser.add_argument('--one-step-gain', type=float, default=None,
                        help='一步到位的增益覆盖（None=用全局GAIN）；WQ实测效率~0.73，建议1.35')
    parser.add_argument('--zero-z', action='store_true',
                        help='一步到位只纠XY、不动Z；用于Z向单目噪声大的工位（如WQ）')
    parser.add_argument('--xy-tol-mm', type=float, default=None,
                        help='验证判定的XY容差覆盖（None=用全局XY_TOL_MM；WQ临时用10）')
    parser.add_argument('--rot-comp', action='store_true',
                        help='强制启用末端姿态补偿(rrx/rry/rrz)；默认仅文件名含xl时启用。WQ现场实夹爪不正，需启用')
    args = parser.parse_args()
    if not args.reference.exists():
        emit({'status': 'no_reference', 'motion_sent': False})
        return 2
    if max(abs(args.inject_rx), abs(args.inject_ry)) > 5.0:
        parser.error('测试注入的单轴绝对值不得超过5mm')

    apply_115_runtime_profile()
    import vision_pick_correction as vpc
    from apriltag_vision import AprilTagVision
    from g2_tag_placement.controller import PlacementError
    from g2_tag_placement.gdk_backend import GDKBackend
    from minth import Minth

    vpc._patch_fresh_tolerance()
    vpc._patch_gripper_for_pick()
    reference = json.loads(args.reference.read_text())
    relation = np.array(reference['board_T_right_end_reference'])
    board_model = vpc._load_board()

    read_only = not args.execute_align and not args.execute_test
    robot = None if read_only else Minth.G2()
    # 仅 XL 工位（下料放料）需要末端姿态补偿：实测 XL 加旋转才放得准。
    # WF（弯曲机放料）实测不纠偏时反而放得正——其 rotation 读数是测量偏差而非
    # 真实姿态偏差，补旋转会把正确姿态转歪，故 WF 不做旋转、仅平移+纠 Z。
    # 默认按参考文件名启用旋转（仅 XL）；WQ 现场实测夹爪不正需旋转，
    # 由调用方传 --rot-comp 强制开启（见 all_machin_plc_vision_loop_onestep.py）。
    rot_comp = args.rot_comp or 'xl' in args.reference.name.lower()
    try:
        with GDKBackend(board_model, unused_pedal_fault_confirmed=True) as backend:
            vision = AprilTagVision(**backend.vision_config())
            if any(reference.get(k) != v for k, v in vision._identity.items()):
                emit({'status': 'identity_mismatch', 'motion_sent': False})
                return 2
            baseline = measure(backend, vision, relation)
            print('[safe-align115] 基线 ' + json.dumps(baseline, ensure_ascii=False),
                  file=sys.stderr)

            if read_only:
                emit({'status': 'baseline_ready', 'baseline': baseline, 'motion_sent': False})
                return 0

            if args.execute_align:
                if args.one_step or args.one_step_verify:
                    return align_loop_one_step(backend, vision, relation, robot,
                                               baseline, baseline,
                                               {'rx': 0.0, 'ry': 0.0, 'rz': 0.0},
                                               'production', rot_comp=rot_comp,
                                               verify=args.one_step_verify,
                                               one_step_gain=args.one_step_gain,
                                               zero_z=args.zero_z,
                                               xy_tol=args.xy_tol_mm)
                return align_loop(backend, vision, relation, robot, baseline, baseline,
                                  {'rx': 0.0, 'ry': 0.0, 'rz': 0.0}, 'production',
                                  rot_comp=rot_comp)

            if baseline['xyz_distance_mm'] > TOL_MM or baseline['rotation_deg'] > MAX_ROTATION_DEG:
                emit({'status': 'baseline_not_ready', 'baseline': baseline, 'motion_sent': False})
                return 2
            injection = {'rx': round(args.inject_rx, 3), 'ry': round(args.inject_ry, 3), 'rz': 0.0}
            print(f'[safe-align115] 注入测试偏差 {injection}', file=sys.stderr)
            if not send_offset(robot, injection):
                emit({'status': 'injection_not_confirmed', 'motion_sent': 'unknown'})
                return 3
            time.sleep(SETTLE_S)
            try:
                current = measure(backend, vision, relation)
            except Exception as exc:
                ok, command = rollback(robot, dict(injection))
                emit({'status': 'failed', 'mode': 'test', 'error': str(exc),
                      'code': getattr(exc, 'code', 'ERROR'), 'rollback_ok': ok,
                      'rollback_command': command})
                return 2
            return align_loop(backend, vision, relation, robot, current, baseline,
                              dict(injection), 'test', rot_comp=rot_comp)
    except Exception as exc:
        emit({'status': 'failed_before_motion', 'error': str(exc),
              'code': getattr(exc, 'code', 'ERROR'), 'motion_sent': read_only is False})
        return 2
    finally:
        if robot is not None:
            robot.close()


if __name__ == '__main__':
    raise SystemExit(main())
