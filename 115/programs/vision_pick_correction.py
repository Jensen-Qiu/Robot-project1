#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""取料/放料 视觉纠偏：teach 建参考 + correct 单次读偏移。只读相机，不发动作。

含两个本进程 monkey-patch（不改视觉包源文件）：
  * 夹爪位置阈值放宽：取料空爪 / 夹厚料时 abs(pos) 会超 0.03，放宽上限，
    但受控/使能/无错误码/静止(vel) 等健康检查全部保留。
  * fresh 时钟抖动容差：SDK 时间源与系统墙钟有几毫秒抖动，偶发极小负 age
    会误报 STALE；给一个小负向容差，上限(数据不能太旧)不变。
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

PKG_ROOT = Path('/home/agi/G2_AprilTag_Placement_0.2.0')
sys.path.insert(0, str(PKG_ROOT))

from apriltag_vision import AprilTagVision
from g2_tag_placement import gdk_backend as _gdk
from g2_tag_placement.gdk_backend import GDKBackend, field
from g2_tag_placement.controller import G2AprilTagPlacement, PlacementError
from g2_tag_placement.geometry import pose6_from_matrix

DEFAULT_REF = Path(__file__).resolve().parent / 'pick_reference.json'

MAX_TRANSLATION_MM = 50.0
MAX_ROTATION_DEG = 5.0

# 取料空爪/夹厚料允许的夹爪最大位置（米）
PICK_GRIPPER_MAX_POS = 1.5
# 时钟抖动容差（秒）
FRESH_NEG_TOLERANCE_S = 0.05


def _patch_fresh_tolerance():
    """放宽 fresh 下限，容忍几毫秒时钟抖动导致的极小负 age；上限不变。"""
    import time as _time

    def fresh(timestamp_ns, max_age_s=.5):
        age = (_time.time_ns() - int(timestamp_ns)) / 1e9
        if not (-FRESH_NEG_TOLERANCE_S <= age <= max_age_s):
            raise PlacementError('STALE', f'机器人本机数据过期/时钟异常，age={age:.3f}s')

    _gdk.fresh = fresh


def _patch_gripper_for_pick():
    """完整复制原 snapshot，仅放宽夹爪位置上限；其余检查全保留。"""
    from pathlib import Path as _Path
    fresh = _gdk.fresh
    transform = _gdk.transform
    from g2_tag_placement.geometry import matrix_from_pose7, close

    def snapshot(self):
        if _Path('/proc/sys/kernel/random/boot_id').read_text().strip() != self.boot_id:
            raise PlacementError('BOOT', '开机身份改变')
        p, ch = self.robot.get_chassis_power_state(), self.robot.get_chest_power_state()
        fresh(p.timestamp)
        faults = [p.charge_plug_insert_state, p.emergency_stop_pedal_state,
                  p.chassis_power_board_fault_state, ch.emergency_stop_button_req,
                  ch.emergency_stop_button_fault_state, ch.chest_power_board_fault_state]
        if any(faults) or (p.emergency_stop_pedal_fault_state and not self._pedal_exception):
            raise PlacementError('POWER', '急停/电源/充电状态不允许，检查现场与状态日志')
        task = self.pnc.get_task_state()
        if int(task.state) not in (0, 7, 8, 9):
            raise PlacementError('CHASSIS_ACTIVE', f'底盘任务未终止 state={task.state}')
        s = self.jog.snapshot()
        fresh(s['tf_timestamp_ns'])
        out = dict(right=matrix_from_pose7(s['right_pose7']).tolist(),
                   left=matrix_from_pose7(s['left_pose7']).tolist(),
                   timestamp_ns=s['tf_timestamp_ns'], boot_id=self.boot_id,
                   pedal_fault=int(p.emergency_stop_pedal_fault_state), pnc_state=int(task.state))
        for name, key in [('head_link3', 'head'), ('odom', 'odom'), ('arm_l_end_link', 'left_tf')]:
            stamp = self.tf.get_latest_timestamp(name)
            fresh(stamp, .25)
            out[key] = transform(self.tf.lookup_transform(name, 'base_link', stamp)).tolist()
            out[key + '_timestamp_ns'] = int(stamp)
        if not close(out['left_tf'], out['left']):
            raise PlacementError('LEFT_FEEDBACK', '左臂 TF 与控制器反馈不一致')
        end = field(self.robot.get_end_state(), 'right_end_state')
        states = field(end, 'end_states')
        if len(states) != 1:
            raise PlacementError('GRIPPER', '右夹爪反馈数量不符')
        grip = states[0]
        pos, vel = float(field(grip, 'position')), float(field(grip, 'velocity'))
        if (not field(end, 'controlled') or not field(grip, 'enable') or field(grip, 'err_code') != 0
                or not np.isfinite([pos, vel]).all()
                or abs(pos) > PICK_GRIPPER_MAX_POS or abs(vel) > .01):
            raise PlacementError('GRIPPER', '右夹爪不健康/未受控/未静止（取料模式）')
        out['gripper_position'] = pos
        status = self.robot.get_motion_control_status()
        if int(status.error_code) != 0 or int(status.control_mode) != 1:
            raise PlacementError('MC', '运动控制状态异常')
        w = status.wrenches[list(status.frame_names).index('arm_r_end_link')]
        out['wrench'] = [w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z]
        if not np.isfinite(out['wrench']).all():
            raise PlacementError('WRENCH', '力矩包含非有限值')
        fresh(p.timestamp)
        fresh(out['timestamp_ns'])
        for key in ('odom_timestamp_ns', 'head_timestamp_ns', 'left_tf_timestamp_ns'):
            fresh(out[key], .25)
        return out

    GDKBackend.snapshot = snapshot


def _load_board():
    return json.loads((PKG_ROOT / 'g2_tag_placement/data/board.json').read_text())


def _median_pose(mats):
    out = np.eye(4)
    out[:3, 3] = np.median(np.array([m[:3, 3] for m in mats]), axis=0)
    out[:3, :3] = mats[0][:3, :3]
    return out


def _batch_observe(backend, vision):
    start = backend.snapshot()
    boards, ends = [], []
    for _ in range(20):
        frame = backend.capture()
        G2AprilTagPlacement._unchanged(start, frame['snapshot'])
        stamp = frame['timestamp_ns']
        obs = vision.observe(frame['image'], timestamp_ns=stamp,
                             base_T_camera=frame['base_T_camera'],
                             base_T_camera_timestamp_ns=stamp)
        if not obs['diagnostic_pass']:
            raise PlacementError('VISION', ','.join(obs['rejection_reasons']))
        boards.append(np.array(obs['base_T_board_nominal']))
        ends.append(np.array(frame['base_T_right_end']))
    return boards, ends


def do_teach(ref_path):
    board_model = _load_board()
    with GDKBackend(board_model, unused_pedal_fault_confirmed=True) as backend:
        vision = AprilTagVision(**backend.vision_config())
        _batch_observe(backend, vision)
        frame = backend.capture()
        stamp = frame['timestamp_ns']
        obs = vision.observe(frame['image'], timestamp_ns=stamp,
                             base_T_camera=frame['base_T_camera'],
                             base_T_camera_timestamp_ns=stamp)
        if not obs['diagnostic_pass']:
            raise PlacementError('VISION', ','.join(obs['rejection_reasons']))
        ref = vision.make_reference(obs, base_T_right_end=frame['base_T_right_end'],
                                    right_end_timestamp_ns=stamp,
                                    name='feeder_pick',
                                    correct_preplacement_confirmed=True)
        if not ref['diagnostic_pass']:
            raise PlacementError('REFERENCE', ','.join(ref['rejection_reasons']))
        ref_path.write_text(json.dumps(ref, ensure_ascii=False, indent=2))
        return dict(status='teach_saved', reference_path=str(ref_path),
                    right_end_pose=pose6_from_matrix(np.array(frame['base_T_right_end'])),
                    stream_id=ref['stream_id'])


def do_correct(ref_path):
    if not ref_path.exists():
        return dict(status='no_reference',
                    message=f'未找到参考 {ref_path}，请先运行 teach（当前开机）')
    reference = json.loads(ref_path.read_text())
    board_model = _load_board()
    with GDKBackend(board_model, unused_pedal_fault_confirmed=True) as backend:
        vision = AprilTagVision(**backend.vision_config())
        if any(reference.get(k) != v for k, v in vision._identity.items()):
            return dict(status='identity_mismatch',
                        message='相机/码板/开机身份与参考不一致，请在当前开机重新 teach')
        boards, ends = _batch_observe(backend, vision)
        board_now = _median_pose(boards)
        end_now = _median_pose(ends)
        relation = np.array(reference['board_T_right_end_reference'])
        goal = board_now @ relation
        delta_base_mm = (goal[:3, 3] - end_now[:3, 3]) * 1000.0
        trans_dist = float(np.linalg.norm(delta_base_mm))
        rot = goal[:3, :3] @ end_now[:3, :3].T
        import cv2
        rot_deg = float(np.degrees(np.linalg.norm(cv2.Rodrigues(rot)[0])))
        within = trans_dist <= MAX_TRANSLATION_MM and rot_deg <= MAX_ROTATION_DEG
        return dict(
            status='correction_ok' if within else 'correction_exceeds_limit',
            within_limits=within,
            delta_base_mm=dict(x=float(delta_base_mm[0]), y=float(delta_base_mm[1]),
                               z=float(delta_base_mm[2])),
            translation_distance_mm=trans_dist, rotation_deg=rot_deg,
            limits=dict(max_translation_mm=MAX_TRANSLATION_MM, max_rotation_deg=MAX_ROTATION_DEG),
            goal_pose=pose6_from_matrix(goal), current_pose=pose6_from_matrix(end_now))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['teach', 'correct'])
    parser.add_argument('--reference', type=Path, default=DEFAULT_REF)
    args = parser.parse_args()
    _patch_fresh_tolerance()
    _patch_gripper_for_pick()
    try:
        result = do_teach(args.reference) if args.mode == 'teach' else do_correct(args.reference)
        print('VISION_PICK_RESULT ' + json.dumps(result, ensure_ascii=False))
        return 0
    except Exception as exc:
        print('VISION_PICK_RESULT ' + json.dumps(dict(
            status='failed', error=str(exc),
            code=getattr(exc, 'code', 'ERROR')), ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
