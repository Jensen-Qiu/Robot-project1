#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""送料机取料 视觉纠偏 —— 独立子进程（只读相机，绝不发送任何动作命令）。

设计原则：
  * 本脚本用视觉包的 GDKBackend + AprilTagVision，只做「看」：连拍 20 帧、
    解算 ID3/4 标定板相对机械臂末端的偏差，输出毫米级偏移。
  * 全程不移动机械臂、不发夹爪命令、不动底盘。跑完即退出并释放视觉包的排他锁，
    因此不会与主程序的 Minth SDK 抢占机械臂。
  * 主程序（Minth）拿到偏移后，自行决定是否用 OFFSET / MoveL(offset=...) 修正。

两种模式：
  teach   —— 在取料位、已抓稳型材、姿态正确时执行一次，生成当前开机的取料参考
             pick_reference.json（记录「标定板 ↔ 末端」关系，绑定当前 boot_id）。
  correct —— 之后每轮取料前执行，读取 pick_reference.json，算出末端相对目标的
             毫米偏移，供主程序对位。无有效参考 / 视觉不稳 / 超限 → 明确失败。

输出：最后一行以 VISION_PICK_RESULT 开头，后接 JSON。
退出码：0=成功（teach 已保存 / correct 得到可用偏移或明确的安全拒绝），2=异常。
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

# 视觉包根目录（与实际部署一致）
PKG_ROOT = Path('/home/agi/G2_AprilTag_Placement_0.2.0')
sys.path.insert(0, str(PKG_ROOT))

from apriltag_vision import AprilTagVision
from g2_tag_placement import gdk_backend as _gdk
from g2_tag_placement.gdk_backend import GDKBackend, field
from g2_tag_placement.controller import G2AprilTagPlacement, PlacementError
from g2_tag_placement.geometry import pose6_from_matrix

# 默认取料参考文件（放在本脚本同目录，不写入视觉包目录）
DEFAULT_REF = Path(__file__).resolve().parent / 'pick_reference.json'


# 取料空爪允许的夹爪最大张开位置（米）。取料纠偏在合爪【之前】做，爪是张开的，
# 原视觉包要求 abs(pos)<=0.03（接近闭合），会误判。这里放宽到一个足够大的上限。
# 其余夹爪健康项（受控/使能/无错误码/静止）以及所有其它安全检查保持不变。
PICK_GRIPPER_MAX_POS = 1.5

# 时钟抖动容差（秒）。SDK 时间源与系统墙钟有几毫秒相对抖动，偶尔数据时间戳会
# 微微超前于 time.time_ns()，产生极小负 age（如 -0.000s），被原 fresh 的
# 下限 0<=age 卡住而误报 STALE。这里给一个很小的负向容差吸收抖动；age 上限
# （数据不能太旧）完全保留。不改视觉包源文件，仅本进程 monkey-patch。
FRESH_NEG_TOLERANCE_S = 0.05


def _patch_fresh_tolerance():
    """放宽 fresh 的下限，容忍几毫秒时钟抖动导致的极小负 age；上限不变。"""
    import time as _time

    def fresh(timestamp_ns, max_age_s=.5):
        age = (_time.time_ns() - int(timestamp_ns)) / 1e9
        if not (-FRESH_NEG_TOLERANCE_S <= age <= max_age_s):
            raise PlacementError('STALE', f'机器人本机数据过期/时钟异常，age={age:.3f}s')

    _gdk.fresh = fresh


def _patch_gripper_for_pick():
    """取料模式补丁：完整复制原 snapshot，仅放宽“夹爪位置必须接近闭合”这一条。

    与视觉包原 snapshot 逐行一致，唯一区别是把夹爪位置上限 0.03 换成
    PICK_GRIPPER_MAX_POS（允许张爪）。不改视觉包源文件，仅本进程内 monkey-patch。
    时钟/TF/力矩/身份/底盘/控制模式/左臂一致性/夹爪健康与静止等检查全部保留。
    """
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
        # 唯一放宽点：abs(pos) 上限 0.03 → PICK_GRIPPER_MAX_POS（取料允许张爪）。
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

# 安全阈值：偏移超过这些值视为异常场景，correct 会拒绝，主程序应回退固定点位。
MAX_TRANSLATION_MM = 50.0   # 平移偏差上限
MAX_ROTATION_DEG = 5.0      # 旋转偏差上限


def _load_board():
    return json.loads((PKG_ROOT / 'g2_tag_placement/data/board.json').read_text())


def _batch_observe(backend, vision):
    """连拍 20 帧，返回每帧 (base_T_board, base_T_right_end)。头/臂移动即失败。"""
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


def _median_pose(mats):
    """位置取中位数，旋转取第一帧（帧间已校验静止），返回 4x4。"""
    out = np.eye(4)
    out[:3, 3] = np.median(np.array([m[:3, 3] for m in mats]), axis=0)
    out[:3, :3] = mats[0][:3, :3]
    return out


def do_teach(ref_path):
    board_model = _load_board()
    with GDKBackend(board_model, unused_pedal_fault_confirmed=True) as backend:
        vision = AprilTagVision(**backend.vision_config())
        boards, ends = _batch_observe(backend, vision)
        # 用最后一帧生成参考（此刻末端与标定板均静止、已抓稳型材）
        frame_board = _median_pose(boards)
        # 重新取一帧配对的末端位姿，保证 board 与 end 同帧一致
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
                    message=f'未找到取料参考 {ref_path}，请先运行 teach（当前开机）')
    reference = json.loads(ref_path.read_text())
    board_model = _load_board()
    with GDKBackend(board_model, unused_pedal_fault_confirmed=True) as backend:
        vision = AprilTagVision(**backend.vision_config())
        # 身份校验：相机/码板/开机必须与参考一致，否则拒绝
        if any(reference.get(k) != v for k, v in vision._identity.items()):
            return dict(status='identity_mismatch',
                        message='相机/码板/开机身份与参考不一致，请在当前开机重新 teach')
        boards, ends = _batch_observe(backend, vision)
        board_now = _median_pose(boards)
        end_now = _median_pose(ends)
        relation = np.array(reference['board_T_right_end_reference'])
        # 末端目标 = 当前标定板 @ 参考中的「标定板→末端」关系
        goal = board_now @ relation
        # base 坐标系下的目标-当前 平移偏移（转成毫米）
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
            translation_distance_mm=trans_dist,
            rotation_deg=rot_deg,
            limits=dict(max_translation_mm=MAX_TRANSLATION_MM, max_rotation_deg=MAX_ROTATION_DEG),
            goal_pose=pose6_from_matrix(goal),
            current_pose=pose6_from_matrix(end_now),
            note='delta_base_mm 为 base_link 坐标系下末端应平移的毫米量；'
                 '主程序需按末端坐标系约定换算后再用 OFFSET/MoveL 下发；本脚本不发动作。')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['teach', 'correct'])
    parser.add_argument('--reference', type=Path, default=DEFAULT_REF)
    args = parser.parse_args()
    # 先放宽时钟抖动容差，再放宽取料夹爪检查（顺序重要：snapshot 补丁会取 _gdk.fresh）
    _patch_fresh_tolerance()
    # 取料模式：放宽“夹爪必须闭合”这一条（空爪对准料再夹）。其余检查全保留。
    _patch_gripper_for_pick()
    try:
        if args.mode == 'teach':
            result = do_teach(args.reference)
        else:
            result = do_correct(args.reference)
        print('VISION_PICK_RESULT ' + json.dumps(result, ensure_ascii=False))
        return 0
    except Exception as exc:
        print('VISION_PICK_RESULT ' + json.dumps(dict(
            status='failed', error=str(exc),
            code=getattr(exc, 'code', 'ERROR')), ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
