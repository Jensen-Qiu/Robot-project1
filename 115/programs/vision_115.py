#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""115 专用视觉入口。

只在本进程中绑定 115 已核验的 AID、GDK SDK 和 IK 配置哈希；
不修改视觉包源码，也不修改机器人运动控制配置。

用法：
  python3 vision_115.py preflight
  python3 vision_115.py pick teach --reference pick_reference.json
  python3 vision_115.py pick correct --reference pick_reference.json
  python3 vision_115.py align --reference pick_reference.json --relax-gripper

注意：只有 preflight 和 pick 子命令不发送机器人动作；align 会执行 Minth OFFSET。
"""
import hashlib
import json
import sys
from pathlib import Path

import numpy as np

PKG_ROOT = Path('/home/agi/G2_AprilTag_Placement_0.2.0')
PROGRAMS_ROOT = Path('/data/wxf0721/runtime/programs')
sys.path.insert(0, str(PKG_ROOT))
sys.path.insert(0, str(PROGRAMS_ROOT))

EXPECTED_AID = 'G2A0104C300228'
EXPECTED_IK_SHA256 = 'b6648713e61adc319f0cab4e59101fcc27b0e4fe80b6366ccfbc8c3447d2936b'
EXPECTED_SDK_SHA256 = 'c8fcab6c01b000e5706681533aa663b776d091cdf3da3bfbefcf447bd3817863'


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def apply_115_runtime_profile():
    """严格核验 115 当前运行文件后，仅在本进程中应用固定身份绑定。"""
    from g2_tag_placement import _jog
    from g2_tag_placement import gdk_backend as gdk

    actual_ik = _sha256(gdk.IK_CONFIG)
    if actual_ik != EXPECTED_IK_SHA256:
        raise RuntimeError(
            f'115 IK 配置发生变化：actual={actual_ik}, expected={EXPECTED_IK_SHA256}'
        )

    actual_sdk = _sha256(_jog.SDK_BINARY)
    if actual_sdk != EXPECTED_SDK_SHA256:
        raise RuntimeError(
            f'115 GDK SDK 发生变化：actual={actual_sdk}, expected={EXPECTED_SDK_SHA256}'
        )

    # GDKBackend 后续仍会执行原有哈希与 AID 校验；这里只替换为 115 的固定基线。
    gdk.IK_SHA256 = EXPECTED_IK_SHA256
    _jog.EXPECTED_AID = EXPECTED_AID
    _jog.EXPECTED_SDK_SHA256 = EXPECTED_SDK_SHA256


def run_preflight():
    """只读检查身份、运行文件、底盘/夹爪/双臂和运动控制状态。"""
    import agibot_gdk as agibot
    import vision_pick_correction as vpc
    from g2_tag_placement.gdk_backend import GDKBackend
    from g2_tag_placement.geometry import pose6_from_matrix

    vpc._patch_fresh_tolerance()
    vpc._patch_gripper_for_pick()
    board = vpc._load_board()
    with GDKBackend(board, unused_pedal_fault_confirmed=True) as backend:
        snapshot = backend.snapshot()
        result = {
            'status': 'ready',
            'aid': agibot.get_robot_aid(),
            'boot_id': snapshot['boot_id'],
            'pnc_state': snapshot['pnc_state'],
            'gripper_position': snapshot['gripper_position'],
            'right_end_pose': pose6_from_matrix(np.array(snapshot['right'])),
            'left_end_pose': pose6_from_matrix(np.array(snapshot['left'])),
            'ik_sha256': EXPECTED_IK_SHA256,
            'sdk_sha256': EXPECTED_SDK_SHA256,
            'motion_sent': False,
        }
    print('VISION_115_PREFLIGHT ' + json.dumps(result, ensure_ascii=False))
    return 0


def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ('preflight', 'pick'):
        print(__doc__.strip(), file=sys.stderr)
        return 2

    command = sys.argv[1]
    forwarded = sys.argv[2:]
    try:
        apply_115_runtime_profile()
        if command == 'preflight':
            if forwarded:
                raise ValueError('preflight 不接受额外参数')
            return run_preflight()
        import vision_pick_correction
        sys.argv = ['vision_pick_correction.py'] + forwarded
        return vision_pick_correction.main()
    except Exception as exc:
        prefix = 'VISION_115_PREFLIGHT' if command == 'preflight' else 'VISION_115_RESULT'
        print(prefix + ' ' + json.dumps({
            'status': 'failed',
            'error': str(exc),
            'code': getattr(exc, 'code', 'ERROR'),
            'motion_sent': False if command != 'align' else 'unknown',
        }, ensure_ascii=False), file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
