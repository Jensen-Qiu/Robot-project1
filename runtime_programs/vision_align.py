#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""通用 视觉纠偏 一体化闭环（常驻进程，提速版）。取料/放料共用。

  * GDKBackend 只初始化【一次】，相机常开，闭环内多次读偏移、下发 OFFSET。
  * 每次只采 SAMPLE_FRAMES=5 帧（够稳，省时）。
  * 同一进程内 Minth(发 OFFSET) 与 GDKBackend(只读) 共存（已验证不抢锁）。
  * 单步限幅、发散检测、身份校验；只发 OFFSET rx/ry/rz（右臂平移）。

参数：
  --reference PATH   参考文件（取料 pick_reference.json / 放料 place_reference.json）
  --relax-gripper    放宽“夹爪必须闭合”检查（取料空爪用）；放料抓着料不加此项。

被主程序以子进程调用；打印 VISION_ALIGN_RESULT {json}。
退出码 0=完成(达标/安全回退)，2=异常。
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

PKG = Path('/home/agi/G2_AprilTag_Placement_0.2.0')
sys.path.insert(0, str(PKG))
sys.path.insert(0, '/data/wxf0721/runtime')

import vision_pick_correction as vpc   # 复用 _load_board / _median_pose / 夹爪补丁
from minth import Minth
from g2_tag_placement.gdk_backend import GDKBackend
from apriltag_vision import AprilTagVision
from g2_tag_placement.controller import PlacementError

SAMPLE_FRAMES = 5      # 每次采样帧数
GAIN = 1.1             # 增益
TOL_MM = 3.0           # 达标阈值
MAX_STEP_MM = 12.0     # 单步任一分量限幅
MAX_ITERS = 4          # 最大迭代
MAX_INIT_TRANS_MM = 50.0   # 起始偏差超此值视为异常，拒绝纠偏
SETTLE_S = 0.15        # OFFSET 后再采样的稳定等待


def read_delta(backend, vision, relation):
    boards, ends = [], []
    start = backend.snapshot()
    from g2_tag_placement.controller import G2AprilTagPlacement
    for _ in range(SAMPLE_FRAMES):
        fr = backend.capture()
        G2AprilTagPlacement._unchanged(start, fr['snapshot'])
        st = fr['timestamp_ns']
        ob = vision.observe(fr['image'], timestamp_ns=st,
                            base_T_camera=fr['base_T_camera'],
                            base_T_camera_timestamp_ns=st)
        if not ob['diagnostic_pass']:
            raise PlacementError('VISION', ','.join(ob['rejection_reasons']))
        boards.append(np.array(ob['base_T_board_nominal']))
        ends.append(np.array(fr['base_T_right_end']))
    bd = vpc._median_pose(boards)
    en = vpc._median_pose(ends)
    goal = bd @ relation
    d = (goal[:3, 3] - en[:3, 3]) * 1000.0
    return d, float(np.linalg.norm(d))


def clamp(rx, ry, rz):
    m = max(abs(rx), abs(ry), abs(rz))
    if m > MAX_STEP_MM:
        s = MAX_STEP_MM / m
        return rx * s, ry * s, rz * s
    return rx, ry, rz


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--reference', type=Path, required=True)
    ap.add_argument('--relax-gripper', action='store_true',
                    help='取料空爪：放宽夹爪闭合检查；放料抓料时不要加')
    args = ap.parse_args()

    if not args.reference.exists():
        print('VISION_ALIGN_RESULT ' + json.dumps(
            dict(status='no_reference', message=f'缺参考 {args.reference}，请当前开机 teach'),
            ensure_ascii=False))
        return 0
    ref = json.loads(args.reference.read_text())
    relation = np.array(ref['board_T_right_end_reference'])
    board = vpc._load_board()
    vpc._patch_fresh_tolerance()        # 容忍几毫秒时钟抖动，防偶发 STALE 负 age
    if args.relax_gripper:
        vpc._patch_gripper_for_pick()   # 取料空爪才放宽

    t0 = time.time()
    g = Minth.G2()
    try:
        with GDKBackend(board, unused_pedal_fault_confirmed=True) as backend:
            vision = AprilTagVision(**backend.vision_config())
            if any(ref.get(k) != v for k, v in vision._identity.items()):
                print('VISION_ALIGN_RESULT ' + json.dumps(
                    dict(status='identity_mismatch', message='相机/码板/开机身份与参考不符，请重新 teach'),
                    ensure_ascii=False))
                return 0
            prev = None
            iters = 0
            for it in range(1, MAX_ITERS + 1):
                d, dist = read_delta(backend, vision, relation)
                print(f"[align] 第{it}次 x={d[0]:.2f} y={d[1]:.2f} z={d[2]:.2f} 总{dist:.2f}mm "
                      f"({time.time()-t0:.1f}s)", file=sys.stderr)
                if it == 1 and dist > MAX_INIT_TRANS_MM:
                    print('VISION_ALIGN_RESULT ' + json.dumps(
                        dict(status='exceeds_limit', translation_mm=dist,
                             message=f'起始偏差 {dist:.1f}mm 超限，回退固定点位'), ensure_ascii=False))
                    return 0
                if dist <= TOL_MM:
                    print('VISION_ALIGN_RESULT ' + json.dumps(
                        dict(status='aligned', iterations=iters, final_mm=dist,
                             seconds=round(time.time()-t0, 2)), ensure_ascii=False))
                    return 0
                if prev is not None and dist > prev + 1.0:
                    print('VISION_ALIGN_RESULT ' + json.dumps(
                        dict(status='diverged_stopped', iterations=iters, final_mm=dist,
                             message='残差不降反升，停止，按当前位置继续'), ensure_ascii=False))
                    return 0
                prev = dist
                rx, ry, rz = clamp(d[0]*GAIN, d[1]*GAIN, d[2]*GAIN)
                cmd = {'rx': round(rx, 3), 'ry': round(ry, 3), 'rz': round(rz, 3)}
                print(f"[align] OFFSET {cmd}", file=sys.stderr)
                if not g.OFFSET(cmd):
                    print('VISION_ALIGN_RESULT ' + json.dumps(
                        dict(status='offset_failed', iterations=iters,
                             message='OFFSET 未确认到位，停止'), ensure_ascii=False))
                    return 0
                iters += 1
                time.sleep(SETTLE_S)
            d, dist = read_delta(backend, vision, relation)
            print('VISION_ALIGN_RESULT ' + json.dumps(
                dict(status='max_iters', iterations=iters, final_mm=dist,
                     seconds=round(time.time()-t0, 2)), ensure_ascii=False))
            return 0
    except Exception as exc:
        print('VISION_ALIGN_RESULT ' + json.dumps(
            dict(status='failed', error=str(exc), code=getattr(exc, 'code', 'ERROR')),
            ensure_ascii=False), file=sys.stderr)
        return 2
    finally:
        g.close()


if __name__ == '__main__':
    raise SystemExit(main())
