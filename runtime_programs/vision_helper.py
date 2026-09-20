#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""视觉纠偏封装：供主程序（Minth）在送料机取料前调用。

这一层做的事：
  * 以【独立子进程】方式运行 vision_pick_correction.py，拿到毫米级偏移。
  * 解析结果，做安全判断；任何异常/超限/无参考都返回一个「安全的空结果」，
    让主程序自行决定回退到原固定点位流程。
  * 本模块自身不 import 视觉包、不碰 agibot_gdk，因此不会与 Minth 抢机械臂。

用法（主程序里）：
    from vision_helper import get_pick_offset
    offset = get_pick_offset()          # 返回 dict 或 None
    if offset is not None:
        robot.OFFSET(offset)            # 由主程序决定是否下发
"""
import json
import subprocess
import sys
from pathlib import Path

# 子进程脚本与参考文件位置
_HERE = Path(__file__).resolve().parent
SUBPROCESS = _HERE / 'vision_pick_correction.py'
REFERENCE = _HERE / 'pick_reference.json'

# 加载 SDK 环境的脚本；子进程需要它才能连相机
ENV_SETUP = '/home/agi/app/env.sh'
ENV_ARG = '/home/agi/app'

# 单轮视觉超时（秒）：拍 20 帧 + 解算，给足余量
TIMEOUT_S = 40


def _run(mode):
    """在加载 SDK 环境的子进程里运行视觉脚本，返回解析后的 dict 或 None。"""
    # 用 bash -lc 先 source 环境再运行，保证 agibot_gdk 可用
    cmd = (f'source {ENV_SETUP} {ENV_ARG} >/dev/null 2>&1; '
           f'python3 {SUBPROCESS} {mode} --reference {REFERENCE}')
    try:
        proc = subprocess.run(['bash', '-lc', cmd], capture_output=True,
                              text=True, timeout=TIMEOUT_S)
    except subprocess.TimeoutExpired:
        print('[vision] 子进程超时，视觉纠偏放弃，回退固定点位')
        return None
    # 结果在 stdout 或 stderr 里，找 VISION_PICK_RESULT 那一行
    for stream in (proc.stdout, proc.stderr):
        if not stream:
            continue
        for line in stream.splitlines():
            if line.startswith('VISION_PICK_RESULT '):
                try:
                    return json.loads(line[len('VISION_PICK_RESULT '):])
                except json.JSONDecodeError:
                    return None
    print(f'[vision] 未解析到结果 (rc={proc.returncode})')
    if proc.stderr:
        print('[vision] stderr:', proc.stderr.strip()[:300])
    return None


def teach_pick_reference():
    """现场一次性：在取料位、已抓稳型材、姿态正确时生成当前开机的取料参考。
    返回 True=已保存，False=失败。
    """
    result = _run('teach')
    if result is None:
        print('[vision] 示教失败：无结果')
        return False
    if result.get('status') == 'teach_saved':
        print(f"[vision] 取料参考已保存：{result.get('reference_path')}")
        print(f"[vision] 末端位姿：{result.get('right_end_pose')}")
        return True
    print(f"[vision] 示教失败：{result.get('status')} {result.get('message') or result.get('error')}")
    return False


def get_pick_offset(axis_map=None):
    """取料前调用，返回可直接传给 Minth.OFFSET 的偏移 dict，或 None（回退）。

    参数 axis_map：base 坐标系 x/y/z 毫米偏移 → Minth OFFSET 键的映射函数。
        默认 None 时【不做映射、返回原始 base 偏移诊断】，主程序应先在现场
        标定坐标对应关系后再启用真正的 OFFSET 下发。这样避免坐标系猜错。

    返回：
        None                         —— 视觉不可用/超限/无参考，主程序回退固定点位
        {'_preview': {...}}          —— 预览模式（axis_map=None），只给数值不给可下发偏移
        {'lx':.., 'ly':.., 'lz':..}  —— 已映射、可直接传 OFFSET 的偏移（axis_map 提供时）
    """
    result = _run('correct')
    if result is None:
        return None
    status = result.get('status')
    if status == 'no_reference':
        print(f"[vision] {result.get('message')}")
        return None
    if status == 'identity_mismatch':
        print(f"[vision] {result.get('message')}")
        return None
    if status == 'correction_exceeds_limit':
        print(f"[vision] 偏移超限 {result.get('translation_distance_mm'):.1f}mm / "
              f"{result.get('rotation_deg'):.2f}deg，放弃纠偏，回退固定点位")
        return None
    if status != 'correction_ok':
        print(f"[vision] 视觉失败：{status} {result.get('error') or ''}")
        return None
    delta = result['delta_base_mm']
    print(f"[vision] base 偏移 x={delta['x']:.2f} y={delta['y']:.2f} z={delta['z']:.2f} mm "
          f"(总 {result['translation_distance_mm']:.2f}mm, 旋转 {result['rotation_deg']:.2f}deg)")
    if axis_map is None:
        # 预览模式：不下发，只把诊断值交回主程序打印
        return {'_preview': delta}
    return axis_map(delta)


if __name__ == '__main__':
    # 命令行自测：vision_helper.py teach | correct
    mode = sys.argv[1] if len(sys.argv) > 1 else 'correct'
    if mode == 'teach':
        teach_pick_reference()
    else:
        print('结果:', get_pick_offset())
