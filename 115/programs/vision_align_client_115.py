#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vision_align_client_115.py —— 视觉纠偏常驻服务的【客户端】（替代直调 vision_align_safe_115.py）

与 vision_align_safe_115.py 保持【完全相同的命令行契约】，供主控 _run_align_onestep
无差别调用：相同的 --reference / --execute-align / --one-step-verify / --rot-comp /
--xy-tol-mm / --zero-z 参数，相同的退出码（0 对准 / 2 未对准但安全 / 1,3 危险异常），
相同的 stdout `VISION_ALIGN_SAFE_115_RESULT <json>` 输出。

行为：
  1. 连接常驻服务 /tmp/vision_align_115.sock，发请求、收结果，按 rc 退出、原样打印 RESULT；
  2. 若服务未启动 / 连接失败 / 超时 —— 自动【降级】为原方式直接执行
     vision_align_safe_115.py（subprocess），保证不比现状差，绝不中断产线。

用法（与原脚本一致，仅脚本名不同）：
  python3 vision_align_client_115.py --reference <ref.json> --execute-align --one-step-verify [--rot-comp] [--xy-tol-mm 12] [--zero-z]
"""
import argparse
import json
import os
import socket
import subprocess
import sys
from pathlib import Path

PROGRAMS_ROOT = Path('/data/wxf0721/runtime/programs')
SOCK_PATH = '/tmp/vision_align_115.sock'
FALLBACK_SCRIPT = str(PROGRAMS_ROOT / 'vision_align_safe_115.py')
CONNECT_TIMEOUT = 3.0
REQUEST_TIMEOUT = 200.0


def _emit_result(result):
    if result is not None:
        print('VISION_ALIGN_SAFE_115_RESULT ' + json.dumps(result, ensure_ascii=False),
              flush=True)


def _station_from_ref(ref_name):
    n = ref_name.lower()
    if 'sl1' in n:
        return 'SL1'
    if 'wq' in n:
        return 'WQ'
    if 'xl' in n:
        return 'XL'
    if 'place' in n:
        return 'WF'
    return '?'


def _via_server(args):
    """返回 rc；连接/通信失败抛异常由上层决定降级。"""
    req = {
        'reference': str(Path(args.reference).resolve()),
        'rot_comp': bool(args.rot_comp),
        'xy_tol_mm': args.xy_tol_mm,
        'zero_z': bool(args.zero_z),
        'one_step_verify': bool(args.one_step_verify),
        'station': _station_from_ref(Path(args.reference).name),
    }
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(CONNECT_TIMEOUT)
    s.connect(SOCK_PATH)
    s.settimeout(REQUEST_TIMEOUT)
    s.sendall((json.dumps(req, ensure_ascii=False) + '\n').encode('utf-8'))
    data = b''
    while b'\n' not in data:
        chunk = s.recv(65536)
        if not chunk:
            break
        data += chunk
    s.close()
    if not data:
        raise RuntimeError('服务无响应（空回包）')
    resp = json.loads(data.split(b'\n', 1)[0].decode('utf-8'))
    # 转发服务端 stderr 日志（含 [safe-align115] 行），便于主控日志一致
    stderr_text = resp.get('stderr') or ''
    if stderr_text:
        sys.stderr.write(stderr_text)
        sys.stderr.flush()
    _emit_result(resp.get('result'))
    rc = int(resp.get('rc', 2))
    print(f'[vision-client115] 经常驻服务完成 rc={rc}', flush=True)
    return rc


def _via_fallback(argv):
    """降级：直接 subprocess 执行原脚本，原样透传 stdout/stderr 与退出码。"""
    print('[vision-client115] 常驻服务不可用，降级为直调 vision_align_safe_115.py', flush=True)
    cmd = [sys.executable, FALLBACK_SCRIPT] + argv
    proc = subprocess.run(cmd, text=True)
    return proc.returncode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference', type=Path, required=True)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--execute-align', action='store_true')
    group.add_argument('--execute-test', action='store_true')
    parser.add_argument('--inject-rx', type=float, default=5.0)
    parser.add_argument('--inject-ry', type=float, default=-5.0)
    step_group = parser.add_mutually_exclusive_group()
    step_group.add_argument('--one-step', action='store_true')
    step_group.add_argument('--one-step-verify', action='store_true')
    parser.add_argument('--one-step-gain', type=float, default=None)
    parser.add_argument('--zero-z', action='store_true')
    parser.add_argument('--xy-tol-mm', type=float, default=None)
    parser.add_argument('--rot-comp', action='store_true')
    args = parser.parse_args()

    # 原样保留 argv（降级透传用）
    argv = sys.argv[1:]

    # 服务只处理生产路径（--execute-align + one-step-verify）。
    # 其它模式（test/读基线/多轮闭环）一律走降级原脚本，避免契约偏差。
    server_eligible = (
        args.execute_align and args.one_step_verify
        and not args.execute_test and not args.one_step
    )

    if server_eligible and os.path.exists(SOCK_PATH):
        try:
            return _via_server(args)
        except Exception as e:
            print(f'[vision-client115] 服务调用失败({e})，降级', flush=True)
            return _via_fallback(argv)
    else:
        return _via_fallback(argv)


if __name__ == '__main__':
    raise SystemExit(main())
