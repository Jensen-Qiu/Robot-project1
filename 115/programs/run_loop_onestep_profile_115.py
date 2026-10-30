#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_loop_onestep_profile_115.py —— 带细化节拍记录地运行 onestep 主控。

做的事：
  1. 导入 all_machin_plc_vision_loop_onestep 作为主控模块（不改其任何逻辑）；
  2. [WF2 垂直下放拦截] 当 VISION_USE_SERVER=1 时，把 G2.WBC("WF2") 运行时替换为
     "向视觉常驻服务发 wf_drop 请求 → 在纠偏后姿态基础上垂直下放固定落差"，
     代替 WF2 关节插值（斜向下致料条翻转）。其它 WBC 名字照常。失败自动降级执行原 WF2。
     —— 主程序源码一字不改，纯运行时 monkey-patch。
  3. 用 OnestepProfiler 记录节拍（patch 在 WF2 拦截之外层，记录到的即下放真实耗时）；
  4. 退出时 finalize 落 CSV；调用主控 main() 进入 PLC 循环。

节拍数据落 logs/cycle_detail_115.csv 与 logs/cycle_summary_115.csv。
"""
import atexit
import functools
import json
import os
import socket
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.dirname(HERE))

import all_machin_plc_vision_loop_onestep as mainmod  # noqa: E402
from qjx_profiler_onestep_115 import OnestepProfiler  # noqa: E402

_VISION_SOCK_PATH = '/tmp/vision_align_115.sock'


# 需做垂直下放的工位：原关节下压动作名 → 工位标识
_DROP_STATIONS = {"WF2": "WF", "XL2": "XL"}


def _drop_via_socket(station, timeout=60.0):
    """向视觉常驻服务发 drop 请求（指定工位），返回 rc（0 成功 / 2 未执行需降级 / None 连不上）。"""
    if not os.path.exists(_VISION_SOCK_PATH):
        return None
    try:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(timeout)
        s.connect(_VISION_SOCK_PATH)
        s.sendall((json.dumps({"cmd": "drop", "station": station}) + "\n").encode("utf-8"))
        buf = b''
        while b'\n' not in buf:
            chunk = s.recv(65536)
            if not chunk:
                break
            buf += chunk
        s.close()
        if not buf:
            return None
        resp = json.loads(buf.split(b'\n', 1)[0].decode('utf-8'))
        info = resp.get('info', '')
        print(f"[drop-intercept] {station} drop rc={resp.get('rc')} ({info})", flush=True)
        return int(resp.get('rc', 2))
    except Exception as e:
        print(f"[drop-intercept] {station} drop socket 异常({e})", flush=True)
        return None


def _install_drop_intercept():
    """把 G2.WBC 包一层：动作名是 WF2/XL2 时改为对应工位的垂直下放；其它照常。
    仅在 VISION_USE_SERVER=1 时启用（与视觉常驻一致）；否则不拦截，保持原关节动作。
    任何失败（服务连不上/rc!=0）都降级执行原关节动作，保证安全不漏放。
    """
    if os.environ.get('VISION_USE_SERVER', '') != '1':
        print("[drop-intercept] 未开启常驻(VISION_USE_SERVER!=1)，不拦截，保持原 WF2/XL2 关节动作", flush=True)
        return
    g2 = mainmod.G2
    orig_wbc = g2.WBC

    @functools.wraps(orig_wbc)
    def wbc_wrapper(name, *a, **k):
        station = _DROP_STATIONS.get(name)
        if station is not None:
            print(f"[drop-intercept] 拦截 WBC('{name}') → {station} 垂直下放固定落差（保持纠偏姿态）",
                  flush=True)
            rc = _drop_via_socket(station)
            if rc == 0:
                return True   # 垂直下放成功，等价原关节动作完成
            # rc=2(未标定/失败) 或 None(服务连不上) → 降级执行原关节动作，保证放料
            print(f"[drop-intercept] {station} drop 未成功(rc={rc})，降级执行原 WBC('{name}')", flush=True)
            return orig_wbc(name, *a, **k)
        return orig_wbc(name, *a, **k)

    g2.WBC = wbc_wrapper
    print("[drop-intercept] 已安装 WF2/XL2→垂直下放 拦截", flush=True)


def _install_profiler():
    prof = OnestepProfiler(main_module=mainmod, g2_instance=mainmod.G2)
    prof.install()
    atexit.register(prof.finalize)
    return prof


def main():
    print("[onestep-profile] 启动 onestep 主控 + 细化节拍记录", flush=True)
    # 顺序：先装 WF2/XL2 拦截（替换为垂直下放），再装 profiler（记录墙钟，包在外层）。
    _install_drop_intercept()
    _install_profiler()
    mainmod.main()


if __name__ == "__main__":
    main()
