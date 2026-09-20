#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_fw.py —— 单动作验证：用 FastJointMover 以 arm_speed=0.5 执行一次 FW（初始展开）。

前提：机器人空手、周围无遮挡、有人监护。
本脚本只执行 FW 一个动作并等待到位，不进入上下料流程、不动底盘、不动夹爪。
"""
import sys
import time

sys.path.insert(0, "/data/QJX")
from fast_joint_mover import FastJointMover


def main():
    print("[test] 初始化 FastJointMover（gdk_init + 加载 WBC）...")
    mover = FastJointMover(init_gdk=True, load_wbc=True)
    if not mover.has("FW"):
        print("[test] ❌ 未加载到 FW 动作，终止。")
        mover.close()
        return 1
    print("[test] → 执行 FW，arm_speed=0.5，等待到位...")
    t0 = time.time()
    ok = mover.wbc_wait("FW", arm_speed=0.5)
    dt = time.time() - t0
    print(f"[test] FW 完成={ok}，用时 {dt:.1f}s")
    mover.close()
    print("[test] 已释放 GDK。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
