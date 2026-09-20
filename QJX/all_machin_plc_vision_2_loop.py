#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
all_machin_plc_vision_2_loop.py —— QJX 循环版主程序

在 all_machin_plc_vision_2_fast.py（提速版）基础上加【连续循环】：
每完成一轮完整流程（送料机取料 → 弯曲机放料 → 弯曲机取料 → 下料机放料）后，
自动开始下一轮取料，无需重启程序。

特点
----
- 前台运行，随时 Ctrl+C 手动中断，中断后干净停机（复位信号、释放 GDK）。
- 【不修改任何现有文件】：直接 import 复用 all_machin_plc_vision_2_fast.py 的
  全部对象与函数（Minth 对象 G2/G2_wd/G2_hb、WBC 提速封装、视觉纠偏、看门狗、
  心跳、握手信号等），本文件只重新组织 main 为循环版。
- 初始化（gdk_init/MOVER、看门狗线程、心跳线程）只做一次；任务体按轮循环。

用法
----
    source /home/agi/app/env.sh /home/agi/app
    cd /data/QJX
    python3 all_machin_plc_vision_2_loop.py
（或用启动脚本 run_loop.sh，日志进 /data/QJX/logs/）
"""
import os
import sys
import time
import threading

sys.path.insert(0, "/data/QJX")

# 复用提速版主程序的全部实现（对象、动作函数、看门狗、心跳、信号等）
import all_machin_plc_vision_2_fast as base
from all_machin_plc_vision_2_fast import (
    G2, comm_watchdog, enabled_heartbeat, wait_signal,
    run_pick_right_task, run_place_middle_task, _auto_rebind_references,
    FastJointMover,
)


def main_loop():
    # ── 一次性初始化 ──────────────────────────────────────
    _auto_rebind_references()
    try:
        base.MOVER = FastJointMover(init_gdk=True, load_wbc=True)
    except Exception as e:
        base.MOVER = None
        print(f"[fast] FastJointMover 初始化失败，全程回退原速 G2.WBC：{e}")

    threading.Thread(target=comm_watchdog, daemon=True).start()
    threading.Thread(target=enabled_heartbeat, daemon=True).start()

    # 就绪：通知 PLC，清除残留完成信号
    G2.setData("G2_eanbled", 1)
    G2.setData("pick_right_done", 0)
    G2.setData("place_middle_done", 0)
    G2.TTS("机器人已就绪，循环模式，等待PLC信号")

    cycle = 0
    try:
        # ── 连续循环：每轮做完自动进入下一轮 ──────────────
        while True:
            cycle += 1
            print(f"\n================  第 {cycle} 轮开始  ================")
            G2.TTS(f"第{cycle}轮，等待PLC上料信号")

            # 第一段+第二段：送料机取料 → 弯曲机放料
            wait_signal("pick_right")
            G2.setData("pick_right_done", 1)
            run_pick_right_task()
            wait_signal("pick_right_release")
            G2.setData("pick_right_done", 0)

            # 第三段+第四段：弯曲机取料 → 下料机放料
            G2.TTS("等待PLC下料信号")
            wait_signal("place_middle")
            G2.setData("place_middle_done", 1)
            run_place_middle_task()
            wait_signal("place_middle_release")
            G2.setData("place_middle_done", 0)

            print(f"================  第 {cycle} 轮完成  ================")
            G2.TTS(f"第{cycle}轮完成，准备下一轮")
            # 直接进入下一轮循环

    except KeyboardInterrupt:
        print(f"\n[loop] 收到 Ctrl+C，第 {cycle} 轮结束时手动中断，正在停机 ...")
    finally:
        base._running = False
        try:
            G2.setData("pick_right_done", 0)
            G2.setData("place_middle_done", 0)
            G2.setData("G2_eanbled", 0)
        except Exception:
            pass
        if base.MOVER is not None:
            base.MOVER.close()
        print("[loop] 已停机，信号已复位，GDK 已释放。")


if __name__ == "__main__":
    main_loop()
