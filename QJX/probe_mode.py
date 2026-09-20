#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""probe_mode.py —— 只读：像视觉子进程那样独立 gdk_init 后读 control_mode / error_code。
不发任何运动指令，机器人不动。用于确认视觉纠偏子进程此刻能否看到 control_mode==1。"""
import time
import agibot_gdk as g

def main():
    rc = g.gdk_init()
    print("gdk_init:", rc)
    if rc != g.GDKRes.kSuccess:
        return 1
    try:
        robot = g.Robot()
        time.sleep(2)
        for i in range(5):
            st = robot.get_motion_control_status()
            print(f"[{i}] control_mode={int(st.control_mode)}  error_code={int(st.error_code)}")
            time.sleep(0.4)
    finally:
        g.gdk_release()
    return 0

if __name__ == "__main__":
    main()
