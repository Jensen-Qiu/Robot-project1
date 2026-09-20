#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
diag_mode_gripper.py —— 验证夹爪在不同 control_mode 下能否响应（右爪空爪）。
步骤：读现状 -> mode1 下合爪并读回 -> 切 mode3 合爪读回 -> mode3 张爪读回。
每步打印 control_mode 与爪位，用数据判定夹爪需要哪个模式。
安全：仅右爪空爪张合。
"""
import time
import agibot_gdk as g

OPEN_POS = -0.78
CLOSE_POS = -0.05

def pos(r):
    es = r.get_end_state()
    rs = (es.get('right_end_state') or {}).get('end_states') or []
    return rs[0].get('position') if rs else None

def mode(r):
    return int(r.get_motion_control_status().control_mode)

def send(r, p):
    jss = g.JointStates(); jss.group="right_tool"; jss.target_type="omnipicker"
    js = g.JointState(); js.position = p; jss.states=[js]; jss.nums=1
    return r.move_ee_pos(jss)

def step(r, target, want_mode, tag):
    if mode(r) != want_mode:
        print(f"  切 control_mode -> {want_mode}: rc=", r.set_control_mode(want_mode)); time.sleep(1.0)
    p0 = pos(r)
    rc = send(r, target)
    time.sleep(2.0)
    p1 = pos(r)
    print(f"[{tag}] mode={mode(r)} 目标={target} rc={rc}  爪位 {p0} -> {p1}  {'动了' if p0 is not None and p1 is not None and abs(p1-p0)>0.05 else '没动'}")

def main():
    g.gdk_init(); r=g.Robot(); time.sleep(2)
    try:
        print(f"起始 mode={mode(r)} 爪位={pos(r)}")
        step(r, CLOSE_POS, 1, "A: mode1 合爪")
        step(r, CLOSE_POS, 3, "B: mode3 合爪")
        step(r, OPEN_POS, 3, "C: mode3 张爪")
        step(r, CLOSE_POS, 3, "D: mode3 合爪")
    finally:
        g.gdk_release()

if __name__ == "__main__":
    main()
