#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
diag_gripper2.py —— 慢速分步观察右爪是否物理动作（右爪空爪）。
每步下发后停 3.5s，便于肉眼观察。打印返回码与读回位置。
"""
import time
import agibot_gdk as g

OPEN_POS = -0.78
CLOSE_POS = -0.05


def read_pos(robot, side="right"):
    try:
        es = robot.get_end_state()
        st = (es.get(f"{side}_end_state") or {}).get("end_states") or []
        return float(st[0].get("position", 0.0)) if st else None
    except Exception:
        return None


def send(robot, side, position):
    jss = g.JointStates()
    jss.group = f"{side}_tool"
    jss.target_type = "omnipicker"
    js = g.JointState()
    js.position = position
    jss.states = [js]
    jss.nums = 1
    return robot.move_ee_pos(jss)


def step(robot, side, position, tag):
    print(f"\n>>> 即将 {tag} (目标 {position})，请盯住右爪 ...")
    time.sleep(1.5)
    rc = send(robot, side, position)
    print(f"    已下发, rc={rc}")
    for i in range(7):
        time.sleep(0.5)
        print(f"    +{(i+1)*0.5:.1f}s 位置={read_pos(robot, side)}")


def main():
    if g.gdk_init() != g.GDKRes.kSuccess:
        print("gdk_init 失败"); return 1
    robot = g.Robot()
    time.sleep(2)
    side = "right"
    try:
        st = robot.get_motion_control_status()
        print("control_mode =", int(st.control_mode))
        print("起始位置 =", read_pos(robot, side))
        step(robot, side, CLOSE_POS, "闭合(合爪)")
        step(robot, side, OPEN_POS, "张开(开爪)")
        step(robot, side, CLOSE_POS, "再闭合(合爪)")
        print("\n最终位置 =", read_pos(robot, side))
    finally:
        g.gdk_release()
    return 0


if __name__ == "__main__":
    main()
