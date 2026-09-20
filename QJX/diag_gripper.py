#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
diag_gripper.py —— 诊断独立进程为何闭合不生效（右爪空爪，只读+受控小动作）。

打印每次下发的【返回码】和位置变化，对比：
  A) move_ee_pos 只给 position（服务端 _control_gripper 的方式）
  B) move_ee_pos 给 position + velocity
  C) move_end_effector_joint(positions, velocities, group)
判断哪种方式能真正让右爪闭合，以及是否需要先 set_control_mode / joint_control_request。
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
    except Exception as e:
        print("  read err:", e)
        return None


def send_ee_pos(robot, side, position, velocity=None):
    jss = g.JointStates()
    jss.group = f"{side}_tool"
    jss.target_type = "omnipicker"
    js = g.JointState()
    js.position = position
    if velocity is not None:
        js.velocity = velocity
    jss.states = [js]
    jss.nums = 1
    return robot.move_ee_pos(jss)


def watch(robot, side, tag, secs=1.5):
    t0 = time.time()
    while time.time() - t0 < secs:
        time.sleep(0.1)
    p = read_pos(robot, side)
    print(f"  [{tag}] {secs}s 后位置 = {p}")
    return p


def main():
    if g.gdk_init() != g.GDKRes.kSuccess:
        print("gdk_init 失败"); return 1
    robot = g.Robot()
    time.sleep(2)
    side = "right"
    try:
        try:
            st = robot.get_motion_control_status()
            print("control_mode =", int(st.control_mode))
        except Exception as e:
            print("读 control_mode 失败:", e)

        print("起始位置:", read_pos(robot, side))

        # 先张开，确保有可闭合的行程
        print("\n[准备] 张开")
        rc = send_ee_pos(robot, side, OPEN_POS)
        print("  move_ee_pos(open) rc =", rc)
        watch(robot, side, "open")

        print("\n[A] move_ee_pos 只给 position 闭合")
        rc = send_ee_pos(robot, side, CLOSE_POS)
        print("  rc =", rc)
        watch(robot, side, "A")

        print("\n[准备] 再张开")
        send_ee_pos(robot, side, OPEN_POS); watch(robot, side, "open2")

        print("\n[B] move_ee_pos position+velocity=2.0 闭合")
        rc = send_ee_pos(robot, side, CLOSE_POS, velocity=2.0)
        print("  rc =", rc)
        watch(robot, side, "B")

        print("\n[准备] 再张开")
        send_ee_pos(robot, side, OPEN_POS); watch(robot, side, "open3")

        print("\n[C] move_end_effector_joint([pos],[vel],'right_tool') 闭合")
        try:
            rc = robot.move_end_effector_joint([CLOSE_POS], [2.0], "right_tool")
            print("  rc =", rc)
        except Exception as e:
            print("  move_end_effector_joint 异常:", e)
        watch(robot, side, "C")

        print("\n最终位置:", read_pos(robot, side))
    finally:
        g.gdk_release()
    return 0


if __name__ == "__main__":
    main()
