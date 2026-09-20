#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
probe_gripper_speed.py —— 夹爪测速（右爪空爪）。

目的：
  1. 测出一次张开/闭合从下发到位置稳定的真实耗时；
  2. 对比"只给 position（默认速度）"与"额外给 velocity（尝试提速）"两种下发方式，
     判断 omnipicker 夹爪是否支持速度控制、设 velocity 是否真的更快。

安全：仅右爪空爪张合，幅度用流程里的值（张开 -0.78 / 闭合 -0.05）。
判读：如果两种方式耗时相近 → velocity 无效，慢在电机固定速度或到位判定；
      如果 with_vel 明显更快 → 可通过 velocity 提速。
"""
import sys
import time

import agibot_gdk

OPEN_POS = -0.78    # 张开
CLOSE_POS = -0.05   # 闭合
SETTLE_TOL = 0.02   # 位置稳定判定：两次读数差 < 该值
POLL = 0.02         # 读位置周期
MAX_WAIT = 6.0      # 单次动作最长观测


def read_pos(robot, side="right"):
    try:
        es = robot.get_end_state()
        states = (es.get(f"{side}_end_state") or {}).get("end_states") or []
        if not states:
            return None
        return float(states[0].get("position", 0.0))
    except Exception:
        return None


def send(robot, side, position, velocity=None):
    jss = agibot_gdk.JointStates()
    jss.group = f"{side}_tool"
    jss.target_type = "omnipicker"
    js = agibot_gdk.JointState()
    js.position = position
    if velocity is not None:
        js.velocity = velocity
    jss.states = [js]
    jss.nums = 1
    robot.move_ee_pos(jss)


def timed_move(robot, side, position, velocity=None):
    """下发后持续读位置，直到稳定或超时，返回 (到位耗时, 起始位, 终止位)。"""
    start_pos = read_pos(robot, side)
    t0 = time.time()
    send(robot, side, position, velocity=velocity)
    last = start_pos
    stable_since = None
    reached_t = None
    while time.time() - t0 < MAX_WAIT:
        time.sleep(POLL)
        cur = read_pos(robot, side)
        if cur is None:
            continue
        # 接近目标即记到位时间
        if reached_t is None and abs(cur - position) <= 0.05:
            reached_t = time.time() - t0
        # 位置稳定（连续不变）判定
        if last is not None and abs(cur - last) < SETTLE_TOL:
            if stable_since is None:
                stable_since = time.time()
            elif time.time() - stable_since >= 0.15:
                break
        else:
            stable_since = None
        last = cur
    settle_t = time.time() - t0
    return reached_t, settle_t, start_pos, last


def main():
    if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
        print("[grip] gdk_init 失败")
        return 1
    ok = 0
    try:
        robot = agibot_gdk.Robot()
        time.sleep(2)
        print(f"[grip] 右爪当前位置: {read_pos(robot)}")

        print("\n===== 默认速度（只给 position）=====")
        r, s, p0, p1 = timed_move(robot, "right", OPEN_POS)
        print(f"[grip] 张开: 到位≈{r}s, 稳定≈{s:.2f}s ({p0} -> {p1})")
        time.sleep(0.5)
        r, s, p0, p1 = timed_move(robot, "right", CLOSE_POS)
        print(f"[grip] 闭合: 到位≈{r}s, 稳定≈{s:.2f}s ({p0} -> {p1})")

        time.sleep(0.8)
        print("\n===== 尝试提速（position + velocity=3.0）=====")
        r, s, p0, p1 = timed_move(robot, "right", OPEN_POS, velocity=3.0)
        print(f"[grip] 张开: 到位≈{r}s, 稳定≈{s:.2f}s ({p0} -> {p1})")
        time.sleep(0.5)
        r, s, p0, p1 = timed_move(robot, "right", CLOSE_POS, velocity=3.0)
        print(f"[grip] 闭合: 到位≈{r}s, 稳定≈{s:.2f}s ({p0} -> {p1})")

        print("\n[grip] 提示：对比两组'稳定'耗时。相近=velocity无效；提速组更小=velocity有效。")
    except Exception as e:
        print(f"[grip] 异常: {e}")
        ok = 1
    finally:
        try:
            agibot_gdk.gdk_release()
        except Exception:
            pass
    return ok


if __name__ == "__main__":
    sys.exit(main())
