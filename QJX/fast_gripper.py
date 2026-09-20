#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fast_gripper.py —— 快速夹爪封装（QJX 提速架构）

背景
----
服务端 commands.py 的 handle_grab 到位判定为：下发后每 0.1s 读位置，只要
abs(实际-目标) > 0.05 就判"未到位"并【重发命令】，最长等 GRIPPER_CHECK_TIMEOUT=6s。
问题：空爪张到底（实际停在 ~-0.77）或抓料闭合被料挡住时，实际位置永远到不了
名义目标，导致反复重发、空耗接近 6s。实测夹爪电机开合本身只需 ~0.2-0.7s。

本封装（不改任何现有文件）
--------------------------
用 agibot_gdk 直接 move_ee_pos 下发，到位判定改为【位置稳定即完成】：
  - 下发一次目标位；
  - 持续以较高频率读位置，当连续若干次读数变化很小（爪子已张/合到底或夹住料）
    即判定完成；
  - 短超时兜底（默认 1.2s），不再等 6s，也不反复重发。

闭合抓料安全检查
----------------
闭合（正向/抓料）时可传 expect_grip=True，额外确认"位置确实向闭合方向移动且已稳定"，
避免为了快而没夹住。张开不需要该检查。

与手臂库共存
------------
本库同样用 agibot_gdk。若与 FastJointMover 在同一进程使用，请只 gdk_init 一次：
构造时传 own_gdk=False 并复用外部已创建的 robot 对象。
"""
import time

import agibot_gdk

OPEN_DEFAULT = -0.78
CLOSE_DEFAULT = -0.05


class FastGripper:
    def __init__(self, robot=None, own_gdk=False):
        """
        robot   : 复用外部已创建的 agibot_gdk.Robot()；为 None 则自建。
        own_gdk : True 表示本对象负责 gdk_init/gdk_release（独立使用时）。
        """
        self._own_gdk = own_gdk
        if own_gdk:
            if agibot_gdk.gdk_init() != agibot_gdk.GDKRes.kSuccess:
                raise RuntimeError("gdk_init 失败")
            time.sleep(2)
        self.robot = robot or agibot_gdk.Robot()

    def _read_pos(self, side):
        try:
            es = self.robot.get_end_state()
            states = (es.get(f"{side}_end_state") or {}).get("end_states") or []
            if not states:
                return None
            return float(states[0].get("position", 0.0))
        except Exception:
            return None

    def _send(self, side, position):
        jss = agibot_gdk.JointStates()
        jss.group = f"{side}_tool"
        jss.target_type = "omnipicker"
        js = agibot_gdk.JointState()
        js.position = position
        jss.states = [js]
        jss.nums = 1
        self.robot.move_ee_pos(jss)

    def move(self, side, position, *, expect_grip=False,
             timeout=1.5, poll=0.03, stable_delta=0.01, stable_reads=3,
             min_wait=0.25, min_move=0.03, near_tol=0.08):
        """下发夹爪到 position，按"确实动起来后再稳定"判定完成。

        side        : "left" / "right"
        position    : 目标位（负=张开，正/接近0=闭合），沿用流程里的 -0.78 / -0.05
        expect_grip : True=抓料闭合，额外确认向闭合方向移动并稳定
        timeout     : 最长等待（秒），到点即返回（兜底，不再等 6s）
        stable_delta: 连续读数变化小于该值视为"不再动"
        stable_reads: 连续多少次"不再动"才判完成
        min_wait    : 下发后至少观测这么久再允许判完成（给电机启动时间，
                      避免"还没动就被判稳定"导致命令被秒过）
        min_move    : 认为"确实动过"的最小位移；未达到且未接近目标时不判完成
        near_tol    : 距目标 <= 该值视为已到位（张到底/合到料时目标可能到不了名义值）

        返回 dict：{done, start, final, elapsed, note}
        """
        start = self._read_pos(side)
        t0 = time.time()
        self._send(side, position)

        last = start
        good = 0
        final = start
        moved_enough = False
        while time.time() - t0 < timeout:
            time.sleep(poll)
            cur = self._read_pos(side)
            if cur is None:
                continue
            final = cur
            # 是否已"确实动过"或"已接近目标"
            if start is not None:
                if abs(cur - start) >= min_move:
                    moved_enough = True
            if abs(cur - position) <= near_tol:
                moved_enough = True
            # 稳定计数
            if last is not None and abs(cur - last) < stable_delta:
                good += 1
            else:
                good = 0
            last = cur
            # 必须：过了最小启动观测期 + 确实动过/接近目标 + 连续稳定
            if (time.time() - t0) >= min_wait and moved_enough and good >= stable_reads:
                break

        elapsed = time.time() - t0
        if not moved_enough:
            note = "no_move"     # 命令似乎未生效（没动、也没接近目标）
        elif good >= stable_reads:
            note = "stable"
        else:
            note = "timeout"

        # 抓料闭合安全检查
        if expect_grip and start is not None and final is not None:
            moved = final - start  # 闭合方向通常是 position 增大（更接近 0）
            if moved < min_move and abs(final - position) > 0.15:
                note = "grip_warn"  # 几乎没动且离目标远，可能没夹到
        return {"done": note in ("stable",), "start": start, "final": final,
                "elapsed": elapsed, "note": note}

    def open(self, side="right", position=OPEN_DEFAULT, **kw):
        r = self.move(side, position, expect_grip=False, **kw)
        print(f"[grip] {side} 张开 -> {position}: {r['note']}, "
              f"{r['start']}→{r['final']}, {r['elapsed']:.2f}s")
        return r

    def close(self, side="right", position=CLOSE_DEFAULT, expect_grip=True, **kw):
        r = self.move(side, position, expect_grip=expect_grip, **kw)
        print(f"[grip] {side} 闭合 -> {position}: {r['note']}, "
              f"{r['start']}→{r['final']}, {r['elapsed']:.2f}s")
        return r

    def release(self):
        if self._own_gdk:
            try:
                agibot_gdk.gdk_release()
            except Exception:
                pass
