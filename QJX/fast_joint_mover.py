#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fast_joint_mover.py —— 可调速关节执行库（QJX 提速架构）

背景与目的
----------
服务端 services/joints.py 中手臂速度写死 ARM_SPEED=0.2，且每个 WBC 段之间还有
time.sleep(0.2) 的固定等待，导致上下料整节拍偏慢。本库【不修改任何现有文件】，
而是另起一套等价的关节执行逻辑：用 agibot_gdk 直接下发关节运动，速度作为参数
【逐动作、逐部位】可调，从而在空手转移段提速、抓料段保持原速。

已在真机验证（QJX/probe_*.py）：
  - 服务端运行时，本进程可并存 gdk_init() 并读取 14 个手臂关节角；
  - 本进程可用自定义速度 move_arm_joint 真实驱动手臂并回位，无冲突。

动作一致性
----------
关节分组顺序、move_arm_joint 的 mode=2、move_head_joint / move_waist_joint 的调用
方式，均与服务端 joints.py 完全一致；WBC 关节角在线通过 MQTT 的 read 命令从服务端
内存数据库获取（与服务端执行 WBC 时用的是同一份数据），仅速度不同。

用法
----
    from fast_joint_mover import FastJointMover
    mover = FastJointMover()          # gdk_init + 加载 WBC 关节角
    mover.wbc("FW", arm_speed=0.5)    # 提速执行 FW
    mover.wbc("SL2", arm_speed=0.2)   # 抓料段保持原速
    mover.close()

安全
----
- 速度参数由调用方给定；建议空手转移 0.5，抓料/贴料段 0.2（保持原速）。
- 与服务端共用机械臂，请勿在服务端正在下发 WBC 的同时调用本库（主程序里两者
  本就串行，不会重叠）。
"""
import json
import time
import threading

import agibot_gdk
import paho.mqtt.client as mqtt

# ── 关节名分组（与 services/joints.py 完全一致）──────────────
HEAD_JOINT_KEYS = [
    "idx11_head_joint1", "idx12_head_joint2", "idx13_head_joint3",
]
WAIST_JOINT_KEYS = [
    "idx01_body_joint1", "idx02_body_joint2", "idx03_body_joint3",
    "idx04_body_joint4", "idx05_body_joint5",
]
LEFT_ARM_JOINT_KEYS = [
    "idx21_arm_l_joint1", "idx22_arm_l_joint2", "idx23_arm_l_joint3",
    "idx24_arm_l_joint4", "idx25_arm_l_joint5", "idx26_arm_l_joint6",
    "idx27_arm_l_joint7",
]
RIGHT_ARM_JOINT_KEYS = [
    "idx61_arm_r_joint1", "idx62_arm_r_joint2", "idx63_arm_r_joint3",
    "idx64_arm_r_joint4", "idx65_arm_r_joint5", "idx66_arm_r_joint6",
    "idx67_arm_r_joint7",
]

# 与服务端一致的默认速度（作为回退/抓料段用）
DEFAULT_HEAD_SPEED = 0.3
DEFAULT_WAIST_SPEED = 0.3
DEFAULT_ARM_SPEED = 0.2
ARM_MOVE_MODE = 2  # 与服务端 _move_both_arms 一致

# MQTT（在线拉取 WBC 关节角，与服务端同一份数据库）
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
TOPIC_SAVE = "/humanoid/joints/save"
TOPIC_DATA = "/humanoid/joints/data"


def _extract(data, keys):
    """按 keys 顺序提取关节角，缺失补 0.0（与服务端 _extract_positions 一致）。"""
    return [data.get(k, 0.0) for k in keys]


class FastJointMover:
    def __init__(self, init_gdk=True, load_wbc=True, settle_s=0.05):
        """
        Parameters
        ----------
        init_gdk : bool
            是否在本进程 gdk_init 并创建 Robot（主程序场景为 True）。
        load_wbc : bool
            是否启动时在线拉取所有 WBC 关节角。
        settle_s : float
            每个部位下发之间的停顿（秒）。服务端为 0.2；这里默认收紧到 0.05
            以去除多余等待，可按现场稳定性调整或置 0。
        """
        self._settle_s = settle_s
        self._wbc = {}

        if init_gdk:
            rc = agibot_gdk.gdk_init()
            if rc != agibot_gdk.GDKRes.kSuccess:
                raise RuntimeError(f"gdk_init 失败: {rc}")
            self.robot = agibot_gdk.Robot()
            time.sleep(2)  # 等 DDS 连接就绪
        else:
            self.robot = agibot_gdk.Robot()

        if load_wbc:
            self.reload_wbc()

    # ── 在线加载 WBC 关节角 ──────────────────────────────
    def reload_wbc(self, timeout=10.0):
        """通过 MQTT read 命令从服务端拉取全部 WBC 动作关节角。"""
        result = {"items": None}
        evt = threading.Event()

        def on_connect(client, userdata, flags, rc, properties=None):
            client.subscribe(TOPIC_DATA, qos=0)

        def on_message(client, userdata, msg):
            if msg.topic != TOPIC_DATA:
                return
            try:
                payload = json.loads(msg.payload.decode("utf-8"))
            except Exception:
                return
            if payload.get("command") == "response":
                result["items"] = payload.get("data", [])
                evt.set()

        cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="fast_joint_mover_read")
        cli.on_connect = on_connect
        cli.on_message = on_message
        cli.connect(MQTT_BROKER, MQTT_PORT)
        cli.loop_start()
        time.sleep(0.5)
        cli.publish(TOPIC_SAVE, json.dumps({"command": "read"}), qos=0)
        ok = evt.wait(timeout=timeout)
        cli.loop_stop()
        cli.disconnect()

        if not ok:
            raise RuntimeError("加载 WBC 关节角超时（服务端未返回 read 数据）")

        wbc = {}
        for it in (result["items"] or []):
            if it.get("type") == "WBC":
                wbc[it.get("name")] = it.get("value", {})
        self._wbc = wbc
        print(f"[fast] 已加载 {len(wbc)} 个 WBC 动作")
        return wbc

    def has(self, name):
        return name in self._wbc

    # ── 分部位下发（速度可调）────────────────────────────
    def _move_head(self, pos_data, speed):
        pos = _extract(pos_data, HEAD_JOINT_KEYS)
        self.robot.move_head_joint(pos, [speed] * len(pos))

    def _move_waist(self, pos_data, speed):
        pos = _extract(pos_data, WAIST_JOINT_KEYS)
        self.robot.move_waist_joint(pos, [speed] * len(pos))

    def _move_both_arms(self, pos_data, speed):
        left = _extract(pos_data, LEFT_ARM_JOINT_KEYS)
        right = _extract(pos_data, RIGHT_ARM_JOINT_KEYS)
        positions = left + right
        self.robot.move_arm_joint(positions, [speed] * len(positions), ARM_MOVE_MODE)

    # ── 对外：执行一个 WBC 动作（等价服务端 WBC，但速度可调）──
    def wbc(self, name, arm_speed=DEFAULT_ARM_SPEED,
            head_speed=DEFAULT_HEAD_SPEED, waist_speed=DEFAULT_WAIST_SPEED):
        """执行 WBC 动作 name（head→waist→arms 顺序，与服务端一致）。

        arm_speed/head_speed/waist_speed 逐动作可调。
        返回 True 表示已下发完成；找不到动作返回 False。
        """
        pos_data = self._wbc.get(name)
        if pos_data is None:
            print(f"[fast] ✗ 找不到 WBC 动作: {name}")
            return False
        print(f"[fast] → WBC {name} (arm={arm_speed}, head={head_speed}, waist={waist_speed})")
        # 顺序与服务端 _make_joint_handler("WBC", ["head","waist","arms"]) 一致
        try:
            self._move_head(pos_data, head_speed)
        except Exception as e:
            print(f"[fast]   head 失败: {e}")
        if self._settle_s:
            time.sleep(self._settle_s)
        try:
            self._move_waist(pos_data, waist_speed)
        except Exception as e:
            print(f"[fast]   waist 失败: {e}")
        if self._settle_s:
            time.sleep(self._settle_s)
        try:
            self._move_both_arms(pos_data, arm_speed)
        except Exception as e:
            print(f"[fast]   arms 失败: {e}")
        return True

    # ── 到位等待（基于关节角读回）──────────────────────
    def _read_arm_angles(self):
        states = self.robot.get_joint_states()
        cur = {s['name']: s['motor_position'] for s in states['states']}
        return cur

    def wait_arms_settled(self, name, tol_rad=0.02, timeout=15.0, stable_reads=2, period=0.1):
        """等待手臂到达 WBC 动作 name 的目标角（读回轮询）。

        tol_rad     : 每个手臂关节与目标的允许残差（弧度，约 1.1°）。
        timeout     : 最长等待（秒），超时按当前位置继续（兜底，不卡死）。
        stable_reads: 连续多少次读数都在容差内才算到位（防抖）。
        period      : 轮询周期（秒）。

        仅判手臂 14 个关节（头/腰行程小、到位快，不单独判）。找不到动作则不等待。
        """
        pos_data = self._wbc.get(name)
        if pos_data is None:
            return False
        target = _extract(pos_data, LEFT_ARM_JOINT_KEYS + RIGHT_ARM_JOINT_KEYS)
        keys = LEFT_ARM_JOINT_KEYS + RIGHT_ARM_JOINT_KEYS
        deadline = time.time() + timeout
        good = 0
        while time.time() < deadline:
            time.sleep(period)
            try:
                cur = self._read_arm_angles()
            except Exception:
                continue
            max_err = 0.0
            for k, t in zip(keys, target):
                v = cur.get(k)
                if v is None:
                    max_err = float('inf')
                    break
                max_err = max(max_err, abs(v - t))
            if max_err <= tol_rad:
                good += 1
                if good >= stable_reads:
                    return True
            else:
                good = 0
        print(f"[fast]   {name} 到位等待超时({timeout}s)，按当前位置继续")
        return False

    def wbc_wait(self, name, arm_speed=DEFAULT_ARM_SPEED,
                 head_speed=DEFAULT_HEAD_SPEED, waist_speed=DEFAULT_WAIST_SPEED,
                 tol_rad=0.02, timeout=15.0):
        """执行 WBC 动作并等待手臂到位（下发 + 到位判定，供主程序串行调用）。"""
        if not self.wbc(name, arm_speed=arm_speed, head_speed=head_speed, waist_speed=waist_speed):
            return False
        return self.wait_arms_settled(name, tol_rad=tol_rad, timeout=timeout)

    def close(self):
        try:
            agibot_gdk.gdk_release()
        except Exception:
            pass
