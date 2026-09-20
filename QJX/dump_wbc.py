#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dump_wbc.py —— 只读导出：通过 MQTT 向服务端请求 WBC 动作关节角，确认全部存在。

安全性：只读，不发任何运动指令，机器人不会动。

做法：向 /humanoid/joints/save 发 {"command":"read"}，服务端会把内存数据库里
所有 joints/positions 数据发布到 /humanoid/joints/data。本脚本订阅该主题，
筛出 type=="WBC" 的动作，打印每个动作名及其关节数，并把结果存成 JSON 备查。
"""
import json
import sys
import time
import threading

import paho.mqtt.client as mqtt

BROKER = "localhost"
PORT = 1883
TOPIC_SAVE = "/humanoid/joints/save"
TOPIC_DATA = "/humanoid/joints/data"

WANT = ["FW", "SL0", "SL2", "SL3", "WF1", "WF2", "WF3",
        "WQ1", "WQ2", "WQ3", "XL1", "XL2", "XL3"]

_result = {"items": None}
_evt = threading.Event()


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
        _result["items"] = payload.get("data", [])
        _evt.set()


def main():
    cli = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="dump_wbc")
    cli.on_connect = on_connect
    cli.on_message = on_message
    cli.connect(BROKER, PORT)
    cli.loop_start()
    time.sleep(1.0)

    cli.publish(TOPIC_SAVE, json.dumps({"command": "read"}), qos=0)
    print("[dump] 已请求 read，等待服务端返回数据...")

    if not _evt.wait(timeout=10):
        print("[dump] ❌ 超时未收到数据。")
        cli.loop_stop()
        return 1

    items = _result["items"] or []
    wbc = {}
    for it in items:
        if it.get("type") == "WBC":
            wbc[it.get("name")] = it.get("value", {})

    print(f"[dump] 数据库中 WBC 动作共 {len(wbc)} 个: {sorted(wbc.keys())}")
    print("[dump] ── 目标动作检查 ──")
    missing = []
    for name in WANT:
        if name in wbc:
            print(f"  ✅ {name:<4} 关节数={len(wbc[name])}")
        else:
            print(f"  ❌ {name:<4} 缺失")
            missing.append(name)

    out = "/data/QJX/wbc_dump.json"
    try:
        with open(out, "w", encoding="utf-8") as f:
            json.dump(wbc, f, ensure_ascii=False, indent=2)
        print(f"[dump] 已保存全部 WBC 关节角到 {out}")
    except Exception as e:
        print(f"[dump] 保存失败（不影响判断）: {e}")

    cli.loop_stop()
    if missing:
        print(f"[dump] ⚠️ 缺失: {missing}")
        return 1
    print("[dump] ✅ 目标 WBC 动作全部存在。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
