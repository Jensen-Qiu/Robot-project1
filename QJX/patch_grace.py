#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""给 all_machin_plc_vision_2_fast.py 的 comm_watchdog 加启动宽限，不动原程序。"""
import io, sys

PATH = "/data/QJX/all_machin_plc_vision_2_fast.py"
with io.open(PATH, encoding="utf-8") as f:
    src = f.read()

# 1) 在 comm_watchdog 函数体开头加宽限变量
old_head = (
    "def comm_watchdog():\n"
    "    global _comm_ok, _running\n"
    "    timeout_count = 0\n"
    "    timeout_limit = 3\n"
    "    while True:\n"
)
new_head = (
    "def comm_watchdog():\n"
    "    global _comm_ok, _running\n"
    "    timeout_count = 0\n"
    "    timeout_limit = 3\n"
    "    # ★ 启动宽限：程序刚起来时，PLC 还没看到\"启动中\"来置位\"总控自动中\"(plc_alive)。\n"
    "    #   宽限期内 plc_alive=0 只等待、不退出；期内读到 1 即进入正常运行。\n"
    "    #   宽限期结束仍为 0 才判总控未进自动而退出。\n"
    "    STARTUP_GRACE_S = 15.0\n"
    "    _grace_deadline = time.monotonic() + STARTUP_GRACE_S\n"
    "    _entered_auto = False\n"
    "    while True:\n"
)
if old_head not in src:
    print("ERR: 未找到 comm_watchdog 头部，未修改")
    sys.exit(2)
src = src.replace(old_head, new_head, 1)

# 2) 把 plc_alive==0 立即退出，改为宽限逻辑
old_zero = (
    "            timeout_count = 0\n"
    "            if val == 0:\n"
    "                _comm_ok = False\n"
    "                _running = False\n"
    "                print(\"[watchdog] 未接收到总控信号(plc_alive=0)，机器人立即停止！\")\n"
    "                try:\n"
    "                    G2.TTS(\"未接收到总控信号，立即停止\")\n"
    "                    G2.setData(\"G2_eanbled\", 0)\n"
    "                except Exception:\n"
    "                    pass\n"
    "                os._exit(1)\n"
)
new_zero = (
    "            timeout_count = 0\n"
    "            if val == 1:\n"
    "                _entered_auto = True\n"
    "            if val == 0:\n"
    "                if not _entered_auto and time.monotonic() < _grace_deadline:\n"
    "                    # 宽限期内：等 PLC 看到\"启动中\"后置位\"总控自动中\"，不退出\n"
    "                    remain = _grace_deadline - time.monotonic()\n"
    "                    print(f\"[watchdog] 启动宽限中，等待总控自动中(plc_alive=1)... 剩余 {remain:.1f}s\")\n"
    "                    time.sleep(0.3)\n"
    "                    continue\n"
    "                _comm_ok = False\n"
    "                _running = False\n"
    "                print(\"[watchdog] 未接收到总控信号(plc_alive=0)，机器人立即停止！\")\n"
    "                try:\n"
    "                    G2.TTS(\"未接收到总控信号，立即停止\")\n"
    "                    G2.setData(\"G2_eanbled\", 0)\n"
    "                except Exception:\n"
    "                    pass\n"
    "                os._exit(1)\n"
)
if old_zero not in src:
    print("ERR: 未找到 plc_alive==0 处理块，未修改")
    sys.exit(3)
src = src.replace(old_zero, new_zero, 1)

with io.open(PATH, "w", encoding="utf-8") as f:
    f.write(src)
print("OK: 已加入启动宽限 (STARTUP_GRACE_S=15s)")
