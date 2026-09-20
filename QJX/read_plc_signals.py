#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""read_plc_signals.py —— 只读：连续读 PLC 关键信号，看心跳是否翻转、各状态位真实值。"""
import sys, time
sys.path.insert(0, "/data/wxf0721/runtime")
from minth import Minth

NAMES = ["plc_heartbeat", "plc_alive", "pick_right", "place_middle",
         "pick_right_release", "place_middle_release"]

def main():
    g = Minth.G2(timeout=3)
    try:
        for i in range(8):
            vals = {n: g.readData(n) for n in NAMES}
            print(f"[{i}] " + "  ".join(f"{n}={vals[n]}" for n in NAMES))
            time.sleep(0.5)
    finally:
        g.close()

if __name__ == "__main__":
    main()
