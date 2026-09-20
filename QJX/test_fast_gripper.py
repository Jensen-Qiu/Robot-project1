#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_fast_gripper.py —— 验证 FastGripper 的到位耗时（右爪空爪张合各两次）。"""
import sys
import time

sys.path.insert(0, "/data/QJX")
from fast_gripper import FastGripper


def main():
    g = FastGripper(own_gdk=True)
    try:
        for i in range(2):
            print(f"--- 第 {i+1} 轮 ---")
            g.open("right", -0.78)
            time.sleep(0.3)
            g.close("right", -0.05)   # 空爪，expect_grip 检查可能提示 grip_warn（正常，因为没夹料）
            time.sleep(0.3)
    finally:
        g.release()
    return 0


if __name__ == "__main__":
    sys.exit(main())
