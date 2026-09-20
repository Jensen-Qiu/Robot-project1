#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_grab_speed.py —— 走服务端 (Minth.GRIPPER) 实测夹爪张合耗时。

与整程序完全同一条路：发 grab 命令 -> 服务端 handle_grab 到位判定 -> 返回 done。
测得的耗时即整程序里每次夹爪动作的真实耗时。用于验证 commands.py 参数改动
（超时 6s->1.5s、容差 0.05->0.12、周期 0.1->0.05）的提速效果。

安全：右爪空爪张合各 3 次。
"""
import sys
import time

sys.path.insert(0, "/data/wxf0721/runtime")
from minth import Minth

OPEN_POS = -0.78
CLOSE_POS = -0.05


def main():
    g = Minth.G2()
    try:
        for i in range(3):
            print(f"--- 第 {i+1} 轮 ---")
            t0 = time.time()
            g.GRIPPER({"right": OPEN_POS})
            print(f"  张开 耗时 {time.time()-t0:.2f}s")
            time.sleep(0.3)
            t0 = time.time()
            g.GRIPPER({"right": CLOSE_POS})
            print(f"  闭合 耗时 {time.time()-t0:.2f}s")
            time.sleep(0.3)
    finally:
        g.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
