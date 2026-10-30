#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把机器人运动控制模式切到关节位控(control_mode=1)。只切模式，不发任何运动。

用途：开机自动把机器人置为位控（关节位控），供视觉纠偏及其它动作使用。
底层走 GDK Robot.set_control_mode(1)（即 /MotionControlService/ControlMode/request）。

背景（2026-10-21 实测）：
  运动控制守护进程 genie_motion_control 以
  --default-action=REACTIVE_CONTROL_FORCE_CONTROL 启动，它会在开机后某个时刻
  （实测晚于"首次切1成功"数分钟）把模式一次性改回 3（力控）。之后不再回弹。
  因此开机不能"切一次就退出"，要在开机后一段窗口内持续看护，发现被改回就再切。

用法：
  python3 set_control_mode_115.py                     # 只检查/切一次（默认）
  python3 set_control_mode_115.py --watch-max-s 480   # 开机看护：持续 N 秒

返回码：0=看护结束时确认为位控；1=未就绪/切换未生效（可重试）。

注意：需先 source /home/agi/app/env.sh 才能 import agibot_gdk。
"""
import argparse
import time

TARGET_MODE = 1  # 1=关节位控(CTRL_JOINT_POSITION)


def _read_mode(robot):
    """回读当前控制模式，顺带确认运动控制状态健康。"""
    st = robot.get_motion_control_status()
    if int(st.error_code) != 0:
        raise RuntimeError(f"运动控制状态异常 error_code={int(st.error_code)}")
    return int(st.control_mode)


def _switch_to_target(robot):
    """切到位控并回读确认，返回回读到的模式。"""
    rc = robot.set_control_mode(TARGET_MODE)
    print(f"[control-mode] set_control_mode({TARGET_MODE}) 返回 {rc}")
    time.sleep(1.0)
    return _read_mode(robot)


def _ensure_once(robot):
    mode = _read_mode(robot)
    if mode == TARGET_MODE:
        print(f"[control-mode] 已是位控(control_mode={mode})，跳过")
        return 0
    print(f"[control-mode] 当前 mode={mode} → 切到位控({TARGET_MODE})")
    mode = _switch_to_target(robot)
    if mode == TARGET_MODE:
        print(f"[control-mode] 切换成功 control_mode={mode}")
        return 0
    print(f"[control-mode] 切换后仍为 mode={mode}")
    return 1


def _watch(robot, max_s, interval):
    """开机看护窗口内持续确认位控；被改回则重新切换。"""
    start = time.monotonic()
    print(f"[control-mode] 进入看护模式 {max_s:.0f}s（每 {interval:.0f}s 检查一次）")
    while True:
        elapsed = time.monotonic() - start
        try:
            mode = _read_mode(robot)
        except Exception as exc:  # noqa: BLE001
            print(f"[control-mode] t={elapsed:.0f}s 状态读取失败: {exc}")
            mode = None
        if mode is not None and mode != TARGET_MODE:
            print(f"[control-mode] t={elapsed:.0f}s 检测到被改为 mode={mode}，切回位控")
            try:
                mode = _switch_to_target(robot)
                print(f"[control-mode] t={elapsed:.0f}s 纠正后 mode={mode}")
            except Exception as exc:  # noqa: BLE001
                print(f"[control-mode] t={elapsed:.0f}s 纠正失败: {exc}")
        elif mode is not None:
            # 每次采样都打印：用于开机后回溯"PC 连上之前模式到底是什么"
            print(f"[control-mode] t={elapsed:.0f}s mode={mode}")
        if elapsed >= max_s:
            break
        time.sleep(min(interval, max(0.0, max_s - elapsed)))
    try:
        mode = _read_mode(robot)
    except Exception as exc:  # noqa: BLE001
        print(f"[control-mode] 看护结束，状态读取失败: {exc}")
        return 1
    print(f"[control-mode] 看护结束 control_mode={mode}")
    return 0 if mode == TARGET_MODE else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--watch-max-s', type=float, default=0.0,
                        help='开机看护时长(秒)；0=只切一次')
    parser.add_argument('--interval', type=float, default=10.0,
                        help='看护期间的检查间隔(秒)')
    args = parser.parse_args()

    try:
        import agibot_gdk as g
    except ImportError as exc:
        print(f"[control-mode] 未找到 agibot_gdk（需先 source /home/agi/app/env.sh）: {exc}")
        return 1

    if g.gdk_init() != g.GDKRes.kSuccess:
        print("[control-mode] GDK 未就绪")
        return 1

    try:
        robot = g.Robot()
        time.sleep(2)  # 等 DDS 连接就绪
        if args.watch_max_s > 0:
            return _watch(robot, args.watch_max_s, args.interval)
        return _ensure_once(robot)
    except Exception as exc:  # noqa: BLE001
        print(f"[control-mode] 异常: {exc}")
        return 1
    finally:
        try:
            g.gdk_release()
        except Exception:  # noqa: BLE001
            pass


if __name__ == '__main__':
    raise SystemExit(main())
