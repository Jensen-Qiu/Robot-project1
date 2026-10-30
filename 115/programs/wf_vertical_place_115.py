#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
wf_vertical_place_115.py —— 放料工位"保持纠偏姿态垂直下放"模块（通用多工位，不改主程序）

问题
----
WF(弯曲放料)/XL(下料) 的放料顺序：纠偏 → GRIPPER 张爪 → WBC("WF2"/"XL2")。
WF2/XL2 是【关节插值】动作，末端斜向下移动且姿态变化；叠加纠偏带的旋转补偿(rry)，
会把料条斜放/翻转，导致下游检测误判或放错。

本模块把 WF2/XL2 换成：在【纠偏成功后的末端姿态】基础上，沿世界 Z 轴【垂直直线下放】
一个【固定落差 dz】。底层用 OFFSET({"rz": -dz*1000}) 且不带任何旋转——
目标位置=当前位置加偏移、姿态保持不变、笛卡尔直线插值、坐标系 Z+ 向上
（见 services/EndEffectorController.py adjust_arms_relative）。配合"纠偏关旋转补偿
(rot_comp=False)"，末端保持原始正姿态垂直下降，消除翻转/放偏。

固定落差 dz（按工位）
--------------------
dz[WF] = Z(WF1末端) − Z(WF2末端)；dz[XL] = Z(XL1末端) − Z(XL2末端)。
上/下位姿都是固定关节角，末端 Z 落差为常量，标定一次即可（每工位各标一次）。

配置文件 wf_vertical_place_dz.json（按工位）：
  {"WF": {"dz_m":0.0736, ...}, "XL": {"dz_m":..., ...}}
向后兼容：若为旧单值 {"dz_m":...}，load_dz('WF') 仍能读到。

接入（不改主程序）：视觉常驻服务加 cmd="drop"(station=WF/XL)；启动器对 G2.WBC 打补丁，
WF2→drop(WF)、XL2→drop(XL)，失败降级执行原关节动作。

命令行：
  python3 wf_vertical_place_115.py show                      # 查看所有工位 dz
  python3 wf_vertical_place_115.py calibrate --station WF    # 标定 WF（WF1/WF2，会动右臂）
  python3 wf_vertical_place_115.py calibrate --station XL    # 标定 XL（XL1/XL2，会动右臂）
"""
import json
import os
import sys
import time
from pathlib import Path

PROGRAMS_ROOT = Path('/data/wxf0721/runtime/programs')
RUNTIME_ROOT = Path('/data/wxf0721/runtime')
for p in (PROGRAMS_ROOT, RUNTIME_ROOT):
    sp = str(p)
    if sp not in sys.path:
        sys.path.insert(0, sp)

RIGHT_END_LINK = 'arm_r_end_link'
CONFIG_PATH = str(PROGRAMS_ROOT / 'wf_vertical_place_dz.json')
MAX_DROP_M = 0.20   # 单次垂直下放安全上限（米）

# 工位 → (上位姿, 下位姿) 关节动作名，用于标定 dz=Z(上)-Z(下)
STATION_WBC = {
    'WF': ('WF1', 'WF2'),
    'XL': ('XL1', 'XL2'),
}


def read_right_end_z(gdk_robot):
    """读右臂末端笛卡尔 Z（米，世界系 Z+向上）。"""
    status = gdk_robot.get_motion_control_status()
    for i, name in enumerate(status.frame_names):
        if name == RIGHT_END_LINK:
            return float(status.frame_poses[i].position.z)
    raise RuntimeError(f"末端帧 {RIGHT_END_LINK} 未找到")


def read_right_end_pose(gdk_robot):
    status = gdk_robot.get_motion_control_status()
    for i, name in enumerate(status.frame_names):
        if name == RIGHT_END_LINK:
            p = status.frame_poses[i]
            return ([p.position.x, p.position.y, p.position.z],
                    [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
    raise RuntimeError(f"末端帧 {RIGHT_END_LINK} 未找到")


def _load_cfg():
    try:
        with open(CONFIG_PATH, encoding='utf-8') as f:
            return json.load(f)
    except FileNotFoundError:
        return {}
    except Exception as e:
        print(f"[wf-drop] 读配置失败: {e}")
        return {}


def load_dz(station):
    """读某工位固定落差 dz（米）。无/非法返回 None（调用方回退关节动作）。
    向后兼容：旧单值 {"dz_m":...} 视为 WF。"""
    cfg = _load_cfg()
    entry = None
    if station in cfg and isinstance(cfg[station], dict):
        entry = cfg[station]
    elif station == 'WF' and 'dz_m' in cfg:   # 旧单值兼容
        entry = cfg
    if not entry:
        return None
    try:
        dz = float(entry.get('dz_m'))
    except (TypeError, ValueError):
        return None
    if dz is None or dz <= 0 or dz > MAX_DROP_M:
        print(f"[wf-drop] {station} dz={dz} 非法(应 0<dz<={MAX_DROP_M})，视为未标定")
        return None
    return dz


def save_dz(station, dz_m, note=""):
    cfg = _load_cfg()
    # 若原来是旧单值，迁移为按工位结构
    if 'dz_m' in cfg and not any(k in cfg for k in STATION_WBC):
        cfg = {'WF': {'dz_m': cfg['dz_m'], 'note': cfg.get('note', ''),
                      'ts': cfg.get('ts', '')}}
    cfg[station] = {"dz_m": round(float(dz_m), 5), "note": note,
                    "ts": time.strftime("%Y-%m-%d %H:%M:%S")}
    with open(CONFIG_PATH, 'w', encoding='utf-8') as f:
        json.dump(cfg, f, ensure_ascii=False, indent=2)
    print(f"[wf-drop] 已保存 {station} 固定落差 dz={cfg[station]['dz_m']}m 到 {CONFIG_PATH}")


def vertical_drop(minth_g2, dz_m):
    """当前末端姿态基础上沿世界 Z 轴垂直下放 dz_m 米（OFFSET rz，无旋转，姿态不变）。"""
    if dz_m is None or dz_m <= 0 or dz_m > MAX_DROP_M:
        raise ValueError(f"dz 非法: {dz_m}（应 0<dz<={MAX_DROP_M}）")
    dz_mm = round(dz_m * 1000.0, 2)
    print(f"[wf-drop] 垂直下放 {dz_mm}mm（OFFSET rz=-{dz_mm}，无旋转，保持纠偏姿态）",
          flush=True)
    return bool(minth_g2.OFFSET({"rz": -dz_mm}))


def calibrate(station, settle_s=1.0):
    """标定某工位 dz = Z(上位姿) - Z(下位姿)。会真实驱动右臂！仅安全无料时手动跑。
    走 GDK（独占文件锁）——运行前确保视觉常驻服务已停。"""
    if station not in STATION_WBC:
        raise ValueError(f"未知工位 {station}，支持 {list(STATION_WBC)}")
    up, down = STATION_WBC[station]
    import agibot_gdk as g
    from minth import Minth

    if g.gdk_init() != g.GDKRes.kSuccess:
        raise RuntimeError("GDK 初始化失败")
    robot = g.Robot()
    time.sleep(1.0)
    m = Minth.G2()
    try:
        print(f"[wf-drop][标定 {station}] 运动到 {up} ...")
        if not m.WBC(up):
            raise RuntimeError(f"{up} 运动失败")
        time.sleep(settle_s)
        z1 = read_right_end_z(robot)
        print(f"[wf-drop][标定 {station}] {up} 末端 Z = {z1:.4f}m")

        print(f"[wf-drop][标定 {station}] 运动到 {down} ...")
        if not m.WBC(down):
            raise RuntimeError(f"{down} 运动失败")
        time.sleep(settle_s)
        z2 = read_right_end_z(robot)
        print(f"[wf-drop][标定 {station}] {down} 末端 Z = {z2:.4f}m")

        dz = z1 - z2
        print(f"[wf-drop][标定 {station}] 固定落差 dz = Z({up}) - Z({down}) = {dz:.4f}m")
        if dz <= 0:
            raise RuntimeError(f"dz={dz:.4f} ≤0，{down} 不比 {up} 低？检查关节角")
        if dz > MAX_DROP_M:
            raise RuntimeError(f"dz={dz:.4f} 超安全上限 {MAX_DROP_M}m，拒绝保存")
        save_dz(station, dz, note=f"calib Z({up})={z1:.4f}-Z({down})={z2:.4f}")
        print(f"[wf-drop][标定 {station}] 复位回 {up}")
        m.WBC(up)
        return dz
    finally:
        try:
            m.close()
        except Exception:
            pass
        try:
            g.gdk_release()
        except Exception:
            pass


def _cli():
    import argparse
    ap = argparse.ArgumentParser(description="放料工位 垂直下放 标定/查看（WF/XL）")
    ap.add_argument('cmd', choices=['calibrate', 'show'])
    ap.add_argument('--station', default='WF', choices=list(STATION_WBC))
    args = ap.parse_args()
    if args.cmd == 'show':
        cfg = _load_cfg()
        print(f"配置 {CONFIG_PATH}:")
        for st in STATION_WBC:
            print(f"  {st}: dz = {load_dz(st)} m")
        return 0
    if args.cmd == 'calibrate':
        print(f"⚠ 标定 {args.station} 将真实驱动右臂到 "
              f"{STATION_WBC[args.station][0]}/{STATION_WBC[args.station][1]}。"
              f"确保无料、无遮挡、视觉服务已停。")
        dz = calibrate(args.station)
        print(f"标定完成 {args.station} dz={dz:.4f}m")
        return 0


if __name__ == '__main__':
    raise SystemExit(_cli())
