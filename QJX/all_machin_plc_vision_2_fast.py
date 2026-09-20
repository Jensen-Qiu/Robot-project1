import math
import os
import sys
import threading
import time

# minth.py 在 runtime/ 下；本文件位于 /data/QJX/，需把 runtime 目录与 QJX 目录都加入搜索路径。
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))          # /data/QJX（fast_joint_mover.py）
sys.path.insert(0, "/data/wxf0721/runtime")                              # minth.py
sys.path.insert(0, "/data/wxf0721/runtime/programs")                    # vision_helper.py 等视觉脚本

from minth import Minth
from fast_joint_mover import FastJointMover

# ══════════════════════════════════════════════════════════════
#  QJX 提速版主程序
#  与 all_machin_plc_vision_2.py 的唯一区别：
#    手臂/头/腰的 WBC 动作改由 FastJointMover 用【可调速】直接下发，
#    绕开服务端固定的 ARM_SPEED=0.2；底盘 GO/REL、夹爪、视觉纠偏、
#    握手、看门狗、心跳等全部沿用原逻辑（走 Minth/MQTT），未改动。
#  速度分组（用户确认）：
#    可提速 0.5：FW, SL0, SL3, WQ1, WF3, WQ3, XL2, XL3
#    保持 0.2 ：SL2, WF1, WF2, WQ2, XL1
# ══════════════════════════════════════════════════════════════

# 各 WBC 动作的手臂速度表（未列出的默认 0.2 保守）
FAST = 0.5   # 空手转移 / 已抓稳退出等，可提速
SLOW = 0.2   # 贴料、抓料、放料等对精度/安全敏感，保持原速
WBC_ARM_SPEED = {
    "FW":  FAST,
    "SL0": FAST,
    "SL2": SLOW,
    "SL3": FAST,
    "WF1": SLOW,
    "WF2": SLOW,
    "WF3": FAST,
    "WQ1": FAST,
    "WQ2": SLOW,
    "WQ3": FAST,
    "XL1": SLOW,
    "XL2": FAST,
    "XL3": FAST,
}

# 上料机→弯曲机 强制右转角度（度）。改这里即可调整，无需算弧度。
TURN_RIGHT_DEG = 90

# ── 视觉纠偏开关（与原程序一致）──────────────────────────────
VISION_ENABLED = True
VISION_APPLY = True
VISION_GAIN = 1.0
VISION_TOL_MM = 3.0
VISION_MAX_STEP = 12.0
VISION_MAX_ITERS = 4

try:
    from vision_helper import get_pick_offset
    _VISION_IMPORT_OK = True
except Exception as _e:
    print(f"[vision] 未能导入 vision_helper，视觉纠偏禁用：{_e}")
    _VISION_IMPORT_OK = False


def _base_delta_to_offset(delta):
    rx = delta['x'] * VISION_GAIN
    ry = delta['y'] * VISION_GAIN
    rz = delta['z'] * VISION_GAIN
    m = max(abs(rx), abs(ry), abs(rz))
    if m > VISION_MAX_STEP:
        s = VISION_MAX_STEP / m
        rx, ry, rz = rx * s, ry * s, rz * s
    return {'rx': round(rx, 3), 'ry': round(ry, 3), 'rz': round(rz, 3)}


def vision_correct_pick():
    _run_vision_align('取料', 'pick_reference.json', relax_gripper=True)


def vision_correct_place():
    _run_vision_align('弯曲放料', 'place_reference.json', relax_gripper=True)


def vision_correct_unload():
    _run_vision_align('下料放料', 'unload_reference.json', relax_gripper=True)


def vision_correct_wqpick():
    _run_vision_align('弯曲取料', 'wqpick_reference.json', relax_gripper=True)


def turn_right(deg):
    """底盘原地右转 deg 度。右转为负 yaw。"""
    print(f"[turn] 底盘右转 {deg} 度")
    return G2.REL({"yaw_rad": -math.radians(abs(deg))})


def turn_left(deg):
    """底盘原地左转 deg 度。左转为正 yaw。"""
    print(f"[turn] 底盘左转 {deg} 度")
    return G2.REL({"yaw_rad": math.radians(abs(deg))})


def _run_vision_align(station, reference_name, *, relax_gripper):
    """通用视觉纠偏（与原程序一致，走独立子进程，安全回退）。视觉参考仍在
    /data/wxf0721/runtime/programs/ 下（vision_align.py 与 *_reference.json）。"""
    if not (VISION_ENABLED and _VISION_IMPORT_OK):
        return
    try:
        import subprocess
        import json
        vis_dir = "/data/wxf0721/runtime/programs"
        ref = os.path.join(vis_dir, reference_name)
        relax = ' --relax-gripper' if relax_gripper else ''
        if not VISION_APPLY:
            print(f"[vision][预览] {station}：VISION_APPLY=False，未启用下发")
            return
        cmd = (f"source /home/agi/app/env.sh /home/agi/app >/dev/null 2>&1; "
               f"python3 {vis_dir}/vision_align.py --reference {ref}{relax}")
        try:
            proc = subprocess.run(['bash', '-lc', cmd], capture_output=True,
                                  text=True, timeout=30)
        except subprocess.TimeoutExpired:
            print(f"[vision] {station}纠偏超时，按当前位置继续")
            return
        result = None
        for stream in (proc.stdout, proc.stderr):
            if not stream:
                continue
            for line in stream.splitlines():
                if line.startswith('[align]'):
                    print(line)
                if line.startswith('VISION_ALIGN_RESULT '):
                    try:
                        result = json.loads(line[len('VISION_ALIGN_RESULT '):])
                    except json.JSONDecodeError:
                        result = None
        if result is None:
            print(f"[vision] {station}未解析到纠偏结果(rc={proc.returncode})，按当前位置继续")
            if proc.stderr:
                print("[vision] stderr:", proc.stderr.strip()[-300:])
            return
        st = result.get('status')
        if st == 'aligned':
            print(f"[vision] ✅ {station}已对准 {result.get('final_mm'):.2f}mm，"
                  f"纠偏 {result.get('iterations')} 次，用时 {result.get('seconds')}s")
        elif st == 'max_iters':
            print(f"[vision] {station}迭代用尽，残差 {result.get('final_mm'):.2f}mm，按当前位置继续")
        elif st in ('exceeds_limit', 'diverged_stopped', 'offset_failed',
                    'no_reference', 'identity_mismatch'):
            print(f"[vision] {station} {st}: {result.get('message') or ''}，按原固定点位执行")
        else:
            print(f"[vision] {station} {st}: {result.get('error') or result.get('message') or ''}，按原固定点位执行")
    except Exception as e:
        print(f"[vision] {station}纠偏异常，回退固定点位：{e}")


# ── 机器人对象：Minth 走 MQTT（底盘/夹爪/信号）；MOVER 走 GDK（可调速关节）──
G2 = Minth.G2()
G2_wd = Minth.G2(timeout=2)
G2_hb = Minth.G2(timeout=5)
MOVER = None  # 在 main() 里初始化（gdk_init 只做一次）

_comm_ok = True
_running = True


def WBC(name):
    """提速版 WBC：查速度表，用 MOVER 可调速下发并等待手臂到位。

    未在速度表中的动作按保守 0.2 执行。MOVER 未就绪时兜底回退到 G2.WBC（原服务端 0.2）。
    """
    speed = WBC_ARM_SPEED.get(name, SLOW)
    if MOVER is not None and MOVER.has(name):
        return MOVER.wbc_wait(name, arm_speed=speed)
    print(f"[fast] MOVER 不可用或无动作 {name}，回退 G2.WBC 原速执行")
    return G2.WBC(name)


def enabled_heartbeat():
    while True:
        if _comm_ok and _running:
            try:
                G2_hb.setData("G2_eanbled", 1)
            except Exception:
                pass
        time.sleep(0.5)


def check_comm():
    if _comm_ok:
        return
    try:
        G2.setData("G2_eanbled", 0)
    except Exception:
        pass
    print("[comm] PLC通信中断，机器人立即停止")
    os._exit(1)


def comm_watchdog():
    global _comm_ok, _running
    timeout_count = 0
    timeout_limit = 3
    # ★ 启动宽限：程序刚起来时，PLC 还没看到"启动中"来置位"总控自动中"(plc_alive)。
    #   宽限期内 plc_alive=0 只等待、不退出；期内读到 1 即进入正常运行。
    #   宽限期结束仍为 0 才判总控未进自动而退出。
    STARTUP_GRACE_S = 15.0
    _grace_deadline = time.monotonic() + STARTUP_GRACE_S
    _entered_auto = False
    while True:
        try:
            val = G2_wd.readData("plc_alive")
            if val is None:
                timeout_count += 1
                print(f"[watchdog] readData 超时 {timeout_count}/{timeout_limit}")
                if timeout_count >= timeout_limit:
                    _comm_ok = False
                    _running = False
                    print("[watchdog] MQTT通信中断（服务无响应），机器人立即停止！")
                    try:
                        G2.setData("G2_eanbled", 0)
                    except Exception:
                        pass
                    os._exit(1)
                time.sleep(0.3)
                continue
            timeout_count = 0
            if val == 1:
                _entered_auto = True
            if val == 0:
                if not _entered_auto and time.monotonic() < _grace_deadline:
                    # 宽限期内：等 PLC 看到"启动中"后置位"总控自动中"，不退出
                    remain = _grace_deadline - time.monotonic()
                    print(f"[watchdog] 启动宽限中，等待总控自动中(plc_alive=1)... 剩余 {remain:.1f}s")
                    time.sleep(0.3)
                    continue
                _comm_ok = False
                _running = False
                print("[watchdog] 未接收到总控信号(plc_alive=0)，机器人立即停止！")
                try:
                    G2.TTS("未接收到总控信号，立即停止")
                    G2.setData("G2_eanbled", 0)
                except Exception:
                    pass
                os._exit(1)
        except Exception:
            _comm_ok = False
            _running = False
            print("[watchdog] 通信异常，机器人立即停止！")
            try:
                G2.setData("G2_eanbled", 0)
            except Exception:
                pass
            os._exit(1)
        time.sleep(0.3)


def wait_signal(name, value=1):
    while True:
        check_comm()
        val = G2.readData(name)
        if val is None:
            time.sleep(0.2)
            continue
        if val == value:
            return
        time.sleep(0.2)


def handshake(req, done, hold_s=0.3, poll_s=0.2):
    G2.setData(done, 1)
    t0 = time.monotonic()
    while time.monotonic() - t0 < hold_s:
        check_comm()
        time.sleep(0.05)
    wait_signal(req, 0)
    G2.setData(done, 0)


def run_pick_right_task():
    """执行送料机取料和弯曲机放料。"""
    G2.GRIPPER({"right": -0.78})  # 爪子张开
    WBC("FW")

    # 第一段：上料。
    G2.TTS("从送料机上抓料，底盘运动到送料机前面")
    G2.GO(1)
    WBC("SL0")
    # ── 视觉纠偏挂载点：合爪抓料前 ──
    vision_correct_pick()
    WBC("SL2")
    G2.GRIPPER({"right": -0.05})  # 爪子关闭
    WBC("SL3")
    G2.REL({"x": -0.3})  # 底盘后退0.3米

    # 第二段：弯曲机放料。
    G2.TTS("向弯曲机放料，运动到设备前方")
    turn_right(TURN_RIGHT_DEG)  # 强制先右转，再导航到弯曲机
    G2.GO(2)
    WBC("WF1")
    # ── 视觉纠偏挂载点：弯曲机放料，张爪前 ──
    vision_correct_place()
    G2.GRIPPER({"right": -0.78})
    WBC("WF2")
    WBC("WF3")
    G2.TTS("在弯曲机上放料完成，后退")
    G2.REL({"x": -0.3})  # 底盘后退0.3米


def run_place_middle_task():
    """执行弯曲机取料和下料机放料。"""
    # 第三段：弯曲机取料。
    G2.TTS("从弯曲机上取料，运动到设备前方")
    G2.GO(2)
    WBC("WQ1")
    # ── 视觉纠偏挂载点：弯曲机取料，WQ2 之前 ──
    vision_correct_wqpick()
    WBC("WQ2")
    G2.GRIPPER({"right": -0.05})  # 爪子关闭
    WBC("WQ3")
    G2.REL({"x": -0.3})  # 底盘后退0.3米
    G2.TTS("从弯曲机上取料完成，后退至下料机")

    # 第四段：下料。
    G2.GO(3)
    WBC("XL1")
    # ── 视觉纠偏挂载点：下料机放料，张爪前 ──
    vision_correct_unload()
    G2.GRIPPER({"right": -0.78})  # 爪子张开
    WBC("XL2")
    WBC("XL3")
    G2.TTS("下料完成，后退")
    G2.REL({"x": -0.4})  # 底盘后退0.4米


def _auto_rebind_references():
    """启动时自动把视觉参考的 boot_id 更新为当前开机（免重新示教）。参考文件仍在
    /data/wxf0721/runtime/programs/ 下。"""
    if not VISION_ENABLED:
        return
    try:
        import json
        vis_dir = "/data/wxf0721/runtime/programs"
        boot = open('/proc/sys/kernel/random/boot_id').read().strip()
        new_stream = f'{boot}/head_color'
        for name in ('pick_reference.json', 'place_reference.json', 'unload_reference.json', 'wqpick_reference.json'):
            p = os.path.join(vis_dir, name)
            if not os.path.exists(p):
                continue
            with open(p, encoding='utf-8') as f:
                ref = json.load(f)
            if ref.get('stream_id') == new_stream:
                continue
            old = ref.get('stream_id', '')
            ref['stream_id'] = new_stream
            with open(p, 'w', encoding='utf-8') as f:
                json.dump(ref, f, ensure_ascii=False, indent=2)
            print(f"[vision] 参考 {name} 已绑定当前开机 ({old} -> {new_stream})")
    except Exception as e:
        print(f"[vision] 自动 rebind 失败（纠偏将回退固定点位）：{e}")


def main():
    global MOVER
    # 充电/重启后自动把视觉参考绑定到当前开机，免重新示教。
    _auto_rebind_references()

    # 初始化可调速关节执行器（gdk_init 一次 + 在线加载 WBC 关节角）。
    # 若失败，则 MOVER=None，WBC() 会自动回退到 G2.WBC 原速执行，保证不影响流程。
    try:
        MOVER = FastJointMover(init_gdk=True, load_wbc=True)
    except Exception as e:
        MOVER = None
        print(f"[fast] FastJointMover 初始化失败，全程回退原速 G2.WBC：{e}")

    threading.Thread(target=comm_watchdog, daemon=True).start()
    threading.Thread(target=enabled_heartbeat, daemon=True).start()

    G2.setData("G2_eanbled", 1)
    G2.setData("pick_right_done", 0)
    G2.setData("place_middle_done", 0)
    G2.TTS("机器人已就绪，等待PLC信号")

    try:
        G2.TTS("等待PLC上料信号")
        wait_signal("pick_right")
        G2.setData("pick_right_done", 1)
        run_pick_right_task()
        wait_signal("pick_right_release")
        G2.setData("pick_right_done", 0)

        G2.TTS("等待PLC下料信号")
        wait_signal("place_middle")
        G2.setData("place_middle_done", 1)
        run_place_middle_task()
        wait_signal("place_middle_release")
        G2.setData("place_middle_done", 0)

        G2.TTS("全部任务完成")
    finally:
        global _running
        _running = False
        try:
            G2.setData("pick_right_done", 0)
            G2.setData("place_middle_done", 0)
            G2.setData("G2_eanbled", 0)
        except Exception:
            pass
        if MOVER is not None:
            MOVER.close()


if __name__ == "__main__":
    main()
