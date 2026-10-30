#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 本文件由 all_machin_plc_vision_loop.py 复制生成，用于"一步到位"改动的工作副本。
# 源文件 all_machin_plc_vision_loop.py 不受影响。
# 与源文件唯一的差异：本文件被规划为一步到位改动的修改入口，
# 默认内容与源文件完全一致，可在此基础上自行修改。
import json
import math
import os
import sys
import threading
import time

# minth.py 在上级目录 runtime/ 下，需加入搜索路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from minth import Minth

# 上料机→弯曲机 强制右转角度（度）。改这里即可调整，无需算弧度。
TURN_RIGHT_DEG = 90

# [到位停稳] GO 的 done 回执是"导航任务完成"信号，可能先于底盘物理停稳返回
# （底盘仍在沉降/微动，snapshot 会看到 CHASSIS_ACTIVE state=2）。若此时就挂载伸手，
# 会出现"没到位就往前伸"的现象（现场在弯曲机 WF、下料机 XL 观察到）。故在这两段
# GO 到位后、挂载 WBC 之前，显式等底盘停稳 SETTLE_AFTER_GO_S 秒再伸手。
# 2026-10-28：1.0s 实测到位正常、有余量，调小到 0.5s 回收节拍（WF+XL 共省 ~1s/轮）。
# 若现场又出现"没到位就伸手"，调回 0.8~1.0。
SETTLE_AFTER_GO_S = 0.5

# 无限循环：完整作业（上料→弯曲机放料→弯曲机取料→下料）持续执行，
# 机器人启停由 PLC 信号（plc_alive）控制，无需预设循环次数。
# （看门狗检测到 plc_alive=0 或通信中断时，自动停机退出）

# 视觉纠偏：改为一步到位（验证判定版），直调本文件 _run_align_onestep()。
# 原生产封装 vision_correct_*_via_shell() 不再使用（如需切回，取消下面注释并
# 把 _run_align_onestep 调用点换回对应封装）：
# from vision_plc_helper_115 import vision_correct_pick_via_shell
# from vision_plc_helper_115 import vision_correct_place_via_shell
# from vision_plc_helper_115 import vision_correct_pick_wq_via_shell
# from vision_plc_helper_115 import vision_correct_place_xl_via_shell


G2 = Minth.G2()
# 看门狗使用独立实例和短超时，快速检测通信中断。
G2_wd = Minth.G2(timeout=2)
# 心跳线程使用独立实例，避免与主线程的 setData 调用竞争。
G2_hb = Minth.G2(timeout=5)

# PLC 通信状态：True=正常，False=通信中断。
_comm_ok = True
# 程序运行状态：True=程序运行中（等待或执行动作都算），False=已退出。
# "启动中"(G2_eanbled) 在整个程序运行期间都保持为 1，与是否运动无关。
_running = True


def enabled_heartbeat():
    """程序运行全程持续向 PLC 写入 G2_eanbled=1（启动中）。

    无论机器人正在等待 PLC 信号、握手，还是执行动作，只要程序在运行、
    通信正常，就周期性重写"启动中"=1，防止总控侧因超时自行复位该位，
    也避免出现"后退等待期间启动中信号断开"的情况。
    """
    while True:
        if _comm_ok and _running:
            try:
                G2_hb.setData("G2_eanbled", 1)
            except Exception:
                # 通信是否真正中断由 comm_watchdog 连续检测后判定。
                pass
        time.sleep(0.5)


def check_comm():
    """在等待 PLC 信号期间检查通信状态。"""
    if _comm_ok:
        return

    try:
        G2.setData("G2_eanbled", 0)
    except Exception:
        pass
    print("[comm] PLC通信中断，机器人立即停止")
    os._exit(1)


def wait_for_plc_ready():
    """启动握手：保持“启动中”=1，等待总控进入自动状态。

    启动阶段 plc_alive=0 是正常的握手等待状态，不能立即触发运行期看门狗，
    否则机器人刚置位 G2_eanbled 就会自行清零，PLC 无法完成互锁。
    一旦首次收到 plc_alive=1，再启动运行期看门狗；之后掉为 0 仍立即停机。
    """
    timeout_count = 0
    timeout_limit = 3

    print("[startup] 已发送机器人启动中，等待总控自动信号(plc_alive=1)...")
    while True:
        val = G2_wd.readData("plc_alive")
        if val is None:
            timeout_count += 1
            print(f"[startup] readData 超时 {timeout_count}/{timeout_limit}")
            if timeout_count >= timeout_limit:
                raise RuntimeError("启动握手期间MQTT通信中断")
        else:
            timeout_count = 0
            if int(val) == 1:
                print("[startup] 已收到总控自动信号，启动握手完成")
                return
        time.sleep(0.5)


def comm_watchdog():
    """持续读取 PLC 心跳；连续超时或 plc_alive=0 时退出程序。"""
    global _comm_ok, _running

    timeout_count = 0
    timeout_limit = 3

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
            if val == 0:
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
    """等待 PLC 信号变为指定值；偶发读取超时时继续重试。"""
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
    """标准握手：完成信号只发一次，等总控应答后清零。

    时序（对应总控接入说明）：
      1) done=1 只写一次；随后保持 hold_s（约 3 个 100ms 写周期），保证
         服务端轮询把该位稳定写到 PLC、被总控锁存——但期间不反复重写，
         避免出现"完成信号一直在发送"。
      2) 等待总控复位请求 req=0 作为应答；此阶段只读 req，不再写 done。
      3) done=0 只写一次，握手结束。

    两段作业串行执行、每段握手结束即把 done 清 0，故任意时刻取料完成与
    放料完成最多只有一个为 1。
    """
    # 1) 置位完成应答，只写一次
    G2.setData(done, 1)
    # 2) 保持一小段时间，确保 100ms 写轮询把该位稳定写入 PLC 并被锁存
    t0 = time.monotonic()
    while time.monotonic() - t0 < hold_s:
        check_comm()
        time.sleep(0.05)
    # 3) 等待总控复位请求（应答）；期间不再重写 done
    wait_signal(req, 0)
    # 4) 复位完成应答，只写一次
    G2.setData(done, 0)


def turn_right(deg):
    """底盘原地右转 deg 度（直接输入角度，内部转弧度）。右转为负 yaw。"""
    print(f"[turn] 底盘右转 {deg} 度")
    return G2.REL({"yaw_rad": -math.radians(abs(deg))})


def _run_align_onestep(reference_name, station_label):
    """视觉纠偏：一步到位（验证判定版），四个工位通用。

    直调 vision_align_safe_115.py --one-step-verify：
      一次测量 → 一次下发完整偏差（不限幅） → 验证判定；
      验证残差超阈值或验证失败时自动撤销 OFFSET。
    姿态补偿由 safe_115 按参考文件名自动决定（仅 xl 启用 rrx/rry/rrz）。
    退出码语义与生产 guard 一致：
      0=已对准；2=未对准但安全（已回退/未发动作，可走固定点）；
      1/3=异常或危险 → 抛 RuntimeError 中止整机（与 vision_correct_*_via_shell 相同）。
    若想改回生产纯开环版，把 --one-step-verify 换成 --one-step 即可。
    """
    import subprocess
    here = os.path.dirname(os.path.abspath(__file__))
    mode_script = os.path.join(here, 'set_control_mode_115.py')
    ref = os.path.join(here, reference_name)
    # 工位级参数（结构化，socket 直连与 subprocess 降级共用）：
    #  one_step_verify=True ：一步到位+验证，残差超阈值则撤销 OFFSET 回退(rc=2)。
    #  one_step_verify=False：纯开环一步到位，下发纠偏后无条件保留(判 aligned)，不回退。
    #  xy_tol_mm：验证判定的 XY 容差（仅影响"要不要回退"，不影响实际纠偏运动补偿）。
    # [2026-10-29 统一扩大验证容差] 实测各工位验证 XY 残差多为"测量偏差"而非真实偏差
    # （SL1 实测残差 4-5mm 却屡超全局 4mm 阈值被误回退，回退到 40mm 原位反而更差）。
    # 故把所有做验证的工位容差统一扩大，减少误回退（纠偏实际已到位，残差是噪声）：
    #  - SL1：全局4mm → 8mm（实测残差4-5mm，留余量，保留对真大偏差的回退保护）
    #  - WQ ：10mm → 12mm
    #  - XL ：6mm → 10mm
    #  - WF ：保持纯开环不回退（one_step_verify=False），其残差系统性~13.9mm 不可信。
    # [2026-10-29 WF 关旋转补偿] 实测 WF 翻转根因：纠偏带 rry(-2~-4.8°) 旋转补偿把末端
    # 姿态转歪，垂直下放保持歪姿态→料条斜放翻转（rry 越大越翻，-4.78° 那轮翻转）。
    # WF 的 rotation 读数系测量偏差非真实姿态偏差（历史结论），故 WF 关 rot_comp：
    # 只纠平移(XY+Z)不纠姿态，垂直下放时保持原始正姿态，消除翻转。
    # [2026-10-29 WQ 容差 12→18] WQ 实测回退残差 13.6/17.0mm（Z向单目噪声大所致），
    # 扩大到 18 覆盖，减少误回退。
    if station_label == 'WQ':
        st = dict(rot_comp=True, zero_z=True, xy_tol_mm=18.0, one_step_verify=True)
    elif station_label == 'WF':
        st = dict(rot_comp=False, zero_z=False, xy_tol_mm=12.0, one_step_verify=False)
    elif station_label == 'XL':
        # [2026-10-29 XL 关旋转补偿 + 垂直下放] XL 放错根因同 WF：纠偏带 rry≈-3° 旋转补偿
        # 转歪姿态 + XL2 关节插值斜向下。关 rot_comp 只纠平移；XL2 由启动器拦截为垂直下放。
        # [方案A 修挂挡板] 容差10mm的副作用：动作前'达标判定'也用该容差，导致 5-8mm 的
        # Y(左右)偏差被判达标不纠偏，残留偏差蹭左右挡板。改 xy_tol=4mm：Y>4mm 就纠偏
        # 对准，消除挂挡板；one_step_verify=False 纯开环不回退(避免纠后又因验证残差回退)。
        st = dict(rot_comp=False, zero_z=False, xy_tol_mm=4.0, one_step_verify=False)
    else:  # SL1
        st = dict(rot_comp=True, zero_z=False, xy_tol_mm=8.0, one_step_verify=True)

    # [节拍优化-视觉常驻 方案A] VISION_USE_SERVER=1 时，主控【进程内直接连 socket】
    # 向常驻视觉服务发请求，不再起客户端子进程——省掉每工位 python 冷启动 + source env.sh
    # 的 ~3s 外壳开销（服务端内部纠偏仅 ~1.3s）。socket 不可用/异常时自动降级为原
    # subprocess 直调 vision_align_safe_115.py，保留安全兜底。契约(rc 0/2/1/3)不变。
    _use_server = os.environ.get('VISION_USE_SERVER', '') == '1'
    if _use_server:
        rc = _align_via_socket(ref, station_label, st)
        if rc is not None:
            print(f'[onestep-{station_label}] 一步到位(socket) rc={rc}')
            if rc in (0, 2):
                return rc == 0
            raise RuntimeError(f'{station_label}一步到位危险/异常，禁止继续（rc={rc}）')
        # rc is None -> socket 不可用，落到下面 subprocess 降级
        print(f'[onestep-{station_label}] 常驻服务不可用，降级直调原脚本')

    # 降级 / 未开启常驻：subprocess 直调原脚本 vision_align_safe_115.py（与历史一致）。
    script = os.path.join(here, 'vision_align_safe_115.py')
    extra = ' --rot-comp'
    if st['zero_z']:
        extra += ' --zero-z'
    if st['xy_tol_mm'] is not None:
        extra += f" --xy-tol-mm {st['xy_tol_mm']:g}"
    # 验证开关：True→一步到位+验证回退；False→纯开环不回退（方案2，WF 用）。
    step_flag = '--one-step-verify' if st.get('one_step_verify', True) else '--one-step'
    # [2026-10-23] 每工位纠偏前模式自愈：厂商侧运行中会把控制模式改回力控(3)，
    # 视觉层会直接 failed_before_motion。先确保位控，失败则中止本工位（不瞎跑）。
    command = (
        'set +u; source /home/agi/app/env.sh /home/agi/app >/dev/null 2>&1; '
        f'python3 {mode_script} && '
        f'python3 {script} --reference {ref} --execute-align {step_flag}{extra}'
    )
    proc = subprocess.run(
        ['bash', '-lc', command], capture_output=True, text=True, timeout=150,
    )
    for line in (proc.stdout or '').splitlines():
        if '[safe-align115]' in line or 'VISION_ALIGN_SAFE_115_RESULT' in line:
            print(f'[onestep-{station_label}] ' + line)
    rc = proc.returncode
    print(f'[onestep-{station_label}] 一步到位 rc={rc}')
    if rc in (0, 2):
        return rc == 0
    raise RuntimeError(f'{station_label}一步到位危险/异常，禁止继续（rc={rc}）')


# 视觉常驻服务 socket 路径（与 vision_align_server_115.py 一致）。
_VISION_SOCK_PATH = '/tmp/vision_align_115.sock'
# 最近一次视觉纠偏的服务端内部纯耗时（秒）；由 _align_via_socket 写、节拍记录器读。
_LAST_VISION_SERVER_S = None


def _align_via_socket(ref_abspath, station_label, st,
                      connect_timeout=3.0, request_timeout=150.0):
    """进程内直连常驻视觉服务 socket，发一次纠偏请求。

    返回：
      int rc  —— 服务已处理，返回码语义与原脚本一致（0 对准/2 安全/1,3 危险）。
      None    —— socket 不存在/连接失败/通信异常/空回包（调用方据此降级 subprocess）。
    不抛异常（除非上层对 rc=1/3 决定抛）；任何通信问题都转成 None 走降级，绝不中断产线。
    """
    import socket
    if not os.path.exists(_VISION_SOCK_PATH):
        return None
    req = {
        'reference': os.path.abspath(ref_abspath),
        'rot_comp': bool(st.get('rot_comp', True)),
        'xy_tol_mm': st.get('xy_tol_mm', None),
        'zero_z': bool(st.get('zero_z', False)),
        'one_step_verify': bool(st.get('one_step_verify', True)),
        'station': station_label,
    }
    try:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.settimeout(connect_timeout)
        s.connect(_VISION_SOCK_PATH)
        s.settimeout(request_timeout)
        s.sendall((json.dumps(req, ensure_ascii=False) + '\n').encode('utf-8'))
        buf = b''
        while b'\n' not in buf:
            chunk = s.recv(65536)
            if not chunk:
                break
            buf += chunk
        s.close()
        if not buf:
            print(f'[onestep-{station_label}] socket 空回包，降级')
            return None
        resp = json.loads(buf.split(b'\n', 1)[0].decode('utf-8'))
    except Exception as e:
        print(f'[onestep-{station_label}] socket 调用异常({e})，降级')
        return None
    # 转发服务端日志与结果，保持与 subprocess 路径一致的可观测性。
    stderr_text = resp.get('stderr') or ''
    for line in stderr_text.splitlines():
        if '[safe-align115]' in line:
            print(f'[onestep-{station_label}] ' + line)
    result = resp.get('result')
    if result is not None:
        print(f'[onestep-{station_label}] VISION_ALIGN_SAFE_115_RESULT '
              + json.dumps(result, ensure_ascii=False))
    # 暴露服务端纯纠偏耗时给节拍记录器（区分"服务端内部"与"外壳开销"）。
    try:
        globals()['_LAST_VISION_SERVER_S'] = resp.get('server_s')
    except Exception:
        pass
    try:
        return int(resp.get('rc', 2))
    except Exception:
        return 2


def run_pick_right_task():
    """执行送料机取料和弯曲机放料。"""
    G2.GRIPPER({"right": -0.78})  # 爪子张开

    # 第一段：上料。
    # 复位（FW）与导航并行：FW 为收拢标准位姿，底盘直行至送料机时无碰撞风险；
    # SL1 挂载仍严格串行（等底盘到达后再挂载）。
    G2.TTS("从送料机上抓料，底盘运动到送料机前面")
    if not G2.GO_NOWAIT(1):
        raise RuntimeError("上料导航指令发送失败")
    if not G2.WBC("FW"):
        raise RuntimeError("FW 复位摆臂失败，中止")
    if not G2.GO_WAIT():
        raise RuntimeError("上料导航到达等待超时")
    G2.WBC("SL1")  # 挂载点：先挂到 SL1 位姿再一步到位。
    _run_align_onestep('pick_sl1_reference.json', 'SL1')
    G2.WBC("SL2")
    G2.GRIPPER({"right": -0.05})#爪子关闭
    # [节拍优化] 上料后退与右转合二为一：后退(x=-0.4)与右转(-90°)合并为一个
    # 复合底盘相对运动（边退边转），省掉中间一次停稳+启动；该复合运动仍与
    # WBC("SL3") 收臂并行执行。turn_right(TURN_RIGHT_DEG) 不再单独调用。
    if not G2.REL_NOWAIT({"x": -0.4, "yaw_rad": -math.radians(abs(TURN_RIGHT_DEG))}):
        raise RuntimeError("SL3 后退+右转指令发送失败")
    if not G2.WBC("SL3"):
        raise RuntimeError("SL3 摆臂失败，中止")
    if not G2.REL_WAIT():
        raise RuntimeError("SL3 底盘后退+右转等待超时")


    # 第二段：弯曲机放料。（夹料状态，保持串行）
    # 右转已在上一步与后退合并完成，这里直接导航到弯曲机。
    G2.TTS("向弯曲机放料，运动到设备前方")
    G2.GO(2)
    # [到位停稳] GO done 可能早于底盘物理停稳，先等停稳再挂载，避免没到位就伸手。
    time.sleep(SETTLE_AFTER_GO_S)
    G2.WBC("WF1")  # 挂载点：先挂到 WF1 位姿再一步到位。
    _run_align_onestep('place_reference.json', 'WF')
    G2.GRIPPER({"right": -0.78})#爪子张开
    G2.WBC("WF2")
    # [节拍优化] 放料完成（空爪）后，弯曲机前后退与 WF3 收臂并行执行：
    # 后退用 REL_NOWAIT 异步下发，立刻并行跑 WBC("WF3")，最后 REL_WAIT 汇合。
    # （WF 段导航与 WF1 挂载并行已现场验证会撞，未采用，保持 GO(2) 同步。）
    G2.TTS("在弯曲机上放料完成，后退")
    if not G2.REL_NOWAIT({"x": -0.2}):  # 弯曲机放料后退 400mm→200mm
        raise RuntimeError("WF 后退指令发送失败")
    if not G2.WBC("WF3"):
        raise RuntimeError("WF3 摆臂失败，中止")
    if not G2.REL_WAIT():
        raise RuntimeError("WF 底盘后退等待超时")


def run_place_middle_task():
    """执行弯曲机取料和下料机放料。"""
    # 第三段：弯曲机取料。（导航与挂载并行）
    G2.TTS("从弯曲机上取料，运动到设备前方")
    if not G2.GO_NOWAIT(2):
        raise RuntimeError("WQ 导航指令发送失败")
    # WQ 保留挂载点（Z 向单目噪声大）：挂载与导航并行。
    mount_ok = G2.WBC("WQ1")
    if not G2.GO_WAIT():
        raise RuntimeError("WQ 导航到达等待超时")
    if not mount_ok:
        raise RuntimeError("WQ 挂载摆臂失败，中止")
    _run_align_onestep('pick_wq_reference.json', 'WQ')
    G2.WBC("WQ2")
    G2.GRIPPER({"right": -0.05})#爪子关闭
    if not G2.REL_NOWAIT({"x": -0.4}):
        raise RuntimeError("WQ 后退指令发送失败")
    if not G2.WBC("WQ3"):
        raise RuntimeError("WQ3 摆臂失败，中止")
    if not G2.REL_WAIT():
        raise RuntimeError("WQ 底盘后退等待超时")

    # 第四段：下料。（夹料状态，保持串行）
    G2.TTS("从弯曲机上取料完成，后退至下料机")
    G2.GO(3)
    # [到位停稳] GO done 可能早于底盘物理停稳，先等停稳再挂载，避免没到位就伸手。
    time.sleep(SETTLE_AFTER_GO_S)
    # XL 保留挂载点：无挂载时相机看不到码板（0/20合格帧），必须先挂到 XL1。
    G2.WBC("XL1")
    _run_align_onestep('pick_xl_reference.json', 'XL')
    G2.GRIPPER({"right": -0.78})#爪子张开
    G2.WBC("XL2")
    # [节拍优化] 下料机下料完成后（张爪放料、空爪状态），底盘后退(x=-0.4)与手臂
    # XL3 收臂动作同时执行：后退用 REL_NOWAIT 异步下发，立刻并行跑 WBC("XL3")，
    # 最后 REL_WAIT 等后退到位汇合。两者并行，省去串行等待。
    if not G2.REL_NOWAIT({"x": -0.4}):
        raise RuntimeError("XL 后退指令发送失败")
    if not G2.WBC("XL3"):
        raise RuntimeError("XL3 摆臂失败，中止")
    if not G2.REL_WAIT():
        raise RuntimeError("XL 底盘后退等待超时")
    G2.TTS("下料完成，后退")


def _auto_rebind_references():
    """启动时自动把视觉参考的 boot_id 更新为当前开机（免重新示教）。

    实测本台 G2 重启不改变相机外参/TF/末端标定，故旧参考数值重启后仍有效，
    仅需把参考里绑定的 boot_id 改成当前开机即可复用。本函数在主程序启动时
    自动执行，重启后直接跑主程序即可，无需手动 rebind 或重新对位示教。

    若某次维护物理动过相机/工装，应对相应工位重新 teach（本函数不做标定校验）。
    """
    try:
        import json
        here = os.path.dirname(os.path.abspath(__file__))
        boot = open('/proc/sys/kernel/random/boot_id').read().strip()
        new_stream = f'{boot}/head_color'
        for name in ('pick_sl1_reference.json', 'pick_wq_reference.json',
                     'pick_xl_reference.json', 'place_reference.json'):
            p = os.path.join(here, name)
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
        # rebind 失败不影响主流程；纠偏会因身份不符自动回退固定点位
        print(f"[vision] 自动 rebind 失败（纠偏将回退固定点位）：{e}")


def main():
    global _running

    # 重启后自动把视觉参考绑定到当前开机（纯文件改写，无 GDK 依赖），免重新示教
    _auto_rebind_references()

    # 先启动“启动中”心跳并置位，等待 PLC 回送自动信号；握手完成前不执行动作。
    threading.Thread(target=enabled_heartbeat, daemon=True).start()

    try:
        G2.setData("G2_eanbled", 1)
        # 清除可能残留的完成信号（上一轮被中途停止时可能残留为1）
        G2.setData("pick_right_done", 0)
        G2.setData("place_middle_done", 0)
        G2.TTS("机器人已就绪，等待总控自动信号")

        # PLC 可能要先收到 G2_eanbled=1 才置 plc_alive=1。
        # 因此启动阶段先等待握手，不能提前启动会因 plc_alive=0 停机的看门狗。
        wait_for_plc_ready()
        threading.Thread(target=comm_watchdog, daemon=True).start()

        cycle = 0
        while True:
            cycle += 1
            print(f"[loop] ===== 第 {cycle} 轮循环开始 =====")
            G2.TTS(f"第{cycle}轮循环开始")

            # pick_right=1：执行送料机取料和弯曲机放料。
            G2.TTS("等待PLC上料信号")
            wait_signal("pick_right")
            # 取料完成(pick_right_done)：从上料机取料起置1，动作全程持续发送
            G2.setData("pick_right_done", 1)
            run_pick_right_task()
            # 退出弯曲机后保持取料完成(0.2)，等总控下发 pick_right_release(0.4)=1 才断开
            wait_signal("pick_right_release")
            # 收到总控断开取料完成信号，断开取料完成(pick_right_done)
            G2.setData("pick_right_done", 0)

            # place_middle=1：执行弯曲机取料和下料机放料。
            G2.TTS("等待PLC下料信号")
            wait_signal("place_middle")
            # 放料完成(place_middle_done)：从弯曲机取料起置1，动作全程持续发送
            G2.setData("place_middle_done", 1)
            run_place_middle_task()
            # 退出下料机后保持放料完成(0.3)，等总控下发 place_middle_release(2.5)=1 才断开
            wait_signal("place_middle_release")
            # 收到总控断开放料完成信号，断开放料完成(place_middle_done)
            G2.setData("place_middle_done", 0)

            print(f"[loop] ===== 第 {cycle} 轮循环完成 =====")
            G2.TTS(f"第{cycle}轮循环完成")

    finally:
        # 程序结束（正常或异常）时，才复位“启动中”=0，通知总控机器人退出运行。
        _running = False
        try:
            # 无论正常结束还是异常/中途退出，都清掉两个完成信号
            G2.setData("pick_right_done", 0)
            G2.setData("place_middle_done", 0)
            G2.setData("G2_eanbled", 0)
        except Exception:
            pass


if __name__ == "__main__":
    main()