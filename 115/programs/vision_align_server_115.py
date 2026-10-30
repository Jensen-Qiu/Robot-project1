#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vision_align_server_115.py —— 视觉纠偏【常驻服务】（节拍优化：避免每工位重启子进程）

背景与目的
----------
原生产链路：主控每个工位(SL1/WF/WQ/XL)都 subprocess 启动一次
`vision_align_safe_115.py`，每次重做 GDK 初始化（gdk_init + Robot/TF/Pnc/Camera
构造 + 文件锁 + sleep0.5 + 首帧等待1.5s + MQTT 连接）约 2s，4 工位每轮浪费约 8s。

本服务把 `GDKBackend` 与 `Minth.G2()` 只初始化一次并常驻，循环复用处理每次纠偏请求，
省掉重复初始化开销。对外用 Unix domain socket 提供请求/响应。

重要原则（安全第一）
--------------------
1. **完全复用** `vision_align_safe_115.py` 的核心逻辑函数（measure / align_loop_one_step /
   send_offset / rollback / emit 等），**不改原脚本**。原脚本作为降级后备始终可用。
2. **契约不变**：请求参数语义、返回码 rc(0=对准/2=未对准但安全/1,3=危险异常)、
   以及 `VISION_ALIGN_SAFE_115_RESULT <json>` 结果输出，与原脚本逐一对应。
3. **串行处理**：socket 一次只处理一个请求，天然保证不并发下发右臂 OFFSET。
4. **控制模式自愈**：每次请求在 measure 前先把控制模式切回位控(1)（等价原链路的
   `python3 set_control_mode_115.py &&`）。
5. backend/相机/文件锁常驻独占 `/tmp/apriltag_right_arm_jog.lock`；服务运行期间
   不应再有别的 GDKBackend 进程（原 subprocess 链路改为走本服务的客户端）。

协议（Unix socket，行分隔 JSON）
--------------------------------
请求  (client→server) 一行 JSON：
  {"reference": "<abs path to xxx_reference.json>",
   "rot_comp": true/false, "xy_tol_mm": <float|null>,
   "zero_z": true/false, "one_step_verify": true/false,
   "station": "SL1|WF|WQ|XL"}
响应  (server→client) 一行 JSON：
  {"rc": <int>, "result": <dict|null>, "stderr": "<转发的[safe-align115]日志>"}

rc 语义与原脚本一致（0/2/1/3）。client 收到后：原样 print
`VISION_ALIGN_SAFE_115_RESULT <result_json>` 到 stdout、以 rc 退出，
从而对主控 _run_align_onestep 完全透明。

启动
----
  bash run_vision_align_server_115.sh         # 前台，Ctrl+C 停
或
  python3 vision_align_server_115.py          # 直接跑

健康/停机
---------
  SIGINT/SIGTERM：优雅关闭 backend（close 相机/gdk_release/放锁）与 socket。
"""
import json
import os
import signal
import socket
import sys
import threading
import time
import traceback
from pathlib import Path

PROGRAMS_ROOT = Path('/data/wxf0721/runtime/programs')
PKG_ROOT = Path('/home/agi/G2_AprilTag_Placement_0.2.0')
RUNTIME_ROOT = Path('/data/wxf0721/runtime')
for p in (PROGRAMS_ROOT, PKG_ROOT, RUNTIME_ROOT):
    sp = str(p)
    if sp not in sys.path:
        sys.path.insert(0, sp)

SOCK_PATH = '/tmp/vision_align_115.sock'

# 复用原脚本的全部核心逻辑（不改原脚本）。
import vision_align_safe_115 as VA  # noqa: E402


def _log(msg):
    print(f'[vision-server115] {msg}', flush=True)


class AlignServer:
    def __init__(self, sock_path=SOCK_PATH):
        self.sock_path = sock_path
        self.backend = None
        self.vision = None
        self.robot = None
        self._init_lock = threading.Lock()
        self._req_lock = threading.Lock()   # 串行处理请求
        self._backend_cm = None             # with GDKBackend 的上下文管理器
        self._board_model = None
        self._stop = False

    # ── 一次性初始化（GDK/相机/Minth 常驻） ──────────────
    def setup(self):
        _log('初始化运行时 profile ...')
        VA.apply_115_runtime_profile()
        import vision_pick_correction as vpc
        from apriltag_vision import AprilTagVision
        from g2_tag_placement.gdk_backend import GDKBackend
        from minth import Minth

        vpc._patch_fresh_tolerance()
        vpc._patch_gripper_for_pick()
        self._board_model = vpc._load_board()

        _log('打开 GDKBackend（gdk_init/相机/TF/Pnc/文件锁，仅此一次）...')
        # 手动进入上下文管理器，让 backend 常驻，直到 shutdown 才 __exit__。
        self._backend_cm = GDKBackend(self._board_model, unused_pedal_fault_confirmed=True)
        self.backend = self._backend_cm.__enter__()
        self.vision = AprilTagVision(**self.backend.vision_config())
        _log('创建常驻 Minth.G2() MQTT 连接（OFFSET 下发复用）...')
        self.robot = Minth.G2()
        _log('初始化完成，backend 已常驻。')

    # ── 控制模式自愈（等价原链路的 set_control_mode_115.py） ─
    def _ensure_control_mode(self):
        """确保关节位控(mode=1)。【进程内直接读/切 GDK，不再每次起子进程】。

        原实现每次请求都 subprocess 跑一次 set_control_mode_115.py（~2s），
        但厂商守护实测"开机数分钟后把模式改回力控一次、之后不再回弹"，稳态下
        模式不漂移，每次切纯属浪费。改为：复用常驻 backend 的 GDK robot 直接读
        control_mode，只有 ≠1 时才切一次。读/切都在进程内，毫秒级。
        失败不抛致命，由后续 snapshot 的 MC 校验兜底(rc=2)。
        """
        try:
            robot = self.backend.robot  # GDK Robot（非 Minth）
            status = robot.get_motion_control_status()
            mode = int(status.control_mode)
            if mode == 1:
                return True
            _log(f'control_mode={mode} ≠1，进程内切回位控 ...')
            rc = robot.set_control_mode(1)
            _log(f'set_control_mode(1) 返回 {rc}')
            # 简短确认
            time.sleep(0.2)
            mode2 = int(robot.get_motion_control_status().control_mode)
            _log(f'切后 control_mode={mode2}')
            return mode2 == 1
        except Exception as e:
            _log(f'control_mode 读/切异常: {e}（继续，由后续 snapshot 校验兜底）')
            return False

    # ── 处理一次纠偏请求（复用原 main 的对位主体） ─────────
    def handle(self, req):
        """复用 vision_align_safe_115 的 measure + align_loop_one_step，
        但在常驻 backend 上执行。返回 (rc, result_dict, stderr_text)。"""
        import numpy as np

        reference_path = Path(req['reference'])
        rot_comp = bool(req.get('rot_comp', False))
        xy_tol = req.get('xy_tol_mm', None)
        zero_z = bool(req.get('zero_z', False))
        one_step_verify = bool(req.get('one_step_verify', True))

        # 捕获原脚本 emit() 的结果（原脚本 emit 直接 print；这里改为捕获）。
        captured = {'result': None}
        orig_emit = VA.emit

        def _capture_emit(result):
            captured['result'] = result
        VA.emit = _capture_emit

        # 捕获 [safe-align115] stderr 日志转发给 client。
        stderr_buf = []
        orig_stderr = sys.stderr

        class _Tee:
            def write(self, s):
                stderr_buf.append(s)
                orig_stderr.write(s)

            def flush(self):
                orig_stderr.flush()
        sys.stderr = _Tee()

        rc = 2
        try:
            if not reference_path.exists():
                VA.emit({'status': 'no_reference', 'motion_sent': False})
                return 2, captured['result'], ''.join(stderr_buf)

            reference = json.loads(reference_path.read_text())
            relation = np.array(reference['board_T_right_end_reference'])

            # identity 校验（与原脚本 L475-477 一致）
            if any(reference.get(k) != v for k, v in self.vision._identity.items()):
                VA.emit({'status': 'identity_mismatch', 'motion_sent': False})
                return 2, captured['result'], ''.join(stderr_buf)

            # [2026-10-29] 旋转补偿完全听主控传入的 rot_comp，不再按"文件名含 xl 自动启用"。
            # 因为 XL 要关旋转补偿（配合垂直下放消除翻转/放偏），而其参考名含 'xl' 会被旧规则
            # 强制开启，故移除文件名自动规则。主控对各工位已显式指定 rot_comp。
            effective_rot = rot_comp

            baseline = VA.measure(self.backend, self.vision, relation)
            print('[safe-align115] 基线 ' + json.dumps(baseline, ensure_ascii=False),
                  file=sys.stderr)

            # 一步到位 + 验证（生产四工位均走此路径）
            rc = VA.align_loop_one_step(
                self.backend, self.vision, relation, self.robot,
                baseline, baseline,
                {'rx': 0.0, 'ry': 0.0, 'rz': 0.0},
                'production', rot_comp=effective_rot,
                verify=one_step_verify,
                one_step_gain=None,
                zero_z=zero_z,
                xy_tol=xy_tol,
            )
            return rc, captured['result'], ''.join(stderr_buf)
        except Exception as exc:
            # 与原脚本 except 分支一致：failed_before_motion → rc=2
            VA.emit({'status': 'failed_before_motion', 'error': str(exc),
                     'code': getattr(exc, 'code', 'ERROR'), 'motion_sent': True})
            _log('handle 异常:\n' + traceback.format_exc())
            return 2, captured['result'], ''.join(stderr_buf)
        finally:
            VA.emit = orig_emit
            sys.stderr = orig_stderr

    # ── WF 垂直下放（代替 WF2 关节插值） ─────────────────
    def _handle_drop(self, station):
        """在当前末端姿态基础上沿世界 Z 轴垂直下放该工位固定落差 dz（WF/XL 通用）。
        返回 (rc, info)。rc 语义与主控对齐：0=下放成功；2=未执行/降级(dz未标定或失败，
        调用方据此回退原关节动作 WF2/XL2)。绝不抛致命异常中断产线。
        """
        try:
            import wf_vertical_place_115 as wfd
        except Exception as e:
            return 2, f'import 垂直下放模块失败:{e}'
        dz = wfd.load_dz(station)
        if dz is None:
            return 2, f'{station} dz 未标定(回退关节动作)'
        # [闭环下放] 实测 OFFSET 执行有系统性欠程（73.6mm 只走 64-68mm），且偶发
        # 0mm 漏放（offset_move done 回执被上一条命令串扰，瞬时返回未真正执行）。
        # 故改为闭环：读 Z → 发 OFFSET(剩余量) → 等稳读 Z 验证 → 不足则补发，最多 3 次。
        import time as _t
        TOL = 0.005          # 到位容差 5mm（实降达到 dz-5mm 即算到位）
        SETTLE = 0.3         # OFFSET 后读 Z 前的回稳等待
        MAX_TRY = 3
        try:
            z0 = wfd.read_right_end_z(self.backend.robot)
        except Exception as e:
            # 读不到 Z 无法闭环验证——退一步做一次开环下放后返回（保守）
            _log(f'wf_drop 读起始 Z 失败,降级单次开环: {e}')
            try:
                ok = wfd.vertical_drop(self.robot, dz)
                return (0, f'dz={dz:.4f}m 开环(未验证)') if ok else (2, 'OFFSET未确认(回退关节动作)')
            except Exception as ee:
                return 2, f'OFFSET异常:{ee}(回退关节动作)'

        z_target = z0 - dz
        z_cur = z0
        for attempt in range(1, MAX_TRY + 1):
            remain = z_cur - z_target          # 还需再下降的量（米，正=还要降）
            if remain <= TOL:
                break
            try:
                # 用 wfd.vertical_drop 发剩余量（内部 OFFSET rz=-remain*1000，无旋转）
                ok = wfd.vertical_drop(self.robot, remain)
            except Exception as e:
                _log(f'wf_drop 第{attempt}次 OFFSET 异常: {e}')
                return 2, f'OFFSET异常:{e}(回退关节动作)'
            if not ok:
                return 2, 'OFFSET未确认(回退关节动作)'
            _t.sleep(SETTLE)
            try:
                z_cur = wfd.read_right_end_z(self.backend.robot)
            except Exception:
                break
            _log(f'wf_drop 第{attempt}次: 目标降{remain*1000:.1f}mm 后 Z={z_cur:.4f} '
                 f'(距目标 {(z_cur - z_target)*1000:.1f}mm)')

        dropped = z0 - z_cur
        info = (f'dz={dz:.4f}m Z {z0:.4f}->{z_cur:.4f} (实降{dropped:.4f}m, '
                f'残差{(z_cur - z_target)*1000:.1f}mm)')
        # 残差仍超 2 倍容差 → 判定未到位，返回 2 让主控降级原 WF2（避免高位放料）
        if (z_cur - z_target) > 2 * TOL:
            _log(f'wf_drop 补发后仍欠程 {(z_cur - z_target)*1000:.1f}mm，判未到位')
            return 2, info + ' 欠程(回退关节动作)'
        return 0, info

    # ── socket 服务循环 ──────────────────────────────────
    def serve(self):
        if os.path.exists(self.sock_path):
            os.unlink(self.sock_path)
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(self.sock_path)
        os.chmod(self.sock_path, 0o666)
        srv.listen(8)
        srv.settimeout(1.0)
        _log(f'监听 {self.sock_path}（Ctrl+C 停）')
        while not self._stop:
            try:
                conn, _ = srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with conn:
                self._handle_conn(conn)
        try:
            srv.close()
        finally:
            if os.path.exists(self.sock_path):
                os.unlink(self.sock_path)
        _log('socket 已关闭')

    def _handle_conn(self, conn):
        try:
            conn.settimeout(200.0)
            data = b''
            while b'\n' not in data:
                chunk = conn.recv(65536)
                if not chunk:
                    break
                data += chunk
            if not data:
                return
            line = data.split(b'\n', 1)[0].decode('utf-8')
            req = json.loads(line)
            if req.get('cmd') == 'ping':
                conn.sendall(b'{"pong": true}\n')
                return
            if req.get('cmd') in ('drop', 'wf_drop'):
                # 放料垂直下放：在纠偏后的末端姿态基础上，沿世界 Z 轴垂直下放工位固定落差 dz，
                # 代替原 WBC("WF2"/"XL2") 的关节插值，避免斜向下放致料条翻转/放偏。
                # station 指定工位(WF/XL)；兼容旧 cmd=wf_drop(默认 WF)。
                station = req.get('station', 'WF') or 'WF'
                with self._req_lock:
                    t0 = time.monotonic()
                    rc, info = self._handle_drop(station)
                    dt = time.monotonic() - t0
                    _log(f"{station} drop 完成 rc={rc} ({info}) 用时 {dt:.2f}s")
                conn.sendall((json.dumps(
                    {'rc': rc, 'info': info, 'server_s': round(dt, 3)}) + '\n').encode('utf-8'))
                return
            # 串行处理纠偏，避免并发下发右臂动作
            with self._req_lock:
                _log(f"请求 station={req.get('station')} ref={os.path.basename(req.get('reference',''))}"
                     f" rot_comp={req.get('rot_comp')} xy_tol={req.get('xy_tol_mm')} zero_z={req.get('zero_z')}")
                self._ensure_control_mode()
                t0 = time.monotonic()
                rc, result, stderr_text = self.handle(req)
                dt = time.monotonic() - t0   # 服务端纯纠偏耗时(不含控制模式切换)
                _log(f"完成 rc={rc} 用时 {dt:.2f}s")
            resp = {'rc': rc, 'result': result, 'stderr': stderr_text,
                    'server_s': round(dt, 3)}
            conn.sendall((json.dumps(resp, ensure_ascii=False) + '\n').encode('utf-8'))
        except Exception:
            _log('连接处理异常:\n' + traceback.format_exc())
            try:
                conn.sendall((json.dumps(
                    {'rc': 2, 'result': {'status': 'server_error'},
                     'stderr': traceback.format_exc()}) + '\n').encode('utf-8'))
            except Exception:
                pass

    # ── 优雅停机 ─────────────────────────────────────────
    def shutdown(self):
        self._stop = True
        try:
            if self._backend_cm is not None:
                self._backend_cm.__exit__(None, None, None)  # close 相机/gdk_release/放锁
                _log('GDKBackend 已释放')
        except Exception:
            _log('释放 backend 异常:\n' + traceback.format_exc())
        try:
            if self.robot is not None:
                self.robot.close()
        except Exception:
            pass


def main():
    server = AlignServer()

    def _sig(_signum, _frame):
        _log('收到停止信号，优雅关闭 ...')
        server.shutdown()

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    server.setup()
    try:
        server.serve()
    finally:
        server.shutdown()


if __name__ == '__main__':
    main()
