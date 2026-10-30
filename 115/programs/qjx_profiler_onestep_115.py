#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
qjx_profiler_onestep_115.py —— 115 QJX onestep 主控【节拍记录器 v2】（精准/可对账）

相对 v1 的改进（本次重设计目标：记录的时间精准且有效）：
  1. 双时钟：
       - 时长测量一律用 time.monotonic()（不受系统时间/NTP 跳变影响，精准）；
       - time.time() 仅用于人读的墙钟时间戳（CSV 的 wall_start 列）。
  2. 墙钟区间并集（解决并行重复计数）：
       底盘 NOWAIT + 上半身 WBC 并行时，两段物理上【时间重叠】。v1 把二者耗时
       分别全额计入会虚高、掩盖并行收益。v2 记录每个动作的真实 [start,end] 区间，
       汇总时对所有动作区间求【并集】得到"动作实际占用的墙钟"，并单独报告
       "并行重叠时间"（= 各段时长之和 − 并集），直观体现并行省了多少。
  3. 可对账：每轮
       net（机器人净耗时） = 轮墙钟总时长 − PLC等待
       net = busy_union（动作并集墙钟） + idle_gap（未被任何动作覆盖的间隙）
       三者在 CSV 里都有，busy_union + idle_gap == net（可核对）。
  4. 视觉细分：socket 路径额外记录"服务端内部纠偏耗时"(server_s)与"墙钟"(wall_s)，
       差值即"外壳开销"(python/socket)，便于判断视觉优化是否落到实处。
  5. 并行语义清晰：NOWAIT 只记"发出点"(dur≈0, extra=nowait_issue, 不参与并集)；
       真正占墙钟的是其后的 WBC 与配对 WAIT，均以真实区间入账。

CSV 输出：
  logs/cycle_detail_115.csv   逐条 event（新增 mono_start/mono_end/wall_start 列）
  logs/cycle_summary_115.csv  每轮汇总（新增 busy_union_s / idle_gap_s / overlap_s / vision_server_s）
不改变任何动作逻辑与返回值（纯 monkey-patch 观测）。
"""
import functools
import json
import os
import time

DETAIL_CSV = "/data/wxf0721/runtime/programs/logs/cycle_detail_115.csv"
SUMMARY_CSV = "/data/wxf0721/runtime/programs/logs/cycle_summary_115.csv"

CYCLE_START_SIGNAL = "pick_right"
CYCLE_END_SIGNAL = "place_middle_release"

# 参与"动作墙钟并集"的类别（真实占用机器人时间的动作）。
BUSY_KINDS = ["go", "rel_back", "rel_turn", "wbc", "grip", "vision"]
# 汇总展示的分段类别（含 tts；tts 走同一命令通道、串行，占时极小）。
SEG_KINDS = ["plc_wait", "go", "rel_back", "rel_turn", "wbc", "grip", "vision", "tts"]


def _mono():
    """单调时钟：测时长用，不受系统时间调整影响。"""
    return time.monotonic()


def _wall():
    """墙钟时间戳：仅人读用。"""
    return time.time()


def _interval_union(intervals):
    """对 [(start,end), ...] 求并集总长度（去除重叠）。"""
    if not intervals:
        return 0.0
    segs = sorted((a, b) for a, b in intervals if b > a)
    if not segs:
        return 0.0
    total = 0.0
    cur_s, cur_e = segs[0]
    for s, e in segs[1:]:
        if s > cur_e:
            total += cur_e - cur_s
            cur_s, cur_e = s, e
        else:
            cur_e = max(cur_e, e)
    total += cur_e - cur_s
    return total


class OnestepProfiler:
    def __init__(self, main_module, g2_instance,
                 detail_csv=DETAIL_CSV, summary_csv=SUMMARY_CSV):
        self.mod = main_module
        self.g2 = g2_instance
        self.detail_csv = detail_csv
        self.summary_csv = summary_csv
        self._orig = {}
        self.cycle = 0
        self.seq = 0
        self.cur_events = []
        self.cycle_m0 = None    # 轮起点（monotonic）
        self.cycle_w0 = None    # 轮起点（wall，人读）
        self._detail_inited = False
        self._summary_inited = False
        # 最近一次 REL_NOWAIT 的分类，供配对 REL_WAIT 归类
        self._pending_rel_kind = "rel_back"
        self._pending_rel_extra = ""

    # ── 轮次控制 ─────────────────────────────────────────
    def _begin_cycle(self):
        # 轮间游离事件（上一轮结束后、本轮 pick_right 前的 TTS 等）直接丢弃，不成伪轮。
        if self.cur_events:
            dropped = len(self.cur_events)
            self.cur_events = []
            print(f"[onestep-prof] 丢弃轮间游离事件 {dropped} 条（非完整轮）", flush=True)
        self.cycle += 1
        self.seq = 0
        self.cur_events = []
        self.cycle_m0 = _mono()
        self.cycle_w0 = _wall()
        print(f"\n===============  第 {self.cycle} 轮开始（节拍记录 v2）  ===============", flush=True)

    def _flush_cycle(self):
        if not self.cur_events:
            return
        total = (_mono() - self.cycle_m0) if self.cycle_m0 is not None else 0.0

        # 分类求和（各段"时长之和"，并行会被重复计入——仅用于分段展示）
        seg_sum = {k: [0.0, 0] for k in SEG_KINDS}
        busy_intervals = []           # 动作墙钟区间（求并集）
        plc = 0.0
        vision_server = 0.0           # 视觉服务端内部纯纠偏时间合计
        for ev in self.cur_events:
            k = ev["kind"]
            if k in seg_sum:
                seg_sum[k][0] += ev["dur_s"]
                seg_sum[k][1] += 1
            if k == "plc_wait":
                plc += ev["dur_s"]
            if k in BUSY_KINDS and not ev.get("issue"):
                busy_intervals.append((ev["m_start"], ev["m_end"]))
            if k == "vision" and ev.get("server_s") is not None:
                vision_server += ev["server_s"]
            self._write_detail(ev)

        net = total - plc                              # 机器人净耗时（权威）
        busy_union = _interval_union(busy_intervals)   # 动作实际占用墙钟（去重叠）
        idle_gap = max(0.0, net - busy_union)          # 未被动作覆盖的间隙
        overlap = sum(seg_sum[k][0] for k in BUSY_KINDS) - busy_union  # 并行省下的重叠

        # 打印汇总
        print(f"\n──────── 第 {self.cycle} 轮 汇总(v2) ────────", flush=True)
        print(f"  墙钟总 {total:6.2f}s | PLC等待 {plc:6.2f}s | 机器人净 {net:6.2f}s", flush=True)
        print(f"  动作并集 {busy_union:6.2f}s | 间隙 {idle_gap:6.2f}s | 并行重叠(已省) {overlap:6.2f}s"
              f"  [校验 并集+间隙={busy_union + idle_gap:.2f} 应≈净{net:.2f}]", flush=True)
        for k in ["go", "rel_back", "rel_turn", "wbc", "grip", "vision", "tts"]:
            d, n = seg_sum[k]
            pct = (d / net * 100.0) if net > 0 else 0.0
            line = f"    {k:9s}: {d:6.2f}s ({n:2d}次, 占净 {pct:4.1f}%)"
            if k == "vision" and vision_server > 0:
                line += f"  [服务端内部 {vision_server:.2f}s, 外壳 {d - vision_server:.2f}s]"
            print(line, flush=True)
        print("────────────────────────────────\n", flush=True)

        self._write_summary(total, plc, net, busy_union, idle_gap, overlap,
                            vision_server, seg_sum)
        self.cur_events = []

    def finalize(self):
        # 仅当缓冲是一轮"真实作业"（含 pick_right 起点）时才 flush，避免写伪轮。
        has_start = any(
            ev.get("kind") == "plc_wait" and str(ev.get("param", "")).startswith(CYCLE_START_SIGNAL)
            for ev in self.cur_events
        )
        if has_start:
            self._flush_cycle()
        else:
            self.cur_events = []

    # ── CSV ──────────────────────────────────────────────
    def _write_detail(self, ev):
        cols = ["cycle", "seq", "kind", "param", "wall_start",
                "mono_start", "mono_end", "dur_s", "server_s", "result", "extra"]
        try:
            os.makedirs(os.path.dirname(self.detail_csv), exist_ok=True)
            new = not os.path.exists(self.detail_csv)
            with open(self.detail_csv, "a", encoding="utf-8") as f:
                if new and not self._detail_inited:
                    f.write(",".join(cols) + "\n")
                    self._detail_inited = True
                row = {
                    "cycle": ev["cycle"], "seq": ev["seq"], "kind": ev["kind"],
                    "param": ev["param"], "wall_start": "%.3f" % ev["w_start"],
                    "mono_start": "%.3f" % ev["m_start"], "mono_end": "%.3f" % ev["m_end"],
                    "dur_s": "%.3f" % ev["dur_s"],
                    "server_s": ("%.3f" % ev["server_s"]) if ev.get("server_s") is not None else "",
                    "result": ev["result"], "extra": ev.get("extra", ""),
                }
                f.write(",".join(str(row[c]).replace(",", ";") for c in cols) + "\n")
        except Exception as e:
            print("[onestep-prof] 写 detail csv 失败:", e, flush=True)

    def _write_summary(self, total, plc, net, busy_union, idle_gap, overlap,
                       vision_server, seg_sum):
        seg = ["go", "rel_back", "rel_turn", "wbc", "grip", "vision", "tts"]
        cols = (["cycle", "total_s", "plc_wait_s", "robot_s",
                 "busy_union_s", "idle_gap_s", "overlap_s", "vision_server_s"]
                + sum([[f"{k}_s", f"{k}_n"] for k in seg], []))
        try:
            os.makedirs(os.path.dirname(self.summary_csv), exist_ok=True)
            new = not os.path.exists(self.summary_csv)
            with open(self.summary_csv, "a", encoding="utf-8") as f:
                if new and not self._summary_inited:
                    f.write(",".join(cols) + "\n")
                    self._summary_inited = True
                row = [self.cycle, "%.3f" % total, "%.3f" % plc, "%.3f" % net,
                       "%.3f" % busy_union, "%.3f" % idle_gap, "%.3f" % overlap,
                       "%.3f" % vision_server]
                for k in seg:
                    row += ["%.3f" % seg_sum[k][0], seg_sum[k][1]]
                f.write(",".join(str(x) for x in row) + "\n")
        except Exception as e:
            print("[onestep-prof] 写 summary csv 失败:", e, flush=True)

    # ── 记录一条 event ───────────────────────────────────
    def _record(self, kind, param, m0, m1, w0, result,
                extra="", issue=False, server_s=None):
        self.seq += 1
        ev = {"cycle": self.cycle, "seq": self.seq, "kind": kind, "param": param,
              "w_start": w0, "m_start": m0, "m_end": m1,
              "dur_s": round(m1 - m0, 3), "result": result,
              "extra": extra, "issue": issue, "server_s": server_s}
        self.cur_events.append(ev)
        return ev

    # ── 通用包装 ─────────────────────────────────────────
    def _wrap_bound(self, kind, bound, param_of=lambda a: a):
        orig = bound

        @functools.wraps(orig)
        def wrapper(arg, *a, **k):
            m0, w0 = _mono(), _wall()
            res = orig(arg, *a, **k)
            self._record(kind, param_of(arg), m0, _mono(), w0, res)
            return res
        return wrapper

    def _classify_rel(self, data):
        has_x = isinstance(data, dict) and "x" in data
        has_yaw = isinstance(data, dict) and "yaw_rad" in data
        if has_x and has_yaw:
            return "rel_back", "merged_back_turn"
        if has_yaw:
            return "rel_turn", ""
        return "rel_back", ""

    def _wrap_rel_sync(self, bound):
        orig = bound

        @functools.wraps(orig)
        def wrapper(data, *a, **k):
            kind, extra = self._classify_rel(data)
            m0, w0 = _mono(), _wall()
            res = orig(data, *a, **k)
            self._record(kind, data, m0, _mono(), w0, res, extra=extra)
            return res
        return wrapper

    def _wrap_go_sync(self, bound):
        orig = bound

        @functools.wraps(orig)
        def wrapper(num, *a, **k):
            m0, w0 = _mono(), _wall()
            res = orig(num, *a, **k)
            self._record("go", num, m0, _mono(), w0, res)
            return res
        return wrapper

    # ── 异步底盘：NOWAIT 只记发出点(不入并集)；WAIT 记真实等待 ─
    def _wrap_go_nowait(self, bound):
        orig = bound

        @functools.wraps(orig)
        def wrapper(num, *a, **k):
            m0, w0 = _mono(), _wall()
            res = orig(num, *a, **k)
            self._pending_go_m0 = m0
            self._pending_go_w0 = w0
            self._record("go", num, m0, _mono(), w0, res,
                         extra="nowait_issue", issue=True)
            return res
        return wrapper

    def _wrap_go_wait(self, bound):
        orig = bound

        @functools.wraps(orig)
        def wrapper(*a, **k):
            call_m0 = _mono()
            res = orig(*a, **k)
            m1 = _mono()
            # 导航异步真实占用墙钟 = [GO_NOWAIT 发出, 本 WAIT 结束]，与期间 WBC 并行重叠。
            m0 = getattr(self, '_pending_go_m0', call_m0)
            w0 = getattr(self, '_pending_go_w0', _wall())
            self._record("go", "GO_WAIT", m0, m1, w0, res, extra="wait")
            return res
        return wrapper

    def _wrap_rel_nowait(self, bound):
        orig = bound

        @functools.wraps(orig)
        def wrapper(data, *a, **k):
            self._pending_rel_kind, self._pending_rel_extra = self._classify_rel(data)
            m0, w0 = _mono(), _wall()
            res = orig(data, *a, **k)
            # 记下发出时刻，供配对 REL_WAIT 用"[发出, 等待结束]"作为底盘真实占用区间。
            self._pending_rel_m0 = m0
            self._pending_rel_w0 = w0
            # 发出点本身 dur≈0、不入并集（issue）。
            self._record(self._pending_rel_kind, data, m0, _mono(), w0, res,
                         extra="nowait_issue", issue=True)
            return res
        return wrapper

    def _wrap_rel_wait(self, bound):
        orig = bound

        @functools.wraps(orig)
        def wrapper(*a, **k):
            kind = self._pending_rel_kind
            extra = ("wait;" + self._pending_rel_extra) if self._pending_rel_extra else "wait"
            call_m0 = _mono()
            res = orig(*a, **k)
            m1 = _mono()
            # 底盘异步动作的真实占用墙钟 = [NOWAIT 发出, 本 WAIT 结束]。
            # 这段会与期间并行的 WBC 区间重叠，使 overlap 正确反映并行收益。
            m0 = getattr(self, '_pending_rel_m0', call_m0)
            w0 = getattr(self, '_pending_rel_w0', _wall())
            self._record(kind, "REL_WAIT", m0, m1, w0, res, extra=extra)
            return res
        return wrapper

    # ── wait_signal（轮次边界 + PLC 等待） ────────────────
    def _wrap_wait_signal(self, func):
        @functools.wraps(func)
        def wrapper(name, value=1, *a, **k):
            if name == CYCLE_START_SIGNAL and value == 1:
                self._begin_cycle()
            m0, w0 = _mono(), _wall()
            res = func(name, value, *a, **k)
            self._record("plc_wait", f"{name}={value}", m0, _mono(), w0, "ok")
            if name == CYCLE_END_SIGNAL:
                self._flush_cycle()
            return res
        return wrapper

    # ── 视觉一步到位（记墙钟 + 服务端内部耗时） ───────────
    def _wrap_align(self, func):
        @functools.wraps(func)
        def wrapper(reference_name, station_label, *a, **k):
            m0, w0 = _mono(), _wall()
            res = func(reference_name, station_label, *a, **k)
            m1 = _mono()
            # 尝试从主控进程记录的"最近一次服务端内部耗时"读取（若主控暴露）。
            server_s = getattr(self.mod, '_LAST_VISION_SERVER_S', None)
            try:
                self.mod._LAST_VISION_SERVER_S = None
            except Exception:
                pass
            self._record("vision", station_label, m0, m1, w0, res,
                         extra=f"ref={reference_name}",
                         server_s=(float(server_s) if server_s is not None else None))
            return res
        return wrapper

    # ── 安装 / 卸载 ──────────────────────────────────────
    def install(self):
        g2 = self.g2
        mod = self.mod

        def patch_bound(name, wrapped):
            if hasattr(g2, name):
                self._orig[("g2", name)] = getattr(g2, name)
                setattr(g2, name, wrapped(getattr(g2, name)))

        patch_bound("GO", self._wrap_go_sync)
        patch_bound("REL", self._wrap_rel_sync)
        patch_bound("GO_NOWAIT", self._wrap_go_nowait)
        patch_bound("GO_WAIT", self._wrap_go_wait)
        patch_bound("REL_NOWAIT", self._wrap_rel_nowait)
        patch_bound("REL_WAIT", self._wrap_rel_wait)
        if hasattr(g2, "WBC"):
            self._orig[("g2", "WBC")] = g2.WBC
            g2.WBC = self._wrap_bound("wbc", g2.WBC)
        if hasattr(g2, "GRIPPER"):
            self._orig[("g2", "GRIPPER")] = g2.GRIPPER
            g2.GRIPPER = self._wrap_bound("grip", g2.GRIPPER)
        if hasattr(g2, "TTS"):
            self._orig[("g2", "TTS")] = g2.TTS
            g2.TTS = self._wrap_bound("tts", g2.TTS)

        if hasattr(mod, "wait_signal"):
            self._orig["wait_signal"] = mod.wait_signal
            mod.wait_signal = self._wrap_wait_signal(mod.wait_signal)
        if hasattr(mod, "_run_align_onestep"):
            self._orig["_run_align_onestep"] = mod._run_align_onestep
            mod._run_align_onestep = self._wrap_align(mod._run_align_onestep)

        print("[onestep-prof] 已安装节拍记录 v2（monotonic 时钟 + 墙钟并集 + 可对账）", flush=True)

    def uninstall(self):
        for key, orig in self._orig.items():
            if isinstance(key, tuple) and key[0] == "g2":
                setattr(self.g2, key[1], orig)
            else:
                setattr(self.mod, key, orig)
        self._orig.clear()
