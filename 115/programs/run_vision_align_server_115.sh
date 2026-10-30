#!/bin/bash
# run_vision_align_server_115.sh —— 启动视觉纠偏常驻服务（前台，Ctrl+C 停）。
# 服务常驻 GDKBackend + Minth.G2，监听 /tmp/vision_align_115.sock。
# ⚠ 启动本服务后，主控的视觉调用应走 vision_align_client_115.py（见 VISION_USE_SERVER）。
#   同一时刻不要再有别的 GDKBackend 进程（会抢 /tmp/apriltag_right_arm_jog.lock）。
PROG_DIR=/data/wxf0721/runtime/programs
LOG_DIR="$PROG_DIR/logs"
mkdir -p "$LOG_DIR"
set +u
source /home/agi/app/env.sh /home/agi/app >/dev/null 2>&1
export GLOG_log_dir="$LOG_DIR"
TS=$(date +%Y%m%d_%H%M%S)
RUN_LOG="$LOG_DIR/vision_align_server_115_${TS}.log"
cd "$PROG_DIR"
echo "[vision-server115] 前台启动（Ctrl+C 停），日志: $RUN_LOG"
echo "[vision-server115] socket: /tmp/vision_align_115.sock"
python3 -u vision_align_server_115.py 2>&1 | tee "$RUN_LOG"
