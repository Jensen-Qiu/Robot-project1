#!/bin/bash
# run_fast.sh —— 启动 QJX 提速版主程序 (all_machin_plc_vision_2_fast.py)，日志统一进 /data/QJX/logs/
# 用法:
#   bash /data/QJX/run_fast.sh            # 后台常驻启动
#   bash /data/QJX/run_fast.sh fg         # 前台启动（可直接看日志，Ctrl+C 停止）
set -e

QJX_DIR=/data/QJX
LOG_DIR="$QJX_DIR/logs"
mkdir -p "$LOG_DIR"

# 加载机器人运行环境
source /home/agi/app/env.sh /home/agi/app >/dev/null 2>&1

# 让 GDK/glog 的 python3.10.log.INFO.* 直接写到 logs 目录
export GLOG_log_dir="$LOG_DIR"

TS=$(date +%Y%m%d_%H%M%S)
RUN_LOG="$LOG_DIR/run_fast_${TS}.log"

cd "$QJX_DIR"

if [ "$1" = "fg" ]; then
    echo "[run_fast] 前台启动主程序（Ctrl+C 停止），glog -> $LOG_DIR"
    exec python3 -u all_machin_plc_vision_2_fast.py
fi

# 后台常驻启动
nohup python3 -u all_machin_plc_vision_2_fast.py > "$RUN_LOG" 2>&1 &
PID=$!
echo "$PID" > "$QJX_DIR/run_fast.pid"
echo "[run_fast] 已后台启动主程序, PID=$PID"
echo "[run_fast] 运行日志: $RUN_LOG"
echo "[run_fast] glog 日志目录: $LOG_DIR"
echo "[run_fast] 停止命令: bash /data/QJX/stop_fast.sh"
sleep 2
if kill -0 "$PID" 2>/dev/null; then
    echo "[run_fast] 进程存活，运行中..."
else
    echo "[run_fast] ⚠️ 进程已退出，请查看 $RUN_LOG"
fi
