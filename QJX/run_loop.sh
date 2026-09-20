#!/bin/bash
# run_loop.sh —— 前台启动 QJX 循环版主程序（Ctrl+C 可随时中断）。
# glog 进 /data/QJX/logs/；屏幕输出同时 tee 到带时间戳的日志。
# 用法: bash /data/QJX/run_loop.sh

QJX_DIR=/data/QJX
LOG_DIR="$QJX_DIR/logs"
mkdir -p "$LOG_DIR"

source /home/agi/app/env.sh /home/agi/app >/dev/null 2>&1
export GLOG_log_dir="$LOG_DIR"

TS=$(date +%Y%m%d_%H%M%S)
RUN_LOG="$LOG_DIR/run_loop_${TS}.log"

cd "$QJX_DIR"
echo "[run_loop] 前台循环运行（Ctrl+C 停止），日志: $RUN_LOG"
# 前台运行 + tee 落盘；Ctrl+C 会传给 python 触发干净停机
python3 -u all_machin_plc_vision_2_loop.py 2>&1 | tee "$RUN_LOG"
