#!/bin/bash
# run_service.sh —— 启动机器人控制服务端 (services/main.py)，日志统一进 /data/QJX/logs/
# 用法:
#   bash /data/QJX/run_service.sh            # 后台常驻启动
#   bash /data/QJX/run_service.sh fg         # 前台启动（调试用，Ctrl+C 会停）
set -e

QJX_DIR=/data/QJX
LOG_DIR="$QJX_DIR/logs"
SVC_DIR=/data/wxf0721/services
mkdir -p "$LOG_DIR"

# 加载机器人运行环境
source /home/agi/app/env.sh /home/agi/app >/dev/null 2>&1

# 让 GDK/glog 的 python3.10.log.INFO.* 直接写到 logs 目录
export GLOG_log_dir="$LOG_DIR"

# ★ 防叠加：启动前先清掉所有已存在的服务端，保证同一时刻只有一个
#   （多个服务端会互抢 S7/GDK/MQTT，导致读不到 plc_alive、夹爪乱等问题）
EXIST=$(ps aux | grep python | grep -F 'main.py' | grep -v grep | grep -v 'all_machin' | awk '{print $2}')
if [ -n "$EXIST" ]; then
    echo "[run_service] 发现已存在服务端进程，先清理: $EXIST"
    kill $EXIST 2>/dev/null; sleep 3
    EXIST=$(ps aux | grep python | grep -F 'main.py' | grep -v grep | grep -v 'all_machin' | awk '{print $2}')
    [ -n "$EXIST" ] && { echo "[run_service] 强制清理: $EXIST"; kill -9 $EXIST 2>/dev/null; sleep 1; }
fi

TS=$(date +%Y%m%d_%H%M%S)
STDOUT_LOG="$LOG_DIR/service_${TS}.log"

cd "$SVC_DIR"

if [ "$1" = "fg" ]; then
    echo "[run_service] 前台启动 main.py（Ctrl+C 停止），glog -> $LOG_DIR"
    exec python3 -u main.py
fi

# 后台常驻启动
nohup python3 -u main.py > "$STDOUT_LOG" 2>&1 &
PID=$!
echo "$PID" > "$QJX_DIR/run_service.pid"
echo "[run_service] 已后台启动 main.py, PID=$PID"
echo "[run_service] 标准输出日志: $STDOUT_LOG"
echo "[run_service] glog 日志目录: $LOG_DIR"

# ★ 等 S7 轮询就绪并确认真正连上 PLC（读到 plc_alive/plc_heartbeat）
echo "[run_service] 等待 S7 连接 PLC ..."
sleep 6
if ! kill -0 "$PID" 2>/dev/null; then
    echo "[run_service] ⚠️ 进程已退出，请查看 $STDOUT_LOG"
    exit 1
fi
if grep -q '\[S7\] 轮询线程已启动' "$STDOUT_LOG"; then
    echo "[run_service] ✅ S7 轮询已启动"
else
    echo "[run_service] ⚠️ 未见 [S7] 轮询线程已启动，S7 可能未连上 PLC，请查看 $STDOUT_LOG"
fi
echo "[run_service] 进程存活，运行中... (停止: bash /data/QJX/stop_service.sh)"
