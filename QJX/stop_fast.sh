#!/bin/bash
# stop_fast.sh —— 停止 QJX 提速版主程序（无论前台/后台起的，都能停干净）。
# 用法: bash /data/QJX/stop_fast.sh

QJX_DIR=/data/QJX
PIDFILE="$QJX_DIR/run_fast.pid"

stopped=0

# 1) 先按记录的 PID 停
if [ -f "$PIDFILE" ]; then
    PID=$(cat "$PIDFILE" 2>/dev/null)
    if [ -n "$PID" ] && kill -0 "$PID" 2>/dev/null; then
        echo "[stop_fast] 停止 PID=$PID ..."
        kill "$PID" 2>/dev/null
        stopped=1
    fi
    rm -f "$PIDFILE"
fi

# 2) 兜底：按进程名清理所有残留（防止之前叠了多个）
PIDS=$(ps aux | grep -F 'all_machin_plc_vision_2_fast.py' | grep -v grep | awk '{print $2}')
if [ -n "$PIDS" ]; then
    echo "[stop_fast] 清理残留主程序: $PIDS"
    kill $PIDS 2>/dev/null
    stopped=1
fi

sleep 2

# 3) 确认；仍在则强杀
LEFT=$(ps aux | grep -F 'all_machin_plc_vision_2_fast.py' | grep -v grep | awk '{print $2}')
if [ -n "$LEFT" ]; then
    echo "[stop_fast] 仍存活，强制停止: $LEFT"
    kill -9 $LEFT 2>/dev/null
    sleep 1
fi

LEFT=$(ps aux | grep -F 'all_machin_plc_vision_2_fast.py' | grep -v grep | awk '{print $2}')
if [ -z "$LEFT" ]; then
    echo "[stop_fast] ✅ 主程序已全部停止"
else
    echo "[stop_fast] ⚠️ 仍有残留: $LEFT"
fi
