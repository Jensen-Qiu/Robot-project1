#!/bin/bash
# run_vision_align_server_115_daemon.sh —— 视觉纠偏常驻服务【后台守护版】。
# 用 setsid+nohup 脱离终端运行：关终端 / 断 ssh 都不会杀掉服务
# （解决"前台跑服务、终端一关服务就死、socket 残留致主控 Connection refused"的问题）。
#
# 用法:
#   bash run_vision_align_server_115_daemon.sh start    # 后台启动（默认）
#   bash run_vision_align_server_115_daemon.sh stop     # 停止
#   bash run_vision_align_server_115_daemon.sh status   # 查看状态
#   bash run_vision_align_server_115_daemon.sh restart  # 重启
#
# ⚠ 启动前确保机器人【未插充电】（否则 GDKBackend 初始化报 POWER）。
PROG_DIR=/data/wxf0721/runtime/programs
LOG_DIR="$PROG_DIR/logs"
PID_FILE="$LOG_DIR/vision_align_server_115.pid"
SOCK=/tmp/vision_align_115.sock
mkdir -p "$LOG_DIR"

_is_running() {
    [ -f "$PID_FILE" ] || return 1
    local pid; pid=$(cat "$PID_FILE" 2>/dev/null)
    [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null
}

_start() {
    if _is_running; then
        echo "[vision-server115] 已在运行 (PID=$(cat "$PID_FILE"))，如需重启用 restart"
        return 0
    fi
    # 清理可能残留的陈旧 socket（上次被硬杀留下的），避免新服务 bind 冲突/旧 socket 误导客户端
    [ -S "$SOCK" ] && rm -f "$SOCK"
    set +u
    source /home/agi/app/env.sh /home/agi/app >/dev/null 2>&1
    export GLOG_log_dir="$LOG_DIR"
    local TS; TS=$(date +%Y%m%d_%H%M%S)
    local RUN_LOG="$LOG_DIR/vision_align_server_115_${TS}.log"
    cd "$PROG_DIR"
    # setsid 脱离控制终端 + nohup 忽略 SIGHUP：关终端/断 ssh 不影响服务
    setsid nohup python3 -u vision_align_server_115.py >"$RUN_LOG" 2>&1 &
    local pid=$!
    echo "$pid" > "$PID_FILE"
    echo "[vision-server115] 后台启动 PID=$pid，日志: $RUN_LOG"
    echo "[vision-server115] 等待初始化（GDK/相机/MQTT）..."
    # 等 socket 出现（最多 ~30s），确认初始化成功
    for i in $(seq 1 60); do
        if [ -S "$SOCK" ]; then
            echo "[vision-server115] 就绪：socket $SOCK 已监听"
            return 0
        fi
        if ! kill -0 "$pid" 2>/dev/null; then
            echo "[vision-server115] ✗ 进程已退出（初始化失败），看日志尾部："
            tail -8 "$RUN_LOG"
            rm -f "$PID_FILE"
            return 1
        fi
        sleep 0.5
    done
    echo "[vision-server115] ⚠ 30s 内未出现 socket，检查日志: $RUN_LOG"
    return 1
}

_stop() {
    if ! _is_running; then
        echo "[vision-server115] 未在运行"
        # 清理可能残留的 socket/pid
        [ -S "$SOCK" ] && rm -f "$SOCK"
        rm -f "$PID_FILE"
        return 0
    fi
    local pid; pid=$(cat "$PID_FILE")
    echo "[vision-server115] 停止 PID=$pid ..."
    kill -TERM "$pid" 2>/dev/null
    for i in $(seq 1 20); do
        kill -0 "$pid" 2>/dev/null || break
        sleep 0.3
    done
    kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null
    rm -f "$PID_FILE"
    [ -S "$SOCK" ] && rm -f "$SOCK"
    echo "[vision-server115] 已停止"
}

_status() {
    if _is_running; then
        echo "[vision-server115] 运行中 (PID=$(cat "$PID_FILE"))"
        [ -S "$SOCK" ] && echo "[vision-server115] socket 正常: $SOCK" \
                       || echo "[vision-server115] ⚠ 进程在但 socket 缺失!"
    else
        echo "[vision-server115] 未运行"
        [ -S "$SOCK" ] && echo "[vision-server115] ⚠ 发现残留 socket（无监听进程）: $SOCK"
    fi
}

case "${1:-start}" in
    start)   _start ;;
    stop)    _stop ;;
    restart) _stop; sleep 1; _start ;;
    status)  _status ;;
    *) echo "用法: $0 {start|stop|status|restart}"; exit 1 ;;
esac
