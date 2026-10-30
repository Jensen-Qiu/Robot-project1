#!/bin/bash
# run_loop_onestep_profile_115.sh —— 运行 onestep 主控 + 细化节拍记录。
# 不改任何现有主控逻辑，仅通过 run_loop_onestep_profile_115.py 挂载零副作用记录器。
#
# 用法:
#   bash run_loop_onestep_profile_115.sh            # 视觉走原脚本(vision_align_safe_115.py)
#   bash run_loop_onestep_profile_115.sh server     # 视觉走常驻服务客户端(需先启动服务)
#   VISION_USE_SERVER=1 bash run_loop_onestep_profile_115.sh   # 同上(环境变量方式)
PROG_DIR=/data/wxf0721/runtime/programs
LOG_DIR="$PROG_DIR/logs"
mkdir -p "$LOG_DIR"
source /home/agi/app/env.sh /home/agi/app >/dev/null 2>&1
export GLOG_log_dir="$LOG_DIR"

# 常驻视觉开关：命令行传 "server" 或外部已 export VISION_USE_SERVER=1 均可开启。
if [ "$1" = "server" ]; then
    export VISION_USE_SERVER=1
fi
export VISION_USE_SERVER="${VISION_USE_SERVER:-}"

TS=$(date +%Y%m%d_%H%M%S)
RUN_LOG="$LOG_DIR/run_loop_onestep_profile_115_${TS}.log"
cd "$PROG_DIR"
echo "[onestep-profile115] 前台运行(Ctrl+C 停止)，日志: $RUN_LOG"
if [ "$VISION_USE_SERVER" = "1" ]; then
    echo "[onestep-profile115] 视觉: 走常驻服务 (VISION_USE_SERVER=1)。请确认已启动 run_vision_align_server_115.sh"
    if [ ! -S /tmp/vision_align_115.sock ]; then
        echo "[onestep-profile115] ⚠ 未发现 socket /tmp/vision_align_115.sock —— 客户端将自动降级为直调原脚本！"
    fi
else
    echo "[onestep-profile115] 视觉: 走原脚本 vision_align_safe_115.py (未开启常驻)"
fi
echo "[onestep-profile115] 细化数据: $LOG_DIR/cycle_detail_115.csv 与 cycle_summary_115.csv"
python3 -u run_loop_onestep_profile_115.py 2>&1 | tee "$RUN_LOG"
