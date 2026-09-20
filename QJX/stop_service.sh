#!/bin/bash
# stop_service.sh —— 停止机器人控制服务端 (services/main.py)。
# 用法: bash /data/QJX/stop_service.sh

# 匹配所有形式的服务端进程：无论 'services/main.py' 全路径还是 cd 后的 'main.py' 相对路径。
# 用 python + main.py 双条件，避免误杀其它同名脚本。
_find() {
    ps aux | grep python | grep -F 'main.py' | grep -v grep | grep -v 'all_machin' | awk '{print $2}'
}

PIDS=$(_find)
if [ -z "$PIDS" ]; then
    echo "[stop_service] 无服务端在运行"
    exit 0
fi
echo "[stop_service] 停止服务端: $PIDS"
kill $PIDS 2>/dev/null
sleep 3
LEFT=$(_find)
if [ -n "$LEFT" ]; then
    echo "[stop_service] 仍存活，强制停止: $LEFT"
    kill -9 $LEFT 2>/dev/null
    sleep 1
fi
LEFT=$(_find)
[ -z "$LEFT" ] && echo "[stop_service] ✅ 服务端已停止" || echo "[stop_service] ⚠️ 残留: $LEFT"
