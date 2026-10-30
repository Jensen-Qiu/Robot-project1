#!/bin/bash
# deploy_115.sh —— 把本仓库 115/ 代码部署到目标同型号机器人的硬编码路径，并做冒烟测试。
# 在【目标机器人】上，于克隆下来的仓库 115/ 目录内运行：  bash deploy_115.sh
#
# 它做什么：
#   1) 建硬编码目录 /data/wxf0721/runtime/programs 与 /data/wxf0721/datas、/data/wxf0721/detect
#   2) 铺代码：programs/* -> /data/wxf0721/runtime/programs/；runtime/minth.py -> /data/wxf0721/runtime/
#   3) 赋可执行权限给 *.sh
#   4) 冒烟测试：import agibot_gdk / 视觉包 / 本项目模块
# 它【不做】(机器相关，必须人工，见 部署手册.md)：
#   - 不装 pip 依赖(请先 pip install -r requirements.txt)
#   - 不拷视觉 SDK 包 G2_AprilTag_Placement(请从源机对拷)
#   - 不迁移地图/导航点/robot_data.db
#   - 不标定下放 dz、不示教视觉参考/关节位姿
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
PROG_DST=/data/wxf0721/runtime/programs
RT_DST=/data/wxf0721/runtime

echo "[deploy] 源目录: $HERE"
echo "[deploy] 1) 建目录(需 sudo 建 /data 子目录并授权给当前用户)"
sudo mkdir -p "$PROG_DST" /data/wxf0721/datas /data/wxf0721/detect
sudo chown -R "$(id -un):$(id -gn)" /data/wxf0721

echo "[deploy] 2) 铺代码"
cp -v "$HERE"/programs/* "$PROG_DST"/
cp -v "$HERE"/runtime/minth.py "$RT_DST"/

echo "[deploy] 3) 赋可执行权限"
chmod +x "$PROG_DST"/*.sh

echo "[deploy] 4) 冒烟测试(import 链)"
set +u
source /home/agi/app/env.sh /home/agi/app >/dev/null 2>&1
ok=1
python3 -c "import agibot_gdk; print('[smoke] agibot_gdk OK')" || { echo "[smoke] agibot_gdk 缺失 -> 机器人 SDK 未就绪"; ok=0; }
python3 -c "import sys; sys.path.insert(0,'/home/agi/G2_AprilTag_Placement_0.2.0'); import apriltag_vision, g2_tag_placement; print('[smoke] 视觉包 OK')" || { echo "[smoke] 视觉包缺失 -> 从源机对拷 /home/agi/G2_AprilTag_Placement_0.2.0"; ok=0; }
python3 -c "import cv2,numpy,paho.mqtt.client; print('[smoke] pip 依赖 OK')" || { echo "[smoke] pip 依赖缺失 -> pip install -r requirements.txt"; ok=0; }
cd "$PROG_DST" && python3 -c "import sys; sys.path.insert(0,'/data/wxf0721/runtime'); sys.path.insert(0,'/data/wxf0721/runtime/programs'); import minth; print('[smoke] minth OK')" || { echo "[smoke] minth import 失败"; ok=0; }

echo "----------------------------------------"
if [ "$ok" = "1" ]; then
  echo "[deploy] 代码部署 + 冒烟测试通过。"
  echo "[deploy] 接下来必须完成(见 部署手册.md)： 迁移地图/robot_data.db -> 标定 dz -> 示教视觉参考 -> 现场调参 -> 试跑。"
else
  echo "[deploy] 冒烟测试有缺项，按上面提示补齐后重跑本脚本。"
fi
