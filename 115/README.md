# 115 机器人 QJX 产线节拍优化版

本目录是 **115 机器人（10.20.17.115）** 的 QJX 产线循环程序（上料取料 → 弯曲放料 → 弯曲取料 → 下料放料），
是在 129/126 那套（见仓库根 `QJX/`）之外**独立的一套更新实现**，核心成果：

- **节拍从 ~92s/轮 降到 ~59-61s/轮**（降幅约 35%）。
- **视觉纠偏进程常驻化**（每工位重复 GDK 初始化 → 常驻服务 + 进程内 socket 直连，视觉段 22s→5.4s）。
- **放料垂直下放**（WF/XL 用"保持纠偏姿态垂直下降固定落差"代替关节插值，消除料条翻转/挂挡板）。
- **精准可对账的节拍记录器**（monotonic 时钟 + 墙钟区间并集）。

> 详细原理、前后对比、踩坑教训见本目录 `115机器人节拍优化经验总结.md`。

## 目录结构

```
115/
├── programs/                                  对应机器人 /data/wxf0721/runtime/programs/
│   ├── all_machin_plc_vision_loop_onestep.py  主程序（含所有并行/停稳/视觉常驻接入/垂直下放拦截）
│   ├── vision_align_server_115.py             视觉纠偏常驻服务（GDKBackend+Minth 只 init 一次，Unix socket）
│   ├── vision_align_client_115.py             常驻服务客户端（命令行契约同原脚本，服务不可用自动降级）
│   ├── vision_align_safe_115.py               原视觉纠偏脚本（降级后备 + 被常驻服务复用，未改核心）
│   ├── wf_vertical_place_115.py               放料垂直下放模块（WF/XL 通用 + 标定工具）
│   ├── wf_vertical_place_dz.json              各工位下放固定落差配置（★机器相关，迁移需重标定）
│   ├── qjx_profiler_onestep_115.py            逐条动作节拍记录器（v2，可对账）
│   ├── run_loop_onestep_profile_115.py/.sh    带记录器启动器（server 参数开常驻 + WF2/XL2 垂直下放拦截）
│   ├── run_vision_align_server_115_daemon.sh  视觉常驻服务后台守护启动（start/stop/status/restart）
│   ├── run_vision_align_server_115.sh         视觉常驻服务前台启动（调试用）
│   ├── set_control_mode_115.py                关节位控模式自愈
│   ├── vision_115.py / vision_pick_correction.py  视觉运行时 profile / 对位算法
│   └── *_reference.json                        4 个工位视觉参考（★现场示教，迁移需重示教）
└── runtime/
    └── minth.py                               Minth G2 控制封装（对应 /data/wxf0721/runtime/minth.py）
```

## 硬编码路径（部署时需对应）

程序里 sys.path / 文件路径写死为：
- `programs/` 内文件 → `/data/wxf0721/runtime/programs/`
- `minth.py` → `/data/wxf0721/runtime/`
- 视觉常驻 socket → `/tmp/vision_align_115.sock`

## 日常启动（两步）

```bash
# 1) 启动视觉常驻服务（后台守护，关终端不死；确保机器人未插充电）
bash /data/wxf0721/runtime/programs/run_vision_align_server_115_daemon.sh start
# 2) 带节拍记录器 + 常驻 跑主控（server 参数自动开 VISION_USE_SERVER=1）
bash /data/wxf0721/runtime/programs/run_loop_onestep_profile_115.sh server
```

## 外部依赖（不在本仓库，部署时另装）
- `agibot_gdk`（机器人 SDK，`/home/agi/app/gdk/lib`）
- 视觉包 `G2_AprilTag_Placement`（`/home/agi/G2_AprilTag_Placement_0.2.0`）
- pip：paho-mqtt、numpy、opencv-python、pyapriltags、scipy
- 服务端运行时数据（`robot_data.db` 的 joints/map_points、地图等）

## 迁移到其它同型号机器人
见经验文档第十一节。要点：代码可直接复制；但 **dz 下放落差、视觉参考、各工位容差/关节位姿/地图点必须在新机重标定/重示教**；硬编码路径与机器标识 115 需替换。
