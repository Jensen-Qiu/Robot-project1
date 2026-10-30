# QJX G2 机器人 PLC 视觉循环控制程序

> **本仓库收录两套同任务（QJX 上下料循环）实现 + 一份通用移植经验。先按下表找到你要的部分。**

## 仓库总览

| 你的目的 | 看这里 |
|---|---|
| 部署/维护 **115 机器人**（节拍优化版，~59-61s/轮） | `115/README.md` → `115/部署手册.md` |
| 部署/维护 **129/126 机器人**（原始版） | 本 README 下方 + `QJX/`、`runtime_programs/` |
| 把程序**移植到另一台同型号机器人**（地图/导航点/关避障） | `移植经验_地图迁移与关避障.md` |
| 理解 **115 优化的原理与踩坑** | `115/115机器人节拍优化经验总结.md` |

### 两套实现对比

| 维度 | 129/126 套（原始） | 115 套（节拍优化版） |
|---|---|---|
| 仓库目录 | `QJX/` + `runtime_programs/` | `115/` |
| 主程序 | `all_machin_plc_vision_2_loop.py` | `all_machin_plc_vision_loop_onestep.py` |
| 代码硬编码路径 | `/data/QJX/` + `/data/wxf0721/runtime[/programs]` | `/data/wxf0721/runtime/programs/` + `/data/wxf0721/runtime/` |
| 单轮节拍 | 基线水平 | **~59-61s（较基线 ~92s 降约 35%）** |
| 视觉纠偏 | 每工位 subprocess 重启 | **常驻服务 + 进程内 socket 直连**（视觉段 22s→5.4s） |
| 底盘/上半身 | 多为串行 | **并行**（边退边转合并、后退与收臂并行等） |
| 放料 | 关节插值下压 | **保持姿态垂直下放固定落差**（消除翻转/挂挡板） |
| 节拍记录 | 无 | **可对账记录器**（monotonic + 墙钟并集） |
| 一键部署 | 手动（见本文第一部分） | `115/deploy_115.sh` + `115/requirements.txt` |

> 两套**互不覆盖、互不影响**；地图迁移与关避障对两套通用。

---


本仓库为 G2 机器人（宇树 / agibot 平台）上的 **PLC 联动 + 视觉纠偏** 上下料自动化控制程序。
主入口为循环版主程序 `all_machin_plc_vision_2_loop.py`，配合 PLC 握手信号，实现
「送料机取料 → 弯曲机放料 → 弯曲机取料 → 下料机放料」的连续循环作业。

> 源码原本分散在机器人的两个目录（`/data/QJX/` 与 `/data/wxf0721/runtime/`），
> 本仓库按原目录结构归档为 `QJX/` 与 `runtime_programs/` 两个文件夹。

---

## 目录结构

```
.
├── QJX/                       # 原 /data/QJX/ —— 提速架构主程序与工具
│   ├── all_machin_plc_vision_2_loop.py   # ★主入口：循环版主程序
│   ├── all_machin_plc_vision_2_fast.py   # 提速版核心逻辑（动作、看门狗、心跳、握手、视觉调用）
│   ├── fast_joint_mover.py               # 可调速关节执行库（绕开服务端固定 ARM_SPEED=0.2）
│   ├── fast_gripper.py                   # 夹爪快速控制库
│   ├── run_loop.sh                       # ★启动脚本（前台循环，Ctrl+C 干净停机）
│   ├── run_fast.sh / stop_fast.sh        # 提速版单轮启停脚本
│   ├── run_service.sh / stop_service.sh  # 后台服务方式启停脚本
│   ├── diag_gripper.py / diag_gripper2.py / diag_mode_gripper.py  # 夹爪诊断
│   ├── dump_wbc.py / wbc_dump.json       # WBC 动作导出/快照
│   ├── patch_grace.py                    # 停机优雅处理补丁
│   ├── probe_gripper_speed.py / probe_mode.py  # 速度/模式探测
│   ├── read_plc_signals.py               # 读取 PLC 信号（调试）
│   └── test_fast_gripper.py / test_fw.py / test_grab_speed.py  # 测试脚本
│
└── runtime_programs/          # 原 /data/wxf0721/runtime/ 及 runtime/programs/ 的依赖
    ├── minth.py                          # Minth 机器人控制封装（MQTT，底盘/腰/臂/夹爪/TTS）
    ├── vision_helper.py                  # 视觉纠偏封装（子进程方式调用，取料偏移）
    ├── vision_pick_correction.py         # 取料视觉纠偏子进程（AprilTag 解算，只读相机）
    ├── vision_align.py                   # 通用视觉对位子进程（放料/取料对位）
    ├── pick_reference.json               # 送料机取料参考
    ├── place_reference.json              # 弯曲机放料参考
    ├── unload_reference.json             # 下料机放料参考
    └── wqpick_reference.json             # 弯曲机取料参考
```

---

## 依赖调用链

```
all_machin_plc_vision_2_loop.py            (主循环，一次性初始化 + 每轮任务)
  └─ import all_machin_plc_vision_2_fast.py (复用全部对象/动作函数/看门狗/心跳/信号)
       ├─ import minth.py                   (Minth：走 MQTT 控制底盘/腰/臂/夹爪/TTS/视觉握手)
       ├─ import fast_joint_mover.py        (可调速关节下发；依赖 agibot_gdk、paho.mqtt)
       ├─ import vision_helper.py           (取料偏移，子进程)
       │      └─ subprocess → vision_pick_correction.py
       └─ subprocess → vision_align.py      (4 个工位纠偏：pick/place/unload/wqpick)
              └─ import vision_pick_correction.py (复用解算)

vision_pick_correction.py / vision_align.py
  └─ import apriltag_vision, g2_tag_placement   ← 外部已安装视觉包（见下）
```

---

## 运行环境与依赖部署

以下环境信息在目标机器人（主机名 `G2`）上实机核实。

### 1. 硬件与操作系统

| 项 | 实测值 |
|----|--------|
| 平台 | G2 机器人（agibot），ARM64（aarch64） |
| 操作系统 | Ubuntu 22.04.5 LTS（Jammy） |
| 内核 | 5.10.x PREEMPT_RT（实时内核） |
| Python | Python 3.10.12（系统 `/usr/bin/python3`） |

> 本项目直接控制真实机器人硬件（底盘、腰、双臂、夹爪、相机），并通过 MQTT 与
> 机器人服务端通信，需在真实 G2 机器人上运行；无仿真环境。

### 2. Python 第三方包（pip 安装）

实机已装且验证可用的版本：

| 包 | 实测版本 | 用途 |
|----|---------|------|
| `paho-mqtt` | 2.1.0 | MQTT 通信（`minth.py`、`fast_joint_mover.py`） |
| `numpy` | 1.26.4 | 视觉解算 |
| `opencv-python` | 4.11.0.86 | 相机图像处理（`cv2`） |
| `pyapriltags` | 3.4.3.1 | AprilTag 检测 |
| `scipy` | 1.15.3 | 位姿/几何计算 |

安装示例：

```bash
python3 -m pip install \
    "paho-mqtt==2.1.0" \
    "numpy==1.26.4" \
    "opencv-python==4.11.0.86" \
    "pyapriltags==3.4.3.1" \
    "scipy==1.15.3"
```

> 视觉包自带的 `requirements-vision.txt` 标注的是开发工作站版本
> （`numpy==2.2.6`、`opencv-python==4.13.0.92`），与机器人实机版本不同；
> **以上表格为机器人实机版本，部署时以实机版本为准**。

### 3. 机器人 SDK 环境（`agibot_gdk`）

`agibot_gdk` **不是 pip 包**，随机器人 SDK 一起安装在 `/home/agi/app/gdk/lib/agibot_gdk`，
通过 `env.sh` 注入环境变量后才能 `import`。运行本项目前必须先加载：

```bash
source /home/agi/app/env.sh /home/agi/app
```

`env.sh` 会设置：
- `LD_LIBRARY_PATH`：加入 `/home/agi/app/lib` 下所有 `.so` 目录
- `PATH`：加入 `/home/agi/app/bin`
- `PYTHONPATH`：加入 `/home/agi/app/gdk/lib`（提供 `agibot_gdk`）
- `APP_CONF_PATH`：`/home/agi/app/gdk/config/app_conf.json`

> 未加载 `env.sh` 时 `import agibot_gdk` / `fast_joint_mover` / `fast_gripper` 会失败。
> 启动脚本 `run_loop.sh`、`vision_helper.py`、`vision_align.py` 内部均已自动 source。

### 4. 视觉包 `G2_AprilTag_Placement`（AprilTag 对位）

安装在机器人的 `/home/agi/G2_AprilTag_Placement_0.2.0`，提供 `apriltag_vision`、
`g2_tag_placement`（含 `gdk_backend`、`controller`、`geometry`、`data/board.json` 等）。
`vision_pick_correction.py` 与 `vision_align.py` 通过硬编码路径加载：

```python
PKG_ROOT = Path('/home/agi/G2_AprilTag_Placement_0.2.0')
sys.path.insert(0, str(PKG_ROOT))
```

该包为独立部署件（含 `setup.py` / `pyproject.toml`），需单独获取安装到上述路径；
路径不同时需修改脚本中的 `PKG_ROOT`。

### 5. 机器人服务端运行时数据

`minth.py` 通过 MQTT 与机器人服务端交互，实际动作 JSON 由服务端持有，不在本仓库：
- `detect.json`（底盘/腰纠偏检测结果）
- 动作库：`datas/joints/WBC/{name}.json`、`arms/{name}.json`、`left/{name}.json` 等

这些由机器人服务端提供并加载，属于机器人部署环境，无需随本项目上传。

### 部署检查清单

- [ ] 目标为真实 G2 机器人，Ubuntu 22.04 / Python 3.10 / ARM64
- [ ] 已装 pip 包：`paho-mqtt`、`numpy`、`opencv-python`、`pyapriltags`、`scipy`（版本见上表）
- [ ] 机器人 SDK 已装，`source /home/agi/app/env.sh /home/agi/app` 后可 `import agibot_gdk`
- [ ] 视觉包位于 `/home/agi/G2_AprilTag_Placement_0.2.0`（否则改 `PKG_ROOT`）
- [ ] 机器人服务端已启动（MQTT 可达，动作 JSON 就绪）
- [ ] 本仓库 `QJX/` → `/data/QJX/`，`runtime_programs/` → `/data/wxf0721/runtime/` 与 `.../programs/`（或改硬编码路径）
- [ ] 4 个 `*_reference.json` 视觉参考已按当前开机 `boot_id` 重新示教（换机/重启后）

---

## 外部依赖（不在本仓库，需机器人环境提供）

以下为机器人部署环境 / 第三方 SDK，**不随本仓库上传**，在目标机器上必须已安装配置：

| 依赖 | 说明 | 机器人上位置 |
|------|------|------------|
| `agibot_gdk` | 机器人底层控制 SDK（Python 包） | SDK 环境 |
| `paho-mqtt` | MQTT 客户端（`pip` 包） | Python 环境 |
| `numpy` / `opencv (cv2)` | 数值计算 / 相机图像 | Python 环境 |
| `G2_AprilTag_Placement` 视觉包 | 含 `apriltag_vision`、`g2_tag_placement`（GDKBackend、controller、geometry、board.json 等） | `/home/agi/G2_AprilTag_Placement_0.2.0` |
| `env.sh` | SDK 环境加载脚本 | `/home/agi/app/env.sh` |
| 服务端运行时数据 | `detect.json`、`datas/joints/WBC/*.json`、`arms/*.json` 等动作 JSON，由 MQTT 服务端加载 | `/data/wxf0721/runtime/` 相关目录 |

> `minth.py` 通过 MQTT 与服务端交互，实际动作 JSON（如 `hold.json`）由服务端持有，
> 不属于本控制程序源码。

---

## 部署与运行

程序对路径有硬编码假设，原始部署布局为：

- 主程序目录：`/data/QJX/`
- 运行时依赖：`/data/wxf0721/runtime/`（`minth.py`）与 `/data/wxf0721/runtime/programs/`（视觉脚本 + 参考 json）

`all_machin_plc_vision_2_fast.py` 顶部通过 `sys.path.insert` 把上述目录加入搜索路径：

```python
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))   # /data/QJX
sys.path.insert(0, "/data/wxf0721/runtime")                       # minth.py
sys.path.insert(0, "/data/wxf0721/runtime/programs")             # vision_*.py
```

> 若在新机器上部署，请将本仓库的 `QJX/` 还原到 `/data/QJX/`，
> `runtime_programs/` 内容还原到 `/data/wxf0721/runtime/`（`minth.py`）
> 与 `/data/wxf0721/runtime/programs/`（视觉脚本与 `*_reference.json`），
> 或相应修改上述硬编码路径。

### 启动（前台循环，推荐）

```bash
source /home/agi/app/env.sh /home/agi/app
cd /data/QJX
bash run_loop.sh
# 或直接：python3 -u all_machin_plc_vision_2_loop.py
```

- 前台运行，`Ctrl+C` 可随时中断；中断后复位 PLC 信号、释放 GDK，干净停机。
- 日志写入 `/data/QJX/logs/`。

---

## 工作流程（每轮循环）

1. 等待 PLC `pick_right` 信号 → 送料机取料（含 `pick_reference.json` 视觉纠偏）→ 置 `pick_right_done`
2. 弯曲机放料（`place_reference.json` 纠偏）
3. 等待 PLC `place_middle` 信号 → 弯曲机取料（`wqpick_reference.json` 纠偏）
4. 下料机放料（`unload_reference.json` 纠偏）→ 置 `place_middle_done`
5. 自动进入下一轮

视觉纠偏为独立子进程「只看不动」，解算 AprilTag 偏移后由主程序（Minth）决定是否下发，
任何异常/超限/无参考都安全回退到固定点位流程。

---

## 速度分组说明

`fast_joint_mover.py` 绕开服务端写死的 `ARM_SPEED=0.2`，对部分 WBC 段提速到 0.5，
对贴料/抓料/放料等精度敏感段保持 0.2（见 `all_machin_plc_vision_2_fast.py` 中 `WBC_ARM_SPEED`）。

---

## 注意事项

- 本程序直接操作真实机器人硬件，请在安全环境下运行，确保急停可用。
- 参考文件 `*_reference.json` 与开机 `boot_id` 绑定，换机 / 重启后可能需要重新示教。
- 日志文件、`__pycache__/`、`*.bak*` 备份、`run_service.pid` 未纳入本仓库。

---

## 115 机器人节拍优化版（`115/` 目录）

除根目录 `QJX/` + `runtime_programs/`（129/126 机器人那套）外，仓库新增 **`115/`** 目录，
是 **115 机器人（10.20.17.115）** 的独立更新实现，在同样的 QJX 产线任务上做了节拍优化与放料质量改进：

- 节拍 ~92s/轮 → **~59-61s/轮**（降幅约 35%）。
- 视觉纠偏**进程常驻化** + 进程内 socket 直连（视觉段 22s→5.4s）。
- 放料**垂直下放**（WF/XL 保持纠偏姿态垂直下降固定落差，消除料条翻转/挂挡板）。
- 精准可对账的**节拍记录器**（monotonic 时钟 + 墙钟区间并集）。

主程序为 `115/programs/all_machin_plc_vision_loop_onestep.py`（与根目录 `all_machin_plc_vision_2_loop.py`
是**两套不同实现**，互不覆盖）。详见 `115/README.md` 与 `115/115机器人节拍优化经验总结.md`。

---

## 移植经验文档（`移植经验_地图迁移与关避障.md`）

仓库根目录的 `移植经验_地图迁移与关避障.md` 记录了同型号 G2 机器人整体移植的完整实操经验，三大块：
**① 代码检索与上传 GitHub ② 地图与导航点三层迁移 ③ 关闭避障功能**。
含真实命令、踩坑记录、回滚方法与整体检查清单。其中**地图迁移与关避障对 `QJX/`(129/126) 与 `115/` 两套都适用**；
代码部分 129 套看该文第一部分、115 套看 `115/部署手册.md`。
