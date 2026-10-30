# 115 机器人 QJX 产线节拍优化版

本目录是 **115 机器人（10.20.17.115）** 的 QJX 产线循环程序（上料取料 → 弯曲放料 → 弯曲取料 → 下料放料），
是在 129/126 那套（仓库根 `QJX/`）之外**独立的一套更新实现**。

**成果：机器人净耗时从 ~92s/轮 降到 ~59-61s/轮（降幅约 35%），并解决 WF/XL 放料翻转、挂挡板、纠偏误回退等质量问题。**

> 原理、前后对比、踩坑教训详见本目录 `115机器人节拍优化经验总结.md`；
> **移植到另一台机器人请按 `部署手册.md` 执行。**

---

## 一、做了哪些优化（全部）

### 1. 并行动作优化（底盘与上半身同时运动）
把原本串行的"底盘移动"和"手臂关节动作"改为并行执行，靠服务端双通道（底盘走 commands 通道、上半身 WBC 走独立 joints 通道+线程池）+ 主控异步下发：
```python
G2.REL_NOWAIT({"x": -0.4})   # 底盘后退：异步发出立即返回
G2.WBC("XL3")                # 上半身收臂：同时在另一通道跑
G2.REL_WAIT()                # 回头等底盘完成后汇合
```
具体并行点：
- **上料后退 + 右转合并为"边退边转"**：`REL_NOWAIT({"x":-0.4,"yaw_rad":-radians(90)})` 一个复合动作，且与 `WBC("SL3")` 收臂并行（省约 2.9s/轮）。
- **后退与收臂并行**：WQ3 / XL3 / WF3 的底盘后退与手臂收臂同时做（各省约 1.5s）。
- **导航与复位并行**：上料段 GO 与 FW 复位、弯曲取料段 GO 与 WQ1 挂载并行。

### 2. 弯曲机放料后退距离 400mm → 200mm
WF 放料后退只需安全距离，缩短行程（省约 1s/轮）。

### 3. 导航到位"停稳再伸手"（SETTLE_AFTER_GO_S）
`GO` 的完成回执可能先于底盘物理停稳返回，直接挂载会"没到位就伸手"。在 WF/XL 的 GO 后加停稳等待（现值 0.5s），避免伸早。

### 4. 视觉纠偏进程常驻化（决定性优化，视觉段 ~22s → ~5.4s）
原来每工位都 subprocess 重启视觉脚本、每次重做 GDK 初始化（~2s×4 工位）。改为：
- **常驻服务** `vision_align_server_115.py`：GDKBackend + Minth 只初始化一次，Unix socket 接请求，复用原对位逻辑（不改原脚本）。
- **主控进程内 socket 直连**：不再起客户端子进程，省掉 python 冷启动开销；服务不可用自动降级原脚本。
- 服务端控制模式自愈改为进程内读/切（去掉每次 ~2s 的子进程）。

### 5. 放料垂直下放（解决翻转/挂挡板）
WF/XL 原 `WBC("WF2"/"XL2")` 是关节插值斜向下移动，叠加纠偏旋转补偿会把料条斜放/翻转。改为：
- **关旋转补偿**（WF/XL 的 rotation 读数是测量偏差，补了反而转歪）。
- **WF2/XL2 换成"保持纠偏姿态、沿世界 Z 轴垂直直线下降固定落差 dz"**（`OFFSET({"rz":-dz})` 不带旋转），并用"读 Z 验证 + 补发"闭环修正 OFFSET 的系统性欠程/偶发漏放。
- 不改主程序源码，靠启动器运行时 patch `G2.WBC`（WF2/XL2 → 垂直下放），失败自动降级原关节动作。

### 6. 精准可对账的节拍记录器（v2）
`qjx_profiler_onestep_115.py`：monotonic 时钟（不受 NTP 影响）+ 墙钟区间并集（并行动作不重复计数），保证"动作并集 + 间隙 ≈ 净耗时"可对账。

### 附带修复：纠偏"成功又回退"
视觉验证残差多为测量偏差，超阈值回退反而更差。按工位调整容差/关闭回退（见参数表）。

---

## 二、当前各工位参数（115 实测生效值；换机需按现场重调）

| 工位 | rot_comp | zero_z | xy_tol_mm | one_step_verify | 放料下放 |
|---|:---:|:---:|:---:|:---:|---|
| SL1 上料抓取 | True | False | 8 | True | — |
| WF 弯曲放料 | False | False | 12 | False(不回退) | 垂直下放 66mm |
| WQ 弯曲取料 | True | True | 18 | True | — |
| XL 下料放料 | False | False | 4 | False(不回退) | 垂直下放 85mm |

- `xy_tol_mm` 一参两用：既是"动作前纠偏触发门限"又是"动作后验证回退阈值"，调整时注意副作用。
- 下放 dz 存 `wf_vertical_place_dz.json`，纯配置可随时微调。

---

## 三、目录结构

```
115/
├── README.md                                  本文件
├── 部署手册.md                                 移植到新机器人的完整步骤(★先读)
├── 115机器人节拍优化经验总结.md                  原理/前后对比/踩坑教训
├── requirements.txt                            pip 依赖
├── deploy_115.sh                               一键铺代码+冒烟测试(在目标机 115/ 内跑)
├── programs/                                   -> 机器人 /data/wxf0721/runtime/programs/
│   ├── all_machin_plc_vision_loop_onestep.py   主程序
│   ├── vision_align_server_115.py              视觉纠偏常驻服务
│   ├── vision_align_client_115.py              常驻服务客户端(含降级)
│   ├── vision_align_safe_115.py                原视觉脚本(降级后备+被服务复用)
│   ├── wf_vertical_place_115.py                放料垂直下放模块(含标定工具)
│   ├── wf_vertical_place_dz.json               下放落差配置(★机器相关，需重标定)
│   ├── qjx_profiler_onestep_115.py             节拍记录器 v2
│   ├── run_loop_onestep_profile_115.py/.sh     带记录器启动器(server 开常驻+WF2/XL2拦截)
│   ├── run_vision_align_server_115_daemon.sh   视觉服务后台守护(start/stop/status/restart)
│   ├── run_vision_align_server_115.sh          视觉服务前台(调试)
│   ├── set_control_mode_115.py                 关节位控自愈
│   ├── vision_115.py / vision_pick_correction.py  视觉 profile / 对位算法
│   └── *_reference.json                        4 工位视觉参考(★现场示教，需重示教)
└── runtime/
    └── minth.py                                Minth G2 控制封装 -> /data/wxf0721/runtime/
```

---

## 四、部署（简版，详见 部署手册.md）

```bash
git clone https://github.com/Jensen-Qiu/Robot-project1.git ~/Robot-project1
cd ~/Robot-project1/115
source /home/agi/app/env.sh /home/agi/app
python3 -m pip install -r requirements.txt          # pip 依赖
# 视觉包从源机对拷: scp -r agi@源机:/home/agi/G2_AprilTag_Placement_0.2.0 /home/agi/
bash deploy_115.sh                                   # 铺代码 + 冒烟测试
# 然后(机器相关，必做): 迁移地图/robot_data.db -> 标定 dz -> 示教视觉参考 -> 现场调参
```

**⚠ 代码可直接复制，但以下机器/现场相关项必须在新机重做（无法跳过）**：
地图与导航点迁移、放料下放 dz 标定、4 工位视觉参考示教、各工位容差/停稳等现场调参。

## 五、日常启动（已部署好的机器）
```bash
# 1) 视觉常驻服务(后台守护，确保未插充电)
bash /data/wxf0721/runtime/programs/run_vision_align_server_115_daemon.sh start
# 2) 带节拍记录器 + 常驻 跑主控
bash /data/wxf0721/runtime/programs/run_loop_onestep_profile_115.sh server
```

## 六、外部依赖（不在本仓库）
- `agibot_gdk`（机器人 SDK，随机器人软件）
- 视觉包 `G2_AprilTag_Placement_0.2.0`（从源机 `/home/agi/` 对拷）
- pip：见 `requirements.txt`
- 运行时数据：`robot_data.db`（joints/map_points）、地图三层（见部署手册第 5 节）
