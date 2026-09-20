#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Minth 机器人控制类库

通过 MQTT 向 g2_minth_app_service 发送命令，并同步等待执行完成。

用法：
    from minth import Minth

    robot = Minth.G2()
    robot.GO(9)                 # 导航到地图点位 9
    robot.WBC("hold")           # 执行全身关节动作 hold.json
    robot.ARMS("hold")          # 执行双臂关节动作 arms/hold.json
    robot.TTS("你好")           # 语音播报
    robot.REL({"x": 0.3})       # 底盘前进 0.3 米
    robot.OFFSET({"lx": 20})    # 左末端相对移动 20mm
    robot.GRIPPER({"left": 0.5, "right": 0.5})
    robot.YOLO("7.14.pt")               # YOLO 目标检测（使用服务端默认 IP）
    robot.YOLO("wxf.pt")                # 使用 wxf.pt 模型检测
    robot.YOLO("wxf.pt", "10.2.236.7")  # 指定自定义 YOLO 服务端 IP
    robot.CHASSIS_CORRECT()     # 根据 detect.json 纠正底盘水平偏移
    robot.JOINT("idx11_head_joint1", offset=0.01)   # 单关节增量微调
    robot.JOINT("idx11_head_joint1", value=0.0)     # 单关节运动到指定角度
    robot.WAIST_CORRECT()       # 根据 detect.json 的 angle_rad 纠正腰部旋转
    robot.close()

    # X2 型号（预留）
    # x2 = Minth.X2()
"""

import json
import os
import threading

import paho.mqtt.client as mqtt


# ── MQTT 配置（与 services/main.py 对齐）─────────────────
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
# 关节运动命令主题
JOINTS_TOPIC = "/humanoid/joints/control"
# 动作命令主题
COMMANDS_TOPIC = "/humanoid/commands/data"
# 命令完成通知主题
DONE_TOPIC = "/humanoid/commands/done"
# 相机控制主题
CAMERA_TOPIC = "/humanoid/camera/control"
# 坐标点位控制主题
POSITIONS_CTRL_TOPIC = "/humanoid/positions/control"
POSITIONS_DATA_TOPIC = "/humanoid/positions/data"
# synch 数据同步主题（readData / setData）
DATA_READ_TOPIC = "/humanoid/data/read"
DATA_WRITE_TOPIC = "/humanoid/data/write"
DATA_RESPONSE_TOPIC = "/humanoid/data/response"

# 默认超时时间（秒）
DEFAULT_TIMEOUT = 15

# detect.json 路径（runtime/../detect/detect.json）
_DETECT_JSON = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "detect", "detect.json",
)

# 像素 → 米 转换系数
# 实测：70 像素偏移 → 需向右移动 130 毫米 → 系数 = 130/70/1000 m/px
# 向右 = y 负方向，故加负号
PX_TO_METER = -130.0 / 70.0 / 1000.0


class _RobotBase:
    """机器人基类：封装 MQTT 通信和同步等待逻辑"""

    def __init__(self, broker=MQTT_BROKER, port=MQTT_PORT, timeout=DEFAULT_TIMEOUT, client_id=None):
        self.broker = broker
        self.port = port
        self.timeout = timeout
        self._done_event = threading.Event()
        self._positions_event = threading.Event()
        self._positions_result = None
        # synch 数据读取响应同步
        self._data_value = None
        self._data_name = None
        self._data_ready = threading.Event()
        self._connected = False
        cid = client_id or f"minth_{self.__class__.__name__}_{id(self)}"
        self._client = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id=cid,
        )
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._client.connect(broker, port)
        self._client.loop_start()
        # 等待连接建立
        for _ in range(50):
            if self._connected:
                break
            threading.Event().wait(0.1)
        if not self._connected:
            raise ConnectionError(f"无法连接到 MQTT broker {broker}:{port}")

    # ── MQTT 回调 ──────────────────────────────────────────
    def _on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            client.subscribe(DONE_TOPIC, qos=2)
            client.subscribe(POSITIONS_DATA_TOPIC, qos=0)
            client.subscribe(DATA_RESPONSE_TOPIC, qos=0)
            self._connected = True
        else:
            raise ConnectionError(f"MQTT 连接失败，返回码: {rc}")

    def _on_message(self, client, userdata, msg):
        if msg.topic == DONE_TOPIC:
            self._done_event.set()
        elif msg.topic == POSITIONS_DATA_TOPIC:
            try:
                data = json.loads(msg.payload.decode())
                self._positions_result = data
                self._positions_event.set()
            except Exception:
                pass
        elif msg.topic == DATA_RESPONSE_TOPIC:
            self._on_data_response(msg)

    def _on_data_response(self, msg):
        """处理 synch 数据读取/写入响应"""
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except Exception:
            return
        name = payload.get("name")
        if name == self._data_name:
            self._data_value = payload.get("value")
            self._data_ready.set()

    # ── 核心：发送命令并等待完成 ────────────────────────────
    # 关节命令集合（发送到 /humanoid/joints/control）
    _JOINT_CMDS = {"WBC", "arms", "left", "right", "head", "waist", "joint"}

    def DONE(self, message=None):
        """向 /humanoid/commands/done 发送执行完成信号

        与后端 services 的 done 消息格式一致（{"command":"done"}）。
        可用于自定义脚本结束时通知等待方。

        Args:
            message: 可选，附加信息，如 "program_finished"
        Returns:
            bool: True=信号已成功发出，False=发送失败
        """
        payload = {"command": "done"}
        if message is not None:
            payload["message"] = str(message)
        msg_str = json.dumps(payload, ensure_ascii=False)
        info = self._client.publish(DONE_TOPIC, msg_str, qos=2)
        ok = getattr(info, "rc", 0) == 0
        print(f"[Minth] → done 信号已发送"
              f"{'' if ok else ' [发送失败]'}")
        return ok

    def _send_and_wait(self, cmd, data=None):
        """发送命令并等待 DONE_TOPIC 回复或超时

        关节命令发送到 /humanoid/joints/control，
        动作命令发送到 /humanoid/commands/data。
        """
        payload = {"command": cmd}
        if data is not None:
            payload["data"] = data

        # 选择目标主题
        topic = JOINTS_TOPIC if cmd in self._JOINT_CMDS else COMMANDS_TOPIC

        self._done_event.clear()
        msg_str = json.dumps(payload, ensure_ascii=False)
        self._client.publish(topic, msg_str, qos=2)
        print(f"[Minth] → {cmd}: {data}")

        done = self._done_event.wait(timeout=self.timeout)
        if done:
            print(f"[Minth] ✓ {cmd} 执行完成")
        else:
            print(f"[Minth] ✗ {cmd} 超时 ({self.timeout}s)")
        return done

    # ── 释放资源 ──────────────────────────────────────────
    def close(self):
        if self._client:
            self._client.loop_stop()
            self._client.disconnect()
            self._client = None

    def __del__(self):
        self.close()


class G2(_RobotBase):
    """Minth G2 机器人控制类

    所有方法均为同步阻塞调用：发送 MQTT 命令后等待 /G2_minth_app_done 回复，
    收到后返回 True；15 秒超时返回 False。
    """

    # ── synch 数据同步（readData / setData）─────────────────
    def readData(self, name):
        """读取 synch 变量的值

        通过 MQTT 向 /humanoid/data/read 发送请求，
        等待 /humanoid/data/response 返回结果。

        Args:
            name: synch 变量名（如 "m1"、"w1"）

        Returns:
            变量值或 None（超时 / 未找到）
        """
        self._data_name = name
        self._data_value = None
        self._data_ready.clear()
        payload = {"command": "read", "name": name}
        self._client.publish(
            DATA_READ_TOPIC, json.dumps(payload, ensure_ascii=False), qos=0
        )
        print(f"[Minth] → readData({name})")
        if self._data_ready.wait(timeout=self.timeout):
            print(f"[Minth] ✓ readData({name}) = {self._data_value}")
            return self._data_value
        print(f"[Minth] ✗ readData({name}) 超时")
        return None

    def setData(self, name, value):
        """设置 synch write 变量的值

        通过 MQTT 向 /humanoid/data/write 发送请求，
        服务端收到后更新 synch 中对应 write 条目的 value 并标记 state=1，
        随后 modbus/s7 轮询线程会将值写入设备。

        Args:
            name:  synch write 变量名（如 "m1_"、"w1_"）
            value: 要写入的值

        Returns:
            bool: True=服务端已接收，False=超时
        """
        self._data_name = name
        self._data_value = None
        self._data_ready.clear()
        payload = {"command": "write", "name": name, "value": value}
        self._client.publish(
            DATA_WRITE_TOPIC, json.dumps(payload, ensure_ascii=False), qos=0
        )
        print(f"[Minth] → setData({name}, {value})")
        if self._data_ready.wait(timeout=self.timeout):
            print(f"[Minth] ✓ setData({name}, {value}) 已接收")
            return True
        print(f"[Minth] ✗ setData({name}, {value}) 超时")
        return False

    def GO(self, num):
        """导航到指定地图点位
        Args:
            num: 导航点索引（整数），如 9
        Returns:
            bool: True=执行完成，False=超时
        """
        return self._send_and_wait("go", num)

    def GO_NOWAIT(self, num):
        """导航到指定地图点位（异步，不等待执行完成）

        命令下发后立即返回，不等待 /humanoid/commands/done 回复。
        适用于 fire-and-forget 场景。

        Args:
            num: 导航点索引（整数），如 9
        Returns:
            bool: True=命令已成功下发，False=发送失败
        """
        payload = {"command": "go", "data": num}
        msg_str = json.dumps(payload, ensure_ascii=False)
        info = self._client.publish(COMMANDS_TOPIC, msg_str, qos=2)
        ok = getattr(info, "rc", 0) == 0
        print(f"[Minth] → go (nowait): {num}"
              f"{'' if ok else ' [发送失败]'}")
        return ok

    def WBC(self, name):
        """全身关节运动
        Args:
            name: 动作名称字符串，对应 datas/joints/WBC/{name}.json
                  例如 "hold"
        Returns:
            bool
        """
        return self._send_and_wait("WBC", name)

    def ARMS(self, name):
        """双臂关节运动
        Args:
            name: 动作名称字符串，对应 datas/joints/arms/{name}.json
                  例如 "hold"
        Returns:
            bool
        """
        return self._send_and_wait("arms", name)

    def LEFT(self, name):
        """左臂关节运动（仅左臂）
        Args:
            name: 动作名称字符串，对应 datas/joints/left/{name}.json
                  例如 "A_PLACE_LOOK"
        Returns:
            bool
        """
        return self._send_and_wait("left", name)

    def RIGHT(self, name):
        """右臂关节运动（仅右臂）
        Args:
            name: 动作名称字符串，对应 datas/joints/right/{name}.json
                  例如 "A_PLACE_LOOK"
        Returns:
            bool
        """
        return self._send_and_wait("right", name)

    def HEAD(self, name):
        """头部关节运动
        Args:
            name: 动作名称字符串，对应 datas/joints/head/{name}.json
        Returns:
            bool
        """
        return self._send_and_wait("head", name)

    def WAIST(self, name):
        """腰部关节运动
        Args:
            name: 动作名称字符串，对应 datas/joints/waist/{name}.json
        Returns:
            bool
        """
        return self._send_and_wait("waist", name)

    def OFFSET(self, data):
        """末端执行器相对移动
        Args:
            data: dict，单位毫米，如 {"lx": 20, "ly": 0, "lz": 0,
                  "rx": 0, "ry": 0, "rz": 0}
        Returns:
            bool
        """
        return self._send_and_wait("offset_move", data)

    def MoveL(self, position, offset=None):
        """末端运动到数据库中的坐标点位，可附加相对偏移

        从数据库取出 position 的坐标值，根据 offset 加减后执行到位运动。
        offset 直接修改目标坐标，而非先到位再偏移。

        Args:
            position: 坐标点位名称字符串，如 "P1"
            offset:  可选，相对偏移量（毫米），如 {"lx": 20}、{"ly": 10}、{"lx":10,"ly":10}
                      lx/ly/lz 对应 X/Y/Z 方向平移，rx/ry/rz 对应旋转（度）
                      不传时仅执行到位运动
        Returns:
            bool: True=运动完成，False=超时或失败
        """
        payload = {"command": "goto", "data": {"type": "right", "name": position}}
        if offset is not None:
            payload["data"]["offset"] = offset
        return self._goto_position(position, payload)

    def _goto_position(self, position, payload=None):
        """发送 goto 命令并等待结果"""
        if payload is None:
            payload = {"command": "goto", "data": {"type": "right", "name": position}}
        msg_str = json.dumps(payload, ensure_ascii=False)

        self._positions_event.clear()
        self._positions_result = None
        self._client.publish(POSITIONS_CTRL_TOPIC, msg_str, qos=0)
        print(f"[Minth] → MoveL: {position}")

        done = self._positions_event.wait(timeout=30)
        if not done:
            print(f"[Minth] ✗ MoveL 超时 (30s)")
            return False

        result = self._positions_result or {}
        cmd = result.get("command", "")
        data = result.get("data", {})
        success = data.get("success", False)
        message = data.get("message", "")
        if success:
            print(f"[Minth] ✓ MoveL: {message}")
        else:
            print(f"[Minth] ✗ MoveL: {message}")
        return success

    def REL(self, data):
        """底盘相对运动
        Args:
            data: dict，单位米，如 {"x": 0.3, "y": 0, "yaw_rad": 0}
                  x: 前进(+)/后退(-)
                  y: 左(+)/右(-)
                  yaw_rad: 左转(+)/右转(-)
        Returns:
            bool
        """
        return self._send_and_wait("go_rel", data)

    def REL_NOWAIT(self, data):
        """底盘相对运动（异步，不等待执行完成）

        命令下发后立即返回，不等待 /humanoid/commands/done 回复。
        适用于 fire-and-forget 场景，例如 CHASSIS_CORRECT 纠偏后
        立即返回继续后续逻辑。

        Args:
            data: dict，单位米，同 REL
        Returns:
            bool: True=命令已成功下发，False=发送失败
        """
        payload = {"command": "go_rel", "data": data}
        msg_str = json.dumps(payload, ensure_ascii=False)
        info = self._client.publish(COMMANDS_TOPIC, msg_str, qos=2)
        ok = getattr(info, "rc", 0) == 0
        print(f"[Minth] → go_rel (nowait): {data}"
              f"{'' if ok else ' [发送失败]'}")
        return ok

    def TTS(self, text):
        """语音播报
        Args:
            text: 要播报的文本字符串
        Returns:
            bool
        """
        return self._send_and_wait("tts", text)

    def GRIPPER(self, data):
        """夹爪控制
        Args:
            data: dict，如 {"left": 0.5, "right": 0.5}
                  负值=张开，正值=闭合
        Returns:
            bool
        """
        return self._send_and_wait("grab", data)

    def YOLO(self, model="wxf.pt", ip=None):
        """YOLO 目标检测

        拍摄头部彩色+深度图，发送给 YOLO 服务进行检测，等待完成后返回。

        通过 MQTT 向 /humanoid/camera/control 发送 {"command":"detect","yolo":"<model>","yolo_ip":"<ip>"}，
        camera.py 执行完毕后会向 /humanoid/commands/done 发送 {"command":"done"}。

        Args:
            model: YOLO 模型文件名，如 "wxf.pt"、"7.14.pt"
            ip:   可选，自定义 YOLO 检测服务端 IP 地址；
                  不传或为 None 时使用 camera.py 中的默认配置 YOLO_TCP_HOST
        Returns:
            bool: True=检测完成，False=超时
        """
        payload = {"command": "detect", "yolo": model}
        if ip:
            payload["yolo_ip"] = ip
        self._done_event.clear()
        msg_str = json.dumps(payload, ensure_ascii=False)
        self._client.publish(CAMERA_TOPIC, msg_str, qos=2)
        print(f"[Minth] → YOLO: model={model}, ip={ip or 'default'}")

        # YOLO 检测耗时较长，使用较长超时
        done = self._done_event.wait(timeout=120)
        if done:
            print(f"[Minth] ✓ YOLO 检测完成")
        else:
            print(f"[Minth] ✗ YOLO 超时 (120s)")
        return done

    def CHASSIS_CORRECT(self, detect_json=None, px_to_meter=None):
        """底盘水平偏移纠正

        读取 detect/detect.json 中的 horizontal_offset_px 像素值，
        按转换系数换算为米，执行底盘 Y 方向相对移动。

        转换关系：70 像素 → 向右 130 毫米
        即 1 像素 → 130/70/1000 ≈ 0.001857 米
        向右为 y 负方向，故 y_meters = -px * 130/75/1000

        Args:
            detect_json: 可选，自定义 detect.json 路径；默认使用 ../detect/detect.json
            px_to_meter: 可选，自定义像素到米的转换系数；默认使用 PX_TO_METER
        Returns:
            bool: True=纠正完成，False=超时或无数据
        """
        path = detect_json or _DETECT_JSON
        if not os.path.isfile(path):
            print(f"[Minth] ✗ CHASSIS_CORRECT: 检测结果文件不存在: {path}")
            return False

        try:
            with open(path, "r", encoding="utf-8") as f:
                result = json.load(f)
        except Exception as e:
            print(f"[Minth] ✗ CHASSIS_CORRECT: 读取 JSON 失败: {e}")
            return False

        px = result.get("horizontal_offset_px")
        if px is None:
            print(f"[Minth] ✗ CHASSIS_CORRECT: 结果中无 horizontal_offset_px 字段")
            return False

        try:
            px = float(px)
        except (TypeError, ValueError):
            print(f"[Minth] ✗ CHASSIS_CORRECT: horizontal_offset_px 不是数值: {px}")
            return False

        coef = px_to_meter if px_to_meter is not None else PX_TO_METER
        y_meters = px * coef
        print(f"[Minth] CHASSIS_CORRECT: offset_px={px:.1f}, y_meters={y_meters:.4f}")

        return self._send_and_wait("go_rel", {"x": 0, "y": y_meters, "yaw_rad": 0})

    def JOINT(self, name, offset=None, value=None):
        """单关节控制

        通过 MQTT 向 /humanoid/joints/control 发送 joint 命令，可增量微调或运动到指定角度。

        Args:
            name: 关节名，如 "idx11_head_joint1"、"idx01_body_joint1"
            offset: 增量微调值（弧度），如 0.01
            value: 目标角度（弧度），如 0.0
            注意：offset 和 value 二选一，若都提供则使用 value
        Returns:
            bool: True=执行完成，False=超时
        """
        if value is None and offset is None:
            print("[Minth] ✗ JOINT: 需要提供 offset 或 value 参数")
            return False

        data = {"name": name}
        if value is not None:
            data["value"] = value
            print(f"[Minth] → JOINT: {name} value={value}")
        else:
            data["offset"] = offset
            print(f"[Minth] → JOINT: {name} offset={offset}")

        return self._send_and_wait("joint", data)

    def WAIST_CORRECT(self, detect_json=None, joint_name="idx05_body_joint5"):
        """腰部旋转纠正

        读取 detect/detect.json 中的 angle_rad 弧度值，
        执行腰部关节旋转到该角度。

        Args:
            detect_json: 可选，自定义 detect.json 路径；默认使用 ../detect/detect.json
            joint_name: 腰部旋转关节名，默认 "idx05_body_joint5"
        Returns:
            bool: True=旋转完成，False=超时或无数据
        """
        path = detect_json or _DETECT_JSON
        if not os.path.isfile(path):
            print(f"[Minth] ✗ WAIST_CORRECT: 检测结果文件不存在: {path}")
            return False

        try:
            with open(path, "r", encoding="utf-8") as f:
                result = json.load(f)
        except Exception as e:
            print(f"[Minth] ✗ WAIST_CORRECT: 读取 JSON 失败: {e}")
            return False

        angle = result.get("angle_rad")*(-1)
        if angle is None:
            print(f"[Minth] ✗ WAIST_CORRECT: 结果中无 angle_rad 字段")
            return False

        try:
            angle = float(angle)
        except (TypeError, ValueError):
            print(f"[Minth] ✗ WAIST_CORRECT: angle_rad 不是数值: {angle}")
            return False

        print(f"[Minth] WAIST_CORRECT: {joint_name} angle_rad={angle:.4f}")
        return self.JOINT(joint_name, offset=angle)


class X2(_RobotBase):
    """Minth X2 机器人控制类（预留）

    后续实现时，在此添加 X2 专属方法。
    """
    pass


class Minth:
    """Minth 机器人命名空间

    用法：
        robot = Minth.G2()
        robot.GO(9)

        # X2（预留）
        # robot = Minth.X2()
    """
    G2 = G2
    X2 = X2