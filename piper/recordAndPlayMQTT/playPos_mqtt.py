#!/usr/bin/env python3
# -*-coding:utf8-*-
# 播放点位
import os, time, csv, json
from piper_sdk import *
import paho.mqtt.client as mqtt
from typing import List, Optional

# ================== 可配置常量 ==================
MQTT_TOPIC_CMD = "/Amway/B2/B2_560/roboticArm"
MQTT_TOPIC_STATUS = "/Amway/B2/B2_560/statusArm"
DEFAULT_BROKER_HOST = "10.142.94.37"
DEFAULT_BROKER_PORT = 31883

# 位置文件映射（可继续扩展）
POSITION_FILES = {
    "default": "pos.csv",
    "welcome": "welcome_pos.csv",
    "right": "right_pos.csv",
    "left": "left_pos.csv",
}

# 运动到目标点的最大等待时间（秒）
WAYPOINT_TIMEOUT = 8.0
WAYPOINT_JOINT_TOL = 0.0698  # 约 4°

# 主循环休眠
MAIN_LOOP_SLEEP = 0.1

# ================== 全局状态（尽量集中管理） ==================
init_flag = False          # 收到初始化指令
enter_flag = False         # 收到进入播放指令
exit_flag = False          # 收到退出指令
play_ready = False         # 机械臂准备好播放
current_pos_name = "default"  # 当前选择的点位集名称
mqtt_client = None
track: List[List[float]] = []   # 当前点位序列


def reset_session_state():
    """重置一次播放会话内的动态标志（不含当前点位名）。"""
    global init_flag, enter_flag, exit_flag, play_ready
    init_flag = False
    enter_flag = False
    exit_flag = False
    play_ready = False

def load_track_file(file_name: str) -> Optional[List[List[float]]]:
    """加载点位文件并返回二维浮点数组；失败返回 None。"""
    abs_path = os.path.join(os.path.dirname(__file__), file_name)
    try:
        with open(abs_path, 'r', encoding='utf-8') as f:
            rows = list(csv.reader(f))
        if not rows:
            print(f"ERROR: 点位文件为空: {file_name}")
            return None
        return [[float(c) for c in r] for r in rows]
    except FileNotFoundError:
        print(f"ERROR: 点位文件不存在: {file_name}")
        return None
    except Exception as e:
        print(f"ERROR: 读取点位文件失败({file_name}): {e}")
        return None


def set_pos(pos_name: str):
    """设置当前使用的点位序列。"""
    global track, current_pos_name
    file_name = POSITION_FILES.get(pos_name)
    if not file_name:
        publish_status("error", f"未知的点位集: {pos_name}", "setpos")
        return False
    loaded = load_track_file(file_name)
    if loaded is None:
        publish_status("error", f"点位文件加载失败: {file_name}", "setpos")
        return False
    track = loaded
    current_pos_name = pos_name
    publish_status("success", f"点位[{pos_name}]加载成功, 共{len(track)}条", f"setpos_{pos_name}")
    return True

def on_connect(client, userdata, flags, rc):
    """MQTT连接回调函数"""
    if rc == 0:
        print("INFO: MQTT连接成功")
        client.subscribe(MQTT_TOPIC_CMD)
        print(f"INFO: 已订阅topic: {MQTT_TOPIC_CMD}")
        publish_status("success", "MQTT连接成功并已订阅命令", "mqtt_connect")
    else:
        print(f"ERROR: MQTT连接失败，错误代码: {rc}")
        publish_status("error", f"MQTT连接失败 rc={rc}", "mqtt_connect")

def publish_status(status, message, cmd=""):
    """发布命令执行状态到MQTT"""
    try:
        data = {
            "cmd": cmd,
            "status": status,
            "message": message,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "pos_set": current_pos_name,
            "track_len": len(track),
        }
        if mqtt_client is not None:
            mqtt_client.publish(MQTT_TOPIC_STATUS, json.dumps(data), qos=0, retain=False)
        print(f"STATUS: {data}")
    except Exception as e:
        print(f"ERROR: 发布状态消息失败: {e}")

def on_message(client, userdata, msg):
    """MQTT消息回调函数"""
    global init_flag, enter_flag, exit_flag
    try:
        message = json.loads(msg.payload.decode())
        cmd = message.get('cmd', '')
        print(f"INFO: 收到MQTT消息: {message}")

        if cmd == 'replay_init':
            init_flag = True
            publish_status("success", "收到初始化命令", cmd)
        elif cmd.startswith('setpos_'):
            # setpos_welcome / setpos_right / setpos_left / setpos_default
            pos_name = cmd.replace('setpos_', '', 1)
            if set_pos(pos_name):
                # 成功加载点位后，如果已经 init 过，可立即进入 ready 状态再播放
                publish_status("success", f"点位集[{pos_name}]已准备", cmd)
            else:
                publish_status("error", f"点位集[{pos_name}]加载失败", cmd)
        elif cmd == 'replay_enter':
            if play_ready:
                enter_flag = True
                publish_status("success", "开始执行播放序列", cmd)
            else:
                publish_status("error", "未准备好（尚未初始化）", cmd)
        elif cmd == 'replay_q':
            exit_flag = True
            publish_status("success", "收到退出命令，准备回到初始状态", cmd)
        else:
            publish_status("error", f"未知命令: {cmd}", cmd)
    except json.JSONDecodeError:
        publish_status("error", "JSON解析失败", "json_parse")
    except Exception as e:
        publish_status("error", f"处理消息异常: {e}", "exception")

if __name__ == "__main__":
    # 是否有夹爪
    have_gripper = True
    # 播放次数，0表示无限循环
    play_times = 1
    # 播放间隔，单位：秒，负数表示人工按键控制
    play_interval = 0
    # 运动速度百分比，建议范围：10-100
    move_spd_rate_ctrl = 100
    # 切换CAN模式超时时间，单位：秒
    timeout = 5.0
    
    # 设置默认点位（如果失败仍继续，可稍后再设置）
    set_pos("default")

    # 设置MQTT客户端
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    
    # 连接MQTT服务器
    try:
        mqtt_client.connect(DEFAULT_BROKER_HOST, DEFAULT_BROKER_PORT, 60)
        mqtt_client.loop_start()  # 开始MQTT循环
        print("INFO: 正在连接MQTT服务器...")
    except Exception as e:
        print(f"ERROR: 无法连接MQTT服务器: {e}")
        exit()

    # 初始化并连接机械臂
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    time.sleep(0.1)

    def get_pos():
        '''获取机械臂当前关节弧度和夹爪张开距离'''
        joint_state = piper.GetArmJointMsgs().joint_state
        joint_state = tuple(getattr(joint_state, f"joint_{i+1}") / 1e3 * 0.0174533 for i in range(6))
        if have_gripper:
            return joint_state + (piper.GetArmGripperMsgs().gripper_state.grippers_angle / 1e6, )
        return joint_state
    
    def stop():
        '''停止机械臂；初次退出示教模式需先调用此函数才能使用CAN模式控制机械臂'''
        piper.EmergencyStop(0x01)
        time.sleep(1.0)
        limit_angle = [0.1745, 0.7854, 0.2094]  # 2、3、5关节弧度在限制范围内时才恢复机械臂，防止大弧度直接掉落造成损坏
        pos = get_pos()
        while not (abs(pos[1]) < limit_angle[0] and abs(pos[2]) < limit_angle[0] and pos[4] < limit_angle[1] and pos[4] > limit_angle[2]):
            time.sleep(0.01)
            pos = get_pos()
        # 恢复机械臂
        piper.EmergencyStop(0x02)
        time.sleep(1.0)
    
    def enable():
        '''使能机械臂和夹爪'''
        while not piper.EnablePiper():
            time.sleep(0.01)
        if have_gripper:
            time.sleep(0.01)
            piper.GripperCtrl(0, 1000, 0x01, 0x00)
        piper.ModeCtrl(0x01, 0x01, move_spd_rate_ctrl, 0x00)
        print("INFO: 使能成功")
    
    def init_play():
        '''初始化机械臂进入播放模式'''
        global play_ready
        print("INFO: 正在初始化机械臂进入replay模式...")
        print("INFO: 播放前请确保机械臂已退出示教模式")
        
        if piper.GetArmStatus().arm_status.ctrl_mode != 1:
            stop()  # 初次退出示教模式需先调用此函数才能切换至CAN模式
        over_time = time.time() + timeout
        while piper.GetArmStatus().arm_status.ctrl_mode != 1:
            if over_time < time.time():
                print("ERROR: CAN模式切换失败，请检查是否退出示教模式")
                publish_status("error", "CAN模式切换失败，请检查是否退出示教模式", "replay_init")
                return False
            piper.ModeCtrl(0x01, 0x01, move_spd_rate_ctrl, 0x00)
            time.sleep(0.01)
        
        enable()
        play_ready = True
        print("INFO: 机械臂已准备就绪，可以开始播放点位")
        publish_status("success", "机械臂已准备就绪，可以开始播放点位", "replay_init")
        return True

    print("INFO: 播放程序已启动，等待MQTT命令...")
    print("INFO: 命令: replay_init / replay_enter / replay_q / setpos_<name>")
    print("INFO: 位置集: default, welcome, right, left")
    try:
        while True:  # 主状态循环
            reset_session_state()

            print("INFO: 等待 replay_init 命令...")
            while not init_flag:
                time.sleep(MAIN_LOOP_SLEEP)
            
            # 状态2: 执行初始化
            if not init_play():
                publish_status("error", "初始化失败，回到初始状态", "replay_init")
                continue
            
            print("INFO: 等待 replay_enter 命令开始播放...")
            
            # 状态3: 等待播放命令并执行播放
            while play_ready and not exit_flag:
                if not enter_flag:
                    time.sleep(MAIN_LOOP_SLEEP)
                    continue

                enter_flag = False  # 消费一次播放请求
                publish_status("success", f"开始播放，共{len(track)}个点位", "replay_enter")
                play_completed = True

                for idx, waypoint in enumerate(track):
                    if exit_flag:
                        play_completed = False
                        break

                    # 发送指令并等待达成
                    joints = [round(i / 0.0174533 * 1e3) for i in waypoint[:-1]]
                    start_t = time.time()
                    while True:
                        if exit_flag:
                            play_completed = False
                            break
                        piper.MotionCtrl_2(0x01, 0x01, move_spd_rate_ctrl, 0x00)
                        piper.JointCtrl(*joints)
                        time.sleep(0.02)
                        curr = get_pos()
                        reached = all(abs(curr[i] - waypoint[i]) < WAYPOINT_JOINT_TOL for i in range(6))
                        if reached:
                            break
                        if time.time() - start_t > WAYPOINT_TIMEOUT:
                            publish_status("error", f"第{idx+1}个点位超时未到位", "waypoint_timeout")
                            play_completed = False
                            break

                    if not play_completed:
                        break

                    # 夹爪控制
                    if have_gripper and len(waypoint) == 7:
                        piper.GripperCtrl(round(waypoint[-1] * 1e6), 1000, 0x01, 0x00)
                        time.sleep(0.3)

                    # 播放间隔
                    if play_interval > 0:
                        time.sleep(play_interval)

                if play_completed:
                    publish_status("success", f"播放完成，共{len(track)}个点位", "replay_complete")
                else:
                    publish_status("error", "播放未正常完成", "replay_incomplete")
                break  # 无论成功与否退出本轮，等待下一次 init
            
            # 播放完成或被中断后，回到初始状态
            print("INFO: 退出replay模式，回到初始状态")
            publish_status("success", "结束本轮，回到初始等待", "replay_reset")
            
    except KeyboardInterrupt:
        print("INFO: 程序被用户中断")
    finally:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        print("INFO: 播放程序结束")