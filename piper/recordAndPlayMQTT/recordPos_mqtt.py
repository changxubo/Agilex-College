#!/usr/bin/env python3
# -*-coding:utf8-*-
# 录制点位
import os, time, json
from piper_sdk import *
import paho.mqtt.client as mqtt
from typing import Optional

# ================== 常量配置 ==================
MQTT_TOPIC_CMD = "/Amway/B2/B2_560/roboticArm"
MQTT_TOPIC_STATUS = "/Amway/B2/B2_560/statusArm"
BROKER_HOST = "10.142.94.37"
BROKER_PORT = 31883

TEACH_MODE_TIMEOUT = 10.0            # 示教模式进入超时
MAIN_LOOP_SLEEP = 0.1                # 主循环休眠
MAX_POINTS_PER_SESSION: Optional[int] = None  # 限制一次录制最大点位数量 (None 表示不限制)

# 输出文件名基础
BASE_CSV_NAME = "pos.csv"

# ================== 会话状态 ==================
recording_init_flag = False      # 收到初始化指令
recording_enter_flag = False     # 收到录制点指令
exit_flag = False                # 收到退出指令
teaching_mode_ready = False      # 已进入示教模式
mqtt_client = None               # MQTT客户端实例
current_session_points = 0       # 当前会话已录制点位数量


def reset_session_state():
    """重置本轮会话状态（不关闭连接）。"""
    global recording_init_flag, recording_enter_flag, exit_flag, teaching_mode_ready, current_session_points
    recording_init_flag = False
    recording_enter_flag = False
    exit_flag = False
    teaching_mode_ready = False
    current_session_points = 0

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
    """发布命令执行状态到状态主题，并带上上下文信息。"""
    try:
        payload = {
            "cmd": cmd,
            "status": status,
            "message": message,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "teaching_ready": teaching_mode_ready,
            "points_in_session": current_session_points,
            "max_points": MAX_POINTS_PER_SESSION,
        }
        if mqtt_client:
            mqtt_client.publish(MQTT_TOPIC_STATUS, json.dumps(payload), qos=0, retain=False)
        print(f"STATUS: {payload}")
    except Exception as e:
        print(f"ERROR: 发布状态消息失败: {e}")

def on_message(client, userdata, msg):
    """MQTT消息回调函数"""
    global recording_init_flag, recording_enter_flag, exit_flag
    try:
        message = json.loads(msg.payload.decode())
        cmd = message.get('cmd', '')
        print(f"INFO: 收到MQTT消息: {message}")

        if cmd == 'recording_init':
            recording_init_flag = True
            publish_status("success", "收到初始化命令", cmd)
        elif cmd == 'recording_enter':
            if teaching_mode_ready:
                recording_enter_flag = True
                publish_status("success", "开始录制当前点位", cmd)
            else:
                publish_status("error", "尚未进入示教模式", cmd)
        elif cmd == 'recording_q':
            exit_flag = True
            publish_status("success", "收到退出命令，准备退出本轮", cmd)
        elif cmd == 'recording_stop':  # 额外命令：放弃当前会话
            exit_flag = True
            publish_status("success", "收到停止命令，终止当前会话", cmd)
        else:
            publish_status("error", f"未知命令: {cmd}", cmd)
    except json.JSONDecodeError:
        publish_status("error", "JSON解析失败", "json_parse")
    except Exception as e:
        publish_status("error", f"处理消息异常: {e}", "exception")

if __name__ == "__main__":
    # 是否有夹爪
    have_gripper = True
    # 保存点位的CSV文件路径（基准路径）
    CSV_path = os.path.join(os.path.dirname(__file__), BASE_CSV_NAME)
    
    # 设置MQTT客户端
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    
    # 连接MQTT服务器（请根据实际情况修改服务器地址和端口）
    try:
        mqtt_client.connect(BROKER_HOST, BROKER_PORT, 60)
        mqtt_client.loop_start()
        print("INFO: 正在连接MQTT服务器...")
    except Exception as e:
        print(f"ERROR: 无法连接MQTT服务器: {e}")
        publish_status("error", f"MQTT连接失败: {e}", "mqtt_connect")
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
    
    def init_teaching_mode():
        """初始化进入示教模式：等待机械臂切换 ctrl_mode == 2"""
        global teaching_mode_ready
        print("INFO: 初始化示教模式，等待用户按下示教按钮 ctrl_mode==2")
        start_t = time.time()
        while piper.GetArmStatus().arm_status.ctrl_mode != 2:
            if exit_flag:
                publish_status("error", "初始化被外部退出指令中断", "recording_init")
                return False
            if time.time() - start_t > TEACH_MODE_TIMEOUT:
                publish_status("error", "进入示教模式超时", "recording_init")
                return False
            time.sleep(0.05)
        teaching_mode_ready = True
        publish_status("success", "已进入示教模式，可开始录制", "recording_init")
        return True
    
    print("INFO: 录制程序启动，等待MQTT命令...")
    print("INFO: 命令: recording_init / recording_enter / recording_q / recording_stop")
    if MAX_POINTS_PER_SESSION:
        print(f"INFO: 每次最多录制 {MAX_POINTS_PER_SESSION} 个点位")
    
    try:
        while True:  # 主状态循环
            reset_session_state()
            
            print("INFO: 等待 recording_init 命令...")
            
            # 状态1: 等待初始化命令
            while not recording_init_flag:
                time.sleep(MAIN_LOOP_SLEEP)
            
            # 状态2: 执行示教模式初始化
            if not init_teaching_mode():
                print("ERROR: 示教模式初始化失败，回到初始状态")
                publish_status("error", "示教模式初始化失败，回到初始状态", "recording_init")
                continue
            
            # 创建新的CSV文件用于本次录制
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            current_csv_path = CSV_path.replace(".csv", f"_{timestamp}.csv")
            csv_file = open(current_csv_path, "w")
            count = 1
            current_session_points = 0
            
            print("INFO: 等待 recording_enter 命令开始录制...")
            print(f"INFO: 本次录制将保存到: {current_csv_path}")
            
            # 状态3: 等待录制命令并执行录制
            try:
                while teaching_mode_ready and not exit_flag:
                    if recording_enter_flag:
                        recording_enter_flag = False
                        # 录制当前点位
                        pos_values = get_pos()
                        csv_file.write(",".join(map(str, pos_values)) + "\n")
                        csv_file.flush()
                        count += 1
                        current_session_points += 1
                        publish_status("success", f"录制第{count-1}个点位成功", "recording_enter")
                        # 达到上限自动结束
                        if MAX_POINTS_PER_SESSION and current_session_points >= MAX_POINTS_PER_SESSION:
                            publish_status("success", "达到本次录制上限，自动结束会话", "recording_limit")
                            exit_flag = True
                    time.sleep(MAIN_LOOP_SLEEP)
                    
            except Exception as e:
                print(f"ERROR: 录制过程中出错: {e}")
                publish_status("error", f"录制过程中出错: {e}", "recording_enter")
            finally:
                csv_file.close()
                
                # 将临时文件重命名为标准文件名（保留最新录制）
                if count > 1:
                    try:
                        import shutil
                        shutil.copy2(current_csv_path, CSV_path)
                        publish_status("success", f"录制完成，共录制{count-1}个点位，保存为 {CSV_path}", "recording_complete")
                    except Exception as e:
                        publish_status("error", f"保存失败: {e}", "recording_complete")
                else:
                    publish_status("success", "本次未录制任何点位", "recording_complete")
            
            # 录制完成或被中断后，回到初始状态
            print("INFO: 退出录制模式，回到初始状态")
            publish_status("success", "结束本轮录制，返回初始等待", "recording_reset")
            
    except KeyboardInterrupt:
        print("INFO: 程序被用户中断")
    finally:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        print("INFO: 录制程序结束")