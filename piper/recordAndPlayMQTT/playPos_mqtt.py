#!/usr/bin/env python3
# -*-coding:utf8-*-
# Playback waypoints
import os, time, csv, json
from piper_sdk import *
import paho.mqtt.client as mqtt
from typing import List, Optional

# ================== Configurable Constants ==================
MQTT_TOPIC_CMD = "/Org/B2/B2_560/roboticArm"
MQTT_TOPIC_STATUS = "/Org/B2/B2_560/statusArm"
DEFAULT_BROKER_HOST = "127.0.0.1"
DEFAULT_BROKER_PORT = 31883

# Mapping of position files (extensible)
POSITION_FILES = {
    "default": "pos.csv",
    "welcome": "welcome_pos.csv",
    "right": "right_pos.csv",
    "left": "left_pos.csv",
}

# Maximum wait time (seconds) for each waypoint to be reached
WAYPOINT_TIMEOUT = 8.0
WAYPOINT_JOINT_TOL = 0.0698  # Approx 4 degrees

# Sleep interval in main loop
MAIN_LOOP_SLEEP = 0.1

# ================== Global State (centralized) ==================
init_flag = False          # received initialization command
enter_flag = False         # received playback start command
exit_flag = False          # received exit command
play_ready = False         # arm is ready for playback
current_pos_name = "default"  # current selected position set name
mqtt_client = None
track: List[List[float]] = []   # current waypoint sequence


def reset_session_state():
    """Reset dynamic flags of a single playback session (excluding current position set name)."""
    global init_flag, enter_flag, exit_flag, play_ready
    init_flag = False
    enter_flag = False
    exit_flag = False
    play_ready = False

def load_track_file(file_name: str) -> Optional[List[List[float]]]:
    """Load waypoint file and return 2D list of floats; return None if failed."""
    abs_path = os.path.join(os.path.dirname(__file__), file_name)
    try:
        with open(abs_path, 'r', encoding='utf-8') as f:
            rows = list(csv.reader(f))
        if not rows:
            print(f"ERROR: Waypoint file empty: {file_name}")
            return None
        return [[float(c) for c in r] for r in rows]
    except FileNotFoundError:
        print(f"ERROR: Waypoint file not found: {file_name}")
        return None
    except Exception as e:
        print(f"ERROR: Failed to read waypoint file ({file_name}): {e}")
        return None


def set_pos(pos_name: str):
    """Set current waypoint sequence by position set name."""
    global track, current_pos_name
    file_name = POSITION_FILES.get(pos_name)
    if not file_name:
        publish_status("error", f"Unknown position set: {pos_name}", "setpos")
        return False
    loaded = load_track_file(file_name)
    if loaded is None:
        publish_status("error", f"Failed to load waypoint file: {file_name}", "setpos")
        return False
    track = loaded
    current_pos_name = pos_name
    publish_status("success", f"Waypoint set [{pos_name}] loaded, total {len(track)} entries", f"setpos_{pos_name}")
    return True

def on_connect(client, userdata, flags, rc):
    """MQTT connection callback."""
    if rc == 0:
        print("INFO: MQTT connected successfully")
        client.subscribe(MQTT_TOPIC_CMD)
        print(f"INFO: Subscribed topic: {MQTT_TOPIC_CMD}")
        publish_status("success", "MQTT connected and subscribed command topic", "mqtt_connect")
    else:
        print(f"ERROR: MQTT connection failed, code: {rc}")
        publish_status("error", f"MQTT connection failed rc={rc}", "mqtt_connect")

def publish_status(status, message, cmd=""):
    """Publish execution status to MQTT."""
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
        print(f"ERROR: Failed to publish status message: {e}")

def on_message(client, userdata, msg):
    """MQTT message callback."""
    global init_flag, enter_flag, exit_flag
    try:
        message = json.loads(msg.payload.decode())
        cmd = message.get('cmd', '')
        print(f"INFO: Received MQTT message: {message}")

        if cmd == 'replay_init':
            init_flag = True
            publish_status("success", "Initialization command received", cmd)
        elif cmd.startswith('setpos_'):
            # setpos_welcome / setpos_right / setpos_left / setpos_default
            pos_name = cmd.replace('setpos_', '', 1)
            if set_pos(pos_name):
                # After successful load, if already init, can enter ready state
                publish_status("success", f"Waypoint set [{pos_name}] ready", cmd)
            else:
                publish_status("error", f"Waypoint set [{pos_name}] load failed", cmd)
        elif cmd == 'replay_enter':
            if play_ready:
                enter_flag = True
                publish_status("success", "Start executing playback sequence", cmd)
            else:
                publish_status("error", "Not ready (not initialized)", cmd)
        elif cmd == 'replay_q':
            exit_flag = True
            publish_status("success", "Exit command received, returning to initial state", cmd)
        else:
            publish_status("error", f"Unknown command: {cmd}", cmd)
    except json.JSONDecodeError:
        publish_status("error", "JSON parse failed", "json_parse")
    except Exception as e:
        publish_status("error", f"Exception handling message: {e}", "exception")

if __name__ == "__main__":
    # Whether a gripper exists
    have_gripper = True
    # Playback times; 0 means infinite loop
    play_times = 1
    # Playback interval in seconds; negative means manual key control
    play_interval = 0
    # Motion speed percentage; suggested range: 10-100
    move_spd_rate_ctrl = 100
    # Timeout for switching to CAN control mode in seconds
    timeout = 5.0
    
    # Load default positions (continue even if failed; can set later)
    set_pos("default")

    # Setup MQTT client
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    
    # Connect to MQTT broker
    try:
        mqtt_client.connect(DEFAULT_BROKER_HOST, DEFAULT_BROKER_PORT, 60)
        mqtt_client.loop_start()  # Start MQTT loop
        print("INFO: Connecting to MQTT broker...")
    except Exception as e:
        print(f"ERROR: Failed to connect MQTT broker: {e}")
        exit()

    # Initialize and connect robotic arm
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    time.sleep(0.1)

    def get_pos():
        '''Get current joint radians and gripper opening distance.'''
        joint_state = piper.GetArmJointMsgs().joint_state
        joint_state = tuple(getattr(joint_state, f"joint_{i+1}") / 1e3 * 0.0174533 for i in range(6))
        if have_gripper:
            return joint_state + (piper.GetArmGripperMsgs().gripper_state.grippers_angle / 1e6, )
        return joint_state
    
    def stop():
        '''Stop the arm; first exit from teaching mode must call this before CAN control works.'''
        piper.EmergencyStop(0x01)
        time.sleep(1.0)
        limit_angle = [0.1745, 0.7854, 0.2094]  # Only restore when joints 2,3,5 within safe range to avoid dropping damage
        pos = get_pos()
        while not (abs(pos[1]) < limit_angle[0] and abs(pos[2]) < limit_angle[0] and limit_angle[2] < pos[4] < limit_angle[1]):
            time.sleep(0.01)
            pos = get_pos()
        # Restore the arm
        piper.EmergencyStop(0x02)
        time.sleep(1.0)
    
    def enable():
        '''Enable the arm and gripper.'''
        while not piper.EnablePiper():
            time.sleep(0.01)
        if have_gripper:
            time.sleep(0.01)
            piper.GripperCtrl(0, 1000, 0x01, 0x00)
        piper.ModeCtrl(0x01, 0x01, move_spd_rate_ctrl, 0x00)
        print("INFO: Enable successful")
    
    def init_play():
        '''Initialize the arm into replay (playback) mode.'''
        global play_ready
        print("INFO: Initializing arm into replay mode...")
        print("INFO: Ensure the arm has exited teaching mode before playback")

        if piper.GetArmStatus().arm_status.ctrl_mode != 1:
            stop()  # First exit from teaching mode requires calling this before switching to CAN mode
        over_time = time.time() + timeout
        while piper.GetArmStatus().arm_status.ctrl_mode != 1:
            if over_time < time.time():
                print("ERROR: CAN mode switch failed; check if teaching mode exited")
                publish_status("error", "CAN mode switch failed; check teaching mode exit", "replay_init")
                return False
            piper.ModeCtrl(0x01, 0x01, move_spd_rate_ctrl, 0x00)
            time.sleep(0.01)

        enable()
        play_ready = True
        print("INFO: Arm ready, can start waypoint playback")
        publish_status("success", "Arm ready for waypoint playback", "replay_init")
        return True

    print("INFO: Playback program started, waiting MQTT commands...")
    print("INFO: Commands: replay_init / replay_enter / replay_q / setpos_<name>")
    print("INFO: Position sets: default, welcome, right, left")
    try:
        while True:  # Main state loop
            reset_session_state()

            print("INFO: Waiting for replay_init command...")
            while not init_flag:
                time.sleep(MAIN_LOOP_SLEEP)
            
            # State 2: perform initialization
            if not init_play():
                publish_status("error", "Initialization failed, returning to initial state", "replay_init")
                continue
            
            print("INFO: Waiting for replay_enter command to start playback...")
            
            # State 3: wait for playback command then execute playback
            while play_ready and not exit_flag:
                if not enter_flag:
                    time.sleep(MAIN_LOOP_SLEEP)
                    continue

                enter_flag = False  # consume one playback request
                publish_status("success", f"Start playback, total {len(track)} waypoints", "replay_enter")
                play_completed = True

                for idx, waypoint in enumerate(track):
                    if exit_flag:
                        play_completed = False
                        break

                    # Send motion commands and wait until reached
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
                            publish_status("error", f"Waypoint {idx+1} timeout not reached", "waypoint_timeout")
                            play_completed = False
                            break

                    if not play_completed:
                        break

                    # Gripper control
                    if have_gripper and len(waypoint) == 7:
                        piper.GripperCtrl(round(waypoint[-1] * 1e6), 1000, 0x01, 0x00)
                        time.sleep(0.3)

                    # Playback interval
                    if play_interval > 0:
                        time.sleep(play_interval)

                if play_completed:
                    publish_status("success", f"Playback complete, total {len(track)} waypoints", "replay_complete")
                else:
                    publish_status("error", "Playback did not complete normally", "replay_incomplete")
                break  # exit this round regardless of success, wait next init
            
            # After playback completed or interrupted, return to initial state
            print("INFO: Exit replay mode, back to initial state")
            publish_status("success", "End this round, back to initial waiting", "replay_reset")
            
    except KeyboardInterrupt:
        print("INFO: Program interrupted by user")
    finally:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        print("INFO: Playback program ended")