#!/usr/bin/env python3
# -*-coding:utf8-*-
# Record waypoints
import os, time, json
from piper_sdk import *
import paho.mqtt.client as mqtt
from typing import Optional

# ================== Constant Configuration ==================
MQTT_TOPIC_CMD = "/Org/B2/B2_560/roboticArm"
MQTT_TOPIC_STATUS = "/Org/B2/B2_560/statusArm"
BROKER_HOST = "127.0.0.1"
BROKER_PORT = 31883

TEACH_MODE_TIMEOUT = 10.0            # Timeout for entering teaching mode
MAIN_LOOP_SLEEP = 0.1                # Main loop sleep interval
MAX_POINTS_PER_SESSION: Optional[int] = None  # Limit maximum recorded waypoints per session (None = unlimited)

# Base output filename
BASE_CSV_NAME = "pos.csv"

# ================== Session State ==================
recording_init_flag = False      # received initialization command
recording_enter_flag = False     # received capture command
exit_flag = False                # received exit command
teaching_mode_ready = False      # teaching mode ready (ctrl_mode == 2)
mqtt_client = None               # MQTT client instance
current_session_points = 0       # number of waypoints recorded in current session


def reset_session_state():
    """Reset state for the current recording session (without closing connections)."""
    global recording_init_flag, recording_enter_flag, exit_flag, teaching_mode_ready, current_session_points
    recording_init_flag = False
    recording_enter_flag = False
    exit_flag = False
    teaching_mode_ready = False
    current_session_points = 0

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
    """Publish command execution status (with context) to status topic."""
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
        print(f"ERROR: Failed to publish status message: {e}")

def on_message(client, userdata, msg):
    """MQTT message callback."""
    global recording_init_flag, recording_enter_flag, exit_flag
    try:
        message = json.loads(msg.payload.decode())
        cmd = message.get('cmd', '')
        print(f"INFO: Received MQTT message: {message}")

        if cmd == 'recording_init':
            recording_init_flag = True
            publish_status("success", "Initialization command received", cmd)
        elif cmd == 'recording_enter':
            if teaching_mode_ready:
                recording_enter_flag = True
                publish_status("success", "Recording current waypoint", cmd)
            else:
                publish_status("error", "Teaching mode not ready", cmd)
        elif cmd == 'recording_q':
            exit_flag = True
            publish_status("success", "Exit command received; ending this round", cmd)
        elif cmd == 'recording_stop':  # extra command: abort current session
            exit_flag = True
            publish_status("success", "Stop command received; terminating current session", cmd)
        else:
            publish_status("error", f"Unknown command: {cmd}", cmd)
    except json.JSONDecodeError:
        publish_status("error", "JSON parse failed", "json_parse")
    except Exception as e:
        publish_status("error", f"Exception handling message: {e}", "exception")

if __name__ == "__main__":
    # Whether a gripper exists
    have_gripper = True
    # Base CSV path for storing waypoints
    CSV_path = os.path.join(os.path.dirname(__file__), BASE_CSV_NAME)
    
    # Setup MQTT client
    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message
    
    # Connect to MQTT broker (adjust host/port as needed)
    try:
        mqtt_client.connect(BROKER_HOST, BROKER_PORT, 60)
        mqtt_client.loop_start()
        print("INFO: Connecting to MQTT broker...")
    except Exception as e:
        print(f"ERROR: Failed to connect MQTT broker: {e}")
        publish_status("error", f"MQTT connection failed: {e}", "mqtt_connect")
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
    
    def init_teaching_mode():
        """Initialize teaching mode: wait until arm switches to ctrl_mode == 2."""
        global teaching_mode_ready
        print("INFO: Initializing teaching mode; waiting for user to press teaching button (ctrl_mode==2)")
        start_t = time.time()
        while piper.GetArmStatus().arm_status.ctrl_mode != 2:
            if exit_flag:
                publish_status("error", "Initialization interrupted by external exit command", "recording_init")
                return False
            if time.time() - start_t > TEACH_MODE_TIMEOUT:
                publish_status("error", "Entering teaching mode timeout", "recording_init")
                return False
            time.sleep(0.05)
        teaching_mode_ready = True
        publish_status("success", "Teaching mode entered; ready to record", "recording_init")
        return True
    
    print("INFO: Recording program started; waiting for MQTT commands...")
    print("INFO: Commands: recording_init / recording_enter / recording_q / recording_stop")
    if MAX_POINTS_PER_SESSION:
        print(f"INFO: Max waypoints per session: {MAX_POINTS_PER_SESSION}")
    
    try:
        while True:  # Main state loop
            reset_session_state()
            
            print("INFO: Waiting for recording_init command...")
            
            # State 1: wait for initialization command
            while not recording_init_flag:
                time.sleep(MAIN_LOOP_SLEEP)
            
            # State 2: perform teaching mode initialization
            if not init_teaching_mode():
                print("ERROR: Teaching mode initialization failed; returning to initial state")
                publish_status("error", "Teaching mode initialization failed; returning to initial state", "recording_init")
                continue
            
            # Create a new CSV file for this recording session
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            current_csv_path = CSV_path.replace(".csv", f"_{timestamp}.csv")
            csv_file = open(current_csv_path, "w")
            count = 1
            current_session_points = 0
            
            print("INFO: Waiting for recording_enter command to record...")
            print(f"INFO: This session will save to: {current_csv_path}")
            
            # State 3: wait for recording command and record
            try:
                while teaching_mode_ready and not exit_flag:
                    if recording_enter_flag:
                        recording_enter_flag = False
                        # Record current waypoint
                        pos_values = get_pos()
                        csv_file.write(",".join(map(str, pos_values)) + "\n")
                        csv_file.flush()
                        count += 1
                        current_session_points += 1
                        publish_status("success", f"Recorded waypoint #{count-1} successfully", "recording_enter")
                        # Auto end session when reaching the limit
                        if MAX_POINTS_PER_SESSION and current_session_points >= MAX_POINTS_PER_SESSION:
                            publish_status("success", "Reached session waypoint limit; ending session", "recording_limit")
                            exit_flag = True
                    time.sleep(MAIN_LOOP_SLEEP)
                    
            except Exception as e:
                print(f"ERROR: Error during recording: {e}")
                publish_status("error", f"Error during recording: {e}", "recording_enter")
            finally:
                csv_file.close()
                
                # Copy session file to the base filename (keep latest recording)
                if count > 1:
                    try:
                        import shutil
                        shutil.copy2(current_csv_path, CSV_path)
                        publish_status("success", f"Recording finished; {count-1} waypoints saved as {CSV_path}", "recording_complete")
                    except Exception as e:
                        publish_status("error", f"Save failed: {e}", "recording_complete")
                else:
                    publish_status("success", "No waypoints recorded this session", "recording_complete")
            
            # After recording finished or interrupted, return to initial state
            print("INFO: Exit recording mode, back to initial state")
            publish_status("success", "End this recording round; back to initial waiting", "recording_reset")
            
    except KeyboardInterrupt:
        print("INFO: Program interrupted by user")
    finally:
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        print("INFO: Recording program ended")