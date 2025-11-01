#!/usr/bin/env bash
# ---------------------------------------------------------------
# 自动启动脚本 —— 初始化 CAN 总线并运行播放/录制 MQTT 服务
# 支持: 日志输出 / 失败重试 / 后台运行 / 可配置参数
# ---------------------------------------------------------------

set -Eeuo pipefail
IFS=$'\n\t'

# ========== 可配置参数 ==========
CAN_IFACE="can0"
CAN_BITRATE="1000000"
PIPER_SDK_DIR="${HOME}/piper_sdk/piper_sdk"
PROJECT_DIR="${HOME}/Agilex-College/piper/recordAndPlayMQTT"
PLAY_SCRIPT="playPos_mqtt.py"
RECORD_SCRIPT="recordPos_mqtt.py"
PYTHON_BIN="python3"
LOG_DIR="${PROJECT_DIR}/logs"
RETRY_CAN=3
SLEEP_BETWEEN_RETRY=2

# 是否使用 nohup 后台运行 (true/false)
USE_NOHUP=true

# ========== 日志工具函数 ==========
timestamp() { date '+%Y-%m-%d %H:%M:%S'; }
log() { printf '%s [INFO] %s\n' "$(timestamp)" "$*"; }
warn() { printf '%s [WARN] %s\n' "$(timestamp)" "$*" >&2; }
err() { printf '%s [ERROR] %s\n' "$(timestamp)" "$*" >&2; }

require_cmd() {
	command -v "$1" >/dev/null 2>&1 || { err "缺少命令: $1"; exit 1; }
}

mkdir -p "${LOG_DIR}" || { err "无法创建日志目录 ${LOG_DIR}"; exit 1; }

log "检查必要命令..."
for c in ${PYTHON_BIN} ip link; do
	require_cmd "${c}"
done

# ========== 初始化 CAN ==========
start_can() {
	log "初始化 CAN 接口 ${CAN_IFACE} @ ${CAN_BITRATE}"
	pushd "${PIPER_SDK_DIR}" >/dev/null || { err "进入目录失败: ${PIPER_SDK_DIR}"; return 1; }
	if [[ -f can_activate.sh ]]; then
		bash can_activate.sh "${CAN_IFACE}" "${CAN_BITRATE}" || return 1
	else
		err "未找到 can_activate.sh"
		return 1
	fi
	popd >/dev/null
	log "CAN 初始化完成"
}

attempt=1
while (( attempt <= RETRY_CAN )); do
	if start_can; then
		break
	else
		warn "CAN 初始化失败 (第 ${attempt} 次)，重试中..."
		(( attempt++ ))
		sleep "${SLEEP_BETWEEN_RETRY}"
	fi
done
if (( attempt > RETRY_CAN )); then
	err "CAN 初始化多次失败，退出"
	exit 1
fi

# ========== 启动 Python 服务 ==========
start_service() {
	local script="$1"
	local name="$2"
	pushd "${PROJECT_DIR}" >/dev/null || { err "进入目录失败: ${PROJECT_DIR}"; return 1; }
	if [[ ! -f "${script}" ]]; then
		err "脚本不存在: ${script}"
		popd >/dev/null
		return 1
	fi

	local log_file="${LOG_DIR}/${name}.log"
	if [[ "${USE_NOHUP}" == true ]]; then
		log "后台启动 ${script} -> ${log_file}"
		nohup "${PYTHON_BIN}" "${script}" >>"${log_file}" 2>&1 &
	else
		log "前台启动 ${script} (日志: ${log_file})"
		"${PYTHON_BIN}" "${script}" | tee -a "${log_file}" &
	fi
	popd >/dev/null
}

start_service "${PLAY_SCRIPT}" "play"
start_service "${RECORD_SCRIPT}" "record"

log "所有服务已启动。PID 列表:"
jobs -l || true

cat <<EOF
---------------------------------------------------------------
启动完成:
	播放脚本: ${PLAY_SCRIPT}
	录制脚本: ${RECORD_SCRIPT}
日志目录: ${LOG_DIR}
可使用 'tail -f ${LOG_DIR}/play.log' 查看播放日志
可使用 'tail -f ${LOG_DIR}/record.log' 查看录制日志
结束服务示例:
	pkill -f ${PLAY_SCRIPT}
	pkill -f ${RECORD_SCRIPT}
或使用: kill <PID>
建议长期运行可转为 systemd service。
---------------------------------------------------------------
EOF

exit 0