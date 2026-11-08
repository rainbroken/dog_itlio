#!/usr/bin/env bash
# 只订阅诊断数据；不启动底盘、不发目标或速度、不修改运行参数。
set -eo pipefail

workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
output=""
duration=0
raw=false
usage() {
  cat <<'HELP'
用法：tools/record.sh [-o 输出目录] [--duration 秒数] [--raw]
默认保存到工作空间 rosbag/navigation_时间_随机后缀/，Ctrl+C 正常结束并保存。
--duration 120   录制 120 秒后自动结束（0 表示不限时）。
--raw            额外录制 /interfaces/lidar，供重新运行 LIO 使用。
默认包含去畸变点云、IMU、定位地图、TF、路线、速度指令及底盘反馈。
可通过 RECORD_WEB_URL 指定网页地址，默认 http://127.0.0.1:8080。
请先启动导航，再开始录制，看到“开始录制”后重新规划并复现问题。
HELP
}
while (($#)); do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    -o|--output|--duration)
      option=$1
      if (($#<2)); then echo "$option 缺少参数" >&2; exit 2; fi
      if [[ $option == --duration ]]; then duration=$2; else output=$2; fi
      shift 2 ;;
    --raw) raw=true; shift ;;
    *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
done
if ! [[ $duration =~ ^[0-9]+$ ]] || ((${#duration}>6)); then
  echo '--duration 需要 0～999999 的整数秒数' >&2; exit 2
fi
duration=$((10#$duration))
if [[ -n $output && -e $output ]]; then echo "输出路径已存在，拒绝覆盖：$output" >&2; exit 2; fi
if [[ ! -f $workspace/install/setup.bash ]]; then echo '请先编译工作空间，缺少 install/setup.bash' >&2; exit 1; fi
source /opt/ros/humble/setup.bash
source "$workspace/install/setup.bash"
command -v ros2 >/dev/null
command -v timeout >/dev/null
if [[ -z $output ]]; then
  mkdir -p "$workspace/rosbag"
  output=$(mktemp -d "$workspace/rosbag/navigation_$(date +%Y%m%d_%H%M%S)_XXXXXX")
else
  mkdir -p -- "$output"
fi
output=$(cd -- "$output" && pwd)
mkdir -p "$output/context/config" "$output/context/parameters"

topics=(
  /tf /tf_static /clock /rosout /diagnostics /parameter_events
  /initialpose /goal_pose /navigation/planner_goal
  /lio/odom /lio/cloud_deskewed /interfaces/imu
  /map_localizer/status /map_localizer/pose /map_localizer/map
  /local_navigation/status /local_navigation/global_path /local_navigation/local_path
  /local_navigation/terrain_route /local_navigation/obstacles /local_navigation/stop_required
  /navigation_executor/status /navigation_executor/trajectory /navigation/command
  /velocity_guard/status /cmd_vel /dog/safe_cmd_vel /dog/status /dog/odom
  /navigate_terrain/_action/status /navigate_terrain/_action/feedback
)
if $raw; then topics+=(/interfaces/lidar); fi
printf '%s\n' "${topics[@]}" > "$output/context/topics.txt"
cat > "$output/context/qos.yaml" <<'QOS'
/tf_static:
  history: keep_last
  depth: 100
  reliability: reliable
  durability: transient_local
/map_localizer/map:
  history: keep_last
  depth: 1
  reliability: reliable
  durability: transient_local
QOS
{
  date --iso-8601=seconds
  printf 'workspace=%s\nROS_DOMAIN_ID=%s\nROS_LOCALHOST_ONLY=%s\nRMW_IMPLEMENTATION=%s\n' \
    "$workspace" "${ROS_DOMAIN_ID:-0}" "${ROS_LOCALHOST_ONLY:-0}" "${RMW_IMPLEMENTATION:-default}"
  git -C "$workspace" rev-parse HEAD || true
  git -C "$workspace" status --short || true
  df -h "$output"
} > "$output/context/session.txt" 2>&1
shopt -s nullglob
for file in "$workspace"/src/{dog_bringup,dog_navigation,dog_slam}/config/*.yaml \
            "$workspace"/src/3rd_utils/itlio/itera_lio/config/*.yaml \
            "$workspace"/data/config/motion.json "$workspace"/data/maps/.navigation-motion.json; do
  relative=${file#"$workspace"/}
  mkdir -p -- "$output/context/config/$(dirname -- "$relative")"
  cp -- "$file" "$output/context/config/$relative"
done

# 与录包并行读取，避免参数服务缺失导致错过刚开始的故障。
snapshot() {
  timeout 6 ros2 node list > "$output/context/nodes.txt" 2>&1 &
  timeout 6 ros2 topic list -t > "$output/context/topic-types.txt" 2>&1 &
  for node in local_navigation navigation_executor velocity_guard map_localizer itera_lio_node dog_driver; do
    timeout 8 ros2 param dump --no-daemon --spin-time 1 "/$node" > "$output/context/parameters/$node.yaml" 2>&1 &
  done
  python3 - "$output/context" <<'PY' &
import json, os, pathlib, sys, urllib.request
root = pathlib.Path(sys.argv[1])
opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
base = os.environ.get('RECORD_WEB_URL', 'http://127.0.0.1:8080').rstrip('/')
for name, path in [('map', '/api/map'), ('robot', '/api/robot'),
                   ('navigation-settings', '/api/settings/navigation')]:
    try:
        with opener.open(base + path, timeout=2) as response:
            value = json.load(response)
    except Exception as error:
        value = {'snapshot_error': str(error)}
    (root / (name + '.json')).write_text(json.dumps(value, ensure_ascii=False, indent=2))
PY
  ps -eo pid,ppid,pcpu,pmem,rss,etime,args --sort=-rss > "$output/context/processes.txt"
  cat /proc/meminfo > "$output/context/meminfo.txt"
  wait
}

recorder_pid=""; timer_pid=""; snapshot_pid=""
finish() {
  local result=$?
  trap '' INT TERM
  trap - EXIT
  if [[ -n $timer_pid ]]; then kill "$timer_pid" 2>/dev/null || true; fi
  if [[ -n $recorder_pid ]]; then
    kill -INT "$recorder_pid" 2>/dev/null || true
    wait "$recorder_pid" 2>/dev/null || true
  fi
  if [[ -n $snapshot_pid ]]; then wait "$snapshot_pid" 2>/dev/null || true; fi
  if [[ -f $output/bag/metadata.yaml ]]; then
    ros2 bag info "$output/bag" > "$output/context/bag-info.txt" 2>&1 || true
    echo "录制已保存：$output"
  else
    echo "未生成完整 bag，请检查：$output/recorder.log" >&2
    if ((result==0)); then result=1; fi
  fi
  exit "$result"
}
trap finish EXIT
trap 'exit 0' INT TERM
snapshot > "$output/context/snapshot.log" 2>&1 & snapshot_pid=$!
# 不压缩以降低实时 CPU 负担；缓存 32 MiB，单文件约 512 MiB 后分卷。
ros2 bag record -s sqlite3 -o "$output/bag" --include-hidden-topics \
  --max-cache-size 33554432 --max-bag-size 536870912 \
  --qos-profile-overrides-path "$output/context/qos.yaml" \
  "${topics[@]}" > "$output/recorder.log" 2>&1 & recorder_pid=$!
if ((duration>0)); then
  python3 - "$duration" "$recorder_pid" <<'PY' &
import os, signal, sys, time
time.sleep(int(sys.argv[1]))
try:
    os.kill(int(sys.argv[2]), signal.SIGINT)
except ProcessLookupError:
    pass
PY
  timer_pid=$!
fi
echo "开始录制：$output"
echo '等待约 3 秒完成话题发现，再复现导航问题；故障后保留约 5 秒，按 Ctrl+C 保存。日志：recorder.log'
result=0
wait "$recorder_pid" || result=$?
recorder_pid=""
exit "$result"
