#!/usr/bin/env bash
set -euo pipefail

TERM_WAIT_SECONDS="${TERM_WAIT_SECONDS:-3}"
GAZEBO_FORCE_ROUNDS="${GAZEBO_FORCE_ROUNDS:-3}"
GAZEBO_FORCE_WAIT_SECONDS="${GAZEBO_FORCE_WAIT_SECONDS:-1}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<'USAGE'
Usage: ./kill_ros.sh [--dry-run]

Kill ROS 2, workspace node, RViz, and Gazebo processes related to this machine.

Options:
  --dry-run   Show matching processes without killing them.
USAGE
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ $# -gt 1 || ("${1:-}" != "" && "${1:-}" != "--dry-run") ]]; then
  usage >&2
  exit 2
fi

find_target_pids() {
  ps -eo pid=,comm=,args= | awk -v self="$$" -v parent="$PPID" -v root="$ROOT_DIR" '
    function is_gazebo_process(comm, cmd) {
      return comm ~ /^(gzserver|gzclient|gazebo)(-[0-9.]+)?$/ ||
        cmd ~ /(^|[ /])(gzserver|gzclient|gazebo)(-[0-9.]+)?([[:space:]]|$)/ ||
        cmd ~ /(^|[ /])ign[[:space:]]+gazebo([[:space:]]|$)/ ||
        cmd ~ /(^|[ /])gz[[:space:]]+sim([[:space:]]|$)/
    }
    function is_match(comm, cmd) {
      return cmd ~ /\/opt\/ros\/[^ ]+\/bin\/(ros2|_ros2cli_daemon)( |$)/ ||
        cmd ~ /\/opt\/ros\/[^ ]+\/lib\// ||
        cmd ~ root "/install/[^ ]+/lib/" ||
        cmd ~ /(^|[ /])rviz2([[:space:]]|$)/ ||
        is_gazebo_process(comm, cmd)
    }
    {
      pid = $1
      comm = $2
      sub(/^[[:space:]]*[0-9]+[[:space:]]+[^[:space:]]+[[:space:]]*/, "", $0)
      cmd = $0
      if (pid != self && pid != parent && is_match(comm, cmd)) {
        print pid
      }
    }
  ' | sort -n -u
}

find_gazebo_pids() {
  ps -eo pid=,comm=,args= | awk -v self="$$" -v parent="$PPID" '
    function is_gazebo_process(comm, cmd) {
      return comm ~ /^(gzserver|gzclient|gazebo)(-[0-9.]+)?$/ ||
        cmd ~ /(^|[ /])(gzserver|gzclient|gazebo)(-[0-9.]+)?([[:space:]]|$)/ ||
        cmd ~ /(^|[ /])ign[[:space:]]+gazebo([[:space:]]|$)/ ||
        cmd ~ /(^|[ /])gz[[:space:]]+sim([[:space:]]|$)/
    }
    {
      pid = $1
      comm = $2
      sub(/^[[:space:]]*[0-9]+[[:space:]]+[^[:space:]]+[[:space:]]*/, "", $0)
      cmd = $0
      if (pid != self && pid != parent && is_gazebo_process(comm, cmd)) {
        print pid
      }
    }
  ' | sort -n -u
}

show_processes() {
  local pids=("$@")
  if [[ ${#pids[@]} -eq 0 ]]; then
    echo "No ROS/Gazebo processes matched."
    return
  fi

  echo "Matched processes:"
  ps -o pid,ppid,stat,cmd -p "$(IFS=,; echo "${pids[*]}")"
}

force_kill_gazebo_leftovers() {
  local round
  local gazebo_pids=()

  for ((round = 1; round <= GAZEBO_FORCE_ROUNDS; round++)); do
    mapfile -t gazebo_pids < <(find_gazebo_pids)
    if [[ ${#gazebo_pids[@]} -eq 0 ]]; then
      return
    fi

    echo "Force killing leftover Gazebo/gzserver processes..."
    show_processes "${gazebo_pids[@]}"
    kill -KILL "${gazebo_pids[@]}" 2>/dev/null || true
    sleep "$GAZEBO_FORCE_WAIT_SECONDS"
  done

  mapfile -t gazebo_pids < <(find_gazebo_pids)
  if [[ ${#gazebo_pids[@]} -gt 0 ]]; then
    echo "Gazebo processes are still visible after SIGKILL; they may be zombies waiting for parent cleanup:"
    show_processes "${gazebo_pids[@]}"
  fi
}

mapfile -t pids < <(find_target_pids)
show_processes "${pids[@]}"

if [[ "${1:-}" == "--dry-run" ]]; then
  exit 0
fi

if [[ ${#pids[@]} -gt 0 ]]; then
  echo "Sending SIGTERM..."
  kill -TERM "${pids[@]}" 2>/dev/null || true
  sleep "$TERM_WAIT_SECONDS"

  mapfile -t remaining < <(
    for pid in "${pids[@]}"; do
      if kill -0 "$pid" 2>/dev/null; then
        echo "$pid"
      fi
    done
  )

  if [[ ${#remaining[@]} -gt 0 ]]; then
    echo "Sending SIGKILL to remaining processes..."
    kill -KILL "${remaining[@]}" 2>/dev/null || true
  fi
fi

force_kill_gazebo_leftovers

echo "Done."
