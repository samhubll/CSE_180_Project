#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/ros-env.bash"
cd -- "$CSE180_WS"

case "${1:-help}" in
  build)
    if [[ ! -x /opt/cse180-project-venv/bin/python3 ]]; then
      printf 'The project Python environment is missing at /opt/cse180-project-venv.\n' >&2
      exit 1
    fi
    # Invoke colcon as a module so detector's console script uses the ML venv.
    python3 -m colcon build --packages-select search detector \
      --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    ;;
  warehouse)
    if pgrep -f '[r]os2 launch gazeboenvs tb4_warehouse.launch.py' >/dev/null; then
      printf 'The warehouse launch is already running. View it at http://localhost:8080/vnc.html\n'
      exit 0
    fi
    exec ros2 launch gazeboenvs tb4_warehouse.launch.py use_rviz:=True
    ;;
  detector)
    exec ros2 run detector detector --ros-args -p use_sim_time:=true
    ;;
  search)
    exec ros2 run search search --ros-args -p use_sim_time:=true
    ;;
  topics)
    ros2 topic list -t
    printf '\nNav2 actions:\n'
    ros2 action list -t
    ;;
  teleop)
    exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p use_sim_time:=true
    ;;
  *)
    printf 'Usage: bash scripts/project.sh {build|warehouse|detector|search|topics|teleop}\n'
    ;;
esac
