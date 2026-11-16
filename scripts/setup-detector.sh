#!/usr/bin/env bash
# Run inside the course ROS Jazzy container, not Windows PowerShell.
set -eo pipefail
project_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -m venv --system-site-packages /opt/cse180-project-venv
project_python=/opt/cse180-project-venv/bin/python
"$project_python" -m pip install torch==2.7.1 torchvision==0.22.1 \
  --index-url https://download.pytorch.org/whl/cpu
"$project_python" -m pip install -r "$project_dir/requirements-detector.txt"
mkdir -p /opt/cse180-models
cd /opt/cse180-models
"$project_python" -c 'from ultralytics import YOLO; YOLO("yolov8n.pt"); import rclpy, cv_bridge, vision_msgs; print("Detector runtime ready")'
"$project_python" -m pip check
