# Project setup

This repository is inside the existing CSE180 workspace:

```text
/MRTP/CSE180/
  src/CSE_180_Project/
    search/                 C++ search starter
    detector/               Python YOLO person detector
    scripts/                ROS environment and build/run commands
    .vscode/                terminals, tasks and C++ completion
    README.md               original project assignment
```

## Open and build

Use the VS Code window attached to the `mrtp-jazzy` container. The bottom-left
corner should show a container connection. This gives C++ access to the actual
Linux compiler, ROS headers and MRTP library.

Press **Ctrl+Shift+B** to build both project packages. Use **Terminal > New
Terminal** for a Bash shell that automatically sources ROS Jazzy, MRTP, CSE180
and the project Python environment. The shell starts at `/MRTP/CSE180`.

From a project terminal, the equivalent build command is:

```bash
bash "$CSE180_PROJECT_ROOT/scripts/project.sh" build
```

Use that task/script rather than an unsourced Windows PowerShell terminal.
The build uses `python3 -m colcon` so the detector executable points at its
isolated machine-learning environment.

## Run

Choose **Terminal > Run Task** and run these in separate task terminals:

1. **Start warehouse + RViz**
2. **Run detector**
3. **Run search starter**

The exact task labels are listed in `.vscode/tasks.json`. The warehouse task
reuses the running launch if it is already up. View Gazebo and RViz at:

[Open noVNC](http://localhost:8080/vnc.html?autoconnect=true&resize=scale)

Equivalent commands, after the environment is sourced:

```bash
ros2 launch gazeboenvs tb4_warehouse.launch.py use_rviz:=True
ros2 run detector detector --ros-args -p use_sim_time:=true
ros2 run search search --ros-args -p use_sim_time:=true
```

Stop each running task with Ctrl+C. The starter sends no navigation goals.
Use the keyboard teleop task when you intentionally want to move the robot;
press `k` to stop before quitting teleop.

## What is implemented

- `detector` runs the official YOLOv8n pretrained model on CPU, at up to two
  new frames per second. It publishes only person detections, with confidence
  above a configurable threshold, using `vision_msgs/msg/Detection2DArray`.
  Array and detection headers preserve the input image timestamp/frame.
- `search` compiles against MRTP's `Navigator` and creates its action clients,
  subscriptions for detections/depth/calibration, a TF2 buffer/listener, and
  a marker publisher. It prints connection readiness and camera intrinsics.

**This is a development setup, not a completed final-project submission.**
The C++ starter still needs the waypoint mission, timestamp-matched robust
depth sampling, pixel projection, image-time TF2 conversion into map,
duplicate merging, confirmed-person markers, and return-home reporting.
The waypoint coverage, detector range measurements and two-page report
also remain to be completed and validated in the warehouse.

The initial confidence threshold is a starting value, not a measured claim
about detection accuracy or range.

## Useful topics and code

The installed simulation bridges these camera topics:

```text
/rgbd_camera/image         sensor_msgs/msg/Image
/rgbd_camera/depth_image   sensor_msgs/msg/Image
/rgbd_camera/camera_info   sensor_msgs/msg/CameraInfo
/people/detections        vision_msgs/msg/Detection2DArray
/people/markers           visualization_msgs/msg/MarkerArray
```

Inspect the live system:

```bash
ros2 topic list -t
ros2 action list -t
ros2 topic echo /rgbd_camera/camera_info --once
ros2 node info /search
ros2 topic hz /people/detections
```

MRTP's Navigator API is in:

```text
/MRTP/MRTP/src/navigation/include/navigation/navigation.hpp
```

Relevant methods are `SetInitialPose`, `WaitUntilNav2Active`, `GoToPose`,
`Spin`, `IsTaskComplete`, `GetResult`, and `CancelTask`. Study the library's
own spin behavior before mixing Navigator calls with an executor.

For the later localization code, the pinhole model is
`X=(u-cx)*d/fx`, `Y=(v-cy)*d/fy`, `Z=d`. Use calibration for the image/depth
stream actually being processed. An optical frame normally has x right,
y down and z forward. Transform detections at their image timestamp.

## Python environment

The CPU-only environment is `/opt/cse180-project-venv`, separate from the
course environment. The default model is `/opt/cse180-models/yolov8n.pt`.
These are stored in the existing Docker container, not committed to Git.
Exact installed versions and setup checks are saved in `setup-logs/`.
If this container is deleted/recreated, the additional Python environment
and weights must be recreated as well. From a sourced ROS terminal inside
the course container, run:

```bash
bash "$CSE180_PROJECT_ROOT/scripts/setup-detector.sh"
```

The paired CPU Torch/torchvision versions and direct detector dependencies
are pinned in this script and `requirements-detector.txt`.

The original assignment README is preserved. The initial local setup commit
provides a baseline for the team’s later implementation changes.

## Codex in VS Code

The official extension ID is `openai.chatgpt`. It must be available in the
VS Code environment that has the project open, including the attached
container. Open the Codex tab/icon, or run **Codex: Open Codex Sidebar** from
the Command Palette. If that environment asks for a ChatGPT sign-in, complete
the sign-in yourself. Installation and authentication are separate checks.

`AGENTS.md` gives Codex the ROS workspace paths, build command and project
constraints. `ROADMAP.md` defines the remaining milestones and checks.
Open `CODEX_START.md` and paste its prompt into Codex to begin the first
implementation slice: verified sensor inputs, C++ person localization and
confirmed-person markers.
