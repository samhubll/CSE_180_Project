# Project setup

This repository is inside the existing CSE180 workspace:

```text
/MRTP/CSE180/
  src/CSE_180_Project/
    search/                 C++ stationary person localization
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
3. **Run stationary perception**

The exact task labels are listed in `.vscode/tasks.json`. The warehouse task
reuses the running launch if it is already up. View Gazebo and RViz at:

[Open noVNC](http://localhost:8080/vnc.html?autoconnect=true&resize=scale)

Equivalent commands, after the environment is sourced:

```bash
ros2 launch gazeboenvs tb4_warehouse.launch.py use_rviz:=True
ros2 run detector detector --ros-args -p use_sim_time:=true
ros2 run search search --ros-args -p use_sim_time:=true
```

Stop each running task with Ctrl+C. The perception node sends no navigation goals.
Use the keyboard teleop task when you intentionally want to move the robot;
press `k` to stop before quitting teleop.

The warehouse's default RViz configuration does not display the project people
topic. For a focused map and confirmed-people view, open
`search/config/people.rviz` through **File > Open Config** in the running RViz,
or start a separate RViz from a sourced terminal after building:

```bash
ros2 run rviz2 rviz2 -d "$(ros2 pkg prefix search)/share/search/config/people.rviz" \
  --ros-args -p use_sim_time:=true
```

This configuration uses fixed frame `map`, the warehouse `/map`, and a
**MarkerArray** display for `/people/markers` with reliable/transient-local
QoS. Confirmed people appear as green spheres after observations from at least
two distinct source images. Pan and zoom the top-down view to inspect them.
An empty marker display can mean that no visible person has been confirmed;
check the `search` status and rejection logs before drawing conclusions.

## What is implemented

- `detector` runs the official YOLOv8n pretrained model on CPU. Defaults are
  `cpu_threads=1`, requested `inference_size=640`, `confidence_threshold=0.5`
  and a `max_rate_hz=2.0` simulated-time cap. Actual throughput depends on load.
  The one-thread setting follows the measured course-container benchmark;
  `cpu_threads=0` retains the library's automatic thread setting for comparison.
  Source RGB remains 320 x 240; the recorded inference tensor has a different
  size after rectangular preprocessing. Only person detections above the
  threshold are published as `vision_msgs/msg/Detection2DArray`, preserving
  source image timestamps and frames on both arrays and boxes.
- `search` keeps a bounded, timestamp-sorted depth history and matches RGB
  detections to depth within a configurable tolerance. It takes the median
  valid depth in a clipped central box region, validates calibration, projects
  into the optical frame and transforms into `map` at the source image time.
  Unavailable timestamped TF is retried; localization never substitutes latest
  TF. It merges observations within 0.6 m, averages positions, counts distinct
  source images and publishes stable-ID markers only after two observations.
  It reports camera intrinsics, accepted observations, confirmations and
  rejection counts. Simulation time is required.
- MRTP `Navigator` remains linked and configured for simulation time, outside
  the perception executor because its methods spin internally. This slice
  sends no navigation goals.
- [search/include/search/coverage_plan.hpp](search/include/search/coverage_plan.hpp)
  records the geometry-based warehouse waypoint and viewing-heading plan for
  milestone 4. It is planning data; the current search executable does not
  execute the route. The annotated map, assumptions and remaining coverage
  gaps are in [docs/coverage-plan.md](docs/coverage-plan.md).

**This is a development setup, not a completed final-project submission.**
Autonomous waypoint execution, return-home behavior and the final mission
report remain unfinished. Milestone 3 measurements describe specific distance,
angle, occlusion and localization conditions; they do not establish complete
warehouse coverage, whole-world person counts or final mission accuracy.
Implementation policies, checks and limitations are documented in
[docs/perception.md](docs/perception.md). The actual sensor calibration,
registration and timing evidence are in
[docs/sensor-contract.md](docs/sensor-contract.md); recorded runtime outcomes
belong in [docs/perception-validation.md](docs/perception-validation.md),
[docs/detector-range.md](docs/detector-range.md) and [ROADMAP.md](ROADMAP.md).

## Detector and coverage measurements

[docs/detector-range.md](docs/detector-range.md) records the numerical reliability
criterion, thread benchmark, matched-threshold comparisons, actual trial
results and their limits. It distinguishes recognition from usable localization,
which also needs valid ROI depth, source-time TF and acceptable validation error.
The coverage plan uses these measurements together with the warehouse map,
shelf occlusions and planned stationary views. Route execution and return-home
acceptance remain milestone 4 work.

For a measurement run, use one detector and one search process in separate
terminals. Stop an earlier instance before changing profiles. From a sourced
project terminal:

```bash
ros2 run detector detector --ros-args -p use_sim_time:=true \
  -p cpu_threads:=1 -p inference_size:=640 -p max_rate_hz:=2.0 \
  -p confidence_threshold:=0.5 -p log_performance:=true
ros2 run search search --ros-args -p use_sim_time:=true \
  -p min_confidence:=0.5 -p log_measurements:=true
```

`log_performance` and `log_measurements` are optional diagnostics and default
to false. The C++ `range_probe` independently records processed source images,
scores, depth/TF replay outcomes and arrival latency. The detector's own
simulation clock can lag during synchronous inference; use the independent
probe's arrival-age measurements as explained in the range notes. Actual
search acceptance and rejection logs are correlated by source timestamp rather
than inferred solely from the probe's replay.

The validation wrapper supplies truth only at runtime. Set `TRIAL_LABEL` to a
new label and `TARGET_MODEL`, `TARGET_X`, `TARGET_Y`, `TARGET_YAW` and `CONFIDENCE`
from the recorded validation scenario, then run:

```bash
bash scripts/measure-trial.sh "$TRIAL_LABEL" "$TARGET_MODEL" \
  "$TARGET_X" "$TARGET_Y" "$TARGET_YAW" "$CONFIDENCE" 10
ruby scripts/summarize-trials.rb
```

The wrapper checks that live detector and search confidence thresholds match,
records stationary Gazebo poses, and invokes the bounded C++ collector after
warm-up. It refuses to overwrite an existing trial. Raw JSONL, snapshots and
logs are saved under ignored `setup-logs/milestone3/`; concise results and
reproducible commands are tracked in the range notes. Keep any movement inside
Gazebo and record visibility/false-positive annotations from the source images.

`validation_initial_pose` is a separate controlled-experiment helper. It
requires `acknowledge_validation:=true`, simulated time and explicit `x/y/yaw`
parameters before publishing one initial pose. It accepts a supplied validation
pose and is not called by search or used to read Gazebo automatically. Pose-seeded
trials must remain labeled separately from ordinary AMCL trials; see the range
notes for that distinction.

After building, run the project checks from the workspace:

```bash
source scripts/ros-env.bash
cd "$CSE180_WS"
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 python3 -m colcon test \
  --packages-select search detector --event-handlers console_direct+
python3 -m colcon test-result --verbose
```

The C++ checks cover perception geometry/input handling and the stored coverage
plan. Workspace-wide test summaries can also include older course packages;
use the selected package results when reporting this project's verification.

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

The localization code uses the pinhole model
`X=(u-cx)*d/fx`, `Y=(v-cy)*d/fy`, `Z=d`. Use calibration for the image/depth
stream actually being processed. An optical frame normally has x right,
y down and z forward. Recheck RGB/depth registration before remapping camera
topics; matching dimensions alone do not establish registration.

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
`CODEX_START.md` preserves the first-session scope: verified sensor inputs,
C++ person localization and confirmed-person markers. Use the working record
and validation notes to choose the next unfinished milestone.
