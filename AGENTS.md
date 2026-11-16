# Project context

This Git repository is a source folder inside the CSE180 ROS 2 Jazzy workspace,
not a standalone colcon workspace. In the course Docker container:

- Repository: `/MRTP/CSE180/src/CSE_180_Project`
- Workspace: `/MRTP/CSE180`
- MRTP overlay: `/MRTP/MRTP/install/setup.bash`
- Python ML environment: `/opt/cse180-project-venv`

Read `SETUP.md` before building. Use:

```bash
bash scripts/project.sh build
```

The script sources ROS and the overlays, enters the correct workspace, and
uses `python3 -m colcon` so the detector's entry point uses the ML environment.
When running from a Windows host shell, execute Linux commands through
`docker exec mrtp-jazzy` or use the provided VS Code tasks. Do not try to run
ROS commands directly in Windows PowerShell.

`ROADMAP.md` records implementation milestones and acceptance checks.
`CODEX_START.md` contains the initial task prompt. Check off roadmap items
only after observing the stated behavior and record unverified work honestly.

# Assignment constraints

- The `search` node must be C++. Python is allowed only for `detector`.
- Run simulation-facing nodes with `use_sim_time:=true`.
- Preserve source-image headers in detections. Use the image timestamp for
  TF2 person localization, not a latest transform while the robot is turning.
- Read the original assignment in `README.md`; it is the project specification.
- The current `search` executable is a readiness starter, not an implemented
  autonomous mission. Describe unfinished behavior accurately.
- Do not hardcode known person locations into the search algorithm or invent
  detector-range measurements, localization accuracy, or report results.

# Useful entry points

- `search/src/search.cpp`: C++ subscriptions, TF2 and MRTP Navigator starter.
- `detector/detector/detector_node.py`: CPU YOLO person detections.
- `scripts/project.sh`: build, warehouse, detector, search, topics, teleop.
- `setup-logs/`: local runtime checks; ignored by Git.

MRTP Navigator's header is at
`/MRTP/MRTP/src/navigation/include/navigation/navigation.hpp`. Its methods
spin internally, so consider executor ownership before integrating a mission.
