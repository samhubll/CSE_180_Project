# Warehouse search roadmap

Goal: explore the warehouse with Nav2, recognize an unknown number of people,
report confirmed positions in `map`, and return to the starting pose.
The assignment in `README.md` is the specification. Submission is due
December 11 at 11:59 pm, with a demonstration during the assigned lab.

## Verified starting point

- [x] Repository is inside `/MRTP/CSE180/src`; the original assignment README is preserved.
- [x] `search` and `detector` build in ROS 2 Jazzy using `scripts/project.sh build`.
- [x] The warehouse, RViz, map, Nav2 actions and RGB-D streams are available.
- [x] The CPU YOLOv8n detector loads its model and publishes real inference results.
- [x] Detection headers matched source RGB timestamps and frame names in the setup check.
- [x] The C++ starter received depth, calibration, detections and a map transform.
- [x] VS Code has ROS terminal/tasks, C++/Python support, and the Codex extension.

The current camera was observed at 320 x 240, RGB `rgb8`, depth `32FC1`,
with frame `oakd_rgb_camera_optical_frame`. These are recorded observations,
not assumptions to leave unchecked if the simulation changes. The initial
confidence threshold is 0.5 and processing limit is 2 Hz. Neither reliable
detection range nor final search accuracy has been measured. The setup view
produced empty detections; finding the default people is still unverified.

`search` currently checks connections only. Localization, confirmed tracks,
waypoint navigation, mission completion and final report results are unfinished.
Codex sign-in inside the container is a separate user authentication step.

## Milestones

Dates below are working targets. Mark a milestone complete only after its
acceptance checks have actually passed, and record the evidence.

### 1 Verify the sensor contract

Target: November 16 to 18. Dependency: the working setup above.

- [ ] Record live topic names, message types, resolutions, encodings and frames.
- [ ] Record `CameraInfo` intrinsics: fx=K[0], fy=K[4], cx=K[2], cy=K[5].
- [ ] Verify RGB/depth pixel registration and the calibration applicable to each stream.
- [ ] Measure timestamp offsets, image rates and detector latency to size a bounded depth buffer.
- [ ] Verify depth units, range limits and invalid/out-of-range pixel values.
- [ ] Record robot-to-optical TF and explain optical x right, y down, z forward.
- [ ] Record Nav2 action servers, useful MRTP Navigator methods, and both default people's truth positions.

Acceptance: record concise inspection notes with actual values and commands.
Keep truth positions in validation notes, separate from mission decisions.
Do not assume that matching image dimensions prove RGB/depth registration.

### 2 Implement the first working perception slice

Target: November 18 to 23. Dependency: milestone 1.

Implement the following in C++, extending `search`:

- [ ] Buffer depth frames and match each detection's source timestamp to depth
  within a documented, measured tolerance. Reject stale or unmatched inputs.
- [ ] Clip a central bounding-box region to image bounds, filter invalid depth,
  require enough valid support, and use its median. Document the ROI policy.
- [ ] Project pixels into the optical frame using
  `X=(u-cx)*d/fx`, `Y=(v-cy)*d/fy`, `Z=d`, with correct units and calibration.
- [ ] Transform a `PointStamped` into `map` using TF2 at the source image time.
  Reject unavailable timestamped transforms; retry or discard rather than
  substituting a latest transform.
- [ ] Associate observations within M=0.6 m, maintain a running position estimate,
  count distinct source-image observations, and keep stable track IDs.
- [ ] Require at least two observations before reporting a confirmed person.
- [ ] Publish confirmed-person `MarkerArray` markers with stable IDs in `map`.

Acceptance: meaningful C++ checks cover projection, nonidentity transforms,
invalid depth, clipped boxes, unmatched timestamps, duplicate observations,
track confirmation and merge boundaries. Live observations produce finite,
confirmed map markers for a visible person while stationary; controlled
turning checks expose image-time versus latest-TF mistakes. Record actual
results before claiming accuracy. This is the first coding priority.

### 3 Measure the detector and plan coverage

Target: November 23 to 27. Dependency: milestones 1 and 2.

- [ ] Use teleop to approach a person and record the distance at which detections
  become reliable, including view angle, threshold, resolution and processing rate.
- [ ] Choose and justify confidence/rate settings using measured precision,
  false positives and latency on this computer.
- [ ] Inspect the map, shelves and reachable floor in RViz. Choose hardcoded
  C++ waypoints and viewing headings using the approximately 70-degree field
  of view, measured range, shelf occlusions and spin coverage.
- [ ] Save a map picture showing waypoint order, viewing directions and gaps.

Acceptance: range and threshold notes contain measurements; the route covers
each reachable open area from useful viewpoints. Use warehouse geometry and
sensor reach to choose the route, not knowledge of where default people stand.

### 4 Integrate the Nav2 mission

Target: November 27 to December 2. Dependency: milestones 2 and 3.

- [ ] Implement explicit mission states for waiting/localization, visiting a
  waypoint, scanning/spinning, returning home and completing the report.
- [ ] Initialize localization appropriately, wait for Nav2, and record the
  starting pose in `map`, including orientation.
- [ ] Use MRTP Navigator `GoToPose`, `Spin`, `IsTaskComplete` and `GetResult`.
  Handle rejected goals, failures, timeouts and cancellation visibly.
- [ ] Keep perception callbacks running during navigation. MRTP Navigator spins
  its node internally: choose executor ownership deliberately and avoid adding
  it to an executor that conflicts with its internal spin calls.
- [ ] Run search, Navigator and detector on simulated time.
- [ ] Finish all waypoints, return to the recorded pose, then print only
  confirmed people with map coordinates and observation counts.

Acceptance: a complete default-world mission reaches its phases in order,
perception remains live during movement, navigation failures are handled,
and the robot returns within the configured Nav2 goal tolerances. Record the
actual home error and task results. Use Nav2 for motion and obstacle avoidance.

### 5 Validate accuracy and unknown placements

Target: December 2 to 7. Dependency: milestone 4.

- [ ] Run the default world and find both people, with each position error
  within about 0.5 m of Gazebo truth. Record estimates, truth and errors.
- [ ] Add people in new places, including partial shelf occlusion and an area
  with weak coverage; measure misses and false positives.
- [ ] Test repeated views, two people closer than 0.6 m, stale/missing input,
  unavailable TF and an unreachable waypoint.
- [ ] Check that repeated boxes from one image do not inflate observation counts.
- [ ] Observe at least one failure and explain its actual cause. For example,
  test whether the merge rule combines close people; report that only if observed.
- [ ] Fix concrete failures and repeat the relevant end-to-end cases.

Acceptance: keep reproducible scenario descriptions and measured results.
The number and positions of people must come from observations, not fixed
expectations that only work in the default world.

### 6 Prepare the submission and demonstration

Target: December 7 to 10; submit by December 11 at 11:59 pm.

- [ ] Produce the report, at most two pages: inspection answers, waypoint
  rationale/map, detector/threshold/measured range, default-world error table,
  and one observed failure with its cause.
- [ ] Build and run from the documented setup and check dependency/model paths.
- [ ] Package the entire CSE180 workspace as a ZIP and verify the archive opens.
- [ ] Practice explaining optical axes, image timestamps and TF2, robust depth,
  the 0.6 m merge rule, and the limitations when two people stand close together.
- [ ] Prepare a repeatable lab demonstration and record which validations passed.

## Working record

For each milestone, add the commands, test conditions and observed outcomes
to tracked inspection/validation notes. Keep raw large logs, model weights
and generated build files out of Git. Report untested behavior as untested.
The initial setup commit is a baseline for later focused changes.
