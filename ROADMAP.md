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

The camera contract has been rechecked live: registered 320 x 240 RGB `rgb8`
and meter-valued depth `32FC1`, frame `oakd_rgb_camera_optical_frame`, active
camera rate 10 Hz in simulated time. The initial confidence threshold remains
0.5 and processing cap 2 Hz. Milestone 3 selected one CPU thread and explicit
640 inference size: actual trials reached 2 Hz in simulated time. Conditional
range/angle results are recorded; final whole-world search accuracy remains
unmeasured.

`search` now implements the stationary perception slice: bounded timestamp
matching, robust depth projection, image-time TF, distinct-image track
confirmation and stable map markers. A visible default person has been
confirmed live. Waypoint exploration, unknown-person coverage, return-home
behavior and final mission/report results remain unfinished. See
[sensor inspection](docs/sensor-contract.md), [implementation policy](docs/perception.md)
and [actual validation](docs/perception-validation.md).

## Milestones

Dates below are working targets. Mark a milestone complete only after its
acceptance checks have actually passed, and record the evidence.

### 1 Verify the sensor contract

Target: November 16 to 18. Dependency: the working setup above.

- [x] Record live topic names, message types, resolutions, encodings and frames.
- [x] Record `CameraInfo` intrinsics: fx=K[0], fy=K[4], cx=K[2], cy=K[5].
- [x] Verify RGB/depth pixel registration and the calibration applicable to each stream.
- [x] Measure timestamp offsets, image rates and detector latency to size a bounded depth buffer.
- [x] Verify depth units, range limits and invalid/out-of-range pixel values.
- [x] Record robot-to-optical TF and explain optical x right, y down, z forward.
- [x] Record Nav2 action servers, useful MRTP Navigator methods, and both default people's truth positions.

Acceptance: record concise inspection notes with actual values and commands.
Keep truth positions in validation notes, separate from mission decisions.
Do not assume that matching image dimensions prove RGB/depth registration.

### 2 Implement the first working perception slice

Target: November 18 to 23. Dependency: milestone 1.

Implement the following in C++, extending `search`:

- [x] Buffer depth frames and match each detection's source timestamp to depth
  within a documented, measured tolerance. Reject stale or unmatched inputs.
- [x] Clip a central bounding-box region to image bounds, filter invalid depth,
  require enough valid support, and use its median. Document the ROI policy.
- [x] Project pixels into the optical frame using
  `X=(u-cx)*d/fx`, `Y=(v-cy)*d/fy`, `Z=d`, with correct units and calibration.
- [x] Transform a `PointStamped` into `map` using TF2 at the source image time.
  Reject unavailable timestamped transforms; retry or discard rather than
  substituting a latest transform.
- [x] Associate observations within M=0.6 m, maintain a running position estimate,
  count distinct source-image observations, and keep stable track IDs.
- [x] Require at least two observations before reporting a confirmed person.
- [x] Publish confirmed-person `MarkerArray` markers with stable IDs in `map`.

Acceptance: meaningful C++ checks cover projection, nonidentity transforms,
invalid depth, clipped boxes, unmatched timestamps, duplicate observations,
track confirmation and merge boundaries. Live observations produce finite,
confirmed map markers for a visible person while stationary; controlled
turning checks expose image-time versus latest-TF mistakes. Record actual
results before claiming accuracy. This is the first coding priority.

### 3 Measure the detector and plan coverage

Target: November 23 to 27. Dependency: milestones 1 and 2.

- [x] Use teleop to approach a person and record the distance at which detections
  become reliable, including view angle, threshold, resolution and processing rate.
- [x] Choose and justify confidence/rate settings using measured precision,
  false positives and latency on this computer.
- [x] Inspect the map, shelves and reachable floor in RViz. Choose hardcoded
  C++ waypoints and viewing headings using the approximately 70-degree field
  of view, measured range, shelf occlusions and spin coverage.
- [x] Save a map picture showing waypoint order, viewing directions and gaps.
- [ ] Close or explicitly justify the remaining reachable-floor coverage gaps
  and verify full open-area coverage; residual cells remain in the northern
  rack pocket and other near-cutoff strips of the stationary-view model.

Acceptance: range and threshold notes contain measurements; the route covers
each reachable open area from useful viewpoints. Use warehouse geometry and
sensor reach to choose the route, not knowledge of where default people stand.

Measurement and planning deliverables are recorded in
[detector-range.md](docs/detector-range.md) and
[coverage-plan.md](docs/coverage-plan.md). Full coverage acceptance remains
**unproven**: the current 52-stop plan has measured geometric gaps, and map
localization drift caused observed accuracy failures. Neither the geometric
visibility percentage nor read-only planner success proves a completed search.

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

### November 15 session: milestones 1 and 2

Completed the sensor-contract and first-perception acceptance checks in the
running Gazebo warehouse. [Inspection notes](docs/sensor-contract.md) record
active-camera registration/calibration, meter depth and observed `+inf`,
10 Hz simulated-time images, 2.442–3.279 s initial detector lag, optical TF,
Nav2 API/actions and validation-only truth. The measured timing supports the
20 ms depth tolerance, 5 s image-age limit and 8 s/160-frame buffer.

`bash scripts/project.sh build` passed; the selected `search` and `detector`
package checks passed, including 18 C++ tests and source lint/static analysis.
The detector's existing copyright check remains skipped. The workspace-wide
test-result total also includes older course-package results; see the scoped
[validation record](docs/perception-validation.md) for commands and evidence.

After manual Nav2 positioning in Gazebo, the final node confirmed a real
stationary person on its second distinct image. The 40-second stationary
capture and 70-second turning capture each retained one stable marker ID,
with no nonfinite markers or per-box header mismatches. In-window detections
matched depth exactly; startup sources outside new depth/TF history were
rejected rather than localized with latest data. Both controlled spin actions
succeeded. One delayed turning image exhibited 0.772190 rad of image/latest-TF
heading disagreement; the source-time localizer kept the existing track.
Actual RViz inspection showed the map and green person marker with Global
Status Ok. Raw logs/screenshots are ignored under `setup-logs/`.

This completes the first slice, not the final assignment. Whole-world counts,
unknown placements, reliable detector range, coverage, autonomous navigation,
return-home behavior and the final report remain unverified. Next: milestone 3.

### November 15 session: milestone 3 measurements and coverage plan

Recorded [25 trials / 250 distinct processed images](docs/detector-range.md)
with a criterion fixed before testing: at least 8/10 recognized images and
8/10 correctly localized images (within 0.5 m), plus confirmation from two
source images. The original Gazebo/ROS setup was reused; simulated time was
used throughout. Manual Nav2 positioning and short keyboard-teleop approaches
were confined to Gazebo. Both original people were restored after validation.

The frontal 19.961 m case passed 10/10, but side profiles missed at 11.335 and
13.940 m. At 8.741 m, recognition/depth support passed while unseeded map
accuracy failed (0.590 m; 0.675 m after no-motion updates). Those failures
remain in the results. After an explicitly acknowledged, validation-only
robot-pose initialization, three nearby side trials at 8.599–8.741 m passed
10/10 each. Production search still uses image-time TF and no Gazebo truth.
Near-2.058 m front/side/rear images cropped the head/upper body and all missed;
a partly shelf-hidden person at 5.741 m also missed. These observations do
not establish a universal recognition range or a depth-support maximum.

Selected CPU threads 1, requested inference size 640 (observed FP32 tensor
1×3×480×640), confidence 0.5 in both detector/search and simulated-time cap
2 Hz. The 250 trial images arrived at 2 source Hz; independent latency median
was 0.300 simulated seconds. Seven-thread inference was much slower on this
loaded computer. No false person boxes were observed in the inspected static
views, including three ten-image empty controls; this is not an unknown-scene
false-positive-rate guarantee. The measured poses, rates, scores, ROI support,
actual search acceptance/rejections and accuracy are in the tracked
[CSV](docs/data/detector-trials.csv).

The geometry-only [C++ plan](search/include/search/coverage_plan.hpp) has 52
stops and eight 45-degree viewing headings per stop, using a conservative
3.2–8 m annulus, a ±10-degree heading gate, 0.5 s settling and at least 2 s
stationary dwell with two fresh processed-image stamps. The
[annotated map](docs/assets/coverage-plan.png) identifies order, directions,
unknown areas and gaps. Fine-grid visibility is 99.325% of known-free floor
and 99.885% of reachable safe floor. About 8.640 m² remains uncovered by
that model, including 1.1925 m² of reachable safe floor. Three rack-tip
refinements substantially reduced the northern pocket gap; they require
0.05 m actual map-position accuracy before future scan credit. Full coverage
acceptance is still unproven; the new unchecked gap item records this.

Read-only Nav2 checks returned valid paths for all 52 final legs (285.941 m
total, excluding return home), each with zero planned endpoint xy error;
no robot motion was commanded by the checker.
`bash scripts/project.sh build` passes. Fresh selected-package checks pass:
25 C++ behavioral tests plus cppcheck/CMake/style/XML checks (search: 51
reported tests, no failures/skips); detector: 2 checks passed and its existing
copyright check skipped. A cppcheck macro-namespace parse issue was fixed in
the test fixture without suppressions or changes to geometry/test behavior.
Raw results are under ignored `setup-logs/milestone3/`.

Temporary project/teleop nodes are stopped; the original warehouse/Nav2/RViz
launch remains running with the robot stationary at the last test area.
Next is milestone 4 integration, including localization readiness/drift
handling, measured scan completion and return-home states. Route gap review
and final unknown-placement/mission accuracy remain required; no autonomous
search mission or return-home behavior was executed or implemented here.
