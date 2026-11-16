# Perception slice validation

Run in the existing Gazebo warehouse on November 15, 2026 (Los Angeles;
raw logs use November 16 UTC). Changes are local. This validates the first
perception slice; it does not establish autonomous warehouse coverage, measured
detector range or final-world accuracy. Truth is isolated in the validation
section of [sensor-contract.md](sensor-contract.md).

## Build and C++ checks

Commands from the repository:

```bash
bash scripts/project.sh build
/MRTP/CSE180/build/search/test_perception
```

The first full implementation build passed for `search` and `detector`, taking
5 minutes 15 seconds on the bind-mounted workspace. The standalone C++ binary
passed **18 tests from five suites**. Tests cover median/outliers/invalid
support, padded rows and both byte orders, meter/millimeter decoding, clipping
and malformed boxes/images, optical projection and calibration rejection,
nonidentity transforms, actual historical/interpolated TF versus latest TF,
unavailable historical TF, timestamp tolerance/buffer bounds, distinct-image
confirmation, duplicate boxes, merge boundary, stable IDs and invalid limits.
Raw results: `setup-logs/milestone-build.log` and
`setup-logs/perception-unit-tests.log`.

An early build attempt started before the delegated test source existed and
failed CMake generation; rerunning after the source was written passed. This
was a build-order issue, not a reported passing run.

## Live setup

The warehouse and its Nav2/RViz stack were already running. Started the existing
detector with `bash scripts/project.sh detector`. Started the newly built search
binary with `use_sim_time:=true`; initial view showed live calibration, an
approximately 80-frame retained history, empty detection arrays and no confirmed
tracks. The sensor-contract capture measured detector arrival ages of
2.442–3.279 simulated seconds, motivating the documented matching/history
policy. Logs are in `setup-logs/milestone-detector.log`,
`setup-logs/milestone-search-initial.log` and
`setup-logs/sensor-contract-live.txt`.

For a visible-person check, the **validation-only** viewing goal is map
`(x=1,y=2,yaw=-pi/2)`, chosen north of the default standing person with open
floor in the map. This pose is supplied at runtime to the separate manual
helper; it is absent from the search algorithm. A map-occupancy inspection
found free cells in a 0.42 m surrounding square. Motion uses Nav2 in Gazebo.

The first `ros2 action send_goal` attempt was parser-rejected because that CLI
does not accept `--ros-args`; it sent no goal. The C++ helper allows explicit
simulation-time configuration and respects MRTP's internal executor ownership.
Its source was compiled temporarily under `/tmp` to avoid waiting for repeated
bind-mounted builds, with the matching command preserved in
`setup-logs/build-validation-motion-direct.sh`.

```bash
source scripts/ros-env.bash
timeout --kill-after=10s --signal=INT 255s \
  /tmp/cse180-validation-motion/validation_motion --ros-args \
  -p use_sim_time:=true -p mode:=navigate \
  -p goal_x:=1.0 -p goal_y:=2.0 -p goal_yaw:=-1.57079632679 \
  -p timeout_sec:=240.0
```

The installed equivalent after building is `ros2 run search validation_motion`
with the same parameters. The helper is manual test scaffolding, not a search
mission. Concrete stationary and controlled-turn results follow below once
observed.

## Final stationary run

Rebuilt the final node, restarted it at the stationary viewing pose, and ran:

```bash
ros2 run search search_live_probe --ros-args -p use_sim_time:=true \
  -p duration_sec:=40.0
```

The final node's first accepted observation at source **4110.600 s** created
ID 0 with one image and `confirmed=no`. At **4112.901 s**, a second distinct
image confirmed that same ID. Each sampled observation had zero RGB/depth
source-time difference, **1399/1428** valid central-ROI samples, and a finite
map point. The unrounded stationary mean from the marker was
`(1.076977, -0.980161, 0.878577)` meters. Relative to the default standing
person's Gazebo model-origin xy in the validation truth table, this is about
**0.0795 m horizontal error for this one stationary viewpoint**. Height is a
visible surface estimate, not the model's ground origin; this is not a
whole-world accuracy result or a detector range measurement.

The 40.010 s read-only capture received 222 RGB frames, 221 depth frames and
nine arrays, all containing one real YOLO person box. Eight source headers
matched retained RGB frames; one source preceded the probe's capture history.
There were **zero per-box header mismatches**, one unique marker ID and **zero
nonfinite markers**. All eight in-window detections had exact matching depth
stamps and successful image-time TF comparisons. The first out-of-window
source was also older than the probe's new TF history, so its read-only TF
lookup failed as expected. Stationary image-time versus latest-TF comparisons
were zero within printed precision.

The final search itself discarded one startup image after its retry age limit:
that source predated the new node's depth/TF history. It then processed later
images normally. This is observed rejection, not fabricated successful
localization of an unavailable historical image. Raw logs:
`setup-logs/milestone-search-final.log` and
`setup-logs/stationary-probe-final.txt`.

## Controlled turning

With the detector and final search still running, started a 70-wall-second
probe and issued two explicit Nav2 `Spin` actions in sequence:

```bash
ros2 run search search_live_probe --ros-args -p use_sim_time:=true \
  -p duration_sec:=70.0
# In another terminal, first +0.6, then -0.6 radians:
timeout --kill-after=5s --signal=INT 45s ros2 run search validation_motion \
  --ros-args -p use_sim_time:=true -p mode:=spin \
  -p spin_angle:=0.6 -p timeout_sec:=35.0
timeout --kill-after=5s --signal=INT 45s ros2 run search validation_motion \
  --ros-args -p use_sim_time:=true -p mode:=spin \
  -p spin_angle:=-0.6 -p timeout_sec:=35.0
```

Both returned MRTP result **4 = SUCCEEDED**, `complete=yes`, exit 0. Requested
angles are action goals, not claimed measured yaw changes. At detection source
**4150.302 s**, the probe measured image-time/latest optical-forward heading
difference **-0.772190 rad**, with a **2.211197 m** difference for its explicitly
hypothetical 3 m center ray. This ray is a demonstration of timestamp error,
not a measured person-position error.

For that same real detection, search matched depth at **0.000 ms** offset,
used **1707/1728** valid samples and source-time TF, and produced
`map=(1.150,-0.974,0.870)` with **ID 0, 17 distinct images, confirmed=yes**.
The next sampled observation at **4153.701 s** was
`map=(1.001,-0.978,0.871)`, still ID 0 with 18 images. Thus substantial
latest/image TF disagreement was actually exposed while the source-time
localizer retained the existing person track. The post-turn viewpoint differs
slightly; no exact-angle or return-home claim is made.

Logs: `setup-logs/turning-probe.txt`, `validation-spin-positive.log`,
`validation-spin-negative.log`, and the corresponding source stamps in
`milestone-search-final.log`.

The completed 70.003 s turning capture received 379 RGB and 379 depth frames
and 14 nonempty arrays. Thirteen source headers matched retained RGB; the
first source was outside capture history. There were zero per-box header
mismatches, 13 successful image-time TF comparisons, one startup historical-TF
failure, one stable marker ID and zero nonfinite markers.

## RViz and package verification

Started RViz on simulated time with `search/config/people.rviz`. Actual X11
inspection captured a visible warehouse map and a green confirmed-person
sphere, **Global Status: Ok**, and both map/people displays enabled. The
screenshot is `setup-logs/perception-rviz-screen.png`, with observations in
`perception-rviz-inspection.txt`. Numeric marker identity comes from messages
and logs; it is not legible on the sphere itself. RViz logged a GLSL sampler
link error while creating the map, but the subsequent screenshot verifies
that both the map and marker rendered. It did not block this acceptance check.

The final implementation build finished both packages in 4 minutes 42 seconds;
a subsequent incremental build finished both in 24.3 seconds. Package checks:

```bash
source scripts/ros-env.bash
cd "$CSE180_WS"
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 python3 -m colcon test \
  --packages-select search detector --event-handlers console_direct+
python3 -m colcon test-result --verbose
```

The selected packages passed: all **18 C++ tests**, cppcheck, CMake lint,
uncrustify, XML validation and the detector's flake8/docstring checks. The
existing detector copyright test remains skipped. The workspace-wide
`test-result` summary says 72 tests, zero errors/failures and five skips; it
also includes older results for other course packages and is not a count of
newly executed project tests. Logs are `setup-logs/milestone-build-final.log`,
`milestone-build-verify.log`, `milestone-package-tests.log` and
`milestone-test-results.log`.

The optional probe RGB-snapshot feature was added while an earlier compiler
was reading that file. Its source timestamp preceded the finished object, so
CMake initially considered the older binary up-to-date. Forced that helper's
source timestamp and rebuilt it before testing the snapshot feature; this does
not change the perception implementation. That final helper build passed in
1 minute 46 seconds. A 12-second capture saved the real 320 x 240 RGB frame at
source **4241.301 s** (`snapshot-probe.txt`, `visible-person.ppm`). Lossless PNG
conversion and visual inspection of `visible-person.png` show the standing
person in the warehouse camera view. This snapshot is explicitly not matched
to a detector output. The short new probe lacked historical TF for its first
two detections; the continuously running search retained its history and kept
the existing track. Larger raw artifacts remain ignored
under `setup-logs/`; concise evidence and reproducible policies are tracked here.

## Remaining work

Milestone 3 is next: measure reliable detector range and tune rate/threshold
from observations, then plan justified coverage using warehouse geometry and
sensor reach. This session has not run an autonomous mission, detected both
default people through a complete search, validated unknown placements, tested
close people/occlusion live, or returned to the mission's starting pose. Those
acceptance checks remain unchecked in ROADMAP.md. The simulation is left at
the viewing area after manual Nav2 tests, not at a claimed return-home pose.
After evidence collection, stopped this session's detector, search and focused
RViz processes. The original warehouse/Nav2/RViz launch remains running. The
final live Gazebo robot pose was `(1.082870,2.083020)` with yaw `-1.880700` rad,
recorded in `setup-logs/validation-final-pose.txt`. No navigation goal remains
active, and no remote Git changes were published.
