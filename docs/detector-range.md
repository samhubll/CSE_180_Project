# Detector range and settings validation

Milestone 3 validation in the course Gazebo warehouse. This protocol was written
before the new range trials. Actual results must be inserted below from the
recorded distinct source images; an intended pose or configured rate is not a
measurement. Person truth belongs only to this validation record and the
sensor-contract validation section, never the coverage route.

## Numerical criterion fixed before testing

Each stationary trial has a warm-up period of at least **10 wall seconds and
7 simulated seconds** for inference and TF/depth history, followed by **10
distinct processed source images**. A source timestamp counts once, including
empty detection arrays.
Use a wall timeout, report incomplete trials as incomplete, and do not replace
missed processed images by raw camera frames. Moving/transitional images do
not belong to a stationary trial.

- **Recognized image:** at least one retained `person` box corresponds to the
  designated visible test person. Associate against the supplied validation
  pose projected into the image and inspect the corresponding source-image
  snapshot; ambiguous boxes remain ambiguous rather than labeled correct.
- **Recognition reliability at a condition:** recognized in at least **8/10**
  processed images. This empirical 80% threshold describes this small sample,
  not a confidence bound or a guarantee for new models/scenes.
- **Usable localization reliability at a condition:** at least **8/10**
  processed images both recognize that person and yield a successful map
  localization using the unchanged central-ROI support rules, matched depth
  and image-time TF, with **horizontal truth error at most 0.5 m**; additionally
  at least two distinct image observations must confirm a stable track in the
  actual search node. Record localization
  rejections separately, including insufficient ROI depth, missing depth/TF,
  stale data and headers.
- **Boundary evidence:** repeat each candidate reliable range boundary in
  **three trials of 10 distinct processed images**. Report all three outcomes;
  call it the tested reliable boundary only if every repeat meets 8/10.
  If samples stop before a failure, report a tested lower bound, not a maximum.
- **False positives:** record every unmatched person box. Compare against all
  visible real people, not just the designated target, and inspect source
  snapshots before calling an unmatched box false. Report counts per processed
  image and total confidence distribution. A confirmed false track is a
  separate, more consequential event. Zero observed false positives is scoped
  to these views and sample sizes.

Stationary Gazebo renders can repeat identical pixel content despite different
source timestamps. Distinct images prevent duplicate-message inflation, but
do not make these Bernoulli trials independent. Repeated viewpoints, view-angle
changes and the stated sample counts are necessary context for every claim.

## Trial plan and quantities

Start with several unobstructed distances spanning approximately 3, 5, 7, 9 and
12 m where map geometry and Nav2 allow them. Measure actual camera/robot-to-target
range and bearings from source-time TF plus the known validation target pose;
do not label requested navigation distances as actual distances. Define the
reported range explicitly (horizontal camera-to-model-origin range, with the
3D range also retained if available).

At representative useful distances vary optical bearing and person aspect:
centered/front, about 25 degrees off center, side views and a near-FOV-edge
case. A 50- or 90-degree optical bearing is outside the approximately 71.6-degree
horizontal FOV and is a negative visibility control, not an expected detection.
Distinguish optical bearing from person-relative front/side/back aspect angle.
Include at least one observed partial shelf occlusion and a known-empty view.
Record visual occlusion qualitatively from source snapshots; do not invent an
occlusion percentage without a measured mask.

Approach a candidate range boundary with finer distance steps, then use the
three repeated trials above. Compare confidence thresholds **0.35, 0.50 and
0.65** at challenging and known-empty conditions; always set detector
`confidence_threshold` and search `min_confidence` to the same value. Record
recognition/localization differences and any false positives before choosing
final settings. Threshold comparison may be performed on the same saved
per-image hypotheses only if the detector retained scores down to the lowest
threshold and the search acceptance for every threshold is evaluated with
matching settings; do not infer suppressed candidates from higher-threshold
outputs.

For every trial record:

- Actual source-time distance, optical bearing, person aspect and occlusion;
  robot motion status and source-image snapshots.
- Distinct processed-image count, recognized images, boxes, scores, unmatched
  boxes/false positives and any ambiguous labels.
- Actual search localizations/confirmations and rejection reasons, correlated
  by source timestamp. A helper's recomputation alone does not prove that the
  running search node accepted an observation.
- Model/weights, package versions, RGB input resolution, actual inference
  tensor size, device/precision, thread settings, confidence thresholds and
  configured rate cap.
- Measured output rate in both wall and simulation time; source-to-publish or
  source-to-arrival simulated latency; wall inference/preprocess/postprocess
  timing, including a separate first-frame warm-up if observed.

Keep raw JSONL, snapshots and logs under ignored `setup-logs/milestone3/`.
Track concise result tables and commands here. Never use a detector range as
usable localization range until the depth-support criterion also passes.

## Initial setup verification

The original warehouse/Nav2/RViz launch was already running at preflight; no
project detector or search process was running. The camera remains the active
single registered RGB-D source documented in [sensor-contract.md](sensor-contract.md).
The source configuration is 320 x 240 at 10 Hz simulated time; live metadata
and rates must be rechecked for this measurement session.

Installed detector source/library inspection found YOLOv8n weights at
`/opt/cse180-models/yolov8n.pt`, Ultralytics **8.3.228**, default requested
`imgsz=640`, prediction `batch=1`, rectangular preprocessing and FP32 CPU.
Actual tensor dimensions must be logged, not equated with the 320 x 240 source.
Ultralytics CPU setup explicitly resets PyTorch to
`NUM_THREADS=min(8,max(1,os.cpu_count()-1))`; this container exposes eight CPUs.
Consequently OMP/MKL environment settings alone may not control actual inference
threads. Any thread comparison must report the actual PyTorch thread count
after predictor setup. The current machine has no cgroup CPU quota, but Gazebo,
Nav2 and RViz already consume several cores. These are inspected configuration
facts, not measured speedup claims.

## Measurement method and reproducible commands

The detector now has explicit `inference_size` and `cpu_threads` parameters.
`cpu_threads=0` leaves Ultralytics' selected value unchanged; a positive value
is applied after library setup, which would otherwise overwrite it. Optional
`log_performance=true` writes a `PERFORMANCE` JSON record for every processed
source image, including empty outputs. An actual backend pre-forward hook
records tensor NCHW and dtype, rather than assuming the requested square size
equals the input tensor. These diagnostics preserve the image and detection
headers and do not add truth information to detection or search.

For the CPU comparison, the robot and person stayed in the original stationary
view and the detector was restarted for each profile. The direct source command
used for measurement avoids relying on an older installed entry point:

```bash
source scripts/ros-env.bash
/opt/cse180-project-venv/bin/python3 detector/detector/detector_node.py \
  --ros-args -p use_sim_time:=true -p confidence_threshold:=0.5 \
  -p max_rate_hz:=2.0 -p inference_size:=640 \
  -p cpu_threads:=0 -p log_performance:=true
# Repeat with cpu_threads:=2, then cpu_threads:=1; stop the prior process.
```

The unchanged model is YOLOv8n pretrained weights, SHA-256
`f59b3d833e2ff32e194b5bb8e08d211dc7c5bdf144b90d2c8412c47ccfc83b36`.
Installed versions are Ultralytics 8.3.228, Torch 2.7.1+cpu, torchvision
0.22.1+cpu and OpenCV 4.11.0.86. Source RGB is **320 x 240**; every sampled
profile actually inferred on **NCHW [1,3,480,640], torch.float32** with requested
size 640, one image per batch and rectangular preprocessing.

Trial collection and offline correlation use:

```bash
bash scripts/measure-trial.sh LABEL 'Person 1 - Standing' X Y YAW CONFIDENCE 10
ruby scripts/summarize-trials.rb
```

`X,Y,YAW` are validation-only person pose arguments, never mission inputs.
The wrapper records both live confidence parameters and refuses to collect
unless detector and search thresholds match the requested comparison. It also
records Gazebo robot/target poses before the stationary sample and robot pose
afterward. The independent C++ `range_probe` warms up for both clock limits,
retains image/depth history, and records every distinct processed source image
as JSONL with its box scores, depth support and timestamped TF replay outcome.
Matched source-image snapshots support visual occlusion and false-positive
annotation. Raw logs and snapshots live under `setup-logs/milestone3/trials/`.

Recognition labeling is kept independent of AMCL error. The offline Ruby
analysis composes the measured stationary Gazebo robot pose with the camera's
robot-relative mounting transform. An independent live query verified it:

```bash
gz model -m turtlebot4 -l oakd_rgb_camera_frame
```

It reports camera translation **(-0.059600,0,0.243530) m**, zero camera-link
rotation and zero sensor-relative pose. This matches the inspected TF and
active URDF. The robot's measured roll, pitch and yaw rotate this translation
into world; the inverse rotation and optical-axis permutation project a
validation point **1 m above the person's model origin** through the live
`CameraInfo` K (`fx=fy=221.76500408`, `cx=160`, `cy=120`). Projection uses that
streamed calibration; the generic Gazebo inspector also prints a different
lens-intrinsic field. Raw mounting verification is saved in
`gazebo-camera-link-verification.txt`.

Reported **actual range** is horizontal distance from that Gazebo-derived
camera origin to the target's model origin. Optical bearing is angle from
camera forward to the target; person aspect is angle from the person's forward
heading toward the camera. Both are signed wrapped angles. The 1 m point is
used to associate boxes with the visible person, not to set the person's
localized height or alter production localization. Camera/target poses and
snapshots remain validation information; the route planner uses warehouse
geometry independently.

The C++ probe's localization is explicitly an **independent replay of the
production perception core**, not proof that the running search accepted a
box. The Ruby analysis separately correlates actual `search` observations and
rejection logs by exact source nanoseconds. It reports accepted images, stable
IDs/confirmation counts, and map xy error against validation truth. An accepted
but inaccurate localization is recorded as such: search cannot reject using
truth it does not know. Image acceptance by itself does not imply the numerical
usable-localization criterion passed. AMCL/map pose error and surface-depth
bias can both contribute to the resulting map error.

### CPU-thread benchmark

For each profile below, discard the first prediction (which includes model
setup/warm-up), then use the **next ten performance records**. Values are
minimum / median / maximum across those ten. Rates use nine intervals:
`9/(last_log_wall-first_log_wall)` and
`9e9/(last_source_ns-first_source_ns)`. These are measured output/source rates,
not the configured two-Hz timer cap.

| CPU setting (actual threads) | Prediction wall ms, min / median / max | Model inference ms, min / median / max | Wall output Hz | Source simulation Hz |
| --- | --- | --- | --- | --- |
| Library default (7) | 3558.95 / 4223.40 / 5469.94 | 3496.87 / 4182.28 / 5412.05 | 0.223 | 0.398 |
| Explicit 2 | 204.19 / 284.29 / 385.27 | 199.21 / 275.16 / 361.37 | 1.650 | 1.914 |
| Explicit 1 | 211.68 / 258.62 / 329.60 | 208.21 / 252.22 / 323.69 | 1.480 | 1.836 |

First prediction wall times were respectively **7207.69, 3552.15 and 1576.20
ms**, excluded from those summaries. Median preprocess/postprocess times were
**27.68/25.72 ms** for default seven threads, **5.62/3.22 ms** for two and
**1.68/1.36 ms** for one. Gazebo, Nav2, RViz and background compilation were
not controlled or pinned across these sequential profiles. The much slower
seven-thread result is observed on this loaded computer; this is not a general
claim that more threads are slower. The overlapping one/two-thread timings and
different simulation rates do **not** establish a definitive ranking between
one and two. Both made the later two-Hz simulated-time sampling practical;
one thread was used for the range trials to limit contention.

Raw logs: `detector-threads-auto.log`, `detector-threads-2.log` and
`detector-threads-1.log`. Computed values, record boundaries and all timing
components are in `thread-profile-summary.json` under `setup-logs/milestone3/`.

### Latency clock distinction

Detector-local simulated time can lag while synchronous YOLO inference blocks
its single ROS executor from receiving `/clock`. The original benchmark field
`publish_age_sim_sec` was therefore misleading. For source **5414.400 s**, it
reported **0.339 s**, while the independently running search observed
**2.757 s** source-to-callback age for that same image. The diagnostic field
was renamed **`node_clock_age_sim_sec`** and marked
`node_clock_may_lag_during_inference=true`; it is not an authoritative latency
measurement. Wall prediction duration still comes from a monotonic clock.

Trial latency tables use the independent C++ collector's source-to-arrival age
on its own simulation clock. The early one-thread front-view trials recorded
median independent arrival ages of approximately **0.30–0.31 simulated
seconds**, rather than extrapolating from the detector's blocked clock. These
are separate measurements from the benchmark's wall inference milliseconds.
The detailed trial table below must retain each condition's measured ranges.

## Results and final settings

Collected **25 complete stationary trials, 10 distinct processed images each
(250 images)** after both warm-up limits. The original distance grid was
adapted to reachable warehouse views and observed failures; the measured
distances below, rather than the initially suggested grid, are the executed
conditions. The complete per-trial values are tracked in
[data/detector-trials.csv](data/detector-trials.csv). Raw JSONL, matching
parameters, Gazebo poses, first/last source-image snapshots and visual
annotations remain under `setup-logs/milestone3/trials/`.

**R/A/U** below means recognized images / images actually accepted by search /
accepted images satisfying the **0.5 m horizontal validation-error** limit.
Each ordinary row has N=10. Grouped rows give the result **for each** of their
ten-image trials. Scores are the returned person confidences; they were
constant within these stationary conditions. ROI is valid central-depth
pixels / central-ROI pixels. Error is the actual search point's horizontal
distance to the validation model origin, rounded to millimeters from logged
positions; it is not a claim of millimeter measurement precision.

**† means pose-seeded validation:** a known Gazebo robot pose was explicitly
published to AMCL before that group, followed by ordinary navigation and
localization updates. These rows test the stated controlled localization
condition; they do not establish autonomous AMCL accuracy. No row uses a
Gazebo transform inside production search.

| Trial/view | Actual camera range m | Bearing / aspect, degrees | Confidence threshold; N | R/A/U | Score | Valid ROI | Map error m | Outcome/cause |
| --- | ---: | ---: | --- | --- | ---: | --- | ---: | --- |
| `original_view`, front | 3.142 | +15.91 / -1.80 | 0.50; 10 | 10/10/10 | .8872 | 1403/1428 | .022 | Unobstructed |
| `front5_seeded` † | 5.126 | +1.36 / -0.08 | 0.50; 10 | 10/10/10 | .8182 | 417/468 | .323 | Unobstructed |
| `front14`, `front14_t035`, `front14_t065` | 13.940 | +3.68 / -0.04 | 0.50 / 0.35 / 0.65; 3×10 | 10/10/10 each | .6829 | 67/70 | .471 | Same unobstructed front image at three thresholds |
| `front20` | 19.961 | -3.19 / -0.09 | 0.50; 10 | 10/10/10 | .6321 | 40/48 | .326 | Unobstructed; farthest tested front view |
| `side14`, `side14_t035_retry` | 13.940 | +3.68 / +89.96 | 0.50 / 0.35; 2×10 | 0/0/0 each | — | — | — | Visible side profile; no person box |
| `side115` | 11.335 | +1.30 / +89.94 | 0.50; 10 | 0/0/0 | — | — | — | Visible side profile; no person box |
| `side9_a`, unseeded | 8.741 | +2.59 / +90.16 | 0.50; 10 | 10/10/0 | .7591 | 77/140 | .590 | Search accepts valid depth; map accuracy fails |
| `side9_nomotion_retry`, unseeded | 8.741 | +2.59 / +90.16 | 0.50; 10 | 10/10/0 | .7591 | 77/140 | .675 | After three ordinary AMCL no-motion updates; accuracy still fails |
| `side9_seeded_a` † | 8.741 | +2.59 / +90.16 | 0.50; 10 | 10/10/10 | .7591 | 77/140 | .150 | Boundary repeat A |
| `side9_seeded_b` † | 8.674 | +2.63 / +90.18 | 0.50; 10 | 10/10/10 | .7743 | 82/160 | .162 | Boundary repeat B after short Gazebo teleop approach |
| `side9_seeded_c` † | 8.599 | +2.66 / +90.21 | 0.50; 10 | 10/10/10 | .7609 | 80/133 | .140 | Boundary repeat C after another short approach |
| `oblique9_seeded` † | 8.599 | +2.66 / +45.21 | 0.50; 10 | 10/10/10 | .7806 | 151/168 | .132 | Unobstructed oblique aspect |
| `rear9_seeded` † | 8.599 | +2.66 / -179.79 | 0.50; 10 | 10/10/10 | .8165 | 145/160 | .066 | Unobstructed rear aspect |
| `offaxis9_seeded_retry` † | 8.597 | -17.29 / +0.34 | 0.50; 10 | 10/10/10 | .7982 | 196/216 | .247 | Front person, off the optical axis |
| `edge9_seeded` † | 8.591 | -30.87 / +0.43 | 0.50; 10 | 10/10/10 | .8471 | 232/250 | .254 | Near image boundary, full silhouette visible |
| `partial_shelf_seeded` † | 5.741 | -11.01 / -36.21 | 0.50; 10 | 0/0/0 | — | — | — | Person partly visible behind shelf; no box |
| `empty_t035`, `empty_t050`, `empty_t065` | 18.065 to out-of-view target | +43.22 / +129.50 | 0.35 / 0.50 / 0.65; 3×10 | 0/0/0 each | — | — | — | Empty image controls; target outside 71.6° FOV |
| `front2_seeded`, `side2_seeded`, `rear2_seeded` † | 2.058 | +1.55 / +1.63, +91.63, -178.37 | 0.50; 3×10 | 0/0/0 each | — | — | — | Head/upper body cropped above image; lower body visible |

All successful conditions had a stable confirmed track backed by multiple
distinct images in the actual search logs. Across these mixed conditions,
150 images produced a recognized person and an accepted search observation;
130 also met the validation accuracy limit. These totals describe the test
set, not an overall success rate: intentionally empty, occluded, cropped and
difficult views are included. There were **zero observed false-positive boxes
and zero ambiguous boxes** in these sampled views, including 30 known-empty
processed images. This does not establish a false-positive rate for different
backgrounds or unseen people.

Every selected image matched a retained source RGB header and an exactly
timestamped depth image. There were zero per-box header mismatches. The
independent replay reported no frame or box rejection for these selected
samples. The actual search accepted all 150 returned person boxes. Cases with
no box are **recognition misses or negative controls**, not localization
rejections. The two unseeded side trials are **accepted but inaccurate map
localizations**, not depth-support rejections.

### Observed localization failure and controlled separation

The unseeded `side9_a` trial recognized all ten side-profile images and had
77/140 valid central-depth pixels, yet actual map xy error was **0.590 m**.
Three `/request_nomotion_update` calls did not solve it: the repeated image
and depth stayed the same while map error became **0.675 m**. Both failed
accuracy trials are retained in the main table and raw logs.

For diagnostic attribution only, replaying the **same measured ROI median and
pinhole point** through the independently measured stationary Gazebo robot pose
and verified camera mount gave **0.1394 m** horizontal error. The corresponding
point differed horizontally from the production map-TF point by **0.4878 m**. The unchanged
pixels/depth and this counterfactual transform comparison identify pose/TF
accuracy as a substantial contributor; this is not a correction applied by
search. Details are in `trials/side9_a/geometry-audit.json`.

To measure recognition/depth behavior with a controlled localization condition,
the following **explicit validation-only initialization** was then performed:

```bash
ros2 run search validation_initial_pose --ros-args \
  -p use_sim_time:=true -p acknowledge_validation:=true \
  -p x:=1.9726088276654494 -p y:=-14.319767935623513 \
  -p yaw:=-1.6131598121212856
```

The actual run used the identically sourced temporary executable under `/tmp`
while the normal package build was pending. Its complete command, measured
Gazebo pose and publication log are `controlled-initial-pose-command.json`,
`before-controlled-initial-pose.json` and `controlled-initial-pose.log`.
The helper required the acknowledgment flag, published one stamped
`PoseWithCovarianceStamped` in `map` with xy variance **0.0025 m²** and yaw
variance **0.0004 rad²**, and issued no motion command. Production search still
used image-time TF and never read Gazebo truth.

After that initialization the three side repeats at **8.741, 8.674 and
8.599 m** each achieved **10/10 recognized and 10/10 accurate accepted
localizations**, with errors **0.150, 0.162 and 0.140 m**. Short measured Gazebo
teleop approaches changed the images between repeats; first/last images
within each stationary repeat were identical. This meets the preregistered
three-repeat criterion **for the controlled condition** and does not erase
the unseeded failures or prove that AMCL will stay accurate on a mission.

### Recognition, usable range and settings choice

- The unobstructed front trial passed at **19.961 m**, the farthest distance
  tested for that aspect. It is a tested lower bound for this one front-view
  condition, **not the maximum recognition range** and not a warehouse-wide
  usable radius.
- The side-profile condition passed repeated controlled trials around
  **8.6–8.7 m** and missed at **11.335 m** and **13.940 m**. The exact side
  boundary between the passing and failing views remains unmeasured. Reliable
  recognition and reliable localization are distinct: unseeded 8.741 m views
  recognized the person while failing the map-accuracy requirement.
- A valid-depth-support range boundary was **not observed**. Even the
  19.961 m front box retained **40/48 valid ROI pixels**, exceeding both the
  25-pixel and 25% requirements. The occluded 5.741 m view and cropped 2.058 m
  views returned no boxes, so localization was not attempted. Neither is
  evidence of a 25-pixel depth-support failure. Sensor clipping at 100 m is
  unrelated to the measured person-recognition bounds.
- Keep detector `confidence_threshold=0.5` and search `min_confidence=0.5`.
  At 13.940 m, matched 0.35/0.50/0.65 profiles all returned the same .6829
  person hypotheses and all passed. Lowering to 0.35 did not recover the
  missed side-profile view; all three empty-view profiles returned no boxes.
  The observed 19.961 m front score was .6321: a .65 cutoff would suppress
  **those recorded hypotheses**. That last statement is an analytic threshold
  comparison, not a separately executed 20 m/.65 trial. These cases support
  retaining .5; they do not prove a globally optimal threshold or a precision
  advantage over .35.
- Use `cpu_threads=1`, requested `inference_size=640` and FP32 CPU, retaining
  the `max_rate_hz=2.0` simulated-time cap. One thread avoided the measured
  seven-thread slowdown and left CPU capacity for Gazebo/Nav2. The data do
  not establish that it is always faster than two threads. A smaller
  inference size was not compared, so its speed/range tradeoff is unknown.

Across the **250 selected trial images**, measured source processing rate was
**2.000 Hz in simulation time**, with per-trial wall rates **1.270–1.862 Hz**.
Independent source-to-arrival age was **0.231 / 0.300 / 0.351 / 0.396 simulated
seconds** for minimum / median / 95th percentile / maximum. Wall prediction
duration over those matching source images was **162.08 / 233.47 / 371.67 /
464.28 ms** by the same statistics. Quantiles use linear interpolation between
sorted samples. These measurements exclude trial warm-up and are distinct from
the separately reported model-startup timings. The computed aggregate is
`range-final-timing-summary.json`; condition-specific rates/latencies remain
in the CSV and raw summaries.

### Coverage assumption and remaining limits

The geometry-based route uses a **3.2–8.0 m planning annulus**, with the
approximately **71.6°** horizontal FOV and stationary viewing headings. The
outer 8 m value stays inside the three controlled side-repeat ranges rather
than using the exceptional 20 m front result. The inner 3.2 m value is slightly
farther than the successful 3.142 m front view and excludes the observed
2.058 m head/upper-body cropping failures. The exact near boundary was not
measured. This is a conservative **conditional planning assumption**, not
evidence that every pose, aspect and distance in the annulus will work.

The partial-shelf miss occurred inside that annulus, illustrating why alternate
viewpoints and explicit occlusion gaps remain necessary. The route's raycast
coverage is a map-geometry calculation and cannot be read as a recognition
success rate or proof of complete warehouse coverage. See
[coverage-plan.md](coverage-plan.md) for ordered waypoints, viewing headings,
the annotated map and the uncovered areas.

Only one Fuel male-on-phone asset was tested, under this simulation's fixed
lighting and renderer. The other default person and unknown placements have
not been validated here. First and last source-matched images were visually
inspected and all output arrays checked, but static repeated pixels are
strongly correlated and the three repeats are not independent randomized
scenes. No occlusion mask was measured, so no percentage of hidden body is
claimed. AMCL drift, moving-target stale tracks and intermittent setup/service
discovery issues remain distinct from detector false positives. Mission-level
localization, unknown-person coverage and return-home behavior await later
milestones.

### Validation-only poses, commands and retained failed attempts

The standing person's original pose is **(1,-1,0), yaw 1.57**, saved in
`person-original-pose.txt` and the original native-pose captures. The clear
aisle tests temporarily used **(2,-23,0)**: yaw **pi/2** for front views,
**0** for side, **pi/4** for oblique and **-pi/2** for rear. The empty-image
controls placed it at **(13.5,-23,0), yaw 0**, outside the camera FOV. The
partly hidden test used **(-1.4,-22.7,0), yaw pi/2**. These are experimental
truth values only; they are absent from mission waypoint selection and the
production perception algorithm.

One reproducible front-view collection example, after selecting a free robot
view and recording its actual Gazebo pose, is:

```bash
gz service -s /world/warehouse/set_pose --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'name:"Person 1 - Standing", position:{x:2,y:-23,z:0}, orientation:{z:0.7071067811865475,w:0.7071067811865476}'
bash scripts/measure-trial.sh UNIQUE_FRONT_LABEL 'Person 1 - Standing' \
  2.0 -23.0 1.57079632679 0.5 10
ruby scripts/summarize-trials.rb
```

Set both live confidence parameters before a threshold comparison. Use a new
label for every repeat; the wrapper refuses existing directories. The original
person pose is restored with this command and verified against native Gazebo
state after testing:

```bash
gz service -s /world/warehouse/set_pose --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean --timeout 3000 \
  --req 'name:"Person 1 - Standing", position:{x:1,y:-1,z:0}, orientation:{z:0.706825181105366,w:0.7073882691671998}'
```

Restoration was performed: `restore-original-person.log` records `data: true`,
and `restored-world-poses.json` verifies the native Gazebo standing-person pose
at **(1,-1)** with the original quaternion above. The other default person
remained unchanged. Both proof files are under `setup-logs/milestone3/`.
The robot remains stationary at the last test area, approximately
**(1.94154,-21.00344), yaw -1.569 rad**; the return-home mission is still
unimplemented.

The directories `side14_t035`, `side9_nomotion` and `offaxis9_seeded` retain
failed setup attempts with no processed-image JSONL. They are excluded from
the 25 completed trials and are not zero-detection trials. Successful retries
have distinct `_retry` labels. Transient process/parameter and Gazebo-discovery
failures were retained; live confidence checks and pose capture occur before
sampling, and incomplete runs are not counted as measured successes or misses.
Raw service replies, requested poses, before/after states and commands provide
the reproducibility trail. No autonomous warehouse mission was executed.

### Final build, checks and cleanup

`bash scripts/project.sh build` passed after the instrumentation, route and
validation tools were installed. Selected-package regression checks used:

```bash
source scripts/ros-env.bash
cd "$CSE180_WS"
AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1 python3 -m colcon test \
  --packages-select search detector --return-code-on-test-failure \
  --event-handlers console_direct+
python3 -m colcon test-result --test-result-base build/search/test_results --verbose
python3 -m colcon test-result --test-result-base build/detector --verbose
```

All **25 C++ behavioral tests** passed. Search's fresh scoped results report
51 checks with zero errors, failures or skips; detector reports three checks,
zero failures and its preexisting copyright skip. The installed cppcheck
version required the documented performance override. Its first run rejected
a GoogleTest macro inside an anonymous namespace; moving the fixture namespace
closure before the test macros fixed that parser issue without suppressions
or changes to the seven test bodies. Verified logs:
`build-verified.log`, `package-tests-verified.log`,
`search-test-results-verified.log` and `detector-test-results-verified.log`.
Workspace-wide summaries can include older course-package/CTest runs, so the
scoped fresh files are the final evidence for this session.

Stopped this session's detector, search and keyboard-teleop processes after
measurement. The original warehouse/Nav2/RViz launch remains running. Both
people are at their original poses; native `final-world-state.json` also
shows no robot xy displacement during the read-only path checks. The robot
was not returned to a claimed mission-home pose. No changes were pushed.
