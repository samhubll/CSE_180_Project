# First perception slice

`search` is C++ and sends no motion goals. The detector remains the existing
Python CPU YOLO node. Autonomous waypoint coverage and return-home behavior
belong to later milestones.

## Data policy

The verified sensor contract is in [sensor-contract.md](sensor-contract.md).
RGB boxes directly index depth only for the registered, single Gazebo RGB-D
camera inspected there. Recheck registration before remapping sensor topics.

- Keep up to 160 depth frames and at most 8 simulated seconds, sorted by source
  time. An out-of-order arrival cannot extend retention. At the configured
  10 Hz this accommodates approximately 80 frames within the age limit.
- Choose the closest depth timestamp within `depth_tolerance_sec=0.02` seconds;
  ties choose the earlier frame. This is smaller than half the 0.1 s camera
  period. Missing depth is retried rather than replaced with latest depth.
- Require the observed optical frame on calibration, depth and detections;
  require each box header to equal its array header. Reject zero/malformed
  source timestamps, stale images, future images and repeated/out-of-order
  arrays. The default maximum image age is 5 simulated seconds, configurable
  up to 7 seconds to fit within the 8-second history and default 10-second TF
  cache. Future timestamps more than 0.1 seconds ahead of `/clock` are rejected.
  Depth and calibration also have bounded arrival ages before entering state,
  so a future depth stamp cannot evict valid history. Invalid/reordered
  calibration is rejected; a valid calibration change resets tracks and
  rejects delayed detections from before the change.
- Process nonempty arrays in source order. A 50 ms wall timer retries missing
  calibration/depth/image-time TF without blocking sensor callbacks. A missing
  head can delay later arrays until its age limit; all queues are bounded.
- Sample the central 50% of each box dimension, then clip that region to image
  bounds. Require at least 25 valid pixels and at least 25% valid support.
  Reject nonfinite, nonpositive and out-of-range samples. Defaults use the
  observed sensor clip interval 0.3 to 100 meters; these are sensor limits,
  not measured person-detection ranges. Use the valid-depth median (mean of
  the two middle values for even support), and the center of the clipped
  discrete pixel region as `(u,v)`. This is a surface estimate, not a fitted
  anatomical center.
- Decode observed `32FC1` as meters. `16UC1` millimeters is also supported and
  unit-tested but was not observed live. Honor row padding and either byte
  order; reject truncated/malformed images. Reject rotated/invalid boxes.
- Validate matching image/calibration dimensions and frame, positive finite
  canonical K, zero distortion and unsupported crop/binning settings. Project
  with `X=(u-cx)*d/fx`, `Y=(v-cy)*d/fy`, `Z=d`. The optical point carries the
  RGB source header. Query `map <- optical` at that exact, nonzero timestamp
  and apply the transform. There is no latest-TF fallback.
- Associate the nearest track within 0.6 m in the horizontal map plane,
  inclusively. Average x, y and z over observations. Count a source image
  only once for each track, including duplicate boxes. Require two distinct
  images to confirm. IDs remain stable; cap tracks at 256. Nearby people can
  merge and depth on occluders/backgrounds can bias the estimate; these are
  limitations to evaluate in later milestones.
- Publish only confirmed tracks as green spheres on `/people/markers` in
  `map`, namespace `confirmed_people`, with stable integer IDs. The text field
  records ID/image count for inspection. Position is the averaged 3D surface
  point. Markers persist for the session and use reliable/transient-local QoS
  for late subscribers. Startup, clock rewind and calibration changes clear
  old markers and tracks. A new session can reuse IDs after that explicit reset.
  Every retained publication is a complete `DELETEALL` plus current confirmed
  markers snapshot, so a reconnecting RViz can recover missed deletions.

All perception subscriptions and TF callbacks share one executor. Lookups are
nonblocking, so this executor continues receiving depth and TF while waiting.
The linked MRTP Navigator is outside that executor because its mission methods
spin internally. It is configured on simulated time and sends no goals here.

## Build, checks and inspection

```bash
bash scripts/project.sh build
source scripts/ros-env.bash
cd "$CSE180_WS"
python3 -m colcon test --packages-select search detector --event-handlers console_direct+
python3 -m colcon test-result --verbose
```

With the warehouse already running, use separate terminals:

```bash
bash scripts/project.sh detector
bash scripts/project.sh search
```

In RViz, use fixed frame `map`, add a **MarkerArray** display and choose
`/people/markers`. Set reliability **Reliable** and durability **Transient
Local**. Confirmed people appear as green spheres. The existing Nav2 map and
localization displays remain useful; a marker does not mean the autonomous
mission is implemented.

The read-only C++ evidence collector is built when `BUILD_TESTING` is enabled:

```bash
ros2 run search search_live_probe --ros-args -p use_sim_time:=true \
  -p duration_sec:=30.0
```

It checks source-header matches, depth time deltas, input ages, finite marker
positions/stable IDs, and compares image-time/latest TF on a **hypothetical**
3 m optical ray. That displacement is not an accuracy measurement. Optional
`snapshot_path` saves the first RGB frame as PPM, separately stamped and
explicitly not detection-matched.

`validation_motion` is a separate manual C++ Nav2 helper. It requires an
explicit mode plus a pose or angle supplied as parameters; it contains no
route or person positions. Both it and `search` reject wall-time operation.
Its execution timer cancels an unfinished goal; an outer shell timeout is
still needed because MRTP's goal-acceptance/cancellation waits are unbounded.
Use it only with the course Gazebo simulation. Concrete viewing/turn commands
and observed results belong in [perception-validation.md](perception-validation.md).
