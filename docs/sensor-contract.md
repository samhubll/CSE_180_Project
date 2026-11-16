# Sensor contract and validation notes

Inspected in the existing course container on November 15, 2026 (Los Angeles;
raw log dates are November 16 UTC). This records the running default warehouse,
not a detector range measurement or an autonomous search result. The initial
Git status was clean. The warehouse launch was already running; inspection did
not restart it or command movement.

## Commands and evidence

From `/MRTP/CSE180/src/CSE_180_Project`:

```bash
source scripts/ros-env.bash
ros2 topic list -t
ros2 action list -t
ros2 node list
ros2 topic echo /rgbd_camera/camera_info --once
timeout 10 ros2 run tf2_ros tf2_echo base_link oakd_rgb_camera_optical_frame \
  --ros-args -p use_sim_time:=true
gz topic -e -t /world/warehouse/pose/info -n 1
```

Additional bounded C++ inspection artifacts are local, ignored files under
`setup-logs/sensor-probe/`; raw results are `setup-logs/sensor-contract-live.txt`.
The probe was run as
`setup-logs/sensor-probe/probe --ros-args -p use_sim_time:=true -p duration_wall:=30.0`.
`setup-logs/optical-tf.txt` and `setup-logs/warehouse-pose-snapshot.txt` contain
the TF and live Gazebo pose snapshots. Historical setup observations in
`setup-logs/live-readiness.txt` were read but are not substituted for this run.

## Images and calibration

| Topic | ROS message | Resolution | Encoding | Frame |
| --- | --- | --- | --- | --- |
| `/rgbd_camera/image` | `sensor_msgs/msg/Image` | 320 x 240 | `rgb8` | `oakd_rgb_camera_optical_frame` |
| `/rgbd_camera/depth_image` | `sensor_msgs/msg/Image` | 320 x 240 | `32FC1` | `oakd_rgb_camera_optical_frame` |
| `/rgbd_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 320 x 240 | n/a | `oakd_rgb_camera_optical_frame` |
| `/people/detections` | `vision_msgs/msg/Detection2DArray` | source RGB boxes | n/a | copied source image frame |
| `/people/markers` | `visualization_msgs/msg/MarkerArray` | n/a | n/a | `map` |

The last two topics are project outputs and only exist while their nodes run.
Live `CameraInfo` at source stamp 3641.502 s had:

```text
K = [221.76500407999384, 0, 160,
     0, 221.76500407999382, 120,
     0, 0, 1]
fx=K[0], fy=K[4], cx=K[2], cy=K[5]
D = [0,0,0,0,0]; distortion_model=plumb_bob
R = identity
P = [fx,0,cx,0; 0,fy,cy,0; 0,0,1,0]
binning_x=binning_y=0; ROI has zero offsets and dimensions
```

Registration was checked from the **active** launch/model configuration:

- `gazeboenvs/launch/tb4_warehouse.launch.py` selects
  `nav2_minimal_tb4_description/urdf/standard/turtlebot4.urdf.xacro` and includes
  `nav2_minimal_tb4_sim/launch/spawn_tb4.launch.py`.
- `/opt/ros/jazzy/share/nav2_minimal_tb4_description/urdf/sensors/oakd.urdf.xacro`
  defines one `rgbd_camera` sensor on `oakd_rgb_camera_frame`, sharing one
  `<camera>` viewpoint, 320 x 240 image geometry, horizontal FOV 1.25 rad
  (71.62 degrees), and `oakd_rgb_camera_optical_frame` for its RGB and depth.
  It sets `update_rate=10`, near clip 0.3 m and far clip 100 m.
- The active `spawn_tb4.launch.py` and process arguments show separate
  `ros_gz_image/image_bridge` processes for `/rgbd_camera/image` and
  `/rgbd_camera/depth_image`. These transport the same sensor's native outputs;
  no resize, stereo baseline, reprojection or independently calibrated camera
  is inserted. The common calibration is bridged as `CameraInfo`.

Thus RGB box pixels may index depth in this simulation because they come from
one registered RGBD camera model. Equal resolution/frame names alone would not
establish registration for a replacement camera. The similar installed
`turtlebot4_description` package sets 30 Hz, but it is **not** the model selected
by this running warehouse; use the active `nav2_minimal_tb4_description`.

For a valid optical-axis depth `d` in meters:

```text
X = (u-cx)*d/fx
Y = (v-cy)*d/fy
Z = d
```

This uses the calibrated optical axes and zero distortion of the observed
stream. A different distortion model, nonzero distortion or changed binning/ROI
requires checking calibration before using this raw-image pinhole model.

## Timing and depth measurements

The 30-wall-second stationary C++ capture received 165 RGB frames, 162 depth
frames and six detector arrays (all empty in this initial warehouse view).
The detector used CPU YOLOv8n, threshold 0.5 and configured maximum 2 Hz.
Reported timing below is **simulated time**, except the capture duration:

| Measurement | Minimum | Median | Maximum |
| --- | --- | --- | --- |
| RGB source period, seconds | 0.099 | 0.099 | 0.102 |
| Depth source period, seconds | 0.099 | 0.099 | 0.300 |
| Depth arrival age (`now-source`), seconds | 0.012 | 0.045 | 0.102 |
| Detector arrival age (`now-source`), seconds | 2.442 | 3.087 | 3.279 |

Effective RGB source rate was 10.0012 Hz and depth was 9.8183 Hz; the latter
includes a 0.300 s gap. The configured camera period is 0.1 simulated seconds.
Wall callback rates in this capture were 5.5 RGB/s, 5.4 depth/s and 0.2
detector arrays/s. The configured 2 Hz detector maximum is a cap, not a
measured sustained throughput on this loaded CPU.

162 of 165 captured RGB frames had **exactly matching depth source stamps**.
The nearest-depth offset median was zero; the largest was 0.102 s for an RGB
frame without an exact captured depth counterpart. This is why missing depth
must be rejected, not substituted by a frame one camera period away. A 20 ms
matching tolerance is below half the observed 99 ms minimum period and accepts
the observed exact pairs while excluding adjacent frames. Five of six detector
source stamps matched RGB callbacks retained by the probe; one source frame
was not observed in its finite capture. That unmatched observation is not
proof of a header mismatch, and this capture does not establish that every
published RGB frame was received. The detector source code explicitly copies
each source header to its array and boxes; per-box preservation with real
positive detections is checked separately in perception validation.

The measured maximum detector age 3.279 s supports an 8 simulated-second depth
history with a 160-frame cap (approximately 80 frames at the configured rate)
and a 5 s maximum detection age. These permit measured inference lag while
bounding memory and rejecting stale input. Arrival order can differ across
the two bridges; detection processing should retry for matching depth/TF
within the age limit. TF queries use the RGB source time, never arrival time
or latest TF. Recheck the limits if the camera or inference workload changes.
This small six-array sample is not a performance guarantee.

Configured sensor clipping is 0.3 to 100 m; this is not a measured detector
recognition range. Localization must reject nonfinite, nonpositive and
out-of-range samples and must not interpret the 32-bit floating depth as mm.

A native Gazebo depth frame at 3830.301 s was captured with:

```bash
gz topic -e -t /rgbd_camera/depth_image -n 1 --json-output \
  > setup-logs/depth-native.json
jq -r '.data' setup-logs/depth-native.json | base64 -d \
  > setup-logs/depth-native.bin
od -An -v -t f4 setup-logs/depth-native.bin
```

It reported `R_FLOAT32`, 320 x 240, step 1280, the optical frame above, and
contained 76,401 finite pixels and **399 positive infinities**. There were no
NaN, zero or negative-infinity values in that frame. Finite values spanned
0.446838 to 46.3631 m. The bottom-center pixel `(u=160,v=239)` was 0.4468384.
With the observed Gazebo base pose, pitch -0.004396 rad and optical mount,
using that value as meters in the pinhole formula puts this floor sample at
approximately world z=0.001 m. This is a unit/geometry sanity check, not a
claim of person localization accuracy. Scaling this
sample as millimeters would contradict the known camera height and ground
geometry. No-return/out-of-range `+inf` was directly observed; near-clip
behavior was not deliberately induced. Reject NaN and other invalid values
even though they did not occur in this sample.

## Optical TF

Live `base_link <- oakd_rgb_camera_optical_frame`:

```text
translation = (-0.0596, 0.000, 0.24353) m
quaternion (x,y,z,w) = (-0.5, 0.5, -0.5, 0.5)
rotation = [ 0,  0, 1;
            -1,  0, 0;
             0, -1, 0]
RPY = (-pi/2, 0, -pi/2)
```

Optical x points right (robot -y), y points down (robot -z), and z points
forward (robot +x). Image columns increase rightward, image rows downward and
depth extends out through the camera, unlike the mobile base's forward/left/up
axes. Person localization must query `map <- optical` at the **source RGB
stamp**. A latest-transform readiness check does not make a delayed image safe
to localize while turning.

## Nav2 and MRTP Navigator

Live action servers (`nav2_msgs/action/` types):

```text
/assisted_teleop            AssistedTeleop
/backup                     BackUp
/compute_and_track_route    ComputeAndTrackRoute
/compute_path_through_poses  ComputePathThroughPoses
/compute_path_to_pose       ComputePathToPose
/compute_route              ComputeRoute
/dock_robot                 DockRobot
/drive_on_heading           DriveOnHeading
/follow_gps_waypoints       FollowGPSWaypoints
/follow_path                FollowPath
/follow_waypoints           FollowWaypoints
/navigate_through_poses     NavigateThroughPoses
/navigate_to_pose           NavigateToPose
/smooth_path                SmoothPath
/spin                       Spin
/undock_robot               UndockRobot
/wait                       Wait
```

Read `/MRTP/MRTP/src/navigation/include/navigation/navigation.hpp` and
`src/navigation.cpp`. `Navigator` is its own `rclcpp::Node` named `navigator`.
For the later mission use `SetInitialPose(Pose::SharedPtr)`,
`WaitUntilNav2Active()`, `GoToPose(Pose::SharedPtr)`, `Spin(double radians)`,
`IsTaskComplete()`, `GetResult()` and `CancelTask()`. It also supports paths,
waypoints, backup and costmap services. Constructors create clients without
sending goals. The destructor cancels an executing task.

Navigation methods call `rclcpp::spin_until_future_complete`; initial-pose
waiting also calls `spin_some`. Do not simultaneously attach this Navigator
node to another executor that conflicts with its internal spins. Keep
perception on its own node/executor when implementing the later mission, and
set `use_sim_time=true` on both nodes. This session's perception slice sends no
navigation goals.

## Default-world truth (validation only)

These values belong only to validation, never detection, association, mission
waypoints, stopping conditions or an expected person count.

The running Gazebo `/world/warehouse/pose/info` snapshot, and the launch's
generated `/tmp/nav2_anh96py0.sdf`, agree:

| Gazebo entity | World/model origin (x,y,z), m | Yaw, rad |
| --- | --- | --- |
| `Person 1 - Standing` (id 128) | (1, -1, 0) | 1.57 |
| `Person 2 - Walking` (id 132) | (-12, 15, 0) | 0 |

Gazebo omits protobuf scalar fields with their default zero values. These are
model origins, not camera-visible surface centroids. The assignment states
that warehouse world and `map` coincide. This table does not establish
localization accuracy or that either person has been detected in this run.
