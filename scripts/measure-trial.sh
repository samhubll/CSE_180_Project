#!/usr/bin/env bash
# Read-only validation collector. Truth is supplied at runtime, never to search.
main()
{
set -eo pipefail
if [[ $# -lt 6 || $# -gt 7 ]]; then
  printf 'Usage: %s LABEL TARGET_MODEL TARGET_X TARGET_Y TARGET_YAW CONFIDENCE [IMAGES]\n' "$0" >&2
  exit 2
fi
source "$(dirname -- "${BASH_SOURCE[0]}")/ros-env.bash"
trial_label=$1
case "$trial_label" in
  *[!a-zA-Z0-9_-]*|'') printf 'Use a unique alphanumeric trial label.\n' >&2; exit 2 ;;
esac
trial_dir="$CSE180_PROJECT_ROOT/setup-logs/milestone3/trials/$trial_label"
if [[ -e "$trial_dir" ]]; then
  printf 'Refusing to overwrite existing trial: %s\n' "$trial_dir" >&2
  exit 2
fi
mkdir -p "$trial_dir"
{
  date -u '+UTC %Y-%m-%dT%H:%M:%SZ'
  printf 'Invocation:'
  printf ' %q' "$0" "$@"
  printf '\n'
} > "$trial_dir/command.txt"
read_trial_parameter()
{
  local attempt
  for attempt in 1 2 3; do
    if timeout 10 ros2 param get "$1" "$2" > "$3"; then
      return 0
    fi
  done
  return 1
}
read_trial_parameter /person_detector confidence_threshold "$trial_dir/detector-threshold.txt"
read_trial_parameter /search min_confidence "$trial_dir/search-threshold.txt"
detector_threshold=$(awk '{print $NF}' "$trial_dir/detector-threshold.txt")
search_threshold=$(awk '{print $NF}' "$trial_dir/search-threshold.txt")
if ! awk -v expected="$6" -v detector="$detector_threshold" -v search="$search_threshold" \
  'BEGIN {exit !((detector-expected)^2 < 1e-12 && (search-expected)^2 < 1e-12)}'; then
  printf 'Detector/search thresholds must both match the trial threshold.\n' >&2
  exit 2
fi
record_gazebo_pose()
{
  local attempt
  for attempt in 1 2 3; do
    # The native pose topic avoids intermittent /gazebo/worlds discovery
    # timeouts in `gz model` and retains full precision plus the world stamp.
    if timeout 10 gz topic -e -t /world/warehouse/pose/info -n 1 --json-output \
      > "$2.json" 2> "$2.stderr" && ruby -rjson - "$1" "$2.json" "$2" <<'RUBY'
name, input, output = ARGV
# Gazebo can deliver more than one message before --num finishes shutting down.
world = JSON.parse(File.foreach(input).find { |line| line.lstrip.start_with?('{') })
model = world.fetch('pose').find { |pose| pose['name'] == name }
exit 2 unless model
p = model.fetch('position', {})
q = model.fetch('orientation', {})
x, y, z, w = %w[x y z w].map { |key| q.fetch(key, key == 'w' ? 1.0 : 0.0) }
roll = Math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
pitch = Math.asin([[2*(w*y-z*x), -1.0].max, 1.0].min)
yaw = Math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
File.write(output, "Name: #{name}\n[#{%w[x y z].map { |key| p.fetch(key, 0.0) }.join(' ')}]\n[#{[roll, pitch, yaw].join(' ')}]\n")
RUBY
    then
      return 0
    fi
  done
  printf 'Could not read Gazebo pose for %s\n' "$1" >&2
  return 1
}
record_gazebo_pose turtlebot4 "$trial_dir/robot-before.txt"
record_gazebo_pose "$2" "$trial_dir/target-before.txt"
trial_probe="${CSE180_RANGE_PROBE:-$CSE180_WS/install/search/lib/search/range_probe}"
"$trial_probe" --ros-args \
  -p use_sim_time:=true -p trial_label:="$trial_label" \
  -p output_path:="$trial_dir/images.jsonl" -p snapshot_dir:="$trial_dir" \
  -p target_x:="$3" -p target_y:="$4" -p target_yaw:="$5" \
  -p min_confidence:="$6" -p sample_count:="${7:-10}" \
  -p warmup_sim_sec:=7.0 -p warmup_wall_sec:=10.0 -p max_wall_sec:=120.0 \
  > "$trial_dir/probe.log" 2>&1
record_gazebo_pose turtlebot4 "$trial_dir/robot-after.txt"
tail -n 1 "$trial_dir/images.jsonl"
}
main "$@"
