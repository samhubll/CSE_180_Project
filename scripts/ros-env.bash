# Source this file from Bash inside the ROS Docker container.
project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export CSE180_PROJECT_ROOT="$project_root"
export CSE180_WS=$(cd -- "$project_root/../.." && pwd)
course_setup="$CSE180_WS/../MRTP/install/setup.bash"

source /opt/ros/jazzy/setup.bash
if [[ -f "$course_setup" ]]; then
  source "$course_setup"
else
  printf 'MRTP is not built. Build the course workspace at %s/../MRTP first.\n' "$CSE180_WS" >&2
  return 1
fi

if [[ -f /opt/cse180-project-venv/bin/activate ]]; then
  source /opt/cse180-project-venv/bin/activate
fi
if [[ -f "$CSE180_WS/install/setup.bash" ]]; then
  source "$CSE180_WS/install/setup.bash"
fi
unset project_root course_setup
