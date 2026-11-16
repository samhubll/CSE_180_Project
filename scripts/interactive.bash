if [[ -f /root/.bashrc ]]; then
  source /root/.bashrc
fi
source "$(dirname -- "${BASH_SOURCE[0]}")/ros-env.bash"
cd -- "$CSE180_WS"
printf '\nROS 2 Jazzy terminal. Workspace: %s\nProject: %s\n' "$CSE180_WS" "$CSE180_PROJECT_ROOT"
printf 'Build from VS Code with Ctrl+Shift+B, or run: bash "$CSE180_PROJECT_ROOT/scripts/project.sh" build\n\n'
