#!/usr/bin/env bash
set -eo pipefail

# Works from a source checkout; ros2 run already supplies the installed environment.
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "$script_dir/../../.." && pwd)"
if [[ -f "$workspace_dir/install/setup.bash" ]]; then
    source "$workspace_dir/install/setup.bash"
fi
exec ros2 launch teleop_launch teleop_launch.py "$@"
