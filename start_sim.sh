#!/usr/bin/env bash
set -e

# Workspace directory
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$WS_DIR"

# Activate Pixi environment
PIXI_HOOK="$(pixi shell-hook)"
eval "$PIXI_HOOK"
unset PIXI_HOOK

# Source ROS 2 workspace
if [[ ! -f install/setup.bash ]]; then
    echo "Workspace has not been built."
    echo "Run: pixi run colcon build --symlink-install"
    exit 1
fi

source install/setup.bash

# Register cleanup before starting any background processes.
CHILD_PIDS=()
cleanup() {
    trap - EXIT INT TERM
    if (( ${#CHILD_PIDS[@]} )); then
        kill "${CHILD_PIDS[@]}" 2>/dev/null || true
        wait "${CHILD_PIDS[@]}" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "Starting EdgeAI realistic robot simulation and RViz..."

# The launch file opens one RViz window per robot
ros2 launch edgeai_robot_description simulation_real.launch.py "$@" &
SIM_PID=$!
CHILD_PIDS+=("$SIM_PID")

wait "$SIM_PID"
