#!/usr/bin/env bash
set -eo pipefail
package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(cd -- "$package_dir/../.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -f "$workspace_dir/install/setup.bash" ]]; then
    source "$workspace_dir/install/setup.bash"
fi
set -u
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export LP_NUM_THREADS=2
export PYTHONPATH="$package_dir:${PYTHONPATH:-}"
python="$workspace_dir/.venv-voice-nav/bin/python"
mode="${1:-help}"
if [[ $# -gt 0 ]]; then shift; fi
case "$mode" in
    simulation)
        exec ros2 launch turtlebot4_voice_nav voice_sim.launch.py \
            "results_dir:=$package_dir/results/live" "$@"
        ;;
    dashboard)
        exec "$python" -m turtlebot4_voice_nav.dashboard \
            --regions "$package_dir/maps/regions.json" \
            --model "$package_dir/models/ggml-base.bin" "$@"
        ;;
    demo)
        exec "$python" "$package_dir/tools/integration_demo.py" "$@"
        ;;
    microphone)
        exec "$python" -m turtlebot4_voice_nav.speech \
            --model "$package_dir/models/ggml-base.bin" --publish "$@"
        ;;
    wav)
        exec "$python" -m turtlebot4_voice_nav.speech \
            --model "$package_dir/models/ggml-base.bin" --publish --wav "$@"
        ;;
    segment)
        python3 "$package_dir/tools/prepare_assets.py"
        exec python3 "$package_dir/tools/segment_map.py"
        ;;
    tests)
        exec python3 -m unittest discover -s "$package_dir/tests" -v
        ;;
    stop)
        exec python3 "$package_dir/tools/stop_owned_stack.py"
        ;;
    *)
        printf '%s\n' 'Modes: simulation, dashboard, demo, microphone, wav FILE, segment, tests, stop'
        ;;
esac
