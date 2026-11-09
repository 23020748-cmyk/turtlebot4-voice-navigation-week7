#!/usr/bin/env bash
set -euo pipefail
package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_dir="$(cd -- "$package_dir/../.." && pwd)"
python3 -m venv --system-site-packages "$workspace_dir/.venv-voice-nav"
"$workspace_dir/.venv-voice-nav/bin/python" -m pip install -r "$package_dir/requirements-speech.txt"
"$workspace_dir/.venv-voice-nav/bin/python" "$package_dir/tools/download_whisper_model.py"
