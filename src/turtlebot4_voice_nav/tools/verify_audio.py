import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from turtlebot4_voice_nav.speech import VietnameseRecognizer
from turtlebot4_voice_nav.commands import RegionCatalog, CommandError

recognizer = VietnameseRecognizer(ROOT / "models/ggml-base.bin")
catalog = RegionCatalog.load(ROOT / "maps/regions.json")
expected = {
    "01_area_a": ["A"],
    "02_lab": ["B"],
    "03_corridor": ["C"],
    "04_a_to_b": ["A", "B"],
    "05_unknown": None,
}
rows = []
for sample in json.loads(
    (ROOT / "demo/audio/manifest.json").read_text(encoding="utf-8")
):
    transcript, metrics = recognizer.transcribe_wav(
        ROOT / "demo/audio" / sample["file"], sample["source"]
    )
    try:
        route = list(catalog.parse(transcript.text).regions)
        error = None
    except CommandError as exc:
        route = None
        error = str(exc)
    row = dict(
        id=sample["id"],
        expected_text=sample["expected_text"],
        transcript=json.loads(transcript.to_json()),
        expected_route=expected[sample["id"]],
        actual_route=route,
        error=error,
        **metrics,
        passed=route == expected[sample["id"]]
    )
    rows.append(row)
    print(
        json.dumps(
            {k: row[k] for k in ("id", "transcript", "actual_route", "passed")},
            ensure_ascii=False,
        ),
        flush=True,
    )
out = ROOT / "results"
out.mkdir(exist_ok=True)
(out / "speech_results.json").write_text(
    json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8"
)
try:
    import subprocess

    listing = subprocess.run(
        ["ffmpeg", "-hide_banner", "-sources", "pulse"],
        capture_output=True,
        text=True,
        timeout=10,
    )
    probe = subprocess.run(
        [
            "ffmpeg",
            "-nostdin",
            "-hide_banner",
            "-loglevel",
            "error",
            "-f",
            "pulse",
            "-i",
            "default",
            "-t",
            "1",
            "-f",
            "null",
            "-",
        ],
        capture_output=True,
        timeout=10,
    )
    data = {
        "backend": "FFmpeg PulseAudio/WSLg",
        "devices": listing.stdout + listing.stderr,
        "opened_one_second": probe.returncode == 0,
        "human_speech_accuracy": "NOT MEASURED",
    }
except Exception as exc:
    data = {"error": str(exc)}
(out / "microphone_probe.json").write_text(
    json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8"
)
print("Microphone:", json.dumps(data, ensure_ascii=False), flush=True)
raise SystemExit(0 if all(r["passed"] for r in rows) else 1)
