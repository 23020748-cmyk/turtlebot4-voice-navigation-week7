"""Mux the actual dashboard capture with its timestamped synthetic WAV inputs."""

import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT.parents[1] / "reports/bai07_voice_navigation"
OUT.mkdir(parents=True, exist_ok=True)
capture = ROOT / "demo/dashboard_raw.mp4"
metadata = json.loads(capture.with_suffix(".json").read_text())
rows = json.loads((ROOT / "results/integration_results.json").read_text())
voice_rows = [row for row in rows if "audio_start_monotonic" in row]
audio = {
    sample["id"]: sample
    for sample in json.loads((ROOT / "demo/audio/manifest.json").read_text())
}
probe = json.loads(
    subprocess.check_output(
        [
            "ffprobe",
            "-v",
            "error",
            "-show_format",
            "-show_streams",
            "-of",
            "json",
            str(capture),
        ]
    )
)
capture_duration = float(probe["format"]["duration"])
capture_start = metadata["recording_started_monotonic"]
first_input = voice_rows[0]["audio_start_monotonic"] - capture_start
trim_start = max(0, first_input - 2)
last = voice_rows[-1]
trim_end = min(
    capture_duration,
    last["audio_start_monotonic"] - capture_start + last["elapsed_seconds"] + 3,
)
duration = trim_end - trim_start
command = [
    "ffmpeg",
    "-y",
    "-hide_banner",
    "-loglevel",
    "error",
    "-ss",
    str(trim_start),
    "-i",
    str(capture),
]
filters = []
inputs = []
for index, row in enumerate(voice_rows, 1):
    path = ROOT / "demo/audio" / audio[row["id"]]["file"]
    command.extend(["-i", str(path)])
    delay = round((row["audio_start_monotonic"] - capture_start - trim_start) * 1000)
    filters.append(f"[{index}:a]adelay={delay}|{delay}[a{index}]")
    inputs.append(
        dict(
            id=row["id"],
            file=path.name,
            delay_ms=delay,
            sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        )
    )
filters.append(
    "".join(f"[a{i}]" for i in range(1, len(voice_rows) + 1))
    + f"amix=inputs={len(voice_rows)}:duration=longest:normalize=0,apad,atrim=duration={duration}[audio]"
)
caption = OUT / "video_caption.txt"
caption.write_text(
    "DEMO TỰ ĐỘNG • WAV tổng hợp Piper VAIS-1000 • Pose và plan đọc từ ROS/Nav2.\nÂm thanh ghép từ WAV đầu vào theo timestamp nhận dạng. Video giữ thời gian thực."
)
font = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
video_filter = f"pad=iw:ih+40:0:0:color=white,drawtext=fontfile={font}:textfile={caption}:fontsize=12:fontcolor=0x153658:x=12:y=h-35:line_spacing=3"
output = OUT / "demo_voice_navigation.mp4"
command.extend(
    [
        "-filter_complex",
        ";".join(filters),
        "-vf",
        video_filter,
        "-map",
        "0:v:0",
        "-map",
        "[audio]",
        "-t",
        str(duration),
        "-c:v",
        "libx264",
        "-threads",
        "2",
        "-preset",
        "fast",
        "-crf",
        "22",
        "-pix_fmt",
        "yuv420p",
        "-r",
        "6",
        "-c:a",
        "aac",
        "-b:a",
        "96k",
        "-movflags",
        "+faststart",
        str(output),
    ]
)
subprocess.run(command, check=True)
preview_time = max(2, duration - 13)
subprocess.run(
    [
        "ffmpeg",
        "-y",
        "-loglevel",
        "error",
        "-ss",
        str(preview_time),
        "-i",
        str(output),
        "-frames:v",
        "1",
        str(OUT / "demo_preview.png"),
    ],
    check=True,
)
result = dict(
    capture_file=capture.name,
    capture_source=metadata["capture"],
    audio_source="SYNTHETIC_TTS_PIPER",
    audio_edit="original WAV inputs aligned to ASR-start monotonic timestamps",
    trim_start_seconds=trim_start,
    trim_end_seconds=trim_end,
    output_seconds=duration,
    playback_speed=1,
    inputs=inputs,
    output_bytes=output.stat().st_size,
)
(OUT / "video_manifest.json").write_text(
    json.dumps(result, ensure_ascii=False, indent=2)
)
print(json.dumps(result, ensure_ascii=False, indent=2))
