"""Download the CPU multilingual Whisper base GGML model (no Torch/CUDA)."""

import hashlib
import json
from pathlib import Path
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
folder = ROOT / "models"
folder.mkdir(exist_ok=True)
path = folder / "ggml-base.bin"
url = "https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-base.bin"
if not path.exists():
    temporary = path.with_suffix(".download")
    with urllib.request.urlopen(url, timeout=90) as response, temporary.open(
        "wb"
    ) as out:
        size = 0
        while True:
            block = response.read(1024 * 1024)
            if not block:
                break
            size += len(block)
            if size > 180_000_000:
                raise RuntimeError("Unexpected model size")
            out.write(block)
    temporary.replace(path)
data = {
    "model": "Whisper base multilingual GGML",
    "url": url,
    "bytes": path.stat().st_size,
    "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    "license": "MIT, whisper.cpp model distribution",
    "device": "CPU",
}
(folder / "whisper_manifest.json").write_text(json.dumps(data, indent=2))
print(json.dumps(data), flush=True)
