"""Vietnamese offline Whisper/Vosk transcription from WAV or PulseAudio."""

from dataclasses import dataclass, asdict
import json
import math
from pathlib import Path
import subprocess
import tempfile
import time
import uuid
import wave


@dataclass(frozen=True)
class Transcript:
    text: str
    confidence: float | None
    source: str
    command_id: str
    audio_path: str = ""

    def to_json(self):
        return json.dumps(asdict(self), ensure_ascii=False)


class VietnameseRecognizer:
    def __init__(self, model_path):
        self.capture_process = None
        self.backend = "whisper.cpp" if Path(model_path).is_file() else "vosk"
        if self.backend == "whisper.cpp":
            from pywhispercpp.model import Model

            self.model = Model(
                str(model_path),
                n_threads=1,
                context_params={"use_gpu": False},
                language="vi",
                no_context=True,
                print_realtime=False,
                print_progress=False,
                print_timestamps=False,
                redirect_whispercpp_logs_to=None,
            )
            return
        import vosk

        vosk.SetLogLevel(-1)
        self.model = vosk.Model(str(model_path))
        self.vosk = vosk

    def decoder(self, rate=16000):
        recognizer = self.vosk.KaldiRecognizer(self.model, rate)
        recognizer.SetWords(True)
        return recognizer

    def transcribe_wav(self, path, source="wav_file"):
        path = Path(path).resolve()
        start = time.perf_counter()
        parts = []
        words = []
        if self.backend == "whisper.cpp":
            with wave.open(str(path), "rb") as wav:
                if (
                    wav.getnchannels() != 1
                    or wav.getsampwidth() != 2
                    or wav.getframerate() != 16000
                ):
                    raise ValueError("Whisper WAV must be mono PCM 16-bit at 16000 Hz")
            segments = self.model.transcribe(str(path), language="vi")
            text = " ".join(segment.text.strip() for segment in segments).strip()
            return Transcript(text, None, source, str(uuid.uuid4()), path.name), {
                "processing_seconds": time.perf_counter() - start,
                "backend": self.backend,
                "confidence_note": "No calibrated confidence supplied by this wrapper",
            }
        with wave.open(str(path), "rb") as wav:
            if wav.getnchannels() != 1 or wav.getsampwidth() != 2:
                raise ValueError(
                    "WAV must be mono PCM signed 16-bit; resample with ffmpeg first"
                )
            rec = self.decoder(wav.getframerate())
            while True:
                block = wav.readframes(4000)
                if not block:
                    break
                if rec.AcceptWaveform(block):
                    result = json.loads(rec.Result())
                    parts.append(result.get("text", ""))
                    words += result.get("result", [])
            result = json.loads(rec.FinalResult())
            parts.append(result.get("text", ""))
            words += result.get("result", [])
        text = " ".join(p for p in parts if p).strip()
        confidence = (
            sum(float(w.get("conf", 0)) for w in words) / len(words) if words else 0
        )
        return Transcript(text, confidence, source, str(uuid.uuid4()), path.name), {
            "processing_seconds": time.perf_counter() - start,
            "words": words,
        }

    def stop_microphone(self):
        if self.capture_process and self.capture_process.poll() is None:
            self.capture_process.terminate()

    def microphone(self, device=None, stop_event=None):
        """PulseAudio/WSLg -> energy VAD -> offline ASR; temporary audio is removed."""
        from collections import deque
        import numpy as np

        process = subprocess.Popen(
            [
                "ffmpeg",
                "-nostdin",
                "-hide_banner",
                "-loglevel",
                "error",
                "-f",
                "pulse",
                "-i",
                str(device or "default"),
                "-ac",
                "1",
                "-ar",
                "16000",
                "-f",
                "s16le",
                "pipe:1",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        self.capture_process = process
        pre_roll = deque(maxlen=2)
        chunks = []
        quiet = 0
        voiced = 0
        try:
            while stop_event is None or not stop_event.is_set():
                block = process.stdout.read(8000)
                if not block:
                    raise RuntimeError(
                        "Microphone stopped: "
                        + process.stderr.read().decode(errors="replace")[-250:]
                    )
                samples = (
                    np.frombuffer(block, dtype=np.int16).astype(np.float32) / 32768
                )
                active = math.sqrt(float((samples * samples).mean())) > 0.012
                if not chunks:
                    pre_roll.append(block)
                    if not active:
                        continue
                    chunks = list(pre_roll)
                    quiet = 0
                    voiced = 1
                else:
                    chunks.append(block)
                    quiet = 0 if active else quiet + 1
                    voiced += int(active)
                if quiet >= 3 or len(chunks) >= 28:
                    if voiced >= 2:
                        with tempfile.TemporaryDirectory(
                            prefix="voice_nav_mic_"
                        ) as temporary:
                            path = Path(temporary) / "utterance.wav"
                            with wave.open(str(path), "wb") as wav:
                                wav.setnchannels(1)
                                wav.setsampwidth(2)
                                wav.setframerate(16000)
                                wav.writeframes(b"".join(chunks))
                            transcript, _ = self.transcribe_wav(path, "live_microphone")
                            if transcript.text:
                                yield Transcript(
                                    transcript.text,
                                    transcript.confidence,
                                    "live_microphone",
                                    transcript.command_id,
                                )
                    chunks = []
                    pre_roll.clear()
                    quiet = voiced = 0
        finally:
            process.terminate()
            try:
                process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                process.kill()
            self.capture_process = None


def main(args=None):
    import argparse

    parser = argparse.ArgumentParser(
        description="Vietnamese offline ASR -> /voice_nav/command"
    )
    parser.add_argument("--model", required=True)
    parser.add_argument("--wav")
    parser.add_argument("--device")
    parser.add_argument("--publish", action="store_true")
    parser.add_argument("--source", default="wav_file")
    parser.add_argument("--output")
    options = parser.parse_args(args)
    recognizer = VietnameseRecognizer(options.model)
    publisher = node = None
    if options.publish:
        import rclpy
        from std_msgs.msg import String

        rclpy.init()
        node = rclpy.create_node("vietnamese_voice_input")
        publisher = node.create_publisher(String, "/voice_nav/command", 10)
        until = time.monotonic() + 1.5
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.1)
    try:
        iterator = (
            [recognizer.transcribe_wav(options.wav, options.source)[0]]
            if options.wav
            else recognizer.microphone(options.device)
        )
        for transcript in iterator:
            text = transcript.to_json()
            print(text, flush=True)
            if options.output:
                Path(options.output).write_text(text, encoding="utf-8")
            if publisher:
                publisher.publish(String(data=text))
                for _ in range(5):
                    rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        if node:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
