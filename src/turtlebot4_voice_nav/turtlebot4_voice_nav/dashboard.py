"""Lightweight real-time view of actual ROS/AMCL/Nav2 state, not a robot simulator."""

import argparse
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog
from PIL import Image, ImageTk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path as PathMsg
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

from .mapping import GridMap
from .commands import RegionCatalog
from .speech import Transcript


class DashboardBridge(Node):
    def __init__(self):
        super().__init__("voice_navigation_dashboard")
        self.status = {}
        self.pose = None
        self.path = []
        self.progress = {}
        self.pub = self.create_publisher(String, "/voice_nav/command", 10)
        self.status_sub = self.create_subscription(
            String,
            "/voice_nav/status",
            self.on_status,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.on_pose, 10
        )
        self.plan_sub = self.create_subscription(
            PathMsg,
            "/plan",
            lambda m: setattr(
                self, "path", [(p.pose.position.x, p.pose.position.y) for p in m.poses]
            ),
            10,
        )
        self.progress_sub = self.create_subscription(
            String, "/voice_nav/demo", self.on_progress, 10
        )

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except ValueError:
            pass

    def on_progress(self, msg):
        try:
            self.progress = json.loads(msg.data)
        except ValueError:
            pass

    def on_pose(self, msg):
        p = msg.pose.pose
        q = p.orientation
        self.pose = (
            p.position.x,
            p.position.y,
            math.atan2(2 * q.w * q.z, 1 - 2 * q.z * q.z),
        )

    def send(self, transcript):
        self.pub.publish(String(data=transcript.to_json()))


class Dashboard:
    def __init__(self, root, bridge, regions_file, model, record=None, demo_mode=False):
        self.root = root
        self.bridge = bridge
        self.model_path = Path(model)
        self.recognizer = None
        self.catalog = RegionCatalog.load(regions_file)
        self.grid = GridMap.load(
            Path(regions_file).parent / self.catalog.data["map_yaml"]
        )
        self.work = queue.Queue()
        self.stop_event = threading.Event()
        self.busy = False
        self.trace = []
        self.record_process = None
        root.title("Bài 7 | TurtleBot 4 · Giọng nói tiếng Việt · Nav2")
        root.geometry("1120x680+10+0")
        root.minsize(1040, 640)
        ttk.Style(root).theme_use("clam")
        ttk.Label(
            root,
            text="BÀI 7  PHÂN VÙNG BẢN ĐỒ VÀ ĐIỀU HƯỚNG BẰNG GIỌNG NÓI",
            font=("DejaVu Sans", 14, "bold"),
        ).pack(anchor="w", padx=12, pady=(8, 2))
        ttk.Label(
            root,
            text="TurtleBot 4 Lite / Gazebo Fortress headless · Pose TF map (AMCL + odometry) · Nav2 tránh vật cản",
        ).pack(anchor="w", padx=12)
        body = ttk.Frame(root)
        body.pack(fill="both", expand=True, padx=12, pady=5)
        self.canvas = tk.Canvas(
            body, width=600, height=576, highlightthickness=0, bg="#ddd"
        )
        self.canvas.pack(side="left", anchor="n")
        rgb = (
            Image.open(Path(regions_file).parent / "segmented_map.png")
            .convert("RGB")
            .resize((600, 576))
        )
        self.photo = ImageTk.PhotoImage(rgb)
        self.canvas.create_image(0, 0, anchor="nw", image=self.photo)
        self.path_item = self.canvas.create_line(
            0, 0, 0, 0, fill="#1640bb", width=2, dash=(4, 3)
        )
        self.trace_item = self.canvas.create_line(0, 0, 0, 0, fill="#00d8da", width=3)
        self.robot_item = self.canvas.create_oval(
            0, 0, 0, 0, fill="#ec3654", outline="white", width=2
        )
        self.direction_item = self.canvas.create_line(
            0, 0, 0, 0, fill="#a70020", width=3, arrow="last"
        )
        self.target_item = self.canvas.create_oval(
            0, 0, 0, 0, outline="#ff5b00", width=3
        )
        panel = ttk.Frame(body)
        panel.pack(side="left", fill="both", expand=True, padx=(14, 0))
        self.region_text = "\n".join(
            f'{r["id"]} · {r["name"]}: ({r["goal"]["x"]:.2f}, {r["goal"]["y"]:.2f}) m'
            for r in self.catalog.regions.values()
        )
        ttk.Label(panel, text=self.region_text, font=("DejaVu Sans", 10, "bold")).pack(
            anchor="w", pady=(0, 8)
        )
        self.text = tk.StringVar(value="Đi đến khu vực A")
        ttk.Entry(panel, textvariable=self.text, font=("DejaVu Sans", 11)).pack(
            fill="x"
        )
        row = ttk.Frame(panel)
        row.pack(fill="x", pady=5)
        ttk.Button(
            row,
            text="GỬI TEXT",
            command=self.send_text,
            state="disabled" if demo_mode else "normal",
        ).pack(side="left")
        ttk.Button(row, text="DỪNG / HỦY", command=self.cancel).pack(
            side="left", padx=5
        )
        row = ttk.Frame(panel)
        row.pack(fill="x")
        ttk.Button(
            row,
            text="BẬT MIC",
            command=self.start_mic,
            state="disabled" if demo_mode else "normal",
        ).pack(side="left")
        ttk.Button(row, text="TẮT MIC", command=self.stop_mic).pack(side="left", padx=5)
        ttk.Button(
            row,
            text="NHẬN DẠNG WAV",
            command=self.wav,
            state="disabled" if demo_mode else "normal",
        ).pack(side="left")
        self.audio_status = tk.StringVar(
            value="ASR offline · Whisper base CPU · MIC tắt"
        )
        ttk.Label(panel, textvariable=self.audio_status, wraplength=460).pack(
            anchor="w", pady=7
        )
        self.status = tk.StringVar(value="Chờ dữ liệu ROS...")
        self.transcript = tk.StringVar(value="Chưa có câu nhận dạng")
        ttk.Label(
            panel, textvariable=self.status, wraplength=460, font=("DejaVu Sans", 10)
        ).pack(fill="x", pady=5)
        ttk.Label(
            panel, text="CÂU LỆNH / NGUỒN ÂM THANH", font=("DejaVu Sans", 10, "bold")
        ).pack(anchor="w")
        ttk.Label(panel, textvariable=self.transcript, wraplength=460).pack(
            fill="x", pady=5
        )
        ttk.Label(
            panel,
            text="Nét xanh đậm: plan Nav2 · Nét cyan: quỹ đạo TF map\nChấm đỏ: robot · Vòng cam: điểm đích\nXám: ô chưa biết · Ba vùng màu: phân vùng bán tự động",
            wraplength=460,
        ).pack(anchor="w", pady=8)
        self.log = tk.Text(
            panel, height=6, width=50, wrap="word", font=("DejaVu Sans", 9)
        )
        self.log.pack(fill="both", expand=True)
        self.last_event = None
        self.job = root.after(100, self.tick)
        root.protocol("WM_DELETE_WINDOW", self.close)
        if record:
            root.after(500, lambda: self.start_recording(record))

    def xy(self, x, y):
        col, row = self.grid.world_to_pixel(x, y)
        return (col + 0.5) * 600 / self.grid.image.shape[1], (
            row + 0.5
        ) * 576 / self.grid.image.shape[0]

    def send_text(self):
        self.bridge.send(
            Transcript(self.text.get(), None, "text_dashboard", str(time.time_ns()))
        )

    def cancel(self):
        self.bridge.send(
            Transcript("Dừng lại", None, "dashboard_stop_button", str(time.time_ns()))
        )

    def _recognizer(self):
        if self.recognizer is None:
            from .speech import VietnameseRecognizer

            if not self.model_path.exists():
                raise RuntimeError(
                    "Thiếu model. Chạy tools/download_whisper_model.py trước."
                )
            self.recognizer = VietnameseRecognizer(self.model_path)
        return self.recognizer

    def start_mic(self):
        if self.busy:
            return
        self.busy = True
        self.stop_event.clear()
        self.audio_status.set("Đang nạp ASR và mở microphone WSLg...")

        def worker():
            try:
                rec = self._recognizer()
                self.work.put(
                    (
                        "info",
                        "Đang nghe microphone thật · nói lệnh rồi ngừng khoảng 1 giây",
                    )
                )
                for transcript in rec.microphone(stop_event=self.stop_event):
                    self.work.put(("transcript", transcript))
            except Exception as exc:
                if not self.stop_event.is_set():
                    self.work.put(("error", str(exc)))
            finally:
                self.work.put(("done", None))

        threading.Thread(target=worker, daemon=True).start()

    def stop_mic(self):
        self.stop_event.set()
        if self.recognizer:
            self.recognizer.stop_microphone()
        self.audio_status.set("MIC đã tắt")

    def wav(self):
        if self.busy:
            return
        filename = filedialog.askopenfilename(filetypes=[("PCM WAV", "*.wav")])
        if not filename:
            return
        self.busy = True
        self.audio_status.set(
            "Đang nhận dạng WAV; các WAV mẫu trong demo/audio là giọng tổng hợp."
        )

        def worker():
            try:
                transcript, _ = self._recognizer().transcribe_wav(
                    filename, "selected_wav_file"
                )
                self.work.put(("transcript", transcript))
            except Exception as exc:
                self.work.put(("error", str(exc)))
            finally:
                self.work.put(("done", None))

        threading.Thread(target=worker, daemon=True).start()

    def start_recording(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        self.root.update_idletasks()
        data = {
            "recording_started_monotonic": time.monotonic(),
            "window_id": self.root.winfo_id(),
            "file": path.name,
            "capture": "actual X11 dashboard window",
            "robot_source": "AMCL from real Gazebo TurtleBot4/Nav2 simulation",
            "fps": 6,
        }
        self.record_process = subprocess.Popen(
            [
                "ffmpeg",
                "-y",
                "-nostdin",
                "-hide_banner",
                "-loglevel",
                "error",
                "-f",
                "x11grab",
                "-framerate",
                "6",
                "-window_id",
                str(self.root.winfo_id()),
                "-i",
                os.environ.get("DISPLAY", ":0"),
                "-an",
                "-c:v",
                "libx264",
                "-threads",
                "1",
                "-preset",
                "ultrafast",
                "-crf",
                "24",
                "-pix_fmt",
                "yuv420p",
                str(path),
            ],
            stdout=subprocess.DEVNULL,
            stderr=(path.with_suffix(".log")).open("w"),
        )
        path.with_suffix(".json").write_text(
            json.dumps(data, indent=2), encoding="utf-8"
        )

    def tick(self):
        try:
            while True:
                kind, value = self.work.get_nowait()
                if kind == "transcript":
                    self.text.set(value.text)
                    self.bridge.send(value)
                    self.audio_status.set("ASR: " + value.text)
                elif kind == "error":
                    self.audio_status.set("Lỗi ASR: " + value)
                elif kind == "info":
                    self.audio_status.set(value)
                elif kind == "done":
                    self.busy = False
        except queue.Empty:
            pass
        status = dict(self.bridge.status)
        if status:
            pose = status.get("pose") or {}
            route = " → ".join(
                status.get("completed", [])
                + ([status["current_target"]] if status.get("current_target") else [])
                + status.get("pending", [])
            )
            self.status.set(
                f'Trạng thái: {status.get("state")}\nNav2: {status.get("nav2_available")} · AMCL: {status.get("localization_available")}\nLộ trình: {route or "∅"}\nĐích hiện tại: {status.get("current_target") or "∅"}\n{status.get("reason","")}'
            )
            self.transcript.set(
                f'{status.get("text","")}\nNguồn: {status.get("source","")}'
            )
            event = (
                status.get("command_id"),
                status.get("state"),
                status.get("current_target"),
                status.get("reason"),
            )
            if event != self.last_event:
                self.log.insert(
                    "end",
                    f'{status.get("state")}: {status.get("current_target")} · {status.get("reason")}\n',
                )
                self.log.see("end")
                self.last_event = event
            target = self.catalog.regions.get(status.get("current_target"))
            if target:
                x, y = self.xy(target["goal"]["x"], target["goal"]["y"])
                self.canvas.coords(self.target_item, x - 12, y - 12, x + 12, y + 12)
            else:
                self.canvas.coords(self.target_item, 0, 0, 0, 0)
        live_pose = status.get("pose") if status else None
        pose = (
            (live_pose["x"], live_pose["y"], live_pose["yaw"])
            if live_pose
            else self.bridge.pose
        )
        if pose:
            x, y, yaw = pose
            px, py = self.xy(x, y)
            self.canvas.coords(self.robot_item, px - 8, py - 8, px + 8, py + 8)
            tx, ty = self.xy(x + 0.45 * math.cos(yaw), y + 0.45 * math.sin(yaw))
            self.canvas.coords(self.direction_item, px, py, tx, ty)
            if not self.trace or math.dist(self.trace[-1], (px, py)) > 1:
                self.trace.append((px, py))
                self.trace = self.trace[-1500:]
                if len(self.trace) > 1:
                    self.canvas.coords(
                        self.trace_item, *[v for pair in self.trace for v in pair]
                    )
        if len(self.bridge.path) > 1:
            self.canvas.coords(
                self.path_item, *[v for p in self.bridge.path for v in self.xy(*p)]
            )
        progress = dict(self.bridge.progress)
        if progress.get("stage") == "ASR":
            self.audio_status.set("Nhận dạng WAV tổng hợp: " + progress.get("file", ""))
        elif progress.get("stage") == "NAVIGATION":
            self.audio_status.set("Demo tự động · WAV tổng hợp · Chờ kết quả Nav2")
        elif progress.get("stage") == "COMPLETE":
            self.audio_status.set("Đã hoàn tất demo tự động · WAV tổng hợp · MIC tắt")
        self.job = self.root.after(150, self.tick)

    def close(self):
        self.stop_mic()
        self.root.after_cancel(self.job)
        if self.record_process and self.record_process.poll() is None:
            self.record_process.send_signal(2)
            try:
                self.record_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.record_process.kill()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--regions")
    parser.add_argument("--model")
    parser.add_argument("--record")
    parser.add_argument("--auto-close", type=float, default=0)
    parser.add_argument("--demo-mode", action="store_true")
    args = parser.parse_args()
    share = Path(get_package_share_directory("turtlebot4_voice_nav"))
    regions = args.regions or str(share / "maps/regions.json")
    model = args.model or str(
        Path(regions).resolve().parent.parent / "models/ggml-base.bin"
    )
    rclpy.init()
    node = DashboardBridge()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    root = tk.Tk()
    app = Dashboard(root, node, regions, model, args.record, args.demo_mode)
    signal.signal(signal.SIGINT, lambda *args: root.after(0, app.close))
    if args.auto_close:
        root.after(int(args.auto_close * 1000), app.close)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        thread.join(timeout=2)


if __name__ == "__main__":
    main()
