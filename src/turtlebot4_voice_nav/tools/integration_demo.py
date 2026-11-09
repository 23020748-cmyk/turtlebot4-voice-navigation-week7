"""Actual WAV ASR -> ROS command -> NavigateToPose -> observed completion.

No text is substituted for a decoder output. One explicit text-only C goal is
labelled as such to validate all three region goals independently of ASR.
"""

import json
import os
from pathlib import Path
import sys
import time
import uuid
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from turtlebot4_voice_nav.commands import RegionCatalog, CommandError
from turtlebot4_voice_nav.speech import VietnameseRecognizer, Transcript


class Harness(Node):
    def __init__(self):
        super().__init__("voice_nav_integration_demo")
        self.status = {}
        self.events = []
        self.pub = self.create_publisher(String, "/voice_nav/command", 10)
        self.progress = self.create_publisher(String, "/voice_nav/demo", 10)
        self.sub = self.create_subscription(
            String,
            "/voice_nav/status",
            self.callback,
            QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

    def callback(self, msg):
        data = json.loads(msg.data)
        self.status = data
        if data.get("event"):
            self.events.append(data)

    def wait(self, predicate, timeout):
        until = time.monotonic() + timeout
        while time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False


def main():
    rclpy.init()
    node = Harness()
    out = ROOT / "results"
    rows = []
    catalog = RegionCatalog.load(ROOT / "maps/regions.json")
    (out / "integration_process.json").write_text(
        json.dumps({"pid": os.getpid(), "script": __file__})
    )
    try:
        if not node.wait(
            lambda: node.status.get("nav2_available")
            and node.status.get("localization_available"),
            60,
        ):
            raise RuntimeError("Nav2/AMCL not ready")
        recognizer = VietnameseRecognizer(ROOT / "models/ggml-base.bin")
        expected = {
            "01_area_a": ["A"],
            "02_lab": ["B"],
            "03_corridor": ["C"],
            "04_a_to_b": ["A", "B"],
            "05_unknown": None,
        }
        manifest = json.loads(
            (ROOT / "demo/audio/manifest.json").read_text(encoding="utf-8")
        )
        for sample in manifest:
            node.progress.publish(
                String(
                    data=json.dumps(
                        {
                            "stage": "ASR",
                            "file": sample["file"],
                            "source": sample["source"],
                        }
                    )
                )
            )
            audio_start = time.monotonic()
            transcript, metrics = recognizer.transcribe_wav(
                ROOT / "demo/audio" / sample["file"], sample["source"]
            )
            node.progress.publish(
                String(data=json.dumps({"stage": "NAVIGATION", "file": sample["file"]}))
            )
            try:
                actual_route = list(catalog.parse(transcript.text).regions)
            except CommandError:
                actual_route = None
            cid = transcript.command_id
            node.pub.publish(String(data=transcript.to_json()))

            def finished():
                return any(
                    e.get("command_id") == cid
                    and e.get("event")
                    in ("ROUTE_SUCCEEDED", "ROUTE_FAILED", "COMMAND_REJECTED")
                    for e in node.events
                )

            observed = node.wait(finished, 650)
            relevant = [e for e in node.events if e.get("command_id") == cid]
            final = next(
                (
                    e
                    for e in reversed(relevant)
                    if e.get("event")
                    in ("ROUTE_SUCCEEDED", "ROUTE_FAILED", "COMMAND_REJECTED")
                ),
                None,
            )
            goals = [
                e["region_id"]
                for e in relevant
                if e.get("event") == "GOAL_RESULT" and e.get("verified")
            ]
            passed = bool(
                observed
                and final
                and (
                    (
                        expected[sample["id"]] is None
                        and final["event"] == "COMMAND_REJECTED"
                    )
                    or (
                        actual_route == expected[sample["id"]]
                        and goals == expected[sample["id"]]
                        and final["event"] == "ROUTE_SUCCEEDED"
                    )
                )
            )
            row = {
                "id": sample["id"],
                "source": sample["source"],
                "expected_text": sample["expected_text"],
                "transcript": transcript.text,
                "expected_route": expected[sample["id"]],
                "parsed_route": actual_route,
                "verified_goals": goals,
                "final_event": final.get("event") if final else "TIMEOUT",
                "passed": passed,
                "audio_start_monotonic": audio_start,
                "elapsed_seconds": time.monotonic() - audio_start,
                "asr_metrics": metrics,
                "command_id": cid,
            }
            rows.append(row)
            (out / "integration_results.json").write_text(
                json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8"
            )
            print(json.dumps(row, ensure_ascii=False), flush=True)
            if not observed:
                raise RuntimeError("Navigation timed out")
            # Independent validation of goal C: do not label this as voice ASR.
            if sample["id"] == "03_corridor" and not goals:
                cid = str(uuid.uuid4())
                text = "Đi đến hành lang"
                node.pub.publish(
                    String(
                        data=Transcript(
                            text, None, "TEXT_NAVIGATION_VALIDATION", cid
                        ).to_json()
                    )
                )
                if not node.wait(
                    lambda: any(
                        e.get("command_id") == cid
                        and e.get("event")
                        in ("ROUTE_SUCCEEDED", "ROUTE_FAILED", "COMMAND_REJECTED")
                        for e in node.events
                    ),
                    650,
                ):
                    raise RuntimeError("C validation timeout")
                relevant = [e for e in node.events if e.get("command_id") == cid]
                goals = [
                    e["region_id"]
                    for e in relevant
                    if e.get("event") == "GOAL_RESULT" and e.get("verified")
                ]
                rows.append(
                    {
                        "id": "C_text_goal_validation",
                        "source": "TEXT_NAVIGATION_VALIDATION",
                        "transcript": text,
                        "expected_route": ["C"],
                        "verified_goals": goals,
                        "passed": goals == ["C"],
                        "command_id": cid,
                    }
                )
                (out / "integration_results.json").write_text(
                    json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8"
                )
                print("C text navigation verified:", goals, flush=True)
            node.wait(lambda: False, 1)
    finally:
        if rclpy.ok():
            node.progress.publish(String(data=json.dumps({"stage": "COMPLETE"})))
        (out / "integration_events.json").write_text(
            json.dumps(node.events, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
