"""Async Nav2 action client with verified, strictly ordered semantic goals."""

import csv
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_msgs.msg import TFMessage
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State

from .commands import RegionCatalog, RouteSequencer, CommandError
from .mapping import GridMap


class VoiceNavigation(Node):
    def __init__(self):
        super().__init__("semantic_voice_navigation")
        share = Path(get_package_share_directory("turtlebot4_voice_nav"))
        for name, value in [
            ("regions_file", str(share / "maps/regions.json")),
            ("results_dir", str(Path.home() / ".ros/voice_navigation")),
            ("initialize_pose", False),
            ("goal_timeout_sec", 180.0),
            ("min_speech_confidence", 0.5),
        ]:
            self.declare_parameter(name, value)
        catalog_path = Path(self.get_parameter("regions_file").value)
        self.declare_parameter("require_ground_truth", False)
        self.catalog = RegionCatalog.load(catalog_path)
        self.grid = GridMap.load(catalog_path.parent / self.catalog.data["map_yaml"])
        self.seq = RouteSequencer()
        self.state = "WAITING_FOR_NAV2"
        self.active = None
        self.epoch = 0
        self.serial = 0
        self.pose = None
        self.pose_stamp = 0.0
        self.scan_min = None
        self.current_id = ""
        self.last_text = ""
        self.last_source = ""
        self.ground_truth = None
        self.ground_truth_stamp = 0.0
        self.last_reason = "Đợi Nav2 và AMCL"
        self.distance_remaining = None
        self.last_trajectory = -100
        self.goal_began = 0
        self.last_seed = 0
        self.results = Path(self.get_parameter("results_dir").value)
        self.results.mkdir(parents=True, exist_ok=True)
        self.status_pub = self.create_publisher(
            String,
            "/voice_nav/status",
            QoSProfile(
                depth=10,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/voice_nav/regions",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.stop_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.seed_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.command_sub = self.create_subscription(
            String, "/voice_nav/command", self.on_command, 10
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self.on_pose, 10
        )
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.on_scan, qos_profile_sensor_data
        )
        self.truth_sub = self.create_subscription(
            TFMessage,
            "/voice_nav/ground_truth",
            self.on_ground_truth,
            qos_profile_sensor_data,
        )
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.lifecycle_client = self.create_client(GetState, "/bt_navigator/get_state")
        self.nav2_active = False
        self.lifecycle_pending = False
        self.last_lifecycle_poll = -100.0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.heartbeat = self.create_timer(0.5, self.tick)
        self.markers()
        self.emit("STARTUP")

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def snapshot(self):
        return dict(
            state=self.state,
            command_id=self.current_id,
            text=self.last_text,
            source=self.last_source,
            current_target=self.seq.current,
            pending=self.seq.pending,
            completed=self.seq.completed,
            pose=self.pose,
            ground_truth=self.ground_truth,
            distance_remaining=self.distance_remaining,
            reason=self.last_reason,
            nav2_available=self.nav2_ready(),
            localization_available=self.pose is not None,
            frame="map",
            sim_time=self.now_seconds(),
        )

    def nav2_ready(self):
        return self.nav2_active and self.client.server_is_ready()

    def on_lifecycle_state(self, future):
        self.lifecycle_pending = False
        try:
            self.nav2_active = (
                future.result().current_state.id == State.PRIMARY_STATE_ACTIVE
            )
        except Exception:
            self.nav2_active = False

    def on_ground_truth(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id != "turtlebot4":
                continue
            p = transform.transform.translation
            q = transform.transform.rotation
            self.ground_truth = {
                "x": p.x,
                "y": p.y,
                "yaw": math.atan2(
                    2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
                ),
            }
            self.ground_truth_stamp = (
                transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
            )

    def emit(self, event, **values):
        data = dict(self.snapshot(), event=event, **values)
        self.status_pub.publish(String(data=json.dumps(data, ensure_ascii=False)))
        data["wall_time_utc"] = datetime.now(timezone.utc).isoformat()
        with (self.results / "events.jsonl").open("a", encoding="utf-8") as f:
            f.write(json.dumps(data, ensure_ascii=False) + "\n")
        self.get_logger().info(
            f'{event}: {data.get("command_id")} {data.get("current_target")} {data.get("reason")}'
        )

    def on_pose(self, msg):
        p = msg.pose.pose
        q = p.orientation
        self.pose = {
            "x": p.position.x,
            "y": p.position.y,
            "yaw": math.atan2(
                2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
            ),
        }
        self.pose_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.record_pose()

    def record_pose(self):
        if self.pose is None:
            return
        now = self.now_seconds()
        if now - self.last_trajectory < 0.25:
            return
        self.last_trajectory = now
        x, y = self.pose["x"], self.pose["y"]
        col, row = self.grid.world_to_pixel(x, y)
        clearance = (
            float(self.grid.clearance[row, col])
            if self.grid.contains(col, row)
            else 0.0
        )
        path = self.results / "trajectory.csv"
        exists = path.exists()
        record = dict(
            timestamp=now,
            x=x,
            y=y,
            heading=self.pose["yaw"],
            target=self.seq.current or "",
            command_id=self.current_id,
            state=self.state,
            map_clearance_m=clearance,
            min_laser_m=self.scan_min,
            pose_source="map TF (AMCL + wheel odometry)",
        )
        record["ground_truth_x"] = self.ground_truth["x"] if self.ground_truth else None
        record["ground_truth_y"] = self.ground_truth["y"] if self.ground_truth else None
        with path.open("a", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(record))
            if not exists:
                writer.writeheader()
            writer.writerow(record)

    def on_scan(self, msg):
        ranges = [
            r
            for r in msg.ranges
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max
        ]
        self.scan_min = min(ranges) if ranges else None

    def on_command(self, msg):
        try:
            try:
                payload = json.loads(msg.data)
            except json.JSONDecodeError:
                payload = {"text": msg.data, "source": "text_ros_topic"}
            if not isinstance(payload, dict):
                raise CommandError("Dữ liệu lệnh phải là text hoặc JSON object")
            text = str(payload.get("text", ""))
            cid = str(payload.get("command_id") or uuid.uuid4())
            source = str(payload.get("source", "text_ros_topic"))
            command = self.catalog.parse(text)
            confidence = payload.get("confidence", 1)
            if confidence is not None:
                confidence = float(confidence)
                if (
                    not math.isfinite(confidence)
                    or not 0 <= confidence <= 1
                    or confidence
                    < float(self.get_parameter("min_speech_confidence").value)
                ):
                    raise CommandError("Confidence lời nói chưa đủ; hãy nói lại")
            if command.action == "STOP":
                self.epoch += 1
                if self.active:
                    self.active.cancel_goal_async()
                self.active = None
                self.seq.stop()
                self.state = "CANCELED"
                self.stop_pub.publish(Twist())
                self.current_id = cid
                self.last_text = text
                self.last_source = source
                self.last_reason = "Đã hủy đích và dừng Nav2"
                self.emit("CANCELED")
                return
            if self.seq.current:
                raise CommandError("Robot đang chạy. Hãy dừng hoặc chờ kết thúc")
            if (
                not self.nav2_ready()
                or self.pose is None
                or abs(self.now_seconds() - self.pose_stamp) > 5
            ):
                raise CommandError("Nav2 hoặc định vị AMCL chưa sẵn sàng")
            for region_id in command.regions:
                goal = self.catalog.regions[region_id]["goal"]
                if not self.grid.is_safe(
                    goal["x"], goal["y"], self.catalog.data["min_clearance_m"]
                ):
                    raise CommandError("Điểm đích không an toàn: " + region_id)
            self.epoch += 1
            self.current_id = cid
            self.last_text = text
            self.last_source = source
            self.state = "RUNNING"
            self.last_reason = "Thực hiện lần lượt: " + " → ".join(command.regions)
            self.seq.begin(command)
            self.emit(
                "COMMAND_ACCEPTED",
                route=list(command.regions),
                explicit_start=command.explicit_start,
            )
            self.send_current()
        except (ValueError, TypeError, KeyError) as exc:
            rejected_id = locals().get("cid", str(uuid.uuid4()))
            if not self.seq.current:
                self.state = "REJECTED"
                self.current_id = rejected_id
                self.last_text = locals().get("text", msg.data)
                self.last_source = locals().get("source", "unknown")
            self.last_reason = str(exc)
            self.emit(
                "COMMAND_REJECTED",
                command_id=rejected_id,
                rejected_command_id=rejected_id,
            )

    def send_current(self):
        if not self.seq.current:
            self.state = "SUCCEEDED"
            self.last_reason = "Đã đến tất cả khu vực yêu cầu"
            self.emit("ROUTE_SUCCEEDED")
            return
        self.serial += 1
        token = (self.epoch, self.serial)
        region = self.catalog.regions[self.seq.current]
        goal = region["goal"]
        request = NavigateToPose.Goal()
        request.pose = PoseStamped()
        request.pose.header.frame_id = "map"
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.position.x = float(goal["x"])
        request.pose.pose.position.y = float(goal["y"])
        request.pose.pose.orientation.z = math.sin(goal["yaw"] / 2)
        request.pose.pose.orientation.w = math.cos(goal["yaw"] / 2)
        self.goal_began = time.monotonic()
        self.distance_remaining = None
        self.emit("GOAL_SENT", goal=goal, region_id=self.seq.current)
        future = self.client.send_goal_async(
            request, feedback_callback=lambda f: self.on_feedback(f, token)
        )
        future.add_done_callback(lambda f: self.on_goal_response(f, token))

    def on_feedback(self, feedback, token):
        if token != (self.epoch, self.serial):
            return
        self.distance_remaining = float(feedback.feedback.distance_remaining)

    def on_goal_response(self, future, token):
        try:
            handle = future.result()
        except Exception as exc:
            if token == (self.epoch, self.serial):
                self.fail("Lỗi gửi Nav2: " + str(exc))
            return
        if token != (self.epoch, self.serial):
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.fail("Nav2 từ chối goal")
            return
        self.active = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_result(f, token))

    def on_result(self, future, token):
        if token != (self.epoch, self.serial):
            return
        try:
            result = future.result()
            target = self.seq.current
            goal = self.catalog.regions[target]["goal"]
            error = (
                math.hypot(self.pose["x"] - goal["x"], self.pose["y"] - goal["y"])
                if self.pose
                else float("inf")
            )
            fresh = (
                self.pose is not None and abs(self.now_seconds() - self.pose_stamp) <= 5
            )
            truth_error = (
                math.hypot(
                    self.ground_truth["x"] - goal["x"],
                    self.ground_truth["y"] - goal["y"],
                )
                if self.ground_truth
                else None
            )
            truth_fresh = (
                self.ground_truth is not None
                and abs(self.now_seconds() - self.ground_truth_stamp) <= 5
            )
            truth_verified = (
                truth_error is not None and truth_error <= 0.5 and truth_fresh
            )
            verified = (
                result.status == GoalStatus.STATUS_SUCCEEDED
                and error <= 0.5
                and fresh
                and (
                    truth_verified
                    or not self.get_parameter("require_ground_truth").value
                )
            )
            self.active = None
            self.emit(
                "GOAL_RESULT",
                region_id=target,
                nav2_status=int(result.status),
                position_error_m=error,
                ground_truth_error_m=truth_error,
                ground_truth_fresh=truth_fresh,
                amcl_fresh=fresh,
                verified=verified,
            )
            if not verified:
                self.fail("Goal chưa được xác minh bằng AMCL")
                return
            self.seq.result(True, True)
            self.send_current()
        except Exception as exc:
            self.fail("Lỗi kết quả Nav2: " + str(exc))

    def fail(self, reason):
        self.epoch += 1
        if self.active:
            self.active.cancel_goal_async()
        self.active = None
        self.seq.result(False)
        self.state = "FAILED"
        self.last_reason = reason
        self.stop_pub.publish(Twist())
        self.emit("ROUTE_FAILED")

    def tick(self):
        if (
            not self.lifecycle_pending
            and time.monotonic() - self.last_lifecycle_poll > 2
            and self.lifecycle_client.service_is_ready()
        ):
            self.last_lifecycle_poll = time.monotonic()
            self.lifecycle_pending = True
            self.lifecycle_client.call_async(GetState.Request()).add_done_callback(
                self.on_lifecycle_state
            )
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
            t = transform.transform.translation
            q = transform.transform.rotation
            self.pose = {
                "x": t.x,
                "y": t.y,
                "yaw": math.atan2(
                    2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)
                ),
            }
            self.pose_stamp = (
                transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
            )
            self.record_pose()
        except TransformException:
            pass
        if (
            self.pose is None
            and self.get_parameter("initialize_pose").value
            and time.monotonic() - self.last_seed > 2
        ):
            self.last_seed = time.monotonic()
            seed = PoseWithCovarianceStamped()
            seed.header.frame_id = "map"
            seed.header.stamp = self.get_clock().now().to_msg()
            seed.pose.pose.orientation.w = 1.0
            seed.pose.covariance[0] = seed.pose.covariance[7] = 0.04
            seed.pose.covariance[35] = 0.01
            self.seed_pub.publish(seed)
        if (
            self.state == "WAITING_FOR_NAV2"
            and self.nav2_ready()
            and self.pose is not None
        ):
            self.state = "IDLE"
            self.last_reason = "Sẵn sàng nhận lệnh tiếng Việt"
            self.emit("READY")
        if self.state == "RUNNING" and time.monotonic() - self.goal_began > float(
            self.get_parameter("goal_timeout_sec").value
        ):
            self.fail("Quá thời gian thực hiện goal")
        self.status_pub.publish(
            String(data=json.dumps(self.snapshot(), ensure_ascii=False))
        )

    def markers(self):
        array = MarkerArray()
        for i, r in enumerate(self.catalog.regions.values()):
            goal = r["goal"]
            m = Marker()
            m.header.frame_id = "map"
            m.ns = "semantic_regions"
            m.id = i * 2
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(goal["x"])
            m.pose.position.y = float(goal["y"])
            m.pose.position.z = 0.1
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.3
            m.scale.z = 0.15
            m.color.a = 1.0
            m.color.r, m.color.g, m.color.b = (
                (0.2, 0.5, 1.0),
                (1.0, 0.5, 0.1),
                (0.2, 0.9, 0.4),
            )[i % 3]
            array.markers.append(m)
            text = Marker()
            text.header.frame_id = "map"
            text.ns = "semantic_regions"
            text.id = i * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = m.pose.position.x
            text.pose.position.y = m.pose.position.y
            text.pose.position.z = 0.5
            text.pose.orientation.w = 1.0
            text.scale.z = 0.25
            text.color.a = 1.0
            text.color.r = text.color.g = text.color.b = 1.0
            text.text = r["id"] + " " + r["name"]
            array.markers.append(text)
        self.marker_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.active:
            node.active.cancel_goal_async()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
