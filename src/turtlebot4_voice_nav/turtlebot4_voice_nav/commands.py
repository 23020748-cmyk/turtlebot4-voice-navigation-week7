"""Strict Vietnamese semantic route parser, independent of ROS and ASR."""

from dataclasses import dataclass
import json
from pathlib import Path
import re
import unicodedata


def normalize(text):
    text = unicodedata.normalize("NFD", str(text or "").lower().replace("đ", "d"))
    text = "".join(c for c in text if unicodedata.category(c) != "Mn")
    return " ".join(re.sub(r"[^a-z0-9\s]", " ", text).split())


class CommandError(ValueError):
    pass


class UnknownRegion(CommandError):
    pass


@dataclass(frozen=True)
class RouteCommand:
    action: str
    regions: tuple = ()
    explicit_start: bool = False


class RegionCatalog:
    def __init__(self, data):
        self.data = data
        self.regions = {r["id"]: r for r in data["regions"]}
        self.aliases = {}
        for region in data["regions"]:
            for alias in [region["id"], region["name"], *region.get("aliases", [])]:
                key = normalize(alias)
                if key in self.aliases and self.aliases[key] != region["id"]:
                    raise ValueError("Ambiguous alias: " + alias)
                self.aliases[key] = region["id"]

    @classmethod
    def load(cls, path):
        return cls(json.loads(Path(path).read_text(encoding="utf-8")))

    def resolve(self, text):
        key = normalize(text)
        if key not in self.aliases:
            raise UnknownRegion(f"Khu vực không tồn tại: {text}")
        return self.aliases[key]

    def parse(self, text):
        phrase = normalize(text)
        phrase = re.sub(r"^(?:robot|minibot)(?: oi)?\s+", "", phrase)
        if phrase in ("dung", "dung lai", "stop", "huy", "huy lenh"):
            return RouteCommand("STOP")
        if not phrase:
            raise CommandError("Chưa có câu lệnh")
        if re.search(r"\b(khong|chua|dung di)\b", phrase):
            raise CommandError("Không thực thi câu phủ định")
        if re.search(r"\b(co nen|co phai|tai sao|vi sao|o dau|lam the nao)\b", phrase):
            raise CommandError("Câu hỏi chưa phải lệnh di chuyển")
        # Locate directional intent after polite/salutation prefixes; region
        # references remain exact. Never repair a guessed/unknown region name.
        multi = re.search(r"\b(?:di(?: chuyen)?|chuyen) tu (.+)$", phrase)
        single = re.search(r"\b(?:den|toi) (.+)$", phrase)
        if multi:
            refs = re.split(r"\s+(?:roi\s+)?(?:qua|den|toi)\s+", multi.group(1))
            if len(refs) < 2:
                raise CommandError("Cần cả khu vực xuất phát và đích đến")
            explicit = True
        elif single:
            refs = re.split(r"\s+(?:roi\s+)?(?:qua|den|toi)\s+", single.group(1))
            explicit = False
        else:
            raise CommandError(
                "Cú pháp: Đi đến <vùng>, hoặc Đi từ <vùng> qua <vùng> rồi đến <vùng>"
            )
        if len(refs) > 8:
            raise CommandError("Tối đa 8 điểm trong một lệnh")
        route = []
        for ref in refs:
            region = self.resolve(ref)
            if not route or route[-1] != region:
                route.append(region)
        return RouteCommand("NAVIGATE", tuple(route), explicit)


class RouteSequencer:
    """No next waypoint is released until the previous goal is verified."""

    def __init__(self):
        self.pending = []
        self.current = None
        self.completed = []
        self.state = "IDLE"

    def begin(self, command):
        if self.current:
            raise CommandError("Robot đang chạy; hãy dừng trước khi giao lệnh mới")
        self.pending = list(command.regions)
        self.completed = []
        self.state = "RUNNING"
        return self.advance()

    def advance(self):
        self.current = self.pending.pop(0) if self.pending else None
        if self.current is None:
            self.state = "SUCCEEDED"
        return self.current

    def result(self, succeeded, verified=True):
        if not self.current:
            return None
        if not succeeded or not verified:
            self.pending = []
            self.current = None
            self.state = "FAILED"
            return None
        self.completed.append(self.current)
        return self.advance()

    def stop(self):
        self.pending = []
        self.current = None
        self.state = "CANCELED"
