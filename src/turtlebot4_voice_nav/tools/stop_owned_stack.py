"""Stop only process groups created and recorded by these assignment tools."""

import json
import os
from pathlib import Path
import signal
import time

ROOT = Path(__file__).resolve().parents[1]


def descendants(root_pid):
    children = {}
    for status in Path("/proc").glob("[0-9]*/status"):
        try:
            fields = dict(
                line.split(":", 1)
                for line in status.read_text().splitlines()
                if ":" in line
            )
            children.setdefault(int(fields["PPid"]), []).append(int(status.parent.name))
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    pending = [root_pid]
    owned = []
    while pending:
        pid = pending.pop()
        owned.append(pid)
        pending.extend(children.get(pid, []))
    return owned


def signal_processes(pids, sig):
    for pid in reversed(pids):
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            pass


for name in (
    "integration_process.json",
    "dashboard_process.json",
    "navigation_process.json",
    "simulation_process.json",
):
    path = ROOT / "results" / name
    if not path.exists():
        continue
    data = json.loads(path.read_text())
    pid = data["pid"]
    commandline = Path(f"/proc/{pid}/cmdline")
    if not commandline.exists():
        continue
    command = commandline.read_bytes().replace(b"\0", b" ").decode(errors="replace")
    if not any(x in command for x in ("turtlebot4", "integration_demo.py")):
        continue
    owned = descendants(pid)
    try:
        signal_processes(owned, signal.SIGINT)
        print("Stopped owned process:", pid, flush=True)
    except ProcessLookupError:
        pass
    time.sleep(3)
    signal_processes(owned, signal.SIGTERM)
    time.sleep(2)
    signal_processes(owned, signal.SIGKILL)
