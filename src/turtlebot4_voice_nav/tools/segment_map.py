from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from turtlebot4_voice_nav.mapping import write_segmentation

data = write_segmentation(
    ROOT / "maps/navigation_map.yaml", ROOT / "config/region_seeds.yaml", ROOT / "maps"
)
print({r["id"]: r["goal"] for r in data["regions"]})
print("Assigned non-free cells:", data["unknown_or_occupied_assigned"])
