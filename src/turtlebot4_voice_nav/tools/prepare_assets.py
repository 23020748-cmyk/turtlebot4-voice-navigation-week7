"""Copy the existing SLAM map, preserving the original package unchanged."""

from pathlib import Path
import shutil
import sys
import yaml
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from turtlebot4_voice_nav.mapping import GridMap

source = ROOT.parent / "turtlebot4_slam_nav2/maps"
target = ROOT / "maps/original"
target.mkdir(parents=True, exist_ok=True)
for name in ("map.pgm", "map.yaml"):
    if not (target / name).exists():
        shutil.copy2(source / name, target / name)
navigation_map = yaml.safe_load((target / "map.yaml").read_text())
navigation_map["image"] = "navigation_map.pgm"
grid = GridMap.load(target / "map.yaml")
navigation_image = Image.fromarray(grid.image.copy())
obstacle_draw = ImageDraw.Draw(navigation_image)
config = yaml.safe_load((ROOT / "config/region_seeds.yaml").read_text())
for obstacle in config.get("known_obstacles", []):
    xmin, ymin, xmax, ymax = obstacle["bounds"]
    corners = [
        grid.world_to_pixel(xmin, ymin),
        grid.world_to_pixel(xmin, ymax),
        grid.world_to_pixel(xmax, ymax),
        grid.world_to_pixel(xmax, ymin),
    ]
    obstacle_draw.polygon(corners, fill=255 if navigation_map.get("negate", 0) else 0)
navigation_image.save(ROOT / "maps/navigation_map.pgm")
# PGM value 205 is unexplored. Its occupancy probability is 50/255.
# 0.196 leaves that value unknown in Nav2's trinary map loader.
navigation_map["free_thresh"] = 0.196
(ROOT / "maps/navigation_map.yaml").write_text(
    yaml.safe_dump(navigation_map, sort_keys=False)
)
image = (
    Image.open(target / "map.pgm")
    .convert("RGB")
    .resize((grid.image.shape[1] * 2, grid.image.shape[0] * 2))
)
draw = ImageDraw.Draw(image)
for name, x, y in [("START", 0, 0)] + [
    (r["id"], *r["seed"]) for r in config["regions"]
]:
    col, row = grid.world_to_pixel(x, y)
    col, row = col * 2, row * 2
    draw.ellipse((col - 6, row - 6, col + 6, row + 6), fill="red")
    draw.text((col + 10, row - 10), name, fill="red")
out = ROOT / "maps/map_preview.png"
image.save(out)
print(out)
