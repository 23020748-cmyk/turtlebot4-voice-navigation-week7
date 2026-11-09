"""ROS map geometry and conservative, seeded geodesic segmentation.

Markers/names are assigned manually. The partition is a geodesic Voronoi
partition of navigable space, NOT automatic architectural room recognition.
"""

from dataclasses import dataclass
import hashlib
import heapq
import json
import math
from pathlib import Path
import cv2
import numpy as np
import yaml


@dataclass
class GridMap:
    image: np.ndarray
    resolution: float
    origin: tuple
    free: np.ndarray
    clearance: np.ndarray
    yaml_path: Path

    @classmethod
    def load(cls, path):
        path = Path(path).resolve()
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        if data.get("mode", "trinary") != "trinary":
            raise ValueError("Only ROS trinary maps are supported")
        image_path = path.parent / data["image"]
        image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError("Cannot read map image: " + str(image_path))
        resolution = float(data["resolution"])
        origin = tuple(map(float, data["origin"]))
        if resolution <= 0 or len(origin) != 3:
            raise ValueError("Invalid map geometry")
        probability = image.astype(np.float64) / 255
        if not int(data.get("negate", 0)):
            probability = 1 - probability
        ft, ot = float(data["free_thresh"]), float(data["occupied_thresh"])
        if not 0 <= ft < ot <= 1:
            raise ValueError("Invalid occupancy thresholds")
        free = probability < ft
        # Pad as occupied so an image edge never provides artificial clearance.
        distance = (
            cv2.distanceTransform(
                np.pad(free.astype(np.uint8), 1), cv2.DIST_L2, cv2.DIST_MASK_PRECISE
            )[1:-1, 1:-1]
            * resolution
        )
        return cls(image, resolution, origin, free, distance, path)

    def pixel_to_world(self, col, row):
        x = (float(col) + 0.5) * self.resolution
        y = (self.image.shape[0] - float(row) - 0.5) * self.resolution
        ox, oy, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        return ox + c * x - s * y, oy + s * x + c * y

    def world_to_pixel(self, x, y):
        ox, oy, yaw = self.origin
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = float(x) - ox, float(y) - oy
        col = math.floor((c * dx + s * dy) / self.resolution)
        row = self.image.shape[0] - 1 - math.floor((-s * dx + c * dy) / self.resolution)
        return int(col), int(row)

    def contains(self, col, row):
        return 0 <= row < self.image.shape[0] and 0 <= col < self.image.shape[1]

    def is_safe(self, x, y, margin=0.28):
        col, row = self.world_to_pixel(x, y)
        return (
            self.contains(col, row)
            and bool(self.free[row, col])
            and float(self.clearance[row, col]) >= margin
        )


def segment(grid, seeds, min_clearance=0.28):
    safe = grid.free & (grid.clearance >= min_clearance)
    labels = np.zeros(grid.image.shape, np.uint16)
    distances = np.full(grid.image.shape, np.inf)
    heap = []
    regions = []
    for index, seed in enumerate(seeds, 1):
        x, y = seed["seed"]
        col, row = grid.world_to_pixel(x, y)
        if not grid.is_safe(x, y, min_clearance):
            raise ValueError(f'Unsafe seed for {seed["id"]}: {(x,y)}')
        if labels[row, col]:
            raise ValueError("Duplicate seed pixels")
        labels[row, col] = index
        distances[row, col] = 0
        heapq.heappush(heap, (0, index, row, col))
        regions.append(dict(seed, index=index))
    while heap:
        distance, label, row, col = heapq.heappop(heap)
        if distance != distances[row, col] or label != labels[row, col]:
            continue
        for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0)):
            rr, cc = row + dr, col + dc
            if not (
                0 <= rr < safe.shape[0] and 0 <= cc < safe.shape[1] and safe[rr, cc]
            ):
                continue
            nd = distance + grid.resolution
            if nd < distances[rr, cc] - 1e-10:
                distances[rr, cc] = nd
                labels[rr, cc] = label
                heapq.heappush(heap, (nd, label, rr, cc))
    for region in regions:
        mask = labels == region["index"]
        rows, cols = np.where(mask)
        if not len(rows):
            raise ValueError("Empty region")
        region["area_m2"] = round(len(rows) * grid.resolution**2, 4)
        region["centroid_world"] = list(
            grid.pixel_to_world(float(cols.mean()), float(rows.mean()))
        )
        # A raw centroid can fall in a wall; select the safe seed as goal instead.
        gx, gy = region.pop("seed")
        col, row = grid.world_to_pixel(gx, gy)
        region["goal"] = {
            "x": gx,
            "y": gy,
            "yaw": float(region.pop("yaw", 0)),
            "clearance_m": round(float(grid.clearance[row, col]), 3),
        }
    return labels, regions


def write_segmentation(map_yaml, seeds_file, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    grid = GridMap.load(map_yaml)
    config = yaml.safe_load(Path(seeds_file).read_text(encoding="utf-8"))
    labels, regions = segment(
        grid, config["regions"], config.get("min_clearance_m", 0.28)
    )
    cv2.imwrite(str(output / "region_labels.png"), labels)
    preview = cv2.cvtColor(grid.image, cv2.COLOR_GRAY2RGB)
    colors = ((75, 160, 230), (230, 150, 65), (100, 190, 125))
    for region, color in zip(regions, colors):
        preview[labels == region["index"]] = color
    # Saved PNG uses RGB through Pillow so channel names are unambiguous.
    from PIL import Image, ImageDraw, ImageFont

    image = Image.fromarray(preview).resize(
        (grid.image.shape[1] * 2, grid.image.shape[0] * 2)
    )
    draw = ImageDraw.Draw(image)
    font = ImageFont.truetype(
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 19
    )
    for region in regions:
        col, row = grid.world_to_pixel(region["goal"]["x"], region["goal"]["y"])
        x, y = col * 2, row * 2
        draw.ellipse(
            (x - 7, y - 7, x + 7, y + 7), fill="white", outline="black", width=2
        )
        draw.text(
            (x + 10, y - 22),
            region["id"],
            font=font,
            fill="black",
            stroke_width=2,
            stroke_fill="white",
        )
    image.save(output / "segmented_map.png")
    image_path = (
        grid.yaml_path.parent / yaml.safe_load(grid.yaml_path.read_text())["image"]
    )
    data = {
        "schema_version": 1,
        "frame": "map",
        "method": "manual semantic seeds + 4-connected geodesic Voronoi of free cells with obstacle clearance",
        "map_yaml": str(grid.yaml_path.relative_to(output.resolve())),
        "labels_image": "region_labels.png",
        "min_clearance_m": config.get("min_clearance_m", 0.28),
        "map_sha256": hashlib.sha256(image_path.read_bytes()).hexdigest(),
        "known_obstacles": config.get("known_obstacles", []),
        "coordinate_convention": "pixel centers; image y down, ROS map y up; origin yaw applied",
        "regions": regions,
        "segmented_cells": int((labels > 0).sum()),
        "free_cells": int(grid.free.sum()),
        "unknown_or_occupied_assigned": int(((labels > 0) & ~grid.free).sum()),
    }
    (output / "regions.json").write_text(
        json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return data
