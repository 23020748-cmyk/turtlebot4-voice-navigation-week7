import math
from pathlib import Path
import tempfile
import unittest
import cv2
import numpy as np
import yaml
from turtlebot4_voice_nav.mapping import GridMap, segment

ROOT = Path(__file__).resolve().parents[1]


class TestMapping(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.grid = GridMap.load(ROOT / "maps/navigation_map.yaml")

    def test_pixel_roundtrip(self):
        for col, row in ((0, 0), (250, 300), (458, 440)):
            self.assertEqual(
                self.grid.world_to_pixel(*self.grid.pixel_to_world(col, row)),
                (col, row),
            )

    def test_rotated_origin(self):
        g = GridMap(
            np.zeros((4, 6)),
            0.2,
            (1, -2, math.pi / 3),
            np.ones((4, 6), bool),
            np.ones((4, 6)),
            Path("unused"),
        )
        for col, row in ((0, 0), (5, 3), (2, 1)):
            self.assertEqual(g.world_to_pixel(*g.pixel_to_world(col, row)), (col, row))

    def test_known_goals_safe(self):
        for x, y in ((-3.5, 0.5), (-4, 2.5), (-3, 4.5)):
            self.assertTrue(self.grid.is_safe(x, y, 0.28))

    def test_outside_rejected(self):
        self.assertFalse(self.grid.is_safe(100, 100))

    def test_nonfree_not_segmented(self):
        seeds = yaml.safe_load((ROOT / "config/region_seeds.yaml").read_text())[
            "regions"
        ]
        labels, regions = segment(self.grid, seeds, 0.28)
        self.assertEqual(int(((labels > 0) & ~self.grid.free).sum()), 0)
        self.assertEqual(set(np.unique(labels)), {0, 1, 2, 3})

    def test_invalid_seed(self):
        with self.assertRaises(ValueError):
            segment(self.grid, [{"id": "Z", "name": "invalid", "seed": [100, 100]}])

    def test_unexplored_gray_is_blocked(self):
        self.assertFalse(self.grid.free[self.grid.image == 205].any())

    def test_known_chair_is_blocked(self):
        self.assertFalse(self.grid.is_safe(-2, 3.5))

    def test_threshold_and_negate(self):
        with tempfile.TemporaryDirectory() as temp:
            folder = Path(temp)
            cv2.imwrite(str(folder / "m.pgm"), np.array([[0, 205, 255]], np.uint8))
            config = {
                "image": "m.pgm",
                "resolution": 1,
                "origin": [0, 0, 0],
                "occupied_thresh": 0.65,
                "free_thresh": 0.25,
                "negate": 0,
            }
            (folder / "m.yaml").write_text(yaml.safe_dump(config))
            g = GridMap.load(folder / "m.yaml")
            self.assertEqual(g.free.tolist(), [[False, True, True]])
            config["negate"] = 1
            (folder / "m.yaml").write_text(yaml.safe_dump(config))
            g = GridMap.load(folder / "m.yaml")
            self.assertEqual(g.free.tolist(), [[True, False, False]])
