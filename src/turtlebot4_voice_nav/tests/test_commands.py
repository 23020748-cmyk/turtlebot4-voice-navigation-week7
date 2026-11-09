import unittest
from pathlib import Path
from turtlebot4_voice_nav.commands import (
    RegionCatalog,
    CommandError,
    UnknownRegion,
    RouteSequencer,
    normalize,
)

ROOT = Path(__file__).resolve().parents[1]


class TestCommands(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.catalog = RegionCatalog.load(ROOT / "maps/regions.json")

    def test_a(self):
        self.assertEqual(self.catalog.parse("Đi đến khu vực A.").regions, ("A",))

    def test_lab(self):
        self.assertEqual(self.catalog.parse("Đi đến phòng thí nghiệm.").regions, ("B",))

    def test_corridor(self):
        self.assertEqual(self.catalog.parse("Đi đến hành lang").regions, ("C",))

    def test_from_to(self):
        result = self.catalog.parse("Di chuyển từ khu vực A đến khu vực B.")
        self.assertEqual(result.regions, ("A", "B"))
        self.assertTrue(result.explicit_start)

    def test_multi(self):
        self.assertEqual(
            self.catalog.parse("Đi từ phòng A qua hành lang C rồi đến phòng B").regions,
            ("A", "C", "B"),
        )

    def test_unaccented(self):
        self.assertEqual(self.catalog.parse("di den phong thi nghiem").regions, ("B",))

    def test_uppercase(self):
        self.assertEqual(self.catalog.parse("ĐI ĐẾN KHU VỰC A!!!").regions, ("A",))

    def test_polite_prefix(self):
        self.assertEqual(
            self.catalog.parse("Vui lòng cho robot đi đến phòng học").regions, ("A",)
        )

    def test_polite_from_to_is_not_last_leg_only(self):
        self.assertEqual(
            self.catalog.parse("Vui lòng chuyển từ A đến B").regions, ("A", "B")
        )

    def test_question_not_command(self):
        with self.assertRaises(CommandError):
            self.catalog.parse("Có nên đi đến phòng học không?")

    def test_b_phonetic(self):
        self.assertEqual(self.catalog.parse("Đi đến khu vực bê").regions, ("B",))

    def test_c_phonetic(self):
        self.assertEqual(self.catalog.parse("Đi đến khu vực xê").regions, ("C",))

    def test_unknown_region(self):
        with self.assertRaises(UnknownRegion):
            self.catalog.parse("Đi đến khu vực Z")

    def test_unknown_midpoint(self):
        with self.assertRaises(UnknownRegion):
            self.catalog.parse("Đi từ A qua Z rồi đến B")

    def test_refuse_negation(self):
        with self.assertRaises(CommandError):
            self.catalog.parse("Không đi đến A")

    def test_missing_destination(self):
        with self.assertRaises(CommandError):
            self.catalog.parse("Đi từ A")

    def test_empty(self):
        with self.assertRaises(CommandError):
            self.catalog.parse("")

    def test_stop(self):
        self.assertEqual(self.catalog.parse("Dừng lại!").action, "STOP")

    def test_sequence_waits_for_success(self):
        s = RouteSequencer()
        s.begin(self.catalog.parse("Đi từ A đến B"))
        self.assertEqual(s.current, "A")
        self.assertEqual(s.pending, ["B"])
        s.result(True, True)
        self.assertEqual(s.current, "B")
        s.result(True, True)
        self.assertEqual(s.completed, ["A", "B"])
        self.assertEqual(s.state, "SUCCEEDED")

    def test_sequence_failure_does_not_advance(self):
        s = RouteSequencer()
        s.begin(self.catalog.parse("Đi từ A đến B"))
        s.result(False)
        self.assertIsNone(s.current)
        self.assertEqual(s.pending, [])
        self.assertEqual(s.state, "FAILED")

    def test_unverified_success_does_not_advance(self):
        s = RouteSequencer()
        s.begin(self.catalog.parse("Đi từ A đến B"))
        s.result(True, False)
        self.assertEqual(s.completed, [])
        self.assertEqual(s.state, "FAILED")

    def test_busy(self):
        s = RouteSequencer()
        cmd = self.catalog.parse("Đi đến A")
        s.begin(cmd)
        with self.assertRaises(CommandError):
            s.begin(cmd)

    def test_cancel(self):
        s = RouteSequencer()
        s.begin(self.catalog.parse("Đi từ A đến B"))
        s.stop()
        self.assertIsNone(s.current)
        self.assertEqual(s.pending, [])
