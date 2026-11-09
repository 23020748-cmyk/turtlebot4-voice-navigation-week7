"""Save the executed regression test count and failures for the report."""

import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
suite = unittest.defaultTestLoader.discover(str(ROOT / "tests"))
result = unittest.TextTestRunner(verbosity=2).run(suite)
summary = dict(
    tests_run=result.testsRun,
    failures=len(result.failures),
    errors=len(result.errors),
    skipped=len(result.skipped),
    passed=result.wasSuccessful(),
)
(ROOT / "results/unit_tests.json").write_text(json.dumps(summary, indent=2))
raise SystemExit(0 if result.wasSuccessful() else 1)
