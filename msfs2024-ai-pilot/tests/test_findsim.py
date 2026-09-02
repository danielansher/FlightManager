"""The SimConnect.dll finder.

Cannot be exercised on the real machine from here, so the walk is tested
against a synthetic tree: it must find copies at realistic depths, refuse to
descend for ever, skip the directories that are enormous, and stop when it runs
out of time rather than when it runs out of filesystem.
"""

import os
import time

import pytest

from aipilot import findsim


def _touch(path: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as handle:
        handle.write(b"\0" * 2048)


@pytest.fixture
def tree(tmp_path):
    root = str(tmp_path)
    _touch(os.path.join(root, "Little Navmap", "SimConnect.dll"))
    _touch(os.path.join(root, "a", "b", "c", "SimConnect.dll"))
    _touch(os.path.join(root, "Windows", "System32", "SimConnect.dll"))   # pruned
    _touch(os.path.join(root, *["deep"] * 9, "SimConnect.dll"))           # too deep
    _touch(os.path.join(root, "unrelated.dll"))
    return root


def test_it_finds_copies_at_realistic_depths(tree):
    found = list(findsim._walk(tree, time.monotonic() + 30))
    names = {os.path.relpath(p, tree).replace("\\", "/") for p in found}
    assert "Little Navmap/SimConnect.dll" in names
    assert "a/b/c/SimConnect.dll" in names


def test_it_skips_directories_that_are_never_worth_walking(tree):
    found = list(findsim._walk(tree, time.monotonic() + 30))
    assert not any("Windows" in p for p in found), "should not descend into Windows"


def test_it_does_not_descend_for_ever(tree):
    found = list(findsim._walk(tree, time.monotonic() + 30))
    assert not any(p.count("deep") > findsim.MAX_DEPTH for p in found)


def test_it_stops_when_it_runs_out_of_time(tree):
    """A deadline already past must produce nothing rather than a full walk."""
    assert list(findsim._walk(tree, time.monotonic() - 1)) == []


def test_a_missing_directory_does_not_raise(tmp_path):
    assert list(findsim._walk(str(tmp_path / "nope"), time.monotonic() + 5)) == []


def test_the_scan_respects_its_time_budget():
    started = time.monotonic()
    found, timed_out = findsim.find_all(time_budget_s=1.0)
    assert time.monotonic() - started < 20.0
    assert isinstance(found, list)


def test_copies_are_described_by_where_they_came_from():
    assert "Little Navmap" in findsim.describe(r"C:\Program Files\Little Navmap\SimConnect.dll")
    assert "MSFS 2020 SDK" in findsim.describe(r"C:\MSFS SDK\SimConnect SDK\lib\SimConnect.dll")
    assert "MSFS 2024 SDK" in findsim.describe(r"C:\MSFS 2024 SDK\SimConnect SDK\lib\SimConnect.dll")
    assert "MobiFlight" in findsim.describe(r"C:\Program Files\MobiFlight Connector\SimConnect.dll")
    assert "unknown" in findsim.describe(r"D:\somewhere\SimConnect.dll")


def test_duplicates_are_reported_once(tmp_path, monkeypatch):
    root = str(tmp_path)
    target = os.path.join(root, "Little Navmap", "SimConnect.dll")
    _touch(target)
    monkeypatch.setattr(findsim, "LIKELY_DIRECTORIES",
                        (os.path.join(root, "Little Navmap"),) * 3)
    found, _ = findsim.find_all(time_budget_s=2.0, deep=False)
    assert len(found) == 1
