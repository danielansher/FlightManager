"""Find SimConnect.dll anywhere on the machine.

This is the one thing most likely to stop someone before they start. The DLL
ships with the MSFS SDK, which is a large download for one file -- but it is
also bundled by several tools a simmer very likely already has, so on most
machines a copy is already sitting there and the problem is only knowing where.

So: look, rather than send someone away to install something. The scan is
bounded in both depth and time, prunes the directories that are enormous and
never contain it, and reports every copy it finds.
"""

from __future__ import annotations

import os
import platform
import time
from typing import Iterator, Optional

TARGET = "simconnect.dll"

#: Places worth looking at directly, before any scanning. Tools that a simmer
#: is likely to have already, plus the SDK and simulator installs.
LIKELY_DIRECTORIES = (
    r"C:\MSFS SDK\SimConnect SDK\lib",
    r"C:\MSFS 2024 SDK\SimConnect SDK\lib",
    r"C:\Program Files\Little Navmap",
    r"C:\Program Files (x86)\Little Navmap",
    r"C:\Program Files\MobiFlight Connector",
    r"C:\Program Files (x86)\MobiFlight Connector",
    r"C:\Program Files\FSUIPC7",
    r"C:\Program Files (x86)\FSUIPC7",
    r"C:\Program Files\Microsoft Flight Simulator",
    r"C:\Program Files\Microsoft Flight Simulator 2024",
    r"C:\Program Files (x86)\Steam\steamapps\common\MicrosoftFlightSimulator",
    r"C:\Program Files (x86)\Steam\steamapps\common\Microsoft Flight Simulator 2024",
)

#: Directory names never worth descending into: either enormous, or permission
#: denied, or both.
PRUNE = {
    "windows", "$recycle.bin", "system volume information", "winsxs",
    "node_modules", ".git", "drivers", "assemblyPackages", "packages",
    "onedrive", "appdata\\locallow",
}

#: The scan stops after this long regardless. Someone waiting on a progress
#: line has a very different idea of "a moment" than a filesystem walk does.
DEFAULT_TIME_BUDGET_S = 90.0

#: How deep below each root to look. The DLL is always shallow when it is
#: present at all; a deep walk only finds build trees and takes for ever.
MAX_DEPTH = 6


def search_roots() -> list[str]:
    """Where to start looking: program directories, then whole drives."""
    roots: list[str] = []
    for variable in ("ProgramFiles", "ProgramFiles(x86)", "ProgramW6432"):
        value = os.environ.get(variable)
        if value and os.path.isdir(value) and value not in roots:
            roots.append(value)
    for letter in "CDEFGHIJ":
        drive = f"{letter}:\\"
        if os.path.isdir(drive) and drive not in roots:
            roots.append(drive)
    return roots


def _walk(root: str, deadline: float) -> Iterator[str]:
    root_depth = root.rstrip("\\/").count(os.sep)
    for directory, subdirectories, files in os.walk(root, topdown=True,
                                                    onerror=lambda _e: None):
        if time.monotonic() > deadline:
            return
        if directory.count(os.sep) - root_depth >= MAX_DEPTH:
            subdirectories[:] = []
            continue
        subdirectories[:] = [d for d in subdirectories
                             if d.lower() not in PRUNE and not d.startswith("$")]
        for name in files:
            if name.lower() == TARGET:
                yield os.path.join(directory, name)


def find_all(time_budget_s: float = DEFAULT_TIME_BUDGET_S,
             deep: bool = True) -> tuple[list[str], bool]:
    """Every copy of SimConnect.dll found. Returns ``(paths, ran_out_of_time)``."""
    deadline = time.monotonic() + time_budget_s
    found: list[str] = []
    seen: set[str] = set()

    def remember(path: str) -> None:
        key = os.path.normcase(os.path.abspath(path))
        if key not in seen and os.path.isfile(path):
            seen.add(key)
            found.append(path)

    for directory in LIKELY_DIRECTORIES:
        remember(os.path.join(directory, "SimConnect.dll"))

    if deep:
        for root in search_roots():
            if time.monotonic() > deadline:
                break
            for path in _walk(root, deadline):
                remember(path)

    return found, time.monotonic() > deadline


def describe(path: str) -> str:
    """One line about a copy: where it is, how big, and what it came from."""
    try:
        size = os.path.getsize(path)
    except OSError:
        size = 0
    lowered = path.lower()
    if "msfs 2024 sdk" in lowered:
        source = "MSFS 2024 SDK"
    elif "msfs sdk" in lowered:
        source = "MSFS 2020 SDK"
    elif "little navmap" in lowered:
        source = "Little Navmap"
    elif "mobiflight" in lowered:
        source = "MobiFlight"
    elif "fsuipc" in lowered:
        source = "FSUIPC"
    elif "flight simulator 2024" in lowered:
        source = "MSFS 2024 install"
    elif "flightsimulator" in lowered or "flight simulator" in lowered:
        source = "MSFS install"
    else:
        source = "unknown"
    return f"{path}\n      {size / 1024:.0f} KB, looks like: {source}"


def run(args) -> int:
    """The ``find-simconnect`` command."""
    package_dir = os.path.dirname(os.path.abspath(__file__))
    destination = os.path.join(package_dir, "SimConnect.dll")

    if platform.system() != "Windows":
        print("SimConnect.dll only exists on Windows, so there is nothing to "
              "find here. Run this on the machine with the simulator on it.")
        return 0

    if os.path.isfile(destination):
        print(f"Already in place: {destination}")
        print("Nothing to do -- the AI Pilot will use this one.")
        return 0

    print("Looking for SimConnect.dll. This takes up to a minute or two.")
    print()
    found, timed_out = find_all(time_budget_s=args.seconds, deep=not args.quick)

    if not found:
        print("No copy of SimConnect.dll found on this machine.")
        if timed_out:
            print("(The search ran out of time -- try --seconds 300 for a "
                  "longer look.)")
        print()
        print("Get one by installing the MSFS SDK, which is free and comes from "
              "inside the simulator:")
        print("  1. In MSFS: Options -> General -> Developers -> Developer Mode ON")
        print("  2. A 'Developers' menu appears in the top bar")
        print("  3. Help -> SDK Installer, and install it")
        print("  4. Run this again -- it will find it at")
        print(r"     C:\MSFS SDK\SimConnect SDK\lib\SimConnect.dll")
        return 1

    print(f"Found {len(found)} cop{'y' if len(found) == 1 else 'ies'}:")
    print()
    for index, path in enumerate(found, 1):
        print(f"  {index}. {describe(path)}")
    print()

    best = found[0]
    if args.copy:
        try:
            import shutil

            shutil.copy2(best, destination)
        except OSError as exc:
            print(f"Could not copy it: {exc}")
            print(f"Copy it by hand to: {destination}")
            return 1
        print(f"Copied the first one to {destination}")
        print("That is all the setup this needed. Try Check-My-Setup next.")
        return 0

    print("To use the first one, either copy it next to the aipilot package:")
    print(f'  copy "{best}" "{destination}"')
    print()
    print("or just run this again with --copy and it will do that for you:")
    print("  python -m aipilot find-simconnect --copy")
    return 0
