"""Open a completed local TruckSim run in the native VS Visualizer.

Examples (from the repository root)::

    python scripts/open_run_visualizer.py right_side_both_3k_front34_rear335
    python scripts/open_run_visualizer.py runs/right_side_both_3k_front34_rear335
    python scripts/open_run_visualizer.py runs/my_run/model/output/history.vs

This only opens playback. It does not run a simulation or export a video.
"""

from __future__ import annotations

import argparse
import hashlib
import re
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from ddevsim.native_video import (  # noqa: E402
    DEFAULT_VISUALIZER_32,
    default_visualizer,
    default_workdir,
    find_history,
    launch_visualizer,
    stage_history,
    write_animator_par,
)


def resolve_source(value: str) -> Path:
    """Accept a run name, a run directory, an output directory, or a .vs file."""
    supplied = Path(value)
    candidates = [supplied] if supplied.is_absolute() else [ROOT / supplied, ROOT / "runs" / supplied]
    source = next((path for path in candidates if path.exists()), None)
    if source is None:
        raise FileNotFoundError("run/history not found: %s" % value)
    if source.is_file():
        if source.suffix.lower() not in (".vs", ".vsb"):
            raise ValueError("expected a .vs or .vsb history file: %s" % source)
        return source
    for directory in (source / "model" / "output", source / "native", source):
        if directory.is_dir() and any(directory.glob("*.vs")):
            return directory
    raise FileNotFoundError("no TruckSim .vs history below: %s" % source)


def stage_directory(source: Path) -> Path:
    """Give each run an ASCII-only stage to avoid Visualizer's Unicode path bug."""
    source = source.resolve()
    run_name = source.parent.parent.name if source.name == "output" and source.parent.name == "model" else source.name
    slug = re.sub(r"[^A-Za-z0-9_-]+", "_", run_name).strip("_") or "run"
    slug = slug[:48]
    suffix = hashlib.sha256(str(source).encode("utf-8")).hexdigest()[:10]
    stage = Path(tempfile.gettempdir()) / "ddev_visualizer_runs" / (slug + "_" + suffix)
    str(stage).encode("ascii")
    return stage


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", help="name under runs/, run directory, output directory, or .vs/.vsb file")
    parser.add_argument("--resolution", type=int, nargs=2, metavar=("W", "H"), default=(1280, 720))
    parser.add_argument("--dry-run", action="store_true", help="validate and stage without opening a window")
    args = parser.parse_args(argv)

    source = resolve_source(args.run)
    history = find_history(source)
    staged = stage_history(history, stage_directory(source))
    animator = write_animator_par(staged.vs.parent, staged)
    visualizer = DEFAULT_VISUALIZER_32 if DEFAULT_VISUALIZER_32.exists() else default_visualizer()
    workdir = default_workdir()
    if not visualizer.is_file() or not workdir.is_dir():
        raise FileNotFoundError("TruckSim 2019 Visualizer or resource directory is missing")

    print("Run history : %s" % history.vs)
    print("Duration    : %.3f s (%d frames)" % (history.duration_s, history.samples))
    print("Staged at   : %s" % staged.vs.parent)
    print("Visualizer  : %s" % visualizer)
    if args.dry_run:
        print("Validated and staged; no window opened.")
        return 0

    launched = launch_visualizer(
        animator,
        visualizer=visualizer,
        workdir=workdir,
        window_name="DDEV_" + staged.vs.parent.name[:40],
        resolution=args.resolution,
    )
    print("Opened native playback (PID %s). Use the VS Visualizer play button." % launched["pid"])
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (FileNotFoundError, ValueError) as exc:
        print("Cannot open run: %s" % exc, file=sys.stderr)
        raise SystemExit(2) from exc
