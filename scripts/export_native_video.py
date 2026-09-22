"""Export a native TruckSim video from a completed run's history file.

Reads a TruckSim history (``.vs`` + ``.vsb`` + ``_all.par``), validates it, stages
it, creates the ``.vsrap`` package non-interactively and opens VS Visualizer with
the TruckSim 3D scene, then prints the exact remaining GUI steps that write the
AVI.

Why the final step is manual
----------------------------
TruckSim 2019 exposes no command-line video option: the AVI writer sits behind the
``File > Export Video...`` menu and uses the Windows Video-for-Windows dialog to
pick a compressor.  Everything else here is automated.  See
``docs/native_video_export.md``.

Usage
-----
    # a batch case, a run directory, or the .vs file itself all work
    python scripts\\export_native_video.py runs\\hd_utility_ddev_expert_pothole\\native
    python scripts\\export_native_video.py runs\\batch_sweep\\cases\\baseline\\model\\output\\baseline.vs
    python scripts\\export_native_video.py <source> --no-launch     # just validate+stage
    python scripts\\export_native_video.py <source> --resolution 1920 1080
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "src") not in sys.path:
    sys.path.insert(0, str(ROOT / "src"))

from ddevsim.native_video import (  # noqa: E402
    default_visualizer,
    default_workdir,
    export_native_video,
)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path,
                        help="history .vs/.vsb file, or a directory containing one")
    parser.add_argument("--stage-dir", type=Path, default=None,
                        help="where to stage the animator.par and .vsrap "
                             "(default: <history dir>/native_video)")
    parser.add_argument("--resolution", type=int, nargs=2, default=[1280, 720],
                        metavar=("W", "H"))
    parser.add_argument("--no-launch", action="store_true",
                        help="do not open VS Visualizer (validate and stage only)")
    parser.add_argument("--vsrap", action="store_true",
                        help="also attempt non-interactive .vsrap creation "
                             "(best-effort; drives a GUI binary that may block)")
    parser.add_argument("--ascii-stage", action="store_true",
                        help="stage the history under an ASCII path in the system "
                             "temp directory instead of next to the run")
    args = parser.parse_args(argv)

    source = args.source if args.source.is_absolute() else (ROOT / args.source)
    stage_dir = args.stage_dir or (source.parent if source.is_file() else source) / "native_video"

    report = export_native_video(
        source=source,
        stage_dir=stage_dir,
        visualizer=default_visualizer(),
        resolution=tuple(args.resolution),
        create_vsrap_package=args.vsrap,
        launch=not args.no_launch,
        workdir=default_workdir(),
        ascii_stage=args.ascii_stage,
    )

    history = report["history"]
    print("history      : %s" % history["basename"])
    print("  vs         : %s" % history["vs"])
    print("  channels   : %d over %d frames, dt %.4f s, duration %.3f s" % (
        history["channels"], history["samples"], history["time_step_s"], history["duration_s"]))
    print("  stagED     : %s" % report["stage_dir"])
    vsrap = report.get("vsrap")
    if vsrap:
        print("  vsrap      : %s (%s, %d bytes, rc=%s)" % (
            vsrap["vsrap"], "created" if vsrap.get("created") else "failed",
            vsrap.get("size_bytes", 0), vsrap.get("return_code")))
    launch = report.get("launch")
    if launch:
        print("  viewer     : launched pid %s" % launch["pid"])
        print("  command    : %s" % launch["command"])
    print()
    for line in report["manual_steps"]:
        print(("  " + line) if line else "")
    print()
    print("report -> %s" % (Path(report["stage_dir"]) / "native_video_export.json"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
