"""Record the VS Visualizer's own playback of a pothole run into an MP4.

Why this exists
---------------
VS Visualizer exposes no playback or video flag on its command line (its own GUI help
lists only `-i -fs -tb -res -nosound -uimode -vsrap -vsrapoverwrite -altresource -fscf
-pos -c -r -help`), and its `File > Export Video` goes through the Windows VfW modal
compressor dialog, which cannot be scripted.  So the only fully scriptable route to a
video of the *actual rendering* is:

1. launch the visualizer on a history recorded against the current model,
2. let it auto-play (it does), and
3. grab the screen repeatedly, then encode with OpenCV.

Two traps this script encodes, both of which cost real debugging time:

* **The display is 2x DPI here.**  PowerShell's
  ``SystemInformation.VirtualScreen`` reports 1280x800, but the physical framebuffer is
  2560x1600, so a crop computed in the reported coordinate space lands nowhere near the
  vehicle.
* **OpenCV's ``imwrite``/``VideoWriter`` silently fail on non-ASCII paths** (this
  project's path contains Chinese characters).  Intermediate frames therefore go to an
  ASCII directory while the MP4 is written next to the run.

Usage::

    $env:PYTHONPATH='src'
    python scripts\\record_pothole_video.py                # 11 s at 10 fps
    python scripts\\record_pothole_video.py --seconds 11 --fps 10 --model truck
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import cv2
import numpy as np
from PIL import ImageGrab

ROOT = Path(__file__).resolve().parents[1]

VISUALIZER = r"F:\TruckSim2019\TruckSim2019.0_Prog\Programs\VsVisualizer\VsVisualizer.exe"
# The resource root, NOT `..._Data`: relative MTL_FILE / 3D shape paths resolve here.
RESOURCE_ROOT = r"F:\TruckSim2019\TruckSim2019.0_Prog\Resources"
# ASCII staging directory; VS Visualizer also mangles non-ASCII command-line arguments.
STAGE = Path(r"F:\DDEV_TruckSim_Visualizer")
HISTORY = "single_wheel_deep_pothole"

RUNS = {
    "corner_module": ("runs/corner_module_expert_pothole", "Corner Module DDEV"),
    "truck": ("runs/hd_utility_ddev_expert_pothole", "HD Utility DDEV (solid axle)"),
}

#: Crop to the visualizer window in *physical* framebuffer pixels, measured on this
#: machine (2560x1600 with 2x scaling).  A fractional crop of the whole screen also
#: captures the desktop around the window.
CROP = (640, 250, 2010, 1370)


def stage_history(run_dir: Path) -> None:
    native = run_dir / "native"
    missing = [
        name for name in ("%s.vs" % HISTORY, "%s.vsb" % HISTORY, "%s_all.par" % HISTORY)
        if not (native / name).exists()
    ]
    if missing:
        raise SystemExit(
            "history is incomplete in %s (missing %s); run "
            "scripts\\run_expert_pothole.py first" % (native, ", ".join(missing))
        )
    STAGE.mkdir(parents=True, exist_ok=True)
    for name in ("%s.vs" % HISTORY, "%s.vsb" % HISTORY, "%s_all.par" % HISTORY):
        (STAGE / name).write_bytes((native / name).read_bytes())


def animator_par() -> Path:
    path = STAGE / "animator.par"
    text = (
        "PARSFILE\nSET_RUN_SLOT 0\n"
        "DATASET %s\\%s.vs\nPARSFILE %s\\%s_all.par\nEND\n"
        % (STAGE, HISTORY, STAGE, HISTORY)
    )
    path.write_text(text, encoding="ascii")
    return path


def guard_road(par: Path, must_cover_m: float) -> None:
    """Refuse to record a history whose road does not reach the vehicle.

    This is the failure that produced the original "vehicle floating in mid-air"
    report: a history recorded before the road path was lengthened still declares a
    40 m path while the vehicle starts at station 100, so nothing renders under it.
    """
    text = par.read_text(encoding="mbcs", errors="replace")
    import re
    seg = re.search(r"(?m)^\s*SEGMENT_LENGTH\s+([-0-9.eE]+)", text)
    start = re.search(r"(?m)^\s*SPATH_START\s+([-0-9.eE]+)", text)
    sstart = re.search(r"(?m)^\s*SSTART\s+([-0-9.eE]+)", text)
    if not (seg and start and sstart):
        print("WARNING: could not parse the road path from %s; skipping the guard" % par)
        return
    road_end = float(start.group(1)) + float(seg.group(1))
    vehicle_start = float(sstart.group(1))
    print("road path covers 0-%.1f m; vehicle starts at %.1f m" % (road_end, vehicle_start))
    if vehicle_start >= road_end:
        raise SystemExit(
            "history was recorded against a road ending at %.1f m but the vehicle starts "
            "at %.1f m; nothing would render under it. Re-run the case." %
            (road_end, vehicle_start)
        )
    if road_end < must_cover_m:
        raise SystemExit(
            "road ends at %.1f m but this run reaches %.1f m" % (road_end, must_cover_m)
        )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", choices=sorted(RUNS), default="corner_module")
    parser.add_argument("--seconds", type=float, default=11.0)
    parser.add_argument("--fps", type=float, default=10.0)
    parser.add_argument("--width", type=int, default=900)
    parser.add_argument("--height", type=int, default=560)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)

    run_rel, label = RUNS[args.model]
    run_dir = ROOT / run_rel
    stage_history(run_dir)
    par = animator_par()
    guard_road(run_dir / "native" / ("%s_all.par" % HISTORY), must_cover_m=106.5)

    subprocess.run(["taskkill", "/F", "/IM", "VsVisualizer.exe"], capture_output=True)
    time.sleep(1.0)
    subprocess.Popen(
        [VISUALIZER, "-i", "DDEV_Pothole", "-fs", "0",
         "-res", str(args.width), str(args.height), "-tb", "1", str(par)],
        cwd=RESOURCE_ROOT,
    )
    # Trailing-edge padding: shorter waits capture a frame before the history has loaded.
    time.sleep(17)

    full = np.asarray(ImageGrab.grab().convert("RGB"))
    h, w = full.shape[:2]
    x0, y0, x1, y1 = CROP
    x1, y1 = min(x1, w), min(y1, h)
    print("framebuffer %dx%d; cropping to x %d-%d, y %d-%d" % (w, h, x0, x1, y0, y1))

    dump = Path(tempfile.gettempdir()) / "dsh_video_frames"
    dump.mkdir(parents=True, exist_ok=True)
    frames = []
    count = max(1, int(args.seconds * args.fps))
    interval = 1.0 / args.fps
    for i in range(count):
        began = time.time()
        frames.append(np.asarray(ImageGrab.grab().convert("RGB"))[y0:y1, x0:x1])
        if i % 20 == 0:
            print("  captured %d/%d" % (i + 1, count), flush=True)
        if i in (0, count // 3, 2 * count // 3):
            cv2.imwrite(str(dump / ("frame_%03d.png" % i)),
                        cv2.cvtColor(frames[-1], cv2.COLOR_RGB2BGR))
        slack = interval - (time.time() - began)
        if slack > 0:
            time.sleep(slack)

    subprocess.run(["taskkill", "/F", "/IM", "VsVisualizer.exe"], capture_output=True)

    out = args.out or (run_dir / "video" / "pothole_traverse.mp4")
    out.parent.mkdir(parents=True, exist_ok=True)
    fh, fw = frames[0].shape[:2]
    writer = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), args.fps,
                             (fw // 2 * 2, fh // 2 * 2))
    for frame in frames:
        writer.write(cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
    writer.release()

    print("wrote %s" % out)
    print("  %d frames, %dx%d, %.0f fps, %d bytes" % (
        len(frames), fw, fh, args.fps, out.stat().st_size if out.exists() else 0))
    print("  frames dumped for inspection: %s" % dump)
    print("  model: %s" % label)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
