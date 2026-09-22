from __future__ import annotations

import json
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from ddevsim.cosim import run_stepwise
from ddevsim.interface_validation import EXPORT_NAMES, IMPORT_NAMES
from ddevsim.pothole_controller import WheelLiftController
from ddevsim.pothole_case import SCENARIO_EXPORTS
from ddevsim.replay_video import render_engineering_replay


def main() -> int:
    model_dir = ROOT / "models" / "hd_utility_ddev" / "single_wheel_deep_pothole"
    simfile = model_dir / "simfile.sim"
    run_dir = ROOT / "runs" / "hd_utility_ddev_single_wheel_deep_pothole"
    data_dir = run_dir / "data"
    native_dir = run_dir / "native"
    data_dir.mkdir(parents=True, exist_ok=True)
    native_dir.mkdir(parents=True, exist_ok=True)

    result = run_stepwise(
        simfile,
        WheelLiftController(),
        data_dir / "controller_and_dynamics.csv",
        IMPORT_NAMES,
        EXPORT_NAMES + SCENARIO_EXPORTS,
        log_decimation=10,
    )
    model_output = model_dir / "output"
    copied = []
    for source in sorted(model_output.glob("single_wheel_deep_pothole*")):
        if source.is_file():
            target = native_dir / source.name
            shutil.copy2(str(source), str(target))
            copied.append(str(target))

    replay = render_engineering_replay(
        data_dir / "controller_and_dynamics.csv",
        run_dir / "video" / "single_wheel_deep_pothole_engineering_replay.mp4",
    )

    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "model": "HD Utility DDEV 4x4 Active Suspension",
        "scenario": "right single-wheel deep pothole",
        "controller": "sequential FR/RR wheel-load feedback lift with triangular support",
        "input_order": list(IMPORT_NAMES),
        "output_order": list(EXPORT_NAMES + SCENARIO_EXPORTS),
        "result": result,
        "native_history_files": copied,
        "visual_scene": {
            "road": "brown-gray dry dirt, RGB approximately 0.48/0.42/0.32",
            "background": "light grass, RGB approximately 0.82/0.88/0.78",
            "sky": "partly cloudy skybox",
            "camera": "vehicle tracking, azimuth -45 deg, elevation 14 deg, distance 16 m",
        },
        "engineering_replay": replay,
        "native_video_status": (
            "TruckSim history is complete; run scripts/open_pothole_visualizer.ps1 and use "
            "File > Export Video for the native AVI"
        ),
    }
    (run_dir / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest, ensure_ascii=False, indent=2))
    return 0 if result["status"] == "COMPLETED" else 2


if __name__ == "__main__":
    raise SystemExit(main())
