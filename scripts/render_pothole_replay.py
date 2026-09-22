from __future__ import annotations

import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from ddevsim.replay_video import render_engineering_replay


def main() -> int:
    run_dir = ROOT / "runs" / "hd_utility_ddev_single_wheel_deep_pothole"
    video_dir = run_dir / "video"
    result = render_engineering_replay(
        run_dir / "data" / "controller_and_dynamics.csv",
        video_dir / "single_wheel_deep_pothole_engineering_replay.mp4",
    )
    (video_dir / "engineering_replay_manifest.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
