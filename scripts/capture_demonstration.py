"""Render a previously recorded run; no controller replay or video export."""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from ddevsim.demonstration.cameras import load_config
from ddevsim.demonstration.capture import capture_episode


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--config', type=Path, default=ROOT/'configs/demonstration/five_camera.json')
    parser.add_argument('--capture-name', default='episode')
    parser.add_argument('--preview-seconds', type=float, help='short camera placement check; never trainable')
    args = parser.parse_args()
    cfg = load_config(args.config)
    if not cfg['enabled']:
        parser.error('visual collection is disabled in config')
    result = capture_episode(args.run_dir, cfg, capture_name=args.capture_name,
                             preview_duration_s=args.preview_seconds)
    raise SystemExit(0 if result['collection_status'] in ('PASS', 'PREVIEW') else 1)
