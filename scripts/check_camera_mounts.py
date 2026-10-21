"""Verify body-surface mounts and plot their locations against native mesh vertices."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
from ddevsim.demonstration.cameras import load_config
from ddevsim.demonstration.mounts import verify_mounts
from ddevsim.native_video import find_history, default_workdir


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--config', type=Path, default=ROOT/'configs/demonstration/five_camera.json')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    reports = verify_mounts(load_config(args.config), find_history(args.run_dir).par, default_workdir())
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'camera_mounts.json').write_text(json.dumps(reports, indent=2), encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    fig, axes = plt.subplots(1, 2, figsize=(13, 6))
    assets = set()
    for record in reports.values():
        if record['asset'] in assets:
            continue
        assets.add(record['asset'])
        vertices = np.array([list(map(float, l.split()[1:4]))
                            for l in Path(record['asset']).read_text().splitlines() if l.startswith('v ')])
        vertices += record['asset_offset_m']
        for ax, axis in zip(axes, [1, 2]):
            ax.scatter(vertices[:, 0], vertices[:, axis], s=2, c='#8b969e', alpha=.5)
    for name, record in reports.items():
        anchor, lens = record['surface_anchor_body_m'], record['lens_position_body_m']
        for ax, axis in zip(axes, [1, 2]):
            ax.plot([anchor[0], lens[0]], [anchor[axis], lens[axis]], c='red')
            ax.scatter([lens[0]], [lens[axis]], s=35, c='red')
            if axis != 2 or name not in ('wheel_FR', 'wheel_RR'):
                label = ({'wheel_FL': 'wheel_FL / FR', 'wheel_RL': 'wheel_RL / RR'}.get(name, name)
                         if axis == 2 else name)
                ax.annotate(label, (lens[0], lens[axis]), xytext=(6, 12),
                            textcoords='offset points', fontsize=9)
    for ax, label in zip(axes, ['body y / m (left)', 'body z / m (up)']):
        ax.set_xlabel('body x / m (forward)'); ax.set_ylabel(label)
        ax.set_aspect('equal'); ax.grid(alpha=.2)
    fig.suptitle('Native body mesh mounting audit: lens 20 mm outside surface\n'
                 'Geometry diagram only; cameras follow Xo/Yo/Zo + body attitude')
    fig.tight_layout()
    fig.savefig(args.output/'camera_mounts.png', dpi=140)
    print(json.dumps({k: round(v['standoff_m'], 6) for k, v in reports.items()}))


if __name__ == '__main__':
    main()
