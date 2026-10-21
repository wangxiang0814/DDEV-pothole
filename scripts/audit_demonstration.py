"""Recheck saved RGB files and exact state/action timestamp joins independently."""
import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
import numpy as np
from PIL import Image
from ddevsim.demonstration.recording import TransitionReader
from ddevsim.demonstration.capture import sha


def audit(episode):
    episode = Path(episode).resolve()
    meta = json.loads((episode/'episode.json').read_text(encoding='utf-8'))
    reader = TransitionReader(episode.parent)
    times, images = [], 0
    for line in (episode/'frames.jsonl').read_text(encoding='utf-8').splitlines():
        row = json.loads(line)
        sample = reader.sample(row['time_s'])
        for key in sample:
            if row[key] != sample[key]:
                raise ValueError('state/action association mismatch: '+key)
        if set(row['cameras']) != {c['name'] for c in meta['config']['cameras']}:
            raise ValueError('missing camera')
        for cfg in meta['config']['cameras']:
            camera = row['cameras'][cfg['name']]
            if abs(camera['time_s']-row['time_s']) > 1e-9:
                raise ValueError('camera timestamp mismatch')
            with Image.open(episode/camera['image']) as im:
                if im.mode != 'RGB' or im.size != (cfg['width'], cfg['height']):
                    raise ValueError('invalid RGB dimensions')
                im.load()
            images += 1
        times.append(row['time_s'])
    gaps = np.diff(times)
    criteria = {
        'expert_pass': meta['expert_status'] == 'PASS',
        'collection_pass': meta['collection_status'] == 'PASS',
        'full_episode': not meta.get('preview_only', False),
        'strictly_increasing': bool(len(gaps) and np.all(gaps > 0)),
        'maximum_gap': bool(len(gaps) and max(gaps) <= meta['config']['max_gap_s']),
        'start_covered': bool(times and times[0] <= reader.times[0]+meta['config']['start_end_tolerance_s']),
        'end_covered': bool(times and times[-1] >= reader.final_time-meta['config']['start_end_tolerance_s']),
        'history_unchanged': all(sha(p) == h for p, h in meta['source_hashes'].items()),
        'record_count': len(times) == meta['frame_groups'],
    }
    result = dict(status='PASS' if all(criteria.values()) else 'FAIL', criteria=criteria,
                  frame_groups=len(times), rgb_images=images,
                  maximum_gap_s=float(max(gaps)) if len(gaps) else None)
    (episode/'audit.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('episode', type=Path)
    result = audit(parser.parse_args().episode)
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['status'] == 'PASS' else 1)
