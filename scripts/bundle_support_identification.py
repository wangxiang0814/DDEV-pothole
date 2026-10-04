"""Publish a validated three-mode native identification bundle."""
import argparse
import hashlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from ddevsim.static_wheel_lift.system_identification import assemble_support_bundle


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('four-contact', 'fr', 'rr'):
        parser.add_argument('--' + name, type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    paths = dict(zip(('FOUR_CONTACT', 'FR', 'RR'), (args.four_contact, args.fr, args.rr)))
    records = {mode: json.loads(path.read_text(encoding='utf-8')) for mode, path in paths.items()}
    bundle = assemble_support_bundle(records)
    bundle['source_records'] = {mode: {'path': str(path),
        'sha256': hashlib.sha256(path.read_bytes()).hexdigest()} for mode, path in paths.items()}
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(bundle, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(bundle['source_model_sha256'])


if __name__ == '__main__':
    main()
