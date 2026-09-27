"""Keep a small measured reference from a validated native FR-lift CSV."""

from __future__ import annotations

import argparse
import csv
import gzip
from pathlib import Path


FIELDS = ("time_s", "exp_Fz_L1", "exp_Fz_R1", "exp_Fz_L2", "exp_Fz_R2",
          "exp_Jnc_L1", "exp_Jnc_R1", "exp_Jnc_L2", "exp_Jnc_R2",
          "exp_Z_R1", "exp_Zgnd_R1i", "exp_Roll_E", "exp_Pitch")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--decimation", type=int, default=4)
    args = parser.parse_args()
    if args.decimation < 1:
        raise ValueError("decimation must be positive")
    with args.source.open(encoding="utf-8", newline="") as source, gzip.open(
            args.output, "wt", encoding="utf-8", newline="") as output:
        rows = csv.DictReader(source)
        if not set(FIELDS).issubset(rows.fieldnames or ()):
            raise ValueError("native source lacks a required measurement")
        writer = csv.DictWriter(output, fieldnames=FIELDS)
        writer.writeheader()
        for index, row in enumerate(rows):
            if index % args.decimation == 0:
                writer.writerow({field: row[field] for field in FIELDS})


if __name__ == "__main__":
    main()
