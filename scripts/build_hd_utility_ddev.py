from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from ddevsim.channels import write_contract
from ddevsim.hd_ddev_case import build_hd_ddev_case


DEFAULT_SOURCE = Path(
    r"F:\TruckSim2019\TruckSim2019.0_Data\Results\Run_c2c3bc93-4dcf-4714-9285-137101f06551\run_all.par"
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Build the HD Utility 8-channel DDEV TruckSim case")
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--target", type=Path, default=ROOT / "models" / "hd_utility_ddev")
    parser.add_argument("--stop-s", type=float, default=1.0)
    parser.add_argument(
        "--tyre-load-reference-n",
        type=float,
        default=40000.0,
        help=(
            "Rescale the tyre dataset load rating FZ_REF. The stock model labels the "
            "tyre '2000 kg Rating' (FZ_REF 20000 N), which is BELOW this vehicle's "
            "22415 N static corner load, so the solver extrapolates the tyre load "
            "axis above ~39.3 kN and every dynamic load transfer leaves the table. "
            "40000 N (4 t per tyre, 1.79x the static load) makes the model valid up "
            "to ~78 kN. Pass 20000 to keep the stock rating."
        ),
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    artifacts = build_hd_ddev_case(
        args.source,
        args.target,
        program_dir=Path(r"F:\TruckSim2019\TruckSim2019.0_Prog"),
        data_dir=Path(r"F:\TruckSim2019\TruckSim2019.0_Data"),
        stop_s=args.stop_s,
        tyre_load_reference_n=args.tyre_load_reference_n,
    )
    write_contract(args.target / "interface_contract.json")
    manifest = json.loads((args.target / "source_manifest.json").read_text(encoding="utf-8"))
    print(json.dumps(manifest.get("tyre_load_reference_n"), ensure_ascii=False, indent=2))
    print(json.dumps({key: str(value) for key, value in artifacts.items()}, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
