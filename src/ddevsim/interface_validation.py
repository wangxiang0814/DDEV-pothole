from __future__ import annotations

import csv
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, List, Sequence, Tuple

from .cosim import run_stepwise
from .hd_ddev_case import DDEV_EXPORTS


WHEEL_ORDER = ("FL", "FR", "RL", "RR")
IMPORT_NAMES = (
    "IMP_MYUSM_L1", "IMP_MYUSM_R1", "IMP_MYUSM_L2", "IMP_MYUSM_R2",
    "IMP_FS_L1", "IMP_FS_R1", "IMP_FS_L2", "IMP_FS_R2",
)
#: Derived from the builder's own contract rather than repeated here.  The solver
#: binds exports *positionally*, so a list that drifts from the file silently
#: misreads every channel after the first difference -- which is exactly the bug this
#: indirection removes.
EXPORT_NAMES = tuple(line.split(None, 1)[1] for line in DDEV_EXPORTS)
TORQUE_AMPLITUDE_NM = 500.0
ACTIVE_AMPLITUDE_N = 1000.0


def signed_peak_delta(baseline: Sequence[float], case: Sequence[float]) -> float:
    if len(baseline) != len(case) or not baseline:
        raise ValueError("baseline and case must be non-empty and equal length")
    deltas = [float(value) - float(reference) for reference, value in zip(baseline, case)]
    return max(deltas, key=abs)


def build_validation_cases() -> List[Dict[str, Any]]:
    cases: List[Dict[str, Any]] = [
        {"name": "baseline", "port_index": None, "port_name": None, "polarity": 0, "amplitude": 0.0}
    ]
    for port_index, port_name in enumerate(IMPORT_NAMES):
        kind = "torque" if port_index < 4 else "active"
        corner = WHEEL_ORDER[port_index % 4]
        magnitude = TORQUE_AMPLITUDE_NM if port_index < 4 else ACTIVE_AMPLITUDE_N
        for polarity, suffix in ((1, "pos"), (-1, "neg")):
            cases.append(
                {
                    "name": "%s_%s_%s" % (kind, corner, suffix),
                    "port_index": port_index,
                    "port_name": port_name,
                    "polarity": polarity,
                    "amplitude": polarity * magnitude,
                }
            )
    return cases


def command_for_case(
    case: Dict[str, Any],
    start_s: float = 0.25,
    stop_s: float = 0.35,
) -> Callable[[float, Sequence[float]], Tuple[float, ...]]:
    if not 0.0 <= start_s < stop_s:
        raise ValueError("pulse interval must satisfy 0 <= start < stop")
    port_index = case["port_index"]
    amplitude = float(case["amplitude"])

    def command(time_s: float, _exports: Sequence[float]) -> Tuple[float, ...]:
        values = [0.0] * len(IMPORT_NAMES)
        if port_index is not None and start_s <= time_s < stop_s:
            values[int(port_index)] = amplitude
        return tuple(values)

    return command


def run_validation_suite(
    simfile: Path,
    target_dir: Path,
    start_s: float = 0.25,
    stop_s: float = 0.35,
    log_decimation: int = 2,
    export_names: Sequence[str] | None = None,
) -> Dict[str, Any]:
    """Run the 17-case pulse matrix.

    ``export_names`` defaults to the 16-channel DDEV contract; pass a longer list for
    a control object that also exports the scenario channels (pose and wheel stations),
    otherwise the solver rejects the port-count mismatch.
    """
    target_dir = Path(target_dir).resolve()
    target_dir.mkdir(parents=True, exist_ok=True)
    results: Dict[str, Any] = {}
    exports = list(export_names) if export_names is not None else list(EXPORT_NAMES)
    cases = build_validation_cases()
    for case in cases:
        csv_path = target_dir / (case["name"] + ".csv")
        results[case["name"]] = run_stepwise(
            simfile,
            command_for_case(case, start_s=start_s, stop_s=stop_s),
            csv_path,
            IMPORT_NAMES,
            exports,
            log_decimation=log_decimation,
        )
    manifest = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "simfile": str(Path(simfile).resolve()),
        "pulse_window_s": [start_s, stop_s],
        "import_names": list(IMPORT_NAMES),
        "export_names": exports,
        "cases": cases,
        "results": results,
    }
    manifest_path = target_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return manifest


def _read_csv(path: Path) -> List[Dict[str, float]]:
    with Path(path).open("r", encoding="utf-8", newline="") as stream:
        return [{key: float(value) for key, value in row.items()} for row in csv.DictReader(stream)]


def _input_integrity(rows: Sequence[Dict[str, float]], port_index: int | None) -> bool:
    columns = ["imp_" + name for name in IMPORT_NAMES]
    nonzero_seen = port_index is None
    for row in rows:
        for index, column in enumerate(columns):
            value = row[column]
            if value != 0.0:
                if index != port_index:
                    return False
                nonzero_seen = True
    return nonzero_seen


def analyze_validation_directory(target_dir: Path) -> Dict[str, Any]:
    target_dir = Path(target_dir).resolve()
    baseline = _read_csv(target_dir / "baseline.csv")
    baseline_fields = {key: [row[key] for row in baseline] for key in baseline[0]}
    cases = build_validation_cases()
    case_rows = {case["name"]: _read_csv(target_dir / (case["name"] + ".csv")) for case in cases}

    result: Dict[str, Any] = {
        "status": "PASS",
        "input_integrity": {},
        "torque": {},
        "active_suspension": {},
    }
    for case in cases:
        result["input_integrity"][case["name"]] = _input_integrity(case_rows[case["name"]], case["port_index"])

    response_floor = 1e-6
    for index, corner in enumerate(WHEEL_ORDER):
        wheel = ("L1", "R1", "L2", "R2")[index]
        pos_name = "torque_%s_pos" % corner
        neg_name = "torque_%s_neg" % corner
        field = "exp_AVy_" + wheel
        pos = signed_peak_delta(baseline_fields[field], [row[field] for row in case_rows[pos_name]])
        neg = signed_peak_delta(baseline_fields[field], [row[field] for row in case_rows[neg_name]])
        matrix = {}
        for response_index, response_wheel in enumerate(("L1", "R1", "L2", "R2")):
            response_field = "exp_AVy_" + response_wheel
            matrix[WHEEL_ORDER[response_index]] = signed_peak_delta(
                baseline_fields[response_field], [row[response_field] for row in case_rows[pos_name]]
            )
        passed = abs(pos) > response_floor and abs(neg) > response_floor and pos * neg < 0.0
        result["torque"][corner] = {
            "signal": field,
            "positive_signed_peak": pos,
            "negative_signed_peak": neg,
            "positive_response_matrix": matrix,
            "pass": passed,
        }

    for index, corner in enumerate(WHEEL_ORDER):
        wheel = ("L1", "R1", "L2", "R2")[index]
        pos_name = "active_%s_pos" % corner
        neg_name = "active_%s_neg" % corner
        signals = ["exp_Fz_" + wheel, "exp_CmpS_" + wheel, "exp_Vz_Wc_" + wheel]
        metrics = {}
        sign_pair_pass = False
        for field in signals:
            pos = signed_peak_delta(baseline_fields[field], [row[field] for row in case_rows[pos_name]])
            neg = signed_peak_delta(baseline_fields[field], [row[field] for row in case_rows[neg_name]])
            metrics[field] = {"positive_signed_peak": pos, "negative_signed_peak": neg}
            sign_pair_pass = sign_pair_pass or (abs(pos) > response_floor and abs(neg) > response_floor and pos * neg < 0.0)
        fz_matrix = {}
        for response_index, response_wheel in enumerate(("L1", "R1", "L2", "R2")):
            response_field = "exp_Fz_" + response_wheel
            fz_matrix[WHEEL_ORDER[response_index]] = signed_peak_delta(
                baseline_fields[response_field], [row[response_field] for row in case_rows[pos_name]]
            )
        result["active_suspension"][corner] = {
            "signals": metrics,
            "positive_fz_response_matrix": fz_matrix,
            "pass": sign_pair_pass,
        }

    all_ports = all(result["input_integrity"].values())
    all_responses = all(item["pass"] for item in result["torque"].values()) and all(
        item["pass"] for item in result["active_suspension"].values()
    )
    result["status"] = "PASS" if all_ports and all_responses else "FAIL"

    summary_path = target_dir / "verification_summary.json"
    summary_path.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    lines = [
        "# HD Utility DDEV 八执行器接口验证",
        "",
        "结论：**%s**。全部结果来自同一 `HD Utility Vehicle` 生成模型和同一 8 输入合同。" % result["status"],
        "",
        "## 四轮轮端转矩",
        "",
        "|轮位|+500 N·m 对应轮角速度峰值差|−500 N·m 对应轮角速度峰值差|判定|",
        "|---|---:|---:|---|",
    ]
    for corner, item in result["torque"].items():
        lines.append(
            "|%s|%.6g|%.6g|%s|" % (
                corner, item["positive_signed_peak"], item["negative_signed_peak"], "通过" if item["pass"] else "失败"
            )
        )
    lines.extend(
        [
            "",
            "## 四角主动悬架附加力",
            "",
            "|角点|+1000 N 目标角 Fz 峰值差|−1000 N 目标角 Fz 峰值差|判定|",
            "|---|---:|---:|---|",
        ]
    )
    for corner, item in result["active_suspension"].items():
        signal = next(name for name in item["signals"] if name.startswith("exp_Fz_"))
        metric = item["signals"][signal]
        lines.append(
            "|%s|%.6g|%.6g|%s|" % (
                corner, metric["positive_signed_peak"], metric["negative_signed_peak"], "通过" if item["pass"] else "失败"
            )
        )
    lines.extend(
        [
            "",
            "输入独立性判据为：每个试验 CSV 仅目标输入列出现非零值；物理响应允许通过刚性桥、车身和地面耦合传播到其他角点。",
            "",
        ]
    )
    (target_dir / "verification_report.md").write_text("\n".join(lines), encoding="utf-8")
    return result
