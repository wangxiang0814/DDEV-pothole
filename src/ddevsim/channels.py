from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict


WHEEL_ORDER = ("FL", "FR", "RL", "RR")


def _channel(name: str, unit: str, mode: str, status: str, evidence: str) -> Dict[str, str]:
    return {
        "trucksim_name": name,
        "unit": unit,
        "mode": mode,
        "verification_status": status,
        "evidence": evidence,
    }


def build_trucksim2019_contract() -> Dict[str, Any]:
    """Return the explicit logical-to-TruckSim channel contract.

    The contract is shared by Python, Simulink and the generated TruckSim case.
    The checked-in status is backed by the retained same-vehicle pulse matrix in
    runs/hd_utility_ddev_interface_validation/verification_summary.json.
    """
    torque_evidence = (
        "TruckSim2019 Generic/VS_commands in-wheel motor example: "
        "IMP_MYUSM_L1/R1; Run_imp_tab.txt: unsprung-mass/wheel reaction torque, N-m"
    )
    active_force_evidence = (
        "TruckSim2019 Generic/VS_commands lift-axle example: IMP_FS_L2/R2; "
        "Run_imp_tab.txt: suspension spring force contribution, N"
    )
    passed = "same_vehicle_pulse_passed"

    return {
        "schema_version": "2.0",
        "trucksim_version": "2019.0",
        "wheel_order": list(WHEEL_ORDER),
        "sign_conventions": {
            "vehicle_axes": "+X forward, +Y left, +Z up (must be reconfirmed for selected dataset)",
            "wheel_torque": "positive sign is confirmed by generated same-vehicle pulse report",
            "active_force": "positive sign is confirmed by generated same-vehicle pulse report; ADD retains passive suspension",
        },
        "imports": {
            "wheel_torque": {
                "FL": _channel("IMP_MYUSM_L1", "N-m", "ADD", passed, torque_evidence),
                "FR": _channel("IMP_MYUSM_R1", "N-m", "ADD", passed, torque_evidence),
                "RL": _channel("IMP_MYUSM_L2", "N-m", "ADD", passed, torque_evidence),
                "RR": _channel("IMP_MYUSM_R2", "N-m", "ADD", passed, torque_evidence),
            },
            "active_suspension_force": {
                "FL": _channel("IMP_FS_L1", "N", "ADD", passed, active_force_evidence),
                "FR": _channel("IMP_FS_R1", "N", "ADD", passed, active_force_evidence),
                "RL": _channel("IMP_FS_L2", "N", "ADD", passed, active_force_evidence),
                "RR": _channel("IMP_FS_R2", "N", "ADD", passed, active_force_evidence),
            },
        },
        "required_outputs": {
            "vehicle": [
                "time_s", "station_m", "speed_mps", "accel_xyz_mps2", "roll_pitch_yaw_rad",
                "roll_pitch_yaw_rate_radps",
            ],
            "per_wheel": [
                "wheel_load_n", "longitudinal_force_n", "lateral_force_n", "wheel_speed_radps",
                "longitudinal_slip", "contact_flag", "wheel_center_xyz_m",
                "suspension_deflection_m", "suspension_velocity_mps",
            ],
            "controller": [
                "expert_mode", "requested_torque_nm", "filtered_torque_nm", "applied_torque_nm",
                "requested_active_force_n", "filtered_active_force_n", "applied_active_force_n",
                "safety_reason_codes",
            ],
            "resolution_status": "exact same-vehicle imports and 16 solver exports are fixed; runtime status is reported separately",
        },
        "limitations": [
            "IMP_MYUSM applies commanded wheel/unsprung-mass reaction torque; electric motor electromagnetic and thermal dynamics remain external.",
            "IMP_FS adds force at each spring location while the original passive spring and damper remain active.",
            "The HD Utility Vehicle uses solid axles, so independent commands produce mechanically coupled responses.",
            "A generated pulse-test report, not this static contract, is the authority for runtime verification status.",
        ],
    }


def write_contract(path: Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(build_trucksim2019_contract(), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return path
