from ddevsim.pothole_case import PotholeScenario


def _row(time_s, x, fr_load=0.0, rr_load=0.0, rl_load=1000.0, fl_load=1000.0):
    row = {"time_s": str(time_s), "exp_Roll_E": "0", "exp_Yaw": "0", "exp_Yo": "0"}
    for suffix, station, lateral, load in (
        ("L1", x, .63, fl_load), ("R1", x, -.63, fr_load),
        ("L2", x - 1.9, .63, rl_load), ("R2", x - 1.9, -.63, rr_load),
    ):
        row["exp_X_" + suffix] = str(station)
        row["exp_Y_" + suffix] = str(lateral)
        row["exp_Fz_" + suffix] = str(load)
    return row


def test_front_only_crossing_fails_even_when_run_does_not_safe_stop():
    from ddevsim.traversal_metrics import evaluate_traversal

    rows = [_row(0, 101.2), _row(1, 101.5), _row(2, 101.8)]
    report = evaluate_traversal(rows, PotholeScenario(), safe_stop=False)
    assert report["passed"] is False
    assert report["FR"]["seen"] is True
    assert report["RR"]["seen"] is False


def test_zero_diagonal_support_fails_despite_both_targets_unloaded():
    from ddevsim.traversal_metrics import evaluate_traversal

    rows = [
        _row(0, 101.2, rl_load=0),
        _row(1, 101.5, rl_load=0),
        _row(2, 101.8, rl_load=0),
        _row(3, 103.1), _row(4, 103.4), _row(5, 103.7),
    ]
    report = evaluate_traversal(rows, PotholeScenario(), safe_stop=False)
    assert report["passed"] is False
    assert report["FR"]["min_support_load_n"] == 0.0


def test_completed_solver_with_failed_dynamics_has_nonzero_exit_status():
    from ddevsim.traversal_metrics import run_exit_code

    assert run_exit_code("COMPLETED", {"passed": False}) == 3
    assert run_exit_code("COMPLETED", {"passed": True}) == 0
    assert run_exit_code("ERROR", {"passed": False}) == 2


def test_rejects_bump_stop_overtravel_and_landing_impact():
    from ddevsim.traversal_metrics import evaluate_traversal

    rows = [
        _row(0, 101.2), _row(1, 101.8),
        _row(2, 103.1), _row(3, 103.8),
    ]
    for row in rows:
        for suffix in ("L1", "R1", "L2", "R2"):
            row["exp_Jnc_" + suffix] = "50"
    rows[1]["exp_Jnc_R1"] = "138"
    rows[2]["exp_Fz_R1"] = "39000"
    report = evaluate_traversal(
        rows, PotholeScenario(), safe_stop=False,
        jounce_stop_m=0.121, landing_load_limit_n=14000,
    )
    assert "jounce_stop_exceeded" in report["failure_reasons"]
    assert "landing_load_limit" in report["failure_reasons"]
    assert report["max_jounce_m"] == 0.138
    assert report["max_landing_load_n"] == 39000.0


def test_trucksim_jounce_millimetres_are_converted_before_stop_check():
    from ddevsim.traversal_metrics import evaluate_traversal

    rows = [_row(0, 101.2), _row(1, 101.8), _row(2, 103.1)]
    for row in rows:
        for suffix in ("L1", "R1", "L2", "R2"):
            row["exp_Jnc_" + suffix] = "50"
    report = evaluate_traversal(
        rows, PotholeScenario(), safe_stop=False, jounce_stop_m=0.121,
    )
    assert report["max_jounce_m"] == 0.05
    assert "jounce_stop_exceeded" not in report["failure_reasons"]
