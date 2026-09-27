import pytest


def test_preload_roll_target_keeps_four_loads_near_static():
    from ddevsim.identified_suspension import allocate_identified_force

    corners = ("FL", "FR", "RL", "RR")
    static = dict.fromkeys(corners, 3000.0)
    gains = {c: {d: float(c == d) for d in corners} for c in corners}
    result = allocate_identified_force(
        measured_load_n=static, target_load_n=static,
        measured_roll_deg=0.0, target_roll_deg=-2.0,
        gain_command_to_load=gains,
        roll_gain_deg_per_n={"FL": 0.001, "FR": -0.001,
                             "RL": 0.001, "RR": -0.001},
        previous_force_n=dict.fromkeys(corners, 0.0),
        force_min_n=-1000.0, force_max_n=1000.0,
        max_force_step_n=1000.0,
    )
    assert result["FL"] < 0.0 and result["RL"] < 0.0
    assert result["FR"] > 0.0 and result["RR"] > 0.0
    assert max(abs(value) for value in result.values()) <= 1000.0


def test_lifted_load_and_support_floor_compete_inside_force_step_bounds():
    from ddevsim.identified_suspension import allocate_identified_force

    corners = ("FL", "FR", "RL", "RR")
    gains = {c: {d: float(c == d) for d in corners} for c in corners}
    current = {"FL": 3000.0, "FR": 1000.0, "RL": 100.0, "RR": 3000.0}
    target = {"FL": 3000.0, "FR": 0.0, "RL": 500.0, "RR": 3000.0}
    result = allocate_identified_force(
        measured_load_n=current, target_load_n=target,
        measured_roll_deg=0.0, target_roll_deg=0.0,
        gain_command_to_load=gains,
        roll_gain_deg_per_n=dict.fromkeys(corners, 0.0),
        previous_force_n=dict.fromkeys(corners, 0.0),
        force_min_n=-2000.0, force_max_n=2000.0,
        max_force_step_n=100.0,
    )
    assert result["FR"] == pytest.approx(-100.0)
    assert result["RL"] == pytest.approx(100.0)


def test_signed_probe_roll_response_is_normalized_by_force_amplitude():
    from ddevsim.identified_suspension import roll_gain_from_probe

    report = {"force_amplitude_n": 2000.0,
              "roll_response_deg": {"FL": .4, "FR": -.4,
                                    "RL": .8, "RR": -.8}}
    assert roll_gain_from_probe(report)["RL"] == pytest.approx(.0004)


def test_negative_roll_rate_brakes_a_negative_roll_target():
    from ddevsim.identified_suspension import allocate_identified_force

    corners = ("FL", "FR", "RL", "RR")
    static = dict.fromkeys(corners, 3000.0)
    gains = {c: {d: float(c == d) for d in corners} for c in corners}
    kwargs = dict(
        measured_load_n=static, target_load_n=static,
        measured_roll_deg=-3.0, target_roll_deg=-5.0,
        gain_command_to_load=gains,
        roll_gain_deg_per_n={"FL": .001, "FR": -.001,
                             "RL": .001, "RR": -.001},
        previous_force_n=dict.fromkeys(corners, 0.0),
        force_min_n=-1000.0, force_max_n=1000.0,
        max_force_step_n=1000.0,
    )
    undamped = allocate_identified_force(**kwargs)
    damped = allocate_identified_force(**kwargs, measured_roll_rate_deg_s=-20.0,
                                       roll_rate_damping_s=0.15)
    assert damped["FL"] > undamped["FL"]
    assert damped["RR"] < undamped["RR"]
