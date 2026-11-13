from ddevsim.vehicle_profiles.tuned_ii import DEFAULT_PROFILE, center_rear_payload


def test_centered_payload_preserves_mass_and_other_vehicle_parameters():
    import re

    source = (DEFAULT_PROFILE.source_dir / "run_all.par").read_text(encoding="utf-8")
    centered = center_rear_payload(source)
    assert re.findall(r"(?m)^Y_CG_PL (.*)$", centered) == ["375", "-375", "0"]
    assert re.findall(r"(?m)^M_PL (.*)$", centered) == ["200"] * 3
    changes = [(a, b) for a, b in zip(source.splitlines(), centered.splitlines()) if a != b]
    assert set(changes) == {("Y_CG_PL 500", "Y_CG_PL 0"),
                            ("SET_OFFSET_Y 0.5", "SET_OFFSET_Y 0")}
    assert len(changes) == 2
