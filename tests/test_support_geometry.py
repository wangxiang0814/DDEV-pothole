import pytest


def test_triangle_margin_is_positive_inside_and_negative_outside():
    from ddevsim.support_geometry import support_triangle_margin_m

    contacts = {"FL": (-1.0, 1.0), "RL": (1.0, 1.0), "RR": (1.0, -1.0)}
    assert support_triangle_margin_m(contacts, (0.2, 0.2)) > 0.0
    assert support_triangle_margin_m(contacts, (0.0, 0.0)) == pytest.approx(0.0)
    assert support_triangle_margin_m(contacts, (-0.2, -0.2)) < 0.0


def test_triangle_margin_is_independent_of_contact_mapping_order():
    from ddevsim.support_geometry import support_triangle_margin_m

    one = {"FL": (-1.0, 1.0), "RL": (1.0, 1.0), "RR": (1.0, -1.0)}
    two = {"RR": (1.0, -1.0), "FL": (-1.0, 1.0), "RL": (1.0, 1.0)}
    assert support_triangle_margin_m(one, (0.2, 0.2)) == pytest.approx(
        support_triangle_margin_m(two, (0.2, 0.2))
    )


def test_degenerate_support_triangle_is_rejected():
    from ddevsim.support_geometry import support_triangle_margin_m

    with pytest.raises(ValueError, match="degenerate"):
        support_triangle_margin_m({"a": (0, 0), "b": (1, 0), "c": (2, 0)}, (1, 1))


def test_cg_projection_uses_body_roll_and_front_axle_origin():
    from ddevsim.support_geometry import cg_projection_world

    straight = cg_projection_world((100.0, 0.0), 0.9, 0.4, 0.4, 0.0, 0.0, 0.0)
    assert straight == pytest.approx((99.1, 0.0))
    leftward = cg_projection_world((100.0, 0.0), 0.9, 0.4, 0.4, -14.4775, 0.0, 0.0)
    assert leftward[1] == pytest.approx(0.1, abs=0.001)
