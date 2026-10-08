from dataclasses import replace
import pytest


def make_case(coverage='right_track', **changes):
    from ddevsim.scenarios.pit import PitCase
    from ddevsim.pothole_case import corner_module_scenario
    geometry = corner_module_scenario(depth_m=.05, width_m=.9 if coverage == 'right_track' else 1.8,
                                     center_y_m=-.63 if coverage == 'right_track' else 0.)
    return PitCase('shallow_press', coverage, 'tuned_ii_20261005', coverage,
                   replace(geometry, **changes), {'speed_mps': 0., 'vehicle_start_station_m': 99.1}, {})


def test_right_track_excludes_left_footprints_and_wide_covers_both():
    from ddevsim.scenarios.pit import validate_case
    from ddevsim.vehicle_profiles.tuned_ii import DEFAULT_PROFILE
    right, wide = make_case(), make_case('both_tracks')
    validate_case(right, DEFAULT_PROFILE)
    validate_case(wide, DEFAULT_PROFILE)
    assert right.wheel_sequence == (('FR',), ('RR',))
    assert wide.wheel_sequence == (('FL', 'FR'), ('RL', 'RR'))


@pytest.mark.parametrize('coverage,changes', [
    ('both_tracks', {'width_m': .9, 'center_y_m': -.63}),
    ('right_track', {'width_m': 2., 'center_y_m': 0.}),
    ('right_track', {'center_y_m': .63}),
    ('right_track', {'width_m': .1}),
    ('right_track', {'length_m': .1}),
    ('right_track', {'depth_m': float('nan')}),
    ('right_track', {'depth_m': -.1}),
    ('right_track', {'friction': 0.}),
])
def test_bad_coverage_or_geometry_rejected(coverage, changes):
    from ddevsim.scenarios.pit import validate_case
    from ddevsim.vehicle_profiles.tuned_ii import DEFAULT_PROFILE
    with pytest.raises(ValueError):
        validate_case(make_case(coverage, **changes), DEFAULT_PROFILE)


def test_all_wheels_start_before_entry_and_at_rest():
    from ddevsim.scenarios.pit import validate_case
    from ddevsim.vehicle_profiles.tuned_ii import DEFAULT_PROFILE
    case = make_case()
    for initial in ({'speed_mps': .1, 'vehicle_start_station_m': 99.1},
                    {'speed_mps': 0., 'vehicle_start_station_m': 101.},
                    {'speed_mps': 0., 'vehicle_start_station_m': float('nan')}):
        with pytest.raises(ValueError):
            validate_case(replace(case, initial_state=initial), DEFAULT_PROFILE)
