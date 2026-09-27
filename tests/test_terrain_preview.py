from ddevsim.pothole_case import corner_module_scenario
from ddevsim.terrain_preview import TerrainPreview


def test_ground_truth_preview_exposes_geometry_and_wheel_relative_distance():
    preview = TerrainPreview.from_scenario(corner_module_scenario(), timestamp_s=1.25)
    assert preview.source == "ground_truth"
    assert preview.confidence == 1.0
    assert preview.timestamp_s == 1.25
    assert round(preview.distance_to_entry(100.50), 2) == 0.60
    assert preview.contains_wheel(101.50, -0.63)
    assert not preview.contains_wheel(101.50, 0.63)


def test_estimated_preview_uses_the_same_controller_contract():
    preview = TerrainPreview(
        leading_edge_m=102.0,
        trailing_edge_m=103.0,
        lateral_min_m=-1.0,
        lateral_max_m=-0.2,
        depth_m=0.18,
        friction=0.6,
        confidence=0.8,
        timestamp_s=2.0,
        source="perception_estimate",
    )
    assert round(preview.distance_to_entry(101.2), 2) == 0.8
    assert preview.contains_wheel(102.5, -0.6)
