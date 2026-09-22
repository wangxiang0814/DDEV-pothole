from ddevsim.replay_video import ReplayStyle, select_frame_indices


def test_replay_style_uses_clear_scene_colors():
    style = ReplayStyle()

    assert style.sky_bgr == (244, 230, 204)
    assert style.background_bgr == (199, 224, 209)
    assert style.road_bgr == (82, 107, 122)
    assert style.pothole_bgr == (24, 24, 24)


def test_select_frame_indices_covers_complete_nine_second_run():
    times = [index * 0.005 for index in range(1801)]

    indices = select_frame_indices(times, fps=30)

    assert len(indices) == 271
    assert indices[0] == 0
    assert indices[-1] == 1800
