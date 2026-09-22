from __future__ import annotations

import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple


@dataclass(frozen=True)
class ReplayStyle:
    """BGR colors chosen to keep the TruckSim engineering replay legible."""

    sky_bgr: Tuple[int, int, int] = (244, 230, 204)
    background_bgr: Tuple[int, int, int] = (199, 224, 209)
    road_bgr: Tuple[int, int, int] = (82, 107, 122)
    pothole_bgr: Tuple[int, int, int] = (24, 24, 24)
    truck_bgr: Tuple[int, int, int] = (38, 126, 214)
    glass_bgr: Tuple[int, int, int] = (160, 205, 225)


def select_frame_indices(times: Sequence[float], fps: int = 30) -> List[int]:
    if not times:
        return []
    if fps <= 0:
        raise ValueError("fps must be positive")
    start = float(times[0])
    stop = float(times[-1])
    count = int(round((stop - start) * fps)) + 1
    result = []
    for frame in range(count):
        target = min(stop, start + frame / float(fps))
        right = bisect.bisect_left(times, target)
        if right <= 0:
            result.append(0)
        elif right >= len(times):
            result.append(len(times) - 1)
        else:
            left = right - 1
            result.append(left if target - times[left] <= times[right] - target else right)
    result[-1] = len(times) - 1
    return result


def load_history(csv_path: Path) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    with Path(csv_path).open("r", encoding="utf-8", newline="") as stream:
        for row in csv.DictReader(stream):
            rows.append({key: float(value) for key, value in row.items()})
    if not rows:
        raise ValueError("history CSV is empty")
    return rows


def _rotation(roll_deg: float, pitch_deg: float):
    import numpy as np

    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    rx = np.array(((1, 0, 0), (0, cr, -sr), (0, sr, cr)), dtype=float)
    ry = np.array(((cp, 0, sp), (0, 1, 0), (-sp, 0, cp)), dtype=float)
    return ry.dot(rx)


def _projector(vehicle_x: float, width: int, height: int):
    import numpy as np

    camera = np.array((vehicle_x - 9.5, -10.5, 6.2), dtype=float)
    target = np.array((vehicle_x + 0.7, 0.0, 0.7), dtype=float)
    forward = target - camera
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.array((0.0, 0.0, 1.0)))
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    focal = width * 0.88

    def project(point: Iterable[float]):
        relative = np.asarray(tuple(point), dtype=float) - camera
        depth = float(relative.dot(forward))
        if depth <= 0.05:
            return None
        return (
            int(width * 0.50 + focal * float(relative.dot(right)) / depth),
            int(height * 0.49 - focal * float(relative.dot(up)) / depth),
        )

    return project


def _polygon(cv2, image, project, points, color, outline=(45, 45, 45), thickness=1):
    import numpy as np

    pixels = [project(point) for point in points]
    if any(pixel is None for pixel in pixels):
        return
    array = np.asarray(pixels, dtype=np.int32)
    cv2.fillConvexPoly(image, array, color, lineType=cv2.LINE_AA)
    if outline is not None:
        cv2.polylines(image, [array], True, outline, thickness, cv2.LINE_AA)


def render_engineering_replay(
    csv_path: Path,
    output_path: Path,
    *,
    fps: int = 30,
    width: int = 1280,
    height: int = 720,
    style: ReplayStyle | None = None,
) -> Dict[str, object]:
    """Render a labelled MP4 from the completed TruckSim run history.

    This is an engineering data replay, deliberately distinct from the native
    VS Visualizer AVI export. It uses the actual TruckSim states and commands.
    """

    import cv2
    import numpy as np

    style = style or ReplayStyle()
    rows = load_history(csv_path)
    times = [row["time_s"] for row in rows]
    indices = select_frame_indices(times, fps=fps)
    output_path = Path(output_path).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(output_path), cv2.VideoWriter_fourcc(*"mp4v"), float(fps), (width, height)
    )
    if not writer.isOpened():
        raise RuntimeError("OpenCV could not open the MP4 video writer")

    pit_x0, pit_x1 = 101.1, 102.3
    pit_y0, pit_y1 = -1.3875, -0.5875
    wheel_offsets = {
        "FL": (1.4, 0.9875),
        "FR": (1.4, -0.9875),
        "RL": (-2.5, 0.9875),
        "RR": (-2.5, -0.9875),
    }
    # TruckSim names are axle-based (L1/R1/L2/R2), while the overlay uses FL/FR/RL/RR.
    channels = {"FL": "L1", "FR": "R1", "RL": "L2", "RR": "R2"}
    initial_cmp = {wheel: rows[0]["exp_CmpS_" + channel] for wheel, channel in channels.items()}
    thumbnail = None
    try:
        for frame_number, row_index in enumerate(indices):
            row = rows[row_index]
            vehicle_x = row["exp_Xo"]
            project = _projector(vehicle_x, width, height)
            image = np.full((height, width, 3), style.sky_bgr, dtype=np.uint8)

            # Distant light-grass environment and a brown-gray unpaved road.
            cv2.rectangle(image, (0, int(height * 0.36)), (width, height), style.background_bgr, -1)
            _polygon(
                cv2,
                image,
                project,
                ((vehicle_x - 18, -3.2, 0), (vehicle_x + 34, -3.2, 0),
                 (vehicle_x + 34, 3.2, 0), (vehicle_x - 18, 3.2, 0)),
                style.road_bgr,
                outline=(62, 75, 78),
                thickness=2,
            )
            for lateral in (-3.2, 3.2):
                p0, p1 = project((vehicle_x - 15, lateral, 0.01)), project((vehicle_x + 30, lateral, 0.01))
                if p0 and p1:
                    cv2.line(image, p0, p1, (225, 225, 220), 2, cv2.LINE_AA)

            # The only depression is in the right wheel track.
            _polygon(
                cv2,
                image,
                project,
                ((pit_x0, pit_y0, 0.01), (pit_x1, pit_y0, 0.01),
                 (pit_x1, pit_y1, 0.01), (pit_x0, pit_y1, 0.01)),
                style.pothole_bgr,
                outline=(8, 8, 8),
                thickness=3,
            )
            for edge_a, edge_b in (
                ((pit_x0, pit_y0, 0), (pit_x0, pit_y0, -0.45)),
                ((pit_x1, pit_y0, 0), (pit_x1, pit_y0, -0.45)),
                ((pit_x0, pit_y1, 0), (pit_x0, pit_y1, -0.45)),
                ((pit_x1, pit_y1, 0), (pit_x1, pit_y1, -0.45)),
            ):
                a, b = project(edge_a), project(edge_b)
                if a and b:
                    cv2.line(image, a, b, (5, 5, 5), 2, cv2.LINE_AA)

            # Sprung-mass pose follows the actual TruckSim roll and pitch outputs.
            center = np.array((vehicle_x - 0.55, 0.0, 1.42), dtype=float)
            rotation = _rotation(row["exp_Roll_E"], row["exp_Pitch"])
            local_corners = np.array(
                [(x, y, z) for x in (-2.5, 2.5) for y in (-1.1, 1.1) for z in (-0.55, 0.55)],
                dtype=float,
            )
            corners = [center + rotation.dot(point) for point in local_corners]
            faces = (
                ((0, 2, 6, 4), style.truck_bgr),
                ((0, 1, 3, 2), (32, 100, 176)),
                ((4, 6, 7, 5), (56, 148, 232)),
                ((2, 3, 7, 6), style.glass_bgr),
                ((0, 4, 5, 1), (30, 88, 154)),
                ((1, 5, 7, 3), (46, 115, 190)),
            )
            for face, color in faces:
                _polygon(cv2, image, project, [corners[index] for index in face], color, thickness=2)

            # Four independently controlled corners; right-side wheels are emphasized at the pit.
            for wheel, (x_offset, y_offset) in wheel_offsets.items():
                channel = channels[wheel]
                wheel_x = vehicle_x + x_offset
                road_z = -0.45 if (pit_x0 <= wheel_x <= pit_x1 and pit_y0 <= y_offset <= pit_y1) else 0.0
                suspension_delta = (initial_cmp[wheel] - row["exp_CmpS_" + channel]) * 0.001
                wheel_z = road_z + 0.56 + max(-0.18, min(0.18, suspension_delta))
                pixel = project((wheel_x, y_offset, wheel_z))
                upper = project((wheel_x, y_offset, 1.26))
                if pixel and upper:
                    cv2.line(image, upper, pixel, (80, 80, 80), 4, cv2.LINE_AA)
                if pixel:
                    depth_scale = max(18, int(34 - 4 * y_offset))
                    cv2.ellipse(image, pixel, (depth_scale, int(depth_scale * 0.68)), 0, 0, 360,
                                (20, 20, 20), -1, cv2.LINE_AA)
                    cv2.ellipse(image, pixel, (int(depth_scale * 0.48), int(depth_scale * 0.32)),
                                0, 0, 360, (150, 150, 150), 2, cv2.LINE_AA)
                    if wheel in ("FR", "RR") and pit_x0 - 0.2 <= wheel_x <= pit_x1 + 0.2:
                        cv2.ellipse(image, pixel, (depth_scale + 7, int(depth_scale * 0.68) + 7),
                                    0, 0, 360, (45, 45, 230), 3, cv2.LINE_AA)

            # Engineering overlay uses the actual eight commands and primary states.
            overlay = image.copy()
            cv2.rectangle(overlay, (22, 18), (500, 194), (18, 25, 32), -1)
            cv2.addWeighted(overlay, 0.78, image, 0.22, 0, image)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(image, "TruckSim DDEV engineering replay", (42, 48), font, 0.72,
                        (255, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(image, "single right-wheel-track deep pothole", (42, 76), font, 0.55,
                        (210, 225, 240), 1, cv2.LINE_AA)
            cv2.putText(image, "t = %.2f s   X = %.2f m   Vx = %.2f km/h" %
                        (row["time_s"], vehicle_x, row["exp_Vx"]), (42, 108), font, 0.52,
                        (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(image, "roll = %.2f deg   pitch = %.2f deg" %
                        (row["exp_Roll_E"], row["exp_Pitch"]), (42, 134), font, 0.52,
                        (255, 255, 255), 1, cv2.LINE_AA)
            torques = [row["imp_IMP_MYUSM_" + channels[w]] for w in ("FL", "FR", "RL", "RR")]
            forces = [row["imp_IMP_FS_" + channels[w]] for w in ("FL", "FR", "RL", "RR")]
            cv2.putText(image, "T [FL FR RL RR] = " + " ".join("%5.0f" % x for x in torques) + " Nm",
                        (42, 160), font, 0.48, (230, 245, 255), 1, cv2.LINE_AA)
            cv2.putText(image, "Fas [FL FR RL RR] = " + " ".join("%6.0f" % x for x in forces) + " N",
                        (42, 184), font, 0.48, (230, 245, 255), 1, cv2.LINE_AA)
            cv2.putText(image, "Actual TruckSim states/commands | programmatic replay (not native Visualizer AVI)",
                        (28, height - 22), font, 0.46, (30, 30, 30), 1, cv2.LINE_AA)
            cv2.putText(image, "RIGHT TRACK POTHOLE", (width - 300, 42), font, 0.58,
                        (20, 20, 200), 2, cv2.LINE_AA)
            writer.write(image)
            if thumbnail is None and row["time_s"] >= 2.0:
                thumbnail = image.copy()
    finally:
        writer.release()

    thumbnail_path = output_path.with_suffix(".png")
    if thumbnail is not None:
        encoded, buffer = cv2.imencode(".png", thumbnail)
        if not encoded:
            raise RuntimeError("OpenCV could not encode the replay thumbnail")
        thumbnail_path.write_bytes(buffer.tobytes())
    return {
        "video": str(output_path),
        "thumbnail": str(thumbnail_path),
        "fps": fps,
        "resolution": [width, height],
        "frames": len(indices),
        "duration_s": times[-1] - times[0],
        "source_csv": str(Path(csv_path).resolve()),
        "kind": "TruckSim engineering data replay; not native VS Visualizer export",
    }
