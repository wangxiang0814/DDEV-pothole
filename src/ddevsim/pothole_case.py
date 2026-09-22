from __future__ import annotations

import re
import json
from dataclasses import dataclass
from dataclasses import asdict
from pathlib import Path
from typing import Dict

from .hd_ddev_case import parse_protected_parameters


SCENARIO_EXPORTS = (
    "Xo", "Vx", "Roll_E", "Pitch",
    # Wheel-centre stations (m).  The expert controller drives its support-phase
    # state machine from these real, solver-reported wheel positions, so no
    # axle-offset constant is hard-coded anywhere in the control path.
    "X_L1", "X_R1", "X_L2", "X_R2",
)


@dataclass(frozen=True)
class PotholeScenario:
    start_station_m: float = 101.1
    length_m: float = 1.2
    width_m: float = 0.8
    depth_m: float = 0.45
    edge_transition_m: float = 0.05
    center_y_m: float = -0.9875
    friction: float = 0.7
    road_length_m: float = 40.0
    stop_s: float = 9.0
    target_speed_kph: float = 2.8

    # ---- visual scene -----------------------------------------------------
    # Camera framing is part of the scenario because a bad lens makes the
    # manoeuvre unreadable in the exported video.  The stock camera dataset for
    # this model is a 10 deg telephoto aimed at wheel height, which frames only
    # ~2.8 m at 16 m and therefore shows a single wheel instead of the vehicle.
    # (A 10 deg lens at distance d covers 2*d*tan(5 deg).)
    camera_azimuth_deg: float = -45.0
    camera_elevation_deg: float = 16.0
    #: 32 m at a 26 deg lens frames 2*32*tan(13) = 14.8 m, so the whole vehicle plus
    #: a good stretch of road fits with margin -- a presentation shot rather than a
    #: close-up.
    camera_distance_m: float = 32.0
    camera_look_x_m: float = -2.0
    camera_look_z_m: float = 1.0
    camera_field_of_view_deg: float = 26.0

    # ---- road appearance --------------------------------------------------
    # Material names must exist in Animator/Road_Materials/road.mtl, which the block
    # loads via MTL_FILE.  The previous "No Texture" is NOT defined in that library,
    # so the surface had no usable material and the vehicle appeared to float over
    # nothing.  "Road (No Lines)" is the asphalt material and "Dirt" is used, tinted
    # dark, for the hole.
    road_half_width_m: float = 5.0
    road_color: tuple = (0.85, 0.85, 0.87)
    pothole_color: tuple = (0.30, 0.28, 0.26)
    road_material: str = "Road (No Lines)"
    hole_material: str = "Dirt"

    # ------------------------------------------------------------------ geometry
    @property
    def leading_edge_m(self) -> float:
        """Station where the depression starts (the entry lip)."""
        return self.start_station_m

    @property
    def trailing_edge_m(self) -> float:
        """Station where the depression ends (the exit lip)."""
        return self.start_station_m + self.length_m

    @property
    def lateral_min_m(self) -> float:
        return self.center_y_m - self.width_m / 2.0

    @property
    def lateral_max_m(self) -> float:
        return self.center_y_m + self.width_m / 2.0

    def station_span(self) -> tuple:
        """``(leading, trailing)`` depression stations in metres."""
        return (self.leading_edge_m, self.trailing_edge_m)

    def lateral_span(self) -> tuple:
        """``(min, max)`` lateral extent of the depression in metres."""
        return (self.lateral_min_m, self.lateral_max_m)

    def covers_station(self, station_m: float) -> bool:
        """True when ``station_m`` lies between the entry and exit lips."""
        leading, trailing = self.station_span()
        return leading <= station_m <= trailing

    def covers_point(self, station_m: float, lateral_m: float) -> bool:
        """True when a point is inside the depression footprint."""
        low, high = self.lateral_span()
        return self.covers_station(station_m) and low <= lateral_m <= high

    def wheel_span_m(self) -> float:
        """Distance a wheel must travel from lip entry to lip exit."""
        return self.length_m



def _fmt(value: float) -> str:
    text = ("%.6f" % float(value)).rstrip("0").rstrip(".")
    return "0" if text in ("-0", "") else text


def _procedure_block(scenario: PotholeScenario) -> str:
    y0 = scenario.center_y_m - scenario.width_m / 2.0
    y1 = scenario.center_y_m + scenario.width_m / 2.0
    s0 = scenario.start_station_m
    s1 = s0 + scenario.length_m
    se0 = s0 + scenario.edge_transition_m
    se1 = s1 - scenario.edge_transition_m
    z = -scenario.depth_m
    return """ENTER_PARSFILE Procedures\\DDEV_single_wheel_deep_pothole.par
#FullDataName Procedures`HD Utility DDEV - Right Single-Wheel Deep Pothole`DDEV Research
OPT_STOP 0
TSTART 0
SSTART 100
TSTART_WRITE = TSTART;
TSTOP {stop}
SSTOP = SSTART + 20;
OPT_DIRECTION 1
INSTALL_SPEED_CONTROLLER
OPT_SC 3
N_SPEED_TARGET = N_SPEED_TARGET + 1
ISPEED = N_SPEED_TARGET
SPEED_TARGET_ID = ISPEED
SPEED_ID_SC = ISPEED
SPEED_TARGET_COMBINE ADD
SPEED_TARGET_S_CONSTANT 0
set_description SPEED_TARGET_ID HD Utility DDEV crawl reference
SPEED_TARGET_CONSTANT {speed}
OPT_SC_ENGINE_BRAKING 0
ENTER_PARSFILE Control\\Steer\\DDEV_zero_steer.par
#FullDataName Control: Steering (Open Loop)`Constant: 0 deg.`DDEV Research
OPT_DRIVER_MODEL 0
OPT_STEER 0
STEER_SW_CONSTANT 0
LOG_ENTRY DDEV constant zero steering
EXIT_PARSFILE Control\\Steer\\DDEV_zero_steer.par

ENTER_PARSFILE Roads\\3D_Road\\DDEV_single_wheel_pothole.par
#FullDataName Road: 3D Surface (All Properties)`DDEV Right Single-Wheel Deep Pothole`DDEV Research
SET_IROAD_FOR_ID 0
CURRENT_ROAD_ID = ROAD_ID
set_description road_id DDEV Right Single-Wheel Deep Pothole
ENTER_PARSFILE Roads\\Builder\\DDEV_straight_40m.par
#FullDataName Path/Road: Segment Builder (Legacy)`DDEV Straight 40 m`DDEV Research
SET_IPATH_FOR_ID 0
OPT_PATH_LOOP 0
OPT_PATH_START 0
PATH_ID_DM = PATH_ID
set_description path_id DDEV Straight 40 m
SPATH_START 0
NSEGMENTS 1
IPATHSEG 1
SEGMENT_TYPE 0
SEGMENT_LENGTH {road_length}
LOG_ENTRY DDEV straight path
EXIT_PARSFILE Roads\\Builder\\DDEV_straight_40m.par

ROAD_PATH_ID = PATH_ID
set_description road_path_id DDEV Straight 40 m
ENTER_PARSFILE Roads\\dZ_Map\\DDEV_single_wheel_pothole.par
#FullDataName Road: Off-Center Elevation Map, S-L Grid`Right Single-Wheel Deep Pothole`DDEV Research
SET_IROAD_DZ_FOR_ID 0
ROAD_N_DZ = ROAD_N_DZ + 1
IDZ_ROAD = ROAD_N_DZ
RD_DZ_ID = ROAD_DZ_ID
set_description RD_DZ_ID DDEV Right Single-Wheel Deep Pothole
set_description ROAD_DZ_ID DDEV Right Single-Wheel Deep Pothole
ROAD_DZ_CARPET 2D_LINEAR
0, -10, {y0}, {ye0}, {ye1}, {y1}, 10
100, 0, 0, 0, 0, 0, 0
{s0}, 0, 0, 0, 0, 0, 0
{se0}, 0, 0, {z}, {z}, 0, 0
{se1}, 0, 0, {z}, {z}, 0, 0
{s1}, 0, 0, 0, 0, 0, 0
140, 0, 0, 0, 0, 0, 0
ENDTABLE
LOG_ENTRY DDEV physical right-track deep pothole
EXIT_PARSFILE Roads\\dZ_Map\\DDEV_single_wheel_pothole.par

ENTER_PARSFILE Roads\\Friction\\DDEV_dry_dirt_mu_070.par
#FullDataName Road: Friction Map, S-L Grid`DDEV Dry Dirt mu 0.70`DDEV Research
MU_ROAD_COMBINE ADD
MU_ROAD_CONSTANT {friction}
MU_ROAD_L_CONSTANT 0
LOG_ENTRY DDEV dry dirt friction
EXIT_PARSFILE Roads\\Friction\\DDEV_dry_dirt_mu_070.par

ENTER_PARSFILE Roads\\Shapes\\DDEV_single_wheel_pothole.par
#FullDataName Road: Animator Surface Shapes`DDEV Right Single-Wheel Deep Pothole`DDEV Research
NLANES 5
OPTTHRESHOLD 1
MIRROR 0
COLOR(1) {road_rgb}
MATERIAL(1) {road_mat}
SPECULAR(1) 1
SCALE(1) 10
LTILES(1) 100
LIN(1) {road_in}
LINUNITS(1) m
LOUT(1) {road_out}
LOUTUNITS(1) m
SSTART(1) 95
SSTOP(1) {s0}
SINT(1) 10
DZ(1) 0
COLOR(2) {road_rgb}
MATERIAL(2) {road_mat}
SPECULAR(2) 1
SCALE(2) 10
LTILES(2) 100
LIN(2) {road_in}
LINUNITS(2) m
LOUT(2) {road_out}
LOUTUNITS(2) m
SSTART(2) {s1}
SSTOP(2) 140
SINT(2) 10
DZ(2) 0
COLOR(3) {road_rgb}
MATERIAL(3) {road_mat}
SPECULAR(3) 1
SCALE(3) 10
LTILES(3) 100
LIN(3) {y1}
LINUNITS(3) m
LOUT(3) {road_out}
LOUTUNITS(3) m
SSTART(3) {s0}
SSTOP(3) {s1}
SINT(3) 10
DZ(3) 0
COLOR(4) {road_rgb}
MATERIAL(4) {road_mat}
SPECULAR(4) 1
SCALE(4) 10
LTILES(4) 100
LIN(4) {road_in}
LINUNITS(4) m
LOUT(4) {y0}
LOUTUNITS(4) m
SSTART(4) {s0}
SSTOP(4) {s1}
SINT(4) 10
DZ(4) 0
COLOR(5) {hole_rgb}
MATERIAL(5) {hole_mat}
SPECULAR(5) 0
SCALE(5) 10
LTILES(5) 100
LIN(5) {y0}
LINUNITS(5) m
LOUT(5) {y1}
LOUTUNITS(5) m
SSTART(5) {s0}
SSTOP(5) {s1}
SINT(5) 10
DZ(5) {z}
MTL_FILE Animator/Road_Materials/road.mtl
LOG_ENTRY DDEV visual right-track deep pothole
EXIT_PARSFILE Roads\\Shapes\\DDEV_single_wheel_pothole.par

RR_SURF 1.0
L_CAMERA_FRONT 0.5
L_CAMERA_REAR 0.5
add_reference_frame DDEV_Sky_and_Background
reference_frame_ghosts off
ENTER_PARSFILE Animator\\Groups\\DDEV_clear_sky_light_grass.par
#FullDataName Animator: Group`Partly Cloudy Sky with Light Grass`Environment Spheres
ENTER_PARSFILE Animator\\Frames\\DDEV_skybox_vehicle_xyz.par
#FullDataName Animator: Reference Frame`Skybox (Vehicle X-Y-Z)`Tracking: Vehicle
ADD_REFERENCE_FRAME DDEV_skybox_vehicle_xyz
SET_EULER_ANGLES yaw_pitch_roll
SET_X_NAME Xo
SET_Y_NAME Yo
SET_Z_NAME Zo
WRT_Xo
ANI_Xo
WRT_Yo
ANI_Yo
WRT_Zo
ANI_Zo
LOG_ENTRY DDEV tracking skybox
EXIT_PARSFILE Animator\\Frames\\DDEV_skybox_vehicle_xyz.par

ENTER_PARSFILE Animator\\STL\\DDEV_partly_cloudy_sky.par
#FullDataName Animator: Shape File Link`Partly Cloudy Sky`Environment: Sky Boxes
add_obj Animator\\3D_Shape_Files\\Environment\\Sky_Boxes\\Partly_Cloudy_Sky\\Sky_Partly_Cloudy.obj
SET_COLOR 1 1 1
SET_SCALE_X 15
SET_SCALE_Y 15
SET_SCALE_Z 5
SET_ANGLE_Z 180
set_lighting off
set_fogging off
SUN_POSITION 250 -250 600
cam_global_ambient .65 .65 .65 1
cam_global_diffuse .9 .9 .9 1
cam_global_specular .7 .7 .7 1
ENV_MAP_XPOS Animator\\3D_Shape_Files\\Environment\\Sky_Boxes\\Partly_Cloudy_Sky\\cubeXPos.tga
ENV_MAP_XNEG Animator\\3D_Shape_Files\\Environment\\Sky_Boxes\\Partly_Cloudy_Sky\\cubeXNeg.tga
ENV_MAP_YPOS Animator\\3D_Shape_Files\\Environment\\Sky_Boxes\\Partly_Cloudy_Sky\\cubeYPos.tga
ENV_MAP_YNEG Animator\\3D_Shape_Files\\Environment\\Sky_Boxes\\Partly_Cloudy_Sky\\cubeYNeg.tga
ENV_MAP_ZPOS Animator\\3D_Shape_Files\\Environment\\Sky_Boxes\\Partly_Cloudy_Sky\\cubeZPos.tga
ENV_MAP_ZNEG Animator\\3D_Shape_Files\\Environment\\Sky_Boxes\\Partly_Cloudy_Sky\\cubeZNeg.tga
LOG_ENTRY DDEV partly cloudy sky
EXIT_PARSFILE Animator\\STL\\DDEV_partly_cloudy_sky.par

ENTER_PARSFILE Animator\\STL\\DDEV_light_grass_background.par
#FullDataName Animator: Shape File Link`Light Grass`Environment: Land Bowls
add_obj Animator\\3D_Shape_Files\\Environment\\Land_Bowls\\Light_Grass\\Land_Bowl_Light_Grass.obj
SET_COLOR 0.82 0.88 0.78
SET_SCALE_X 15
SET_SCALE_Y 15
SET_SCALE_Z 1
set_lighting off
set_fogging on
LOG_ENTRY DDEV light grass background
EXIT_PARSFILE Animator\\STL\\DDEV_light_grass_background.par
LOG_ENTRY Used Dataset: Animator: Group; {{ Environment Spheres }} Partly Cloudy Sky with Light Grass
EXIT_PARSFILE Animator\\Groups\\DDEV_clear_sky_light_grass.par

LOG_ENTRY Used Dataset: Road: 3D Surface (All Properties); {{ DDEV Research }} Right Single-Wheel Deep Pothole
EXIT_PARSFILE Roads\\3D_Road\\DDEV_single_wheel_pothole.par

LOG_ENTRY Used Dataset: Procedures; {{ DDEV Research }} HD Utility DDEV - Right Single-Wheel Deep Pothole
EXIT_PARSFILE Procedures\\DDEV_single_wheel_deep_pothole.par""".format(
        stop=_fmt(scenario.stop_s),
        speed=_fmt(scenario.target_speed_kph),
        road_length=_fmt(scenario.road_length_m),
        y0=_fmt(y0),
        y1=_fmt(y1),
        ye0=_fmt(y0 + scenario.edge_transition_m),
        ye1=_fmt(y1 - scenario.edge_transition_m),
        s0=_fmt(s0),
        se0=_fmt(se0),
        se1=_fmt(se1),
        s1=_fmt(s1),
        z=_fmt(z),
        friction=_fmt(scenario.friction),
        road_in=_fmt(-abs(scenario.road_half_width_m)),
        road_out=_fmt(abs(scenario.road_half_width_m)),
        road_rgb="%.3f %.3f %.3f" % tuple(scenario.road_color),
        hole_rgb="%.3f %.3f %.3f" % tuple(scenario.pothole_color),
        road_mat=scenario.road_material,
        hole_mat=scenario.hole_material,
    )


def transform_single_wheel_pothole(source: str, scenario: PotholeScenario) -> str:
    protected_before = parse_protected_parameters(source)
    text = source
    replacements = (
        (
            r"(?m)^SET_AZIMUTH\s+[-+0-9.eE]+\s*$",
            "SET_AZIMUTH %s" % _fmt(scenario.camera_azimuth_deg),
            "camera azimuth", True,
        ),
        (
            r"(?m)^SET_ELEVATION\s+[-+0-9.eE]+\s*$",
            "SET_ELEVATION %s" % _fmt(scenario.camera_elevation_deg),
            "camera elevation", True,
        ),
        (
            r"(?m)^SET_DISTANCE\s+[-+0-9.eE]+\s*$",
            "SET_DISTANCE %s" % _fmt(scenario.camera_distance_m),
            "camera distance", True,
        ),
        # The look point must sit at mid-body height, not wheel height: the stock
        # value is 0.25 m, which frames the wheels and cuts the vehicle in half.
        (
            r"(?m)^SET_LOOKPOINT_X\s+[-+0-9.eE]+\s*$",
            "SET_LOOKPOINT_X %s" % _fmt(scenario.camera_look_x_m),
            "camera look point X", False,
        ),
        (
            r"(?m)^SET_LOOKPOINT_Z\s+[-+0-9.eE]+\s*$",
            "SET_LOOKPOINT_Z %s" % _fmt(scenario.camera_look_z_m),
            "camera look point Z", False,
        ),
        # A 10 deg lens covers only 2*d*tan(5 deg) = 2.8 m at 16 m, which cannot
        # contain a 5.5 m vehicle; this was why the exported video showed a wheel
        # instead of the truck.
        (
            r"(?m)^SET_FIELD_OF_VIEW\s+[-+0-9.eE]+\s*$",
            "SET_FIELD_OF_VIEW %s" % _fmt(scenario.camera_field_of_view_deg),
            "camera field of view", False,
        ),
    )
    for pattern, replacement, label, required in replacements:
        text, count = re.subn(pattern, replacement, text, count=1)
        if required and count != 1:
            raise ValueError("expected exactly one %s" % label)
        if count > 1:
            raise ValueError("expected at most one %s, found %d" % (label, count))
    pattern = (
        r"ENTER_PARSFILE\s+Procedures\\[^\r\n]+\r?\n"
        r".*?"
        r"EXIT_PARSFILE\s+Procedures\\[^\r\n]+"
    )
    text, count = re.subn(pattern, lambda _match: _procedure_block(scenario), text, count=1, flags=re.DOTALL)
    if count != 1:
        raise ValueError("expected exactly one procedure block")
    if parse_protected_parameters(text) != protected_before:
        raise ValueError("protected vehicle parameters changed while building pothole scene")
    return text


def build_single_wheel_pothole_case(
    source_run_all: Path,
    source_simfile: Path,
    target_dir: Path,
    scenario: PotholeScenario | None = None,
    history_name: str = "single_wheel_deep_pothole",
) -> Dict[str, Path]:
    """Build a self-contained pothole case from the base DDEV model.

    ``history_name`` becomes the TruckSim history basename (``FILEBASE`` and every
    derived output path).  Batch runs pass a per-case name so that cases executed
    in parallel cannot overwrite each other's native history, echo or ERD files.
    """
    scenario = scenario or PotholeScenario()
    target_dir = Path(target_dir).resolve()
    target_dir.mkdir(parents=True, exist_ok=True)
    output_dir = target_dir / "output"
    output_dir.mkdir(parents=True, exist_ok=True)

    source_text = Path(source_run_all).read_text(encoding="utf-8", errors="replace")
    transformed = transform_single_wheel_pothole(source_text, scenario)
    final_end = transformed.rfind("\nEND")
    if final_end < 0:
        raise ValueError("scenario run has no final END")
    scenario_exports = "\n".join("EXPORT " + name for name in SCENARIO_EXPORTS)
    transformed = transformed[:final_end] + "\n" + scenario_exports + transformed[final_end:]
    run_all = target_dir / "run_all.par"
    run_all.write_text(transformed, encoding="utf-8")
    simfile_text = Path(source_simfile).read_text(encoding="ascii")
    if "hd_utility_ddev" not in simfile_text:
        raise ValueError("source simfile does not use expected hd_utility_ddev history basename")
    simfile_text = simfile_text.replace("hd_utility_ddev", history_name)
    base_exports = 16  # the fixed DDEV feedback contract
    total_exports = base_exports + len(SCENARIO_EXPORTS)
    simfile_text, port_count = re.subn(
        r"(?m)^PORTS_EXP\s+\d+\s*$", "PORTS_EXP %d" % total_exports, simfile_text
    )
    if port_count != 1:
        raise ValueError("source simfile must declare exactly one PORTS_EXP")
    simfile = target_dir / "simfile.sim"
    simfile.write_text(simfile_text, encoding="ascii")
    scenario_path = target_dir / "scenario.json"
    scenario_path.write_text(json.dumps(asdict(scenario), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return {
        "run_all": run_all,
        "simfile": simfile,
        "scenario": scenario_path,
        "output_dir": output_dir,
    }
