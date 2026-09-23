"""Paper-faithful expert strategy for a single-wheel deep pothole.

Reference
---------
S. Liu, L. Zhang, Y. Liu, J. Wang, C. Yang, J. Zhang, "Motion Posture Control of
Corner Module Architecture Intelligent Electric Vehicle on Deep-Potholed Roads",
*IEEE/ASME Transactions on Mechatronics*, vol. 29, no. 6, pp. 4480-4491, 2024.

Wheel numbering in the paper is ``1 = left front, 2 = right front, 3 = left
rear, 4 = right rear``, which is exactly this platform's ``FL, FR, RL, RR``.  The
paper's scenario (a deep pothole ahead of wheel 2, crossed by wheels 2 then 4) is
therefore exactly this platform's right-track single-wheel pothole.

The paper's mechanism, and what this module reproduces
------------------------------------------------------
A vehicle cannot be supported on three wheels with the CG on the FL-RR diagonal:
solving the three-wheel equilibrium for this vehicle gives a *negative* load on
the left-rear wheel::

    contacts FL, RL, RR (wheel FR lifted)
    pitch balance : F_FL = W * b / L
    roll  balance : F_RR = W * c / B
    F_RL          = W - F_FL - F_RR

With the measured static loads of this model (W = 87296 N, L = 3.9 m,
B = 1.975 m) the CG sits 2.0034 m from the rear axle, so ``W*b/L`` exceeds
``W/2`` and ``F_RL`` comes out at about **-1194 N**: the vehicle would tip.

The paper's fix is to *move the CG* by imposing a body attitude with the active
suspension: extend wheels 1 and 4, compress wheel 3.  The resulting body roll
carries the CG laterally, because the CG sits above the roll centre.  For this
model (h_rc ~= 0.463 m, track 1.975 m) the paper's +-0.08 m deflection pattern
gives::

    roll  phi      = -2*0.08 / 1.975      = -4.64 deg
    lateral CG shift = -h_rc * phi        = +0.0375 m  (to the left)
    required shift (d >= B*b/L)           = +0.0270 m
    resulting F_RL                        = +40 N   (paper: "very small", assumed 0)

Every one of those numbers is reproduced by :func:`three_wheel_support`, and the
required 0.114 m of differential travel fits inside this model's +-150 mm travel.

Relationship to the previous controller
---------------------------------------
The earlier ``WheelLiftController`` distributed the support reaction as
``+|unload|/3`` on the three remaining corners.  That preserves the *net vertical
force* but not the *roll moment*: for a lifted FR wheel it injected about
+19 to +26 kN*m of roll moment toward the lifted side, which is the same order as
this vehicle's roll stiffness and is a plausible cause of the sustained 14 deg
roll observed in the retained run.  This controller instead commands the three
wheel normal loads that satisfy the full static equilibrium, so the roll moment
is balanced by the shifted CG rather than left as a residual.

Control structure (mirroring the paper's Fig. 8)
------------------------------------------------
* Step 0 approach, Step 1 lift wheel 2, Step 2 recover, Step 3 lift wheel 4,
  Step 4 recover -- the paper's time windows were 0-1.4, 1.4-3.1, 3.1-5,
  5-6.9, 6.9-9 s.  Here the transitions are driven by the *measured wheel-centre
  stations* versus the pothole geometry, so the same controller works for any
  pothole station, length or crawl speed with no retuning.
* feedforward ``uf``  : the active force that moves the passive 4-wheel load
  distribution onto the required 3-wheel distribution.
* feedback ``u``      : integral sliding-mode trim on the wheel-load error, with
  a boundary layer (the solver step is 0.5 ms, so a discontinuous sign term would
  chatter).
* safety layer        : travel limits, force/torque limits and rate limits, finite
  checks, and a latched ``SAFE_STOP`` that returns the vehicle to passive
  suspension.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from .units import require_verified, to_si
from .vehicle_params import GRAVITY, VehicleControllerParams

CORNERS: Tuple[str, ...] = ("FL", "FR", "RL", "RR")

#: Paper Fig. 1 support-phase numbering.
STEP_APPROACH = 0
STEP_LIFT_FRONT = 1
STEP_RECOVER_FRONT = 2
STEP_LIFT_REAR = 3
STEP_RECOVER_REAR = 4
STEP_DONE = 5

STEP_NAMES = {
    STEP_APPROACH: "0 approach",
    STEP_LIFT_FRONT: "1 lift wheel 2 (FR)",
    STEP_RECOVER_FRONT: "2 recover to four-wheel support",
    STEP_LIFT_REAR: "3 lift wheel 4 (RR)",
    STEP_RECOVER_REAR: "4 recover to normal driving",
    STEP_DONE: "done",
}


@dataclass
class ThreeWheelSupport:
    """Static three-wheel support solution for one lifted wheel."""

    lifted_corner: str
    target_load_n: Dict[str, float]
    feedforward_force_n: Dict[str, float]
    cg_lateral_shift_m: float
    cg_longitudinal_shift_m: float
    roll_angle_rad: float
    pitch_angle_rad: float
    deflection_target_m: Dict[str, float]
    minimal_load_n: float

    def as_dict(self) -> Dict[str, object]:
        return {
            "lifted_corner": self.lifted_corner,
            "target_load_n": self.target_load_n,
            "feedforward_force_n": self.feedforward_force_n,
            "cg_lateral_shift_m": self.cg_lateral_shift_m,
            "cg_longitudinal_shift_m": self.cg_longitudinal_shift_m,
            "roll_angle_deg": math.degrees(self.roll_angle_rad),
            "pitch_angle_deg": math.degrees(self.pitch_angle_rad),
            "deflection_target_m": self.deflection_target_m,
            "minimal_load_n": self.minimal_load_n,
        }


def _cross_side(corner: str) -> str:
    """Return the other wheel on the same axle."""
    return {"FL": "FR", "FR": "FL", "RL": "RR", "RR": "RL"}[corner]


def _diagonal_partner(corner: str) -> str:
    """Return the diagonally opposite wheel (the second support of the pair)."""
    return {"FL": "RR", "FR": "RL", "RL": "FR", "RR": "FL"}[corner]


def _solve3(matrix, rhs) -> List[float]:
    """Solve a 3x3 linear system by Cramer's rule.

    Kept dependency-free (no NumPy import) so the control law can be ported to a
    Simulink MATLAB Function block or embedded target without a matrix library.
    """
    (a, b, c), (d, e, f), (g, h, i) = matrix
    det = (
        a * (e * i - f * h)
        - b * (d * i - f * g)
        + c * (d * h - e * g)
    )
    if abs(det) < 1e-12:
        raise ValueError(
            "degenerate three-wheel support geometry (det=%.3e); the CG projection "
            "lies on a support diagonal" % det
        )

    def _det3(m):
        (a, b, c), (d, e, f), (g, h, i) = m
        return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)

    return [
        _det3([(rhs[0], b, c), (rhs[1], e, f), (rhs[2], h, i)]) / det,
        _det3([(a, rhs[0], c), (d, rhs[1], f), (g, rhs[2], i)]) / det,
        _det3([(a, b, rhs[0]), (d, e, rhs[1]), (g, h, rhs[2])]) / det,
    ]


def solve_linear(matrix: List[List[float]], rhs: List[float], regularization: float = 0.0) -> List[float]:
    """Solve ``matrix x = rhs`` by Gauss-Jordan elimination with partial pivoting.

    ``regularization`` adds ``lambda*I`` (Tikhonov) which is used for the actuator
    feedforward: the measured command-to-load matrix is ill-conditioned
    (condition number ~120), so an unregularised inverse would demand enormous
    commands from small gain errors.  Dependency-free so the solver can be ported.
    """
    n = len(rhs)
    augmented = [
        [float(matrix[i][j]) for j in range(n)] + [float(rhs[i])]
        for i in range(n)
    ]
    for i in range(n):
        augmented[i][i] += float(regularization)

    for column in range(n):
        pivot_row = max(range(column, n), key=lambda r: abs(augmented[r][column]))
        if abs(augmented[pivot_row][column]) < 1e-14:
            raise ValueError("singular system at column %d" % column)
        augmented[column], augmented[pivot_row] = augmented[pivot_row], augmented[column]
        pivot = augmented[column][column]
        augmented[column] = [value / pivot for value in augmented[column]]
        for row in range(n):
            if row == column:
                continue
            factor = augmented[row][column]
            if factor != 0.0:
                augmented[row] = [
                    value - factor * other
                    for value, other in zip(augmented[row], augmented[column])
                ]
    return [augmented[i][n] for i in range(n)]


def feedforward_commands(
    gain_matrix: Optional[Sequence[Sequence[float]]],
    delta_load_n: Sequence[float],
    regularization: float = 0.02,
) -> List[float]:
    """Invert the measured command-to-load matrix to hit a target load change.

    ``gain_matrix[j][i]`` is the load change at corner ``i`` per newton of command
    at corner ``j`` (as produced by ``scripts/probe_actuator_gain.py``).  When it is
    ``None`` the command equals the required load change, i.e. a 1:1 actuator is
    assumed.

    The inverse is required because ``IMP_FS`` injects force at the spring seat, so
    a corner's own actuator is only ~8% effective at that corner and the manoeuvre
    depends on cross-coupling: reaching the paper's three-wheel load target on this
    vehicle needs roughly 78 kN at the strongest corner, not 22 kN.
    """
    if gain_matrix is None:
        return [float(value) for value in delta_load_n]
    matrix = [[float(value) for value in row] for row in gain_matrix]
    if len(matrix) != len(delta_load_n):
        raise ValueError("gain matrix must be square and match the command length")
    return solve_linear(matrix, [float(v) for v in delta_load_n], regularization)


def roll_moment_of(forces: Sequence[float], half_track_m: float) -> float:
    """Moment about the longitudinal axis that a command set applies to the body.

    ``M_x = y * (F_FL + F_RL - F_FR - F_RR)`` with ``y`` the half track, using the
    sign convention that positive corner forces act upwards on the body.
    """
    fl, fr, rl, rr = (float(value) for value in forces)
    return half_track_m * (fl + rl - fr - rr)


def allocate_commands(
    gain_matrix: Optional[Sequence[Sequence[float]]],
    delta_load_n: Sequence[float],
    half_track_m: float,
    load_weight: float = 1.0,
    roll_moment_weight: float = 1.0e-5,
    effort_weight: float = 1.0e-6,
) -> List[float]:
    """Choose commands that reach the load targets while not rolling the body.

    Why this is not a plain matrix inverse
    --------------------------------------
    Two facts measured on this vehicle make the plain inverse unusable:

    1. When one wheel is lifted, the three-wheel equilibrium puts the CG on the
       FL-RR diagonal, so the third wheel load solves to nearly zero.  The vehicle
       is then effectively supported on **two** points, which has no stiffness about
       the longitudinal axis.  A command set that carries a large net roll moment
       therefore rolls the body without limit, which is exactly what the first
       expert run did: the attitude trim saturated and injected about -39 kN*m,
       and the body rolled 0 -> 10 deg while the wheel was over the hole.  The
       wheel then hung below road level and hit the exit lip (173 kN impact).
    2. Creating a real third-wheel load is not possible here: asking for a 10%
       share calls for ~300 kN of command, because the corner gain is only ~0.08.

    So the allocation is posed as a small regularised least-squares problem that
    trades off load tracking against the net roll moment and the command effort::

        minimise  w_l*||G f - delta||^2 + w_m*(M_x(f))^2 + w_e*||f||^2

    The roll-moment term is what keeps the manoeuvre survivable on a vehicle whose
    support degenerates to a two-point couple.
    """
    if gain_matrix is None:
        return [float(value) for value in delta_load_n]

    matrix = [[float(value) for value in row] for row in gain_matrix]
    n = len(matrix)
    if n != len(delta_load_n):
        raise ValueError("gain matrix must be square and match the command length")

    # Rows of the stacked system: weighted load equations, the roll-moment
    # equation, and the effort penalty.
    rows: List[List[float]] = []
    rhs: List[float] = []
    scale = math.sqrt(load_weight)
    for i in range(n):
        rows.append([scale * matrix[j][i] for j in range(n)])
        rhs.append(scale * float(delta_load_n[i]))
    roll_row = [
        math.sqrt(roll_moment_weight) * half_track_m * sign
        for sign in (1.0, -1.0, 1.0, -1.0)
    ]
    rows.append(roll_row)
    rhs.append(0.0)
    effort = math.sqrt(effort_weight)
    for j in range(n):
        row = [0.0] * n
        row[j] = effort
        rows.append(row)
        rhs.append(0.0)

    # Normal equations: (A^T A) f = A^T b
    normal = [[sum(rows[k][i] * rows[k][j] for k in range(len(rows)))
               for j in range(n)] for i in range(n)]
    target = [sum(rows[k][i] * rhs[k] for k in range(len(rows))) for i in range(n)]
    return solve_linear(normal, target)


def three_wheel_support(
    vehicle: VehicleControllerParams,
    lifted_corner: str,
    attitude_deflection_m: float = 0.08,
    roll_centre_height_m: Optional[float] = None,
) -> ThreeWheelSupport:
    """Solve the paper's three-wheel support state for one lifted wheel.

    ``attitude_deflection_m`` is the paper's ``SD`` magnitude for the attitude
    pattern (paper value 0.08 m, giving a 0.02 m margin on a 0.10 m limit; this
    model has +-0.150 m travel so 0.08 m also leaves a large margin).

    ``roll_centre_height_m`` defaults to the vehicle's own measured roll arm
    (CG height above the roll centre), so the CG-shift geometry is derived from
    the model rather than assumed.

    Returns the wheel-load targets, the equivalent feedforward active forces, the
    CG shift produced by the attitude, and the corner deflection targets.
    """
    if lifted_corner not in CORNERS:
        raise KeyError("lifted_corner must be one of %s" % (CORNERS,))
    if roll_centre_height_m is None:
        roll_centre_height_m = vehicle.cg_above_roll_centre_m
    weight = vehicle.total_weight_n
    axle_length = vehicle.wheelbase_m
    track = vehicle.track_m

    # The paper's attitude pattern for a lifted wheel `o`: extend `o` and its
    # **diagonal** partner... no -- compress the *diagonal* partner.  The paper is
    # explicit (Sec. IV, and its own numbers): for a lifted wheel 2 it sets
    # SD1 = SD2 = SD4 = -0.08 m and **SD3 = +0.08 m**, so the odd corner is wheel 3 =
    # left rear = the diagonal partner of the lifted wheel.
    #
    # This matters physically, not cosmetically.  The CG must move inside the triangle
    # of the three remaining contacts; for a lifted front-right wheel those are FL, RL,
    # RR, whose inward normal across the FL-RR diagonal points rearward and left.  The
    # CG follows the *low* end of the body, so the body must sit low at the diagonal
    # partner.  Compressing the cross-side corner instead (the previous behaviour) puts
    # no roll on the body at all and leaves the third contact carrying a **negative**
    # load: solving the three-wheel equilibrium gives F_RL = -589.6 N for a lifted front
    # wheel on this vehicle, i.e. the model is asked to tip over.  With the paper's
    # pattern the same solve gives F_RL = +501.1 N.
    odd = _diagonal_partner(lifted_corner)
    deflection = {c: -attitude_deflection_m for c in CORNERS}
    deflection[odd] = attitude_deflection_m

    # Body attitude is the plane through the three **grounded** corners -- a lifted
    # wheel is not a contact, so its command says nothing about the body plane.  With
    # d > 0 meaning compression (the body sits low at that corner), and phi > 0 = left
    # side up, theta > 0 = nose up:
    #
    #     lifted FR (grounded FL, RL, RR):  phi = (d_RR - d_RL)/B   theta = (d_RL - d_FL)/L
    #     lifted RR (grounded FL, FR, RL):  phi = (d_FR - d_FL)/B   theta = (d_RL - d_FL)/L
    #
    # which for the paper's pattern evaluates to phi = -2h/B in both lift cases but
    # theta = **+2h/L for a lifted front wheel and -2h/L for a lifted rear wheel**.  The
    # old code hard-coded +2h/L for both, so the rear-lift target loads were solved for
    # a CG shift of the wrong sign -- and -2h/B was accidentally the right roll only
    # because it was derived for the paper's pattern rather than the one being built.
    grounded = [c for c in CORNERS if c != lifted_corner]
    rear_pair = [c for c in grounded if c in ("RL", "RR")]
    front_pair = [c for c in grounded if c in ("FL", "FR")]
    if len(front_pair) == 2:          # lifted wheel is at the rear
        roll_angle = (deflection["FR"] - deflection["FL"]) / track
    else:                             # lifted wheel is at the front
        roll_angle = (deflection["RR"] - deflection["RL"]) / track
    _ = rear_pair  # documented above; the roll pair is the one that is fully grounded
    pitch_angle = (deflection["RL"] - deflection["FL"]) / axle_length

    # CG translation caused by that attitude.  The CG sits above the roll/pitch
    # centres, so a rotation moves its horizontal projection.
    cg_lateral = -roll_centre_height_m * roll_angle
    cg_longitudinal = -roll_centre_height_m * pitch_angle

    # Geometry from the shifted CG to each axle and each side.
    # +y is left, so a positive lateral shift moves the CG towards wheels 1 and 3.
    side_sign = {"FL": +1.0, "RL": +1.0, "FR": -1.0, "RR": -1.0}
    front = {"FL": True, "FR": True, "RL": False, "RR": False}

    a1 = vehicle.cg_to_front_axle_m - cg_longitudinal   # CG -> front axle
    b1 = vehicle.cg_to_rear_axle_m + cg_longitudinal    # CG -> rear axle
    c_arm = track / 2.0 - cg_lateral                    # CG -> left wheels
    d_arm = track / 2.0 + cg_lateral                    # CG -> right wheels
    span = a1 + b1

    # Three-wheel static equilibrium about the shifted CG.  With three contacts
    # the system is statically determinate: three unknowns (the contact loads)
    # and three equations (vertical force, pitch moment, roll moment).
    contacts = [c for c in CORNERS if c != lifted_corner]
    span = a1 + b1

    # Signed moment arms of each contact about the shifted CG.
    arm_x = {c: (a1 if front[c] else -b1) for c in contacts}
    arm_y = {c: (c_arm if side_sign[c] > 0 else -d_arm) for c in contacts}

    # Solve  [1 1 1; x1 x2 x3; y1 y2 y3] * F = [W; 0; 0]
    matrix = [
        [1.0, 1.0, 1.0],
        [arm_x[contacts[0]], arm_x[contacts[1]], arm_x[contacts[2]]],
        [arm_y[contacts[0]], arm_y[contacts[1]], arm_y[contacts[2]]],
    ]
    rhs = [weight, 0.0, 0.0]
    solution = _solve3(matrix, rhs)

    target = {c: 0.0 for c in CORNERS}
    for corner, load in zip(contacts, solution):
        target[corner] = load
    target[lifted_corner] = 0.0
    _ = span  # documented above; kept for readability of the balance equations

    feedforward = {
        c: target[c] - vehicle.static_load(c) for c in CORNERS
    }
    # Remove any net vertical force introduced by rounding so the manoeuvre never
    # pushes the vehicle up or down as a whole.
    net = sum(feedforward.values())
    for c in CORNERS:
        if c != lifted_corner and abs(feedforward[c]) > 0.0:
            feedforward[c] -= net / 3.0
            break

    return ThreeWheelSupport(
        lifted_corner=lifted_corner,
        target_load_n=target,
        feedforward_force_n=feedforward,
        cg_lateral_shift_m=cg_lateral,
        cg_longitudinal_shift_m=cg_longitudinal,
        roll_angle_rad=roll_angle,
        pitch_angle_rad=pitch_angle,
        deflection_target_m=deflection,
        minimal_load_n=min(target[c] for c in contacts),
    )


#: Command pattern the roll regulator applies, in ``CORNERS`` order.
ROLL_REGULATOR_PATTERN = (1.0, -1.0, 1.0, -1.0)


def derive_roll_regulator_sign(roll_response_deg: Mapping[str, float]) -> float:
    """Return the sign that makes the roll regulator *reduce* roll.

    The regulator applies :data:`ROLL_REGULATOR_PATTERN` scaled by a gain proportional
    to the measured roll angle, so that pattern's own steady-state roll response
    decides whether the loop is negative (corrective) or positive (regenerative).
    Given ``roll_response_deg[corner]`` -- the measured steady-state roll per newton of
    that corner's ``IMP_FS`` command (see ``scripts/probe_actuator_gain.py``) -- the
    pattern response is::

        R = sum_i pattern_i * roll_response_deg[corner_i]

    and the corrective sign is ``-sign(R)``.

    This is a **measured property of the vehicle, not a tuning knob.**  On the
    corner-module control object the measured response is +4.67 deg per unit of
    pattern, so the long-standing hard-coded ``+1.0`` closed a *regenerative* loop.  On
    the same run, flipping only this sign took the heading excursion from a measured
    223.6 deg down to 32.5 deg and restored forward motion (``Vx`` end -0.58 -> +2.56
    km/h).
    """
    response = sum(
        pattern * float(roll_response_deg[corner])
        for pattern, corner in zip(ROLL_REGULATOR_PATTERN, CORNERS)
    )
    if response == 0.0:
        raise ValueError("actuator roll response is zero; cannot derive a sign")
    return -1.0 if response > 0.0 else 1.0


@dataclass
class ExpertConfig:
    """Tunables of the deep-pothole expert strategy."""

    #: Paper used 10 ms for both controller and plant.
    control_period_s: float = 0.01
    #: Distance before the entry lip at which the lift manoeuvre starts.
    #:
    #: This is a *timing* parameter and was far too small.  The actuator needs
    #: ``force_slew_time_s`` to reach the SD force, and at 2.8 km/h (0.78 m/s) the old
    #: 0.25 m gave only 0.32 s of lead -- less than half the time the force takes to
    #: build -- so the wheel was **still hanging at road level when it reached the hole**.
    #: It then drooped into the hole and met the exit lip as a ~200 mm step, which is the
    #: measured 33-36 kN landing that throws the vehicle.  0.6 m gives 0.77 s, comfortably
    #: more than the rise time, so the wheel is already lifted clear before the lip.
    #:
    #: The paper's own timings agree: its ASS force starts at 1 s and the wheel is
    #: unloaded at 1.4 s, i.e. it allows 0.4 s of lead at a similar speed.
    pre_lift_distance_m: float = 0.6
    #: Time allowed for the force to ramp back to zero in Steps 2 and 4.
    #:
    #: The paper's own low-speed Step 2 lasts 1.9 s (3.1-5 s), but that window is
    #: **geometrically impossible on this control object.**  With the rear wheel
    #: 1.925 m behind the front one, a 0.8 m hole and 2.8 km/h (0.778 m/s), by the time
    #: the front wheel clears the trailing edge the rear wheel is 0.875 m short of its
    #: own pre-lift station, i.e. only 0.875 / 0.778 = 1.12 s away.  A 1.9 s recovery
    #: would therefore hand the rear wheel into the hole before the front corner had
    #: been restored.  1.0 s fits the geometry with margin while still being a genuine
    #: handover, and the state machine additionally refuses to start the rear lift until
    #: this ramp has finished, so the manoeuvre never overlaps itself.
    transition_time_s: float = 0.5
    #: Distance *before* the trailing edge at which the Step 2/4 recovery starts.
    #:
    #: A vehicle-specific adaptation, and a necessary one.  The paper recovers only after
    #: the wheel has passed the hole, which works because its corner module can hold the
    #: lifted wheel at road level.  This vehicle cannot: its rebound travel is about
    #: 100 mm, so an unloaded wheel necessarily hangs roughly that far below road level
    #: while it is over the hole (measured: the crossing wheel's centre sat 244-403 mm
    #: above the ground, i.e. its contact point 19-140 mm *below* road level).  It then
    #: meets the exit lip as a step, which is the measured 33-36 kN landing and the
    #: dominant source of body disturbance.  Starting the ramp this far early lets the
    #: passive suspension bring the wheel back to road level exactly as the ground
    #: returns, so the lip is a gentle touch instead of an impact.
    recovery_lead_m: float = 0.45
    #: Paper's SD magnitude for the attitude pattern (m).
    attitude_deflection_m: float = 0.08
    #: Roll arm (CG height above the roll centre) used for the CG-shift geometry.
    #: ``None`` uses the value derived from the vehicle model.
    roll_centre_height_m: Optional[float] = None
    #: Static gain from an ``IMP_FS`` command to that corner's wheel load, used
    #: only when no measured gain matrix is supplied.  The measured value for this
    #: solid-axle model is ~0.08 (see scripts/probe_actuator_gain.py).
    force_to_load_gain: float = 0.08
    #: Tikhonov factor for inverting the measured gain matrix (it has a condition
    #: number near 120, so an exact inverse would demand unreasonable commands).
    actuator_gain_regularization: float = 0.02
    #: Sliding surface: s = edot + lambda*e + kappa*integral(e)
    sliding_lambda: float = 8.0
    sliding_kappa: float = 2.0
    #: Switching gain (N of active force per unit of sliding variable).
    sliding_gain: float = 6000.0
    #: Boundary layer half-width, in sliding-variable units.
    boundary_layer: float = 4000.0
    #: Integral anti-windup limit on the accumulated load error (N*s).
    integral_limit: float = 4.0e4
    #: Actuator limits.  ``None`` derives them from the vehicle as
    #: ``force_limit_static_multiple`` times the largest static corner load, which is
    #: the only scale-free way to size them: the same +-100 kN that is reasonable for
    #: an 8.9 t truck is 7.5x the entire weight of the 1.36 t corner-module vehicle and
    #: launches it.  Set explicit values to override.
    force_min_n: Optional[float] = None
    force_max_n: Optional[float] = None
    #: Multiple of the largest static corner load used when the limits are derived.
    #: The paper's corner module is a ball-screw active suspension, so a generous
    #: multiple is realistic; 5x keeps the manoeuvre from destroying the model.
    #: Reduced to 2x for the corner-module control object, which has only about 72 mm
    #: of jounce travel left at its static position (measured): a commanded force much
    #: above one static corner load drives a front corner straight onto its stop within
    #: a few hundred milliseconds, and the manoeuvre then degenerates into stop chatter.
    force_limit_static_multiple: float = 2.0
    #: Slew limit on the active force (N/s).  ``None`` derives it from the actuator
    #: limit as ``limit / force_slew_time_s``.
    #:
    #: The old fixed 250 000 N/s let this vehicle's 11.4 kN limit be applied in 46 ms.
    #: On a 1.36 t vehicle four corners doing that together is ~3.5x its own weight, and
    #: the measured run shows exactly that: the summed tyre load peaks at 49.5 kN
    #: (3.7x weight) and then **all four wheels leave the ground** for 0.17 s with the
    #: body 0.10 m up, on flat road 2.2 m past the pothole.  In the Visualizer that
    #: reads as the vehicle floating in the air, which is what the user reported.
    force_rate_limit_n_per_s: Optional[float] = None
    #: Time the actuator takes to reach its limit when the slew rate is derived.
    #:
    #: 0.4 s rather than 1.0 s: with the SD loop the largest force actually commanded is
    #: about 4.4 kN (60 % of the limit), so this rises in ~0.25 s -- fast enough to be
    #: lifted before the entry lip, and far gentler than the 250 000 N/s slew that
    #: previously launched the vehicle, because that one was driving a 11.4 kN saturated
    #: load-tracking command.
    force_slew_time_s: float = 0.4
    torque_min_nm: float = -80.0
    torque_max_nm: float = 200.0
    #: Crawl speed loop.  The paper drives the manoeuvre at a near-constant hub torque
    #: (8 N*m, or 8.5 N*m while three-wheel supported) and explicitly reports that "the
    #: speed decreases when the vehicle is in a three-wheel supported state".  A speed
    #: *regulator* is therefore contrary to the reference strategy: it fights that drop
    #: and closes a feedback path from the wheel-lift dynamics straight into the
    #: longitudinal loop.  ``crawl_mode="constant"`` (the default) applies
    #: ``torque_bias_nm`` to every wheel and leaves the speed free, which is what the
    #: paper does; ``"regulated"`` restores the speed hold for cases that need a
    #: repeatable speed.
    #:
    #: ``torque_bias_nm`` is sized for *this* vehicle rather than copied from the
    #: paper: measured coast-down is 2.80 -> 2.64 km/h in 1.0 s, i.e. 60 N of rolling
    #: resistance, so holding 2.8 km/h needs 60 * 0.263 / 4 = 3.9 N*m per wheel.  The
    #: paper's 8 N*m suits its own much heavier vehicle; applied here it would nearly
    #: double the speed over the 9 s run.
    crawl_mode: str = "constant"
    torque_bias_nm: float = 8.0
    torque_per_kph_nm: float = 15.0
    #: Deadband on the speed error (km/h), so channel noise cannot chatter the command.
    torque_speed_deadband_kph: float = 0.05
    #: Slew limit on the torque command (N*m/s): 200 N*m takes 1 s to reach.
    torque_rate_limit_nm_per_s: float = 200.0
    #: Fraction of the travel limit treated as the guard band.
    travel_guard_fraction: float = 0.92
    #: Roll beyond this magnitude latches SAFE_STOP (deg).
    roll_safe_stop_deg: float = 12.0
    #: Attitude trim: replaced by a roll-drift regulator.  The paper's deflection
    #: pattern (extend wheels 1 and 4, compress wheel 3) is not reachable here,
    #: because those corners carry *more* load during the manoeuvre and therefore
    #: compress further rather than extend; driving it anyway saturated every corner
    #: and injected about -39 kN*m of spurious roll moment.
    #: Roll regulator gain, in newtons of roll-moment pattern force per degree of
    #: roll error (about zero).  Kept modest: it only has to arrest drift, and every
    #: newton it uses is a newton the lift manoeuvre cannot use.  Measured on the
    #: corner-module control object (roll span / pitch span / yaw span / lateral drift
    #: over the manoeuvre):
    #:
    #:     900 N/deg (old default)  19.67 deg / 18.84 deg / 34.54 deg / 1.465 m
    #:     400 N/deg               14.90 deg / 11.55 deg / 30.83 deg / 1.064 m
    #:     regulator disabled      26.70 deg / 13.37 deg / 50.55 deg / 2.116 m
    #:
    #: so the regulator is load-bearing but must stay soft, and 400 N/deg dominates
    #: the old default on every axis.
    roll_gain_n_per_deg: float = 200.0
    #: Ceiling on the roll regulator's own contribution, before the per-corner
    #: actuator clamp.  ``None`` derives it as :data:`roll_limit_static_fraction` of
    #: the actuator limit; the old hard-coded 40 kN exceeded this vehicle's *entire*
    #: actuator limit (11.4 kN), so the regulator saturated all four channels by
    #: itself and left the manoeuvre nothing to work with.
    roll_limit_n: Optional[float] = None
    #: Fraction of the actuator limit the roll regulator may use when deriving it.
    roll_limit_static_fraction: float = 0.35
    #: Sign of the roll regulator.  The control object (the corner-module vehicle)
    #: needs ``-1.0``; write it as the measured value rather than a guess by calling
    #: :func:`derive_roll_regulator_sign` on the probe output.  The default records the
    #: measurement for the corner-module vehicle and is a deliberate change from the
    #: old ``+1.0``, which closed a regenerative loop on this model.
    roll_regulator_sign: float = -1.0
    #: Height hold for the lifted wheel.  Zeroing its load alone lets the suspension
    #: sit wherever the actuator/spring balance lands it -- measured at +67 mm above
    #: the static position, so the wheel met the exit lip with a step and produced a
    #: 154 kN landing.  This term drives the lifted corner's deflection back to its
    #: static value, i.e. keeps the wheel at road level.  Newton per metre of error.
    lift_height_gain_n_per_m: float = 250000.0
    lift_height_limit_n: float = 25000.0
    #: Control the paper's own variable -- the suspension deflection (SD) -- instead of
    #: chasing wheel-load targets.
    #:
    #: This is the single most important stability decision in the controller.  A load
    #: target is not achievable on this vehicle: the three-wheel support solution asks a
    #: front corner for ~6.7 kN when its whole static share is 3.6 kN, so the sliding-mode
    #: trim sits permanently saturated, and four saturated corners press the vehicle into
    #: the road -- measured, the summed tyre load reaches **4x the vehicle weight** and the
    #: sprung mass sees 10 g.  Deflection feedback cannot do that: the command is
    #: proportional to a *displacement* error bounded by the travel envelope, so it is
    #: bounded by construction.  It is also literally what the paper controls ("the desired
    #: value for the absolute value of SD during control is set to 0.08 m").
    sd_tracking: bool = True
    #: SD magnitude actually commanded, as a fraction of the travel available either side
    #: of the static position.  The paper's own +-0.08 m sits inside its +-0.1 m limit,
    #: but this vehicle has far less room: its static travel is 21-38 mm against a 121 mm
    #: jounce stop and a -61 mm rebound stop, i.e. roughly 90 mm each way.  The magnitude
    #: is therefore *derived from the model* rather than copied from the paper, which is
    #: the parameter change the differing vehicle parameters require.
    sd_travel_fraction: float = 0.55
    #: Deflection-loop stiffness (N of active force per metre of travel error).  ``None``
    #: derives it so that a full-SD error uses :data:`sd_authority_fraction` of the
    #: actuator limit.
    sd_stiffness_n_per_m: Optional[float] = None
    #: Fraction of the actuator limit a full-magnitude SD error may command.
    sd_authority_fraction: float = 0.6
    #: Multiple of the corner's unsprung weight that the *lifted* corner may command.
    #:
    #: The lifted wheel is the one corner where a stiff deflection loop is actively
    #: harmful.  Once it is off the ground its strut runs to the rebound stop, so pulling
    #: harder cannot raise the wheel any further -- the reaction simply drags the **body**
    #: down instead.  Measured on the previous revision: the loop commanded -3783 N
    #: (about 10x the unsprung weight) and the body sank 200 mm across the hole, leaving
    #: the wheel resting on the hole floor.
    #:
    #: The paper's own feedforward for this corner is just the unsprung weight
    #: (``Faf = -0.02ks - mu*g``), which carries the wheel without disturbing the body.
    #: That is what this bounds the command to.
    lift_force_unsprung_multiple: float = 1.0
    #: Weight on keeping the command set's net roll moment small.  Larger values
    #: trade load-tracking accuracy for a body that does not roll away.
    roll_moment_weight: float = 1.0e-5
    #: Weight on command magnitude (regularisation of the allocation).
    effort_weight: float = 1.0e-6
    #: Lift the wheel only when the hole is deeper than the rebound travel plus
    #: this margin (``None`` derives it from the vehicle's rebound limit).
    min_depth_for_lift_m: Optional[float] = None


class DeepPotholeExpertController:
    """Feedforward + integral sliding-mode expert for one right-track pothole.

    The controller resolves every exported channel by *name* through
    ``export_names``; no positional index is hard-coded, and every channel used
    must have a measured unit (see :mod:`ddevsim.units`).
    """

    def __init__(
        self,
        scenario,
        vehicle: VehicleControllerParams,
        export_names: Sequence[str],
        config: Optional[ExpertConfig] = None,
        gain_matrix: Optional[Sequence[Sequence[float]]] = None,
        static_deflection_m: Optional[Dict[str, float]] = None,
    ) -> None:
        self.scenario = scenario
        self.vehicle = vehicle
        self.config = config or ExpertConfig()
        self.gain_matrix = (
            [list(row) for row in gain_matrix] if gain_matrix is not None else None
        )
        # Measured static corner deflection, needed to turn the paper's relative
        # deflection pattern into absolute compression targets.
        self.static_deflection_m = (
            {c: float(static_deflection_m[c]) for c in CORNERS}
            if static_deflection_m
            else None
        )
        self.export_index: Dict[str, int] = {
            name: index for index, name in enumerate(export_names)
        }
        self._require = (
            "Fz_L1", "Fz_R1", "Fz_L2", "Fz_R2",
            "CmpS_L1", "CmpS_R1", "CmpS_L2", "CmpS_R2",
            "X_R1", "X_R2", "Xo", "Vx",
        )
        for channel in self._require:
            if channel not in self.export_index:
                raise KeyError(
                    "expert controller needs channel %r in export_names" % (channel,)
                )
            require_verified(channel)

        # Support solution for each lifted wheel, precomputed from the vehicle.
        self.support = {
            corner: three_wheel_support(
                vehicle, corner,
                attitude_deflection_m=self.config.attitude_deflection_m,
                roll_centre_height_m=self.config.roll_centre_height_m,
            )
            for corner in ("FR", "RR")
        }

        # Size the actuator limits from the vehicle unless the caller set them.
        if self.config.force_min_n is None or self.config.force_max_n is None:
            reference = max(vehicle.static_load(c) for c in CORNERS)
            limit = self.config.force_limit_static_multiple * reference
            self.config.force_min_n = -limit
            self.config.force_max_n = limit
            self.force_limit_reference_n = reference
        else:
            self.force_limit_reference_n = None

        # The roll regulator must not be able to saturate the actuators on its own.
        # ``roll_limit_n`` was carried over from the 8.9 t truck, whose actuators are
        # an order of magnitude larger: at 40 kN it exceeded this vehicle's entire
        # actuator limit (11.4 kN), so the regulator alone pinned all four channels
        # from about 13 deg of roll and the manoeuvre had nothing left to work with.
        # Clamping it to a fraction of the actuator limit keeps it in the same scale
        # as the force it is trimming.
        actuator_limit = min(self.config.force_max_n, -self.config.force_min_n)
        # SD magnitude and loop stiffness, both derived from the model rather than copied
        # from the paper, because this vehicle's travel envelope is much smaller.
        if self.static_deflection_m is not None:
            # Room to compress further, and room to extend further, measured from the
            # *static* travel position.  The rebound limit is negative (travel can go down
            # to -rebound_limit), so the extension room is static + |rebound_limit|.
            jounce_room = self.vehicle.jounce_limit_m - max(
                self.static_deflection_m.values()
            )
            rebound_room = min(self.static_deflection_m.values()) + abs(
                self.vehicle.rebound_limit_m
            )
            room = max(1e-4, min(jounce_room, rebound_room))
        else:
            room = min(self.vehicle.jounce_limit_m, abs(self.vehicle.rebound_limit_m))
        self.sd_magnitude_m = self.config.sd_travel_fraction * room
        if self.config.sd_stiffness_n_per_m is None:
            self.sd_stiffness_n_per_m = (
                self.config.sd_authority_fraction * actuator_limit / self.sd_magnitude_m
            )
        else:
            self.sd_stiffness_n_per_m = float(self.config.sd_stiffness_n_per_m)
        if self.config.force_rate_limit_n_per_s is None:
            self.config.force_rate_limit_n_per_s = (
                actuator_limit / self.config.force_slew_time_s
            )
        if self.config.roll_limit_n is None:
            self.roll_limit_n = self.config.roll_limit_static_fraction * actuator_limit
        else:
            self.roll_limit_n = min(self.config.roll_limit_n, actuator_limit)

        self.step = STEP_APPROACH
        # Activation is a *logic* threshold on pothole depth, not a scale factor.
        # The manoeuvre exists to stop the wheel reaching the hole bottom and, more
        # importantly, to stop it hanging below road level and hitting the exit lip.
        # Neither depends on how deep the hole is beyond "deeper than the wheel can
        # reach", so once the depth exceeds the rebound travel plus a margin the
        # same strategy applies unchanged.
        if self.config.min_depth_for_lift_m is None:
            self.min_depth_for_lift_m = self.vehicle.rebound_limit_m + 0.02
        else:
            self.min_depth_for_lift_m = float(self.config.min_depth_for_lift_m)
        self.strategy_active = self.scenario.depth_m >= self.min_depth_for_lift_m

        # Command allocation per support phase.  A plain inverse is not used: the
        # three-wheel equilibrium puts the CG on the FL-RR diagonal, so the support
        # degenerates to a two-point couple with no roll stiffness, and any large
        # net roll moment in the command set rolls the body without limit.
        self.feedforward_command: Dict[str, Dict[str, float]] = {}
        self.required_command_n: Dict[str, float] = {}
        self.command_roll_moment_nm: Dict[str, float] = {}
        half_track = self.vehicle.track_m / 2.0
        for corner in ("FR", "RR"):
            delta = [self.support[corner].feedforward_force_n[c] for c in CORNERS]
            command = allocate_commands(
                self.gain_matrix, delta, half_track,
                roll_moment_weight=self.config.roll_moment_weight,
                effort_weight=self.config.effort_weight,
            )
            self.feedforward_command[corner] = dict(zip(CORNERS, command))
            self.required_command_n[corner] = max(abs(value) for value in command)
            self.command_roll_moment_nm[corner] = roll_moment_of(command, half_track)

        self.safe_stop = False
        self.safe_stop_reason = ""
        self._integral = {c: 0.0 for c in CORNERS}
        self._previous_error = {c: 0.0 for c in CORNERS}
        self._previous_force = {c: 0.0 for c in CORNERS}
        self._applied_force = {c: 0.0 for c in CORNERS}
        self._applied_torque = 0.0
        self._torque_command = 0.0
        self._last_control_time = -1.0
        self._recover_start_time = 0.0
        self._recover_from: Dict[str, float] = {c: 0.0 for c in CORNERS}
        self.trace: List[Dict[str, float]] = []
        self._trace_stride = max(1, int(round(self.config.control_period_s / 0.0005)))

    # ------------------------------------------------------------------ helpers
    def _channel(self, exports: Sequence[float], name: str) -> float:
        value = float(exports[self.export_index[name]])
        return value if math.isfinite(value) else 0.0

    def _station(self, exports: Sequence[float], name: str) -> float:
        """Wheel-centre station in metres (``X_*`` is declared in metres)."""
        return to_si(name, self._channel(exports, name))

    def _loads(self, exports: Sequence[float]) -> Dict[str, float]:
        return {c: self._channel(exports, "Fz_%s" % _suffix(c)) for c in CORNERS}

    def _deflections(self, exports: Sequence[float]) -> Dict[str, float]:
        """Suspension travel per corner, in metres.

        ``Jnc_*`` -- total wheel jounce travel -- is the quantity the jounce/rebound
        limits are expressed in, so it is what the travel guard must compare against.
        ``CmpS_*`` is the **ride-spring compression**, a different quantity with a
        different scale (on this model the front spring compresses 168 mm while the wheel
        travels 80 mm), and the guard used to be fed that value against travel limits:
        the static front corner already read 168 mm against a 121 mm jounce stop, so the
        guard believed the corner was permanently bottomed and could never act
        meaningfully.  ``CmpS`` remains the fallback for models without ``Jnc``.
        """
        channel = "Jnc_%s" if "Jnc_L1" in self.export_index else "CmpS_%s"
        return {
            c: to_si(channel % _suffix(c), self._channel(exports, channel % _suffix(c)))
            for c in CORNERS
        }

    def _speed_kph(self, exports: Sequence[float]) -> float:
        return self._channel(exports, "Vx")

    # ------------------------------------------------------------- state machine
    def _advance_step(self, time_s: float, exports: Sequence[float]) -> None:
        scenario = self.scenario
        leading = scenario.leading_edge_m
        trailing = scenario.trailing_edge_m
        front_station = self._station(exports, "X_R1")   # wheel 2 = FR
        rear_station = self._station(exports, "X_R2")    # wheel 4 = RR

        # A hole shallower than the rebound travel never needs the manoeuvre: the
        # wheel simply follows the road.  Depth is a gate, not a gain.
        if not self.strategy_active:
            self.step = STEP_APPROACH
            return

        if self.step == STEP_APPROACH:
            if front_station >= leading - self.config.pre_lift_distance_m:
                self.step = STEP_LIFT_FRONT
        elif self.step == STEP_LIFT_FRONT:
            if front_station >= trailing - self.config.recovery_lead_m:
                self.step = STEP_RECOVER_FRONT
                self._begin_recovery(time_s)
        elif self.step == STEP_RECOVER_FRONT:
            # The paper restores four-wheel support *before* lifting the next wheel:
            # Step 2 is "After the wheel 2 has passed over the pothole, the vehicle is
            # adjusted to a four-wheeled support state", and only then does Step 3 lift
            # wheel 4.  The station test alone does not enforce that -- in the measured
            # run it fired 0.32 s early and handed over with 682 N still commanded on
            # the front corner, so the two lift phases overlapped.  Gate the rear lift
            # on the recovery ramp having actually finished.
            if self._recovery_finished(time_s) and (
                rear_station >= leading - self.config.pre_lift_distance_m
            ):
                self.step = STEP_LIFT_REAR
                self._integral = {c: 0.0 for c in CORNERS}
        elif self.step == STEP_LIFT_REAR:
            if rear_station >= trailing - self.config.recovery_lead_m:
                self.step = STEP_RECOVER_REAR
                self._begin_recovery(time_s)
        elif self.step == STEP_RECOVER_REAR:
            if self._recovery_finished(time_s):
                self.step = STEP_DONE

    def _force_slew_n_per_s(self) -> float:
        """Force slew limit, derived from the actuator limit when not configured.

        Derived here rather than cached in ``__init__`` because callers legitimately
        swap ``controller.config`` for a fresh one after construction, and a cached
        ``None`` would then reach the arithmetic.
        """
        configured = self.config.force_rate_limit_n_per_s
        if configured is not None:
            return float(configured)
        limit = min(self.config.force_max_n, -self.config.force_min_n)
        return limit / max(1e-6, self.config.force_slew_time_s)

    def _recovery_finished(self, time_s: float) -> bool:
        """True once the Step 2/4 ramp from the held forces to zero has completed."""
        return (
            float(time_s) - self._recover_start_time
        ) >= self.config.transition_time_s

    def _begin_recovery(self, time_s: float) -> None:
        self._recover_start_time = time_s
        self._recover_from = dict(self._applied_force)

    def _active_support(self) -> Optional[ThreeWheelSupport]:
        if self.step == STEP_LIFT_FRONT:
            return self.support["FR"]
        if self.step == STEP_LIFT_REAR:
            return self.support["RR"]
        return None

    # ------------------------------------------------------------------- control
    def _sd_forces(
        self,
        support: ThreeWheelSupport,
        deflections: Mapping[str, float],
        config: ExpertConfig,
    ) -> Dict[str, float]:
        """Active forces from the paper's SD pattern, tracked by deflection feedback.

        The pattern itself is the paper's, reused directly from
        :func:`three_wheel_support` (extend the lifted wheel and the two remaining
        contacts, compress the diagonal partner), scaled from the paper's +-0.08 m to
        whatever this vehicle's travel envelope actually allows.

        Sign convention, fixed by measurement rather than assumption: a positive
        ``IMP_FS`` command *extends* the suspension, i.e. it reduces the measured travel
        (verified on this model -- a +4000 N command drives the front ride-spring
        compression down while a negative one drives it up).  So the force that opposes
        excess travel is ``+k * (travel - target)``.
        """
        scale = self.sd_magnitude_m / max(1e-9, config.attitude_deflection_m)
        # Force available on the lifted corner: enough to carry its unsprung mass, and no
        # more.  See ``lift_force_unsprung_multiple`` for why a stiff loop here drags the
        # body into the hole instead of raising the wheel.
        unsprung_n = (
            self.vehicle.unsprung_mass_per_corner_kg
            * GRAVITY
            * config.lift_force_unsprung_multiple
        )
        forces: Dict[str, float] = {}
        for corner in CORNERS:
            reference = 0.0 if self.static_deflection_m is None else (
                self.static_deflection_m[corner]
            )
            target = reference + scale * support.deflection_target_m[corner]
            error = deflections[corner] - target
            force = self.sd_stiffness_n_per_m * error
            if corner == support.lifted_corner:
                # One-sided and bounded: pull the wheel up at most by its own weight, and
                # never push down on a corner that has no tyre load to react against.
                force = min(0.0, max(-unsprung_n, force))
            forces[corner] = force
        return forces

    def _crawl_torque(self, exports: Sequence[float], dt: float) -> float:
        config = self.config
        if config.crawl_mode == "constant":
            # The paper's own strategy: the same small torque on every wheel, with the
            # speed free to fall while three-wheel supported.  There is no speed feedback
            # at all, so the wheel-lift dynamics cannot couple into this loop.
            target = max(
                config.torque_min_nm, min(config.torque_max_nm, config.torque_bias_nm)
            )
        else:
            error = self.scenario.target_speed_kph - self._speed_kph(exports)
            if abs(error) <= config.torque_speed_deadband_kph:
                error = 0.0
            target = config.torque_bias_nm + config.torque_per_kph_nm * error
            target = max(config.torque_min_nm, min(config.torque_max_nm, target))
        # Ramping rather than stepping keeps the first samples free of a torque impulse.
        slew = config.torque_rate_limit_nm_per_s * max(0.0, dt)
        self._torque_command += max(-slew, min(slew, target - self._torque_command))
        return self._torque_command

    def _sliding_force(
        self, corner: str, error: float, dt: float, config: ExpertConfig
    ) -> float:
        """Integral sliding-mode trim, boundary-layer smoothed."""
        self._integral[corner] += error * dt
        limit = config.integral_limit
        self._integral[corner] = max(-limit, min(limit, self._integral[corner]))
        derivative = (error - self._previous_error[corner]) / dt if dt > 0 else 0.0
        self._previous_error[corner] = error
        surface = (
            derivative
            + config.sliding_lambda * error
            + config.sliding_kappa * self._integral[corner]
        )
        half = max(1e-9, config.boundary_layer)
        if abs(surface) <= half:
            return -config.sliding_gain * surface / half
        return -config.sliding_gain * (1.0 if surface > 0 else -1.0)

    def _travel_guard(
        self, corner: str, deflection_m: float, force: float, config: ExpertConfig
    ) -> Tuple[float, Optional[str]]:
        """Reduce an active force that would drive the suspension into a stop.

        The taper must never go negative.  The previous form,
        ``force * (1 - (|d| - guard) / (limit - guard))``, is +1 at the guard
        boundary, 0 exactly at the limit, and **negative beyond it** -- so once the
        suspension was past its stop the guard flipped the sign of a compressively
        commanded force into an extensional one and then *grew* it with further
        travel.  That positive feedback is what pinned ``CmpS_FR`` at 200 mm and
        produced the 0 <-> 42 kN wheel-load limit cycle seen in the probe runs.
        Clamping the taper to [0, 1] lets the force fade to zero at the stop and the
        passive suspension take over, which is the intended behaviour.
        """
        limit = (
            self.vehicle.jounce_limit_m if force < 0.0 else self.vehicle.rebound_limit_m
        )
        guard = config.travel_guard_fraction * limit
        if abs(deflection_m) <= guard:
            return force, None
        taper = 1.0 - (abs(deflection_m) - guard) / max(1e-6, limit - guard)
        taper = max(0.0, min(1.0, taper))
        return force * taper, "travel_guard:%s" % corner

    def __call__(self, time_s: float, exports: Sequence[float]) -> Tuple[float, ...]:
        config = self.config
        self._advance_step(float(time_s), exports)
        dt = config.control_period_s

        # Sample-and-hold at the controller period, matching the paper's 10 ms.
        due = (self._last_control_time < 0.0) or (
            float(time_s) - self._last_control_time >= dt - 1e-9
        )

        loads = self._loads(exports)
        deflections = self._deflections(exports)
        roll_deg = to_si("Roll_E", self._channel(exports, "Roll_E")) * 180.0 / math.pi

        if not self.safe_stop and abs(roll_deg) > config.roll_safe_stop_deg:
            self.safe_stop = True
            self.safe_stop_reason = "roll_exceeds_%.1f_deg" % config.roll_safe_stop_deg

        if due:
            self._last_control_time = float(time_s)
            support = self._active_support()
            if self.safe_stop:
                forces = {c: 0.0 for c in CORNERS}
            elif support is not None and config.sd_tracking:
                # The paper's own control variable: track the SD pattern.
                forces = self._sd_forces(support, deflections, config)
                roll_correction = config.roll_regulator_sign * (
                    config.roll_gain_n_per_deg * roll_deg
                )
                roll_correction = max(
                    -self.roll_limit_n, min(self.roll_limit_n, roll_correction)
                )
                for index, corner in enumerate(CORNERS):
                    forces[corner] += roll_correction * (1.0, -1.0, 1.0, -1.0)[index]
            elif support is not None:
                forces = {}
                feedforward_table = self.feedforward_command[support.lifted_corner]
                for corner in CORNERS:
                    target_load = support.target_load_n[corner]
                    feedforward = feedforward_table[corner]
                    error = loads[corner] - target_load
                    trim = self._sliding_force(corner, error, dt, config)
                    force = feedforward + trim
                    # Hold the lifted wheel at road level.  Zeroing its load is not
                    # enough: without this the suspension settles 67 mm above its
                    # static position and the wheel meets the exit lip as a step.
                    if (
                        self.static_deflection_m is not None
                        and corner == support.lifted_corner
                    ):
                        height_error = (
                            self.static_deflection_m[corner] - deflections[corner]
                        )
                        hold = config.lift_height_gain_n_per_m * height_error
                        hold = max(
                            -config.lift_height_limit_n,
                            min(config.lift_height_limit_n, hold),
                        )
                        force += hold
                    force, _ = self._travel_guard(
                        corner, deflections[corner], force, config
                    )
                    forces[corner] = force

                # Roll-drift regulation.  With the support degenerated to the FL-RR
                # two-point couple there is almost no stiffness about the
                # longitudinal axis, so any residual roll moment lets the body roll
                # away slowly (0 -> 10 deg over 1.7 s in the first expert run) and
                # the lifted wheel ends up below road level at the exit lip.
                # The (+,-,+,-) pattern produces a net roll moment with zero net
                # vertical force, so it acts on attitude without disturbing support.
                roll_correction = config.roll_regulator_sign * (
                    config.roll_gain_n_per_deg * roll_deg
                )
                roll_correction = max(
                    -self.roll_limit_n, min(self.roll_limit_n, roll_correction)
                )
                for index, corner in enumerate(CORNERS):
                    forces[corner] += roll_correction * (1.0, -1.0, 1.0, -1.0)[index]
            elif self.step in (STEP_RECOVER_FRONT, STEP_RECOVER_REAR):
                elapsed = float(time_s) - self._recover_start_time
                blend = 1.0 - min(1.0, elapsed / max(1e-6, config.transition_time_s))
                forces = {c: self._recover_from[c] * blend for c in CORNERS}
                if blend <= 0.0:
                    self._integral = {c: 0.0 for c in CORNERS}
            else:
                forces = {c: 0.0 for c in CORNERS}

            # Rate limit and saturate, then commit as the held value.
            for corner in CORNERS:
                force = forces[corner]
                previous = self._applied_force[corner]
                step = self._force_slew_n_per_s() * dt
                force = max(previous - step, min(previous + step, force))
                force = max(config.force_min_n, min(config.force_max_n, force))
                if not math.isfinite(force):
                    self.safe_stop = True
                    self.safe_stop_reason = "non_finite_force"
                    force = 0.0
                self._applied_force[corner] = force

            torque = 0.0 if self.safe_stop else self._crawl_torque(exports, dt)
            if not math.isfinite(torque):
                self.safe_stop = True
                self.safe_stop_reason = "non_finite_torque"
                torque = 0.0
            self._applied_torque = torque

            self.trace.append(
                {
                    "time_s": float(time_s),
                    "step": float(self.step),
                    "Fz_FL": loads["FL"], "Fz_FR": loads["FR"],
                    "Fz_RL": loads["RL"], "Fz_RR": loads["RR"],
                    "roll_deg": roll_deg,
                    "force_FL": self._applied_force["FL"],
                    "force_FR": self._applied_force["FR"],
                    "force_RL": self._applied_force["RL"],
                    "force_RR": self._applied_force["RR"],
                    "torque_nm": self._applied_torque,
                }
            )

        return (
            self._applied_torque, self._applied_torque,
            self._applied_torque, self._applied_torque,
            self._applied_force["FL"], self._applied_force["FR"],
            self._applied_force["RL"], self._applied_force["RR"],
        )

    # ------------------------------------------------------------------ reporting
    def step_summary(self) -> Dict[str, object]:
        """Describe the support-phase plan the controller derived."""
        return {
            "pothole_span_m": list(self.scenario.station_span()),
            "pre_lift_distance_m": self.config.pre_lift_distance_m,
            "control_period_s": self.config.control_period_s,
            "actuator_gain_matrix_used": self.gain_matrix is not None,
            "required_command_n": dict(self.required_command_n),
            "force_limits_n": [self.config.force_min_n, self.config.force_max_n],
            "support_solutions": {
                corner: solution.as_dict() for corner, solution in self.support.items()
            },
            "feedforward_commands": {
                corner: {c: round(v, 1) for c, v in table.items()}
                for corner, table in self.feedforward_command.items()
            },
            "final_step": self.step,
            "final_step_name": STEP_NAMES.get(self.step, "unknown"),
            "safe_stop": self.safe_stop,
            "safe_stop_reason": self.safe_stop_reason,
        }


def _suffix(corner: str) -> str:
    return {"FL": "L1", "FR": "R1", "RL": "L2", "RR": "R2"}[corner]
