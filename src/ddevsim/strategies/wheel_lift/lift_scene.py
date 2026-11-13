"""Integer-centimetre single-track geometry and the left-side control boundary."""
from dataclasses import dataclass
import json
import re

from ddevsim.vehicle_profiles.tuned_ii import DEFAULT_PROFILE


# Endpoints for the responsive single-track profile. The long-pit endpoint
# starts braking earlier and reaches rear crossing speed in less distance.
_CRAWL_LENGTH_RANGE_CM = (80, 100)
_FRONT_BRAKE_LEAD_M = (.50, .60)
_REAR_ACCEL_RAMP_S = (2.50, 2.10)
_FRONT_SUPPORT_FEEDBACK_N = (500., 550.)


def length_adjusted_crawl_timing(scene, front_brake_lead_m, rear_accel_ramp_s,
                                front_support_feedback_n):
    """Adjust nominal responsive timing for the actual pit length on either side."""
    low, high = _CRAWL_LENGTH_RANGE_CM
    fraction = max(0., min(1., (scene.length_cm - low) / (high - low)))
    if front_brake_lead_m == _FRONT_BRAKE_LEAD_M[0]:
        front_brake_lead_m += fraction * (_FRONT_BRAKE_LEAD_M[1] - front_brake_lead_m)
    if rear_accel_ramp_s == _REAR_ACCEL_RAMP_S[0]:
        rear_accel_ramp_s += fraction * (_REAR_ACCEL_RAMP_S[1] - rear_accel_ramp_s)
    if front_support_feedback_n == _FRONT_SUPPORT_FEEDBACK_N[0]:
        front_support_feedback_n += fraction * (_FRONT_SUPPORT_FEEDBACK_N[1] - front_support_feedback_n)
    return front_brake_lead_m, rear_accel_ramp_s, front_support_feedback_n


@dataclass(frozen=True)
class LiftScene:
    side: str
    length_cm: int = 80
    width_cm: int = 90
    depth_cm: int = 20
    offset_cm: int = 0  # Positive toward the outside of the target track.

    def __post_init__(self):
        if self.side not in ('left', 'right'):
            raise ValueError('side must be left or right')
        if any(type(n) is not int for n in (self.length_cm, self.width_cm, self.depth_cm, self.offset_cm)):
            raise ValueError('scene dimensions and offset must be integer centimetres')
        if not (50 <= self.length_cm <= 100 and 50 <= self.width_cm <= 100 and 10 <= self.depth_cm <= 40):
            raise ValueError('outside the single-track geometry study bounds')
        # 5 cm lip, 10 cm tyre half-width, 5 cm lateral allowance.
        if abs(self.offset_cm) > min(20, self.width_cm // 2 - 20):
            raise ValueError('offset leaves insufficient target-track coverage')

    @classmethod
    def read(cls, path):
        return cls(**json.loads(path.read_text(encoding='utf-8-sig')))

    @property
    def center_y_m(self):
        sign = 1 if self.side == 'left' else -1
        return sign * (DEFAULT_PROFILE.track_m / 2 + self.offset_cm / 100)

    @property
    def wheel_sequence(self):
        return ('FL', 'RL') if self.side == 'left' else ('FR', 'RR')

    def geometry(self):
        return dict(length_m=self.length_cm / 100, width_m=self.width_cm / 100,
                    depth_m=self.depth_cm / 100, center_y_m=self.center_y_m)


class SideMirror:
    """Reflect native observations into the right-side controller coordinates."""
    def __init__(self, names):
        indices = {name: i for i, name in enumerate(names)}
        lateral = {'Yo', 'Yaw', 'Roll_E', 'Roll_Rd', 'Ay', 'AVx', 'AVz', 'YCG_TM'}
        self.mapping = []
        for name in names:
            other = re.sub(r'([LR])([12]i?)$', lambda m: ('R' if m[1] == 'L' else 'L') + m[2], name)
            sign = -1 if name in lateral or name.startswith(('Y_', 'Yctc_', 'Fy_', 'Alpha_', 'Mz_')) else 1
            self.mapping.append((indices[other], sign))

    def observation(self, values):
        return tuple(values[index] * sign for index, sign in self.mapping)

    @staticmethod
    def action(values):
        return tuple(values[i] for i in (1, 0, 3, 2, 5, 4, 7, 6)) + (-values[8],)

    def wrap(self, controller):
        def command(now_s, values):
            return self.action(controller(now_s, self.observation(values)))
        return command
