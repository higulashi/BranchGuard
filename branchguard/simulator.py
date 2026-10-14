"""The controller receives pixels and noisy incremental odometry only.

World lines are available to the renderer and evaluator, never to C decisions.
"""
from dataclasses import dataclass
import math
import numpy as np
from .native import Controller, STATUS

SCENARIOS = ('fork_left', 'fork_right', 'three_way', 'crossing')
CONDITIONS = ('clean', 'clutter', 'gaps', 'flicker', 'mixed')
DT = .04
_v, _u = np.mgrid[0:120, 0:188]
_s = 18. / np.maximum(_v - 12., 1.)
_lat = (94. - _u) * _s / 80.


@dataclass
class Course:
    scenario: str
    seed: int

    def __post_init__(self):
        rng = np.random.default_rng(self.seed)
        self.junction = 2. + rng.uniform(-.12, .12)
        self.slope = rng.uniform(.42, .62)
        self.phase = rng.uniform(-.5, .5)
        self.route = 1 if self.scenario == 'fork_left' else -1 if self.scenario == 'fork_right' else 0
        self.length = 4.4

    def base(self, x):
        return .055 * np.sin(1.5 * x + self.phase)

    def branch(self, x, sign):
        d = np.maximum(x - self.junction, 0.)
        return self.base(x) + sign * self.slope * (d - .30 * (1. - np.exp(-d / .30)))

    def target(self, x):
        return self.branch(x, self.route) if self.route else self.base(x)

    def lines(self, x):
        if self.scenario.startswith('fork'):
            return [self.branch(x, 1), self.branch(x, -1)]
        if self.scenario == 'three_way':
            return [self.base(x), self.branch(x, 1), self.branch(x, -1)]
        return [self.base(x), .75 * (x - self.junction) + self.base(self.junction)]


def render(course, condition, pose, t, step):
    x, y, heading = pose
    cs, sn = math.cos(heading), math.sin(heading)
    wx = x + cs * _s - sn * _lat
    wy = y + sn * _s + cs * _lat
    if condition in ('flicker','mixed'):
        # Short false-line exposures test whether replacements need evidence.
        # This is synthetic perception corruption, not a physical camera model.
        if 1.04 <= t < 1.12 or 2.40 <= t < 2.48 or 3.44 <= t < 3.52:
            wy = wy + .15
    mask = np.zeros((120, 188), dtype=bool)
    for line in course.lines(wx):
        mask |= np.abs(wy - line) < .021
    if condition in ('gaps', 'mixed'):
        # Missing paint belongs to the world, independent of each method's speed.
        missing = ((wx > course.junction + .06) & (wx < course.junction + .20))
        missing |= ((wx > 3.18) & (wx < 3.29))
        mask &= ~missing
    if condition in ('clutter', 'mixed'):
        ghost = course.base(wx) + .13 + .12 * np.sin(6. * (wx - 1.2))
        mask |= (np.abs(wy - ghost) < .016) & (wx > 1.2) & (wx < 2.45)
        transverse = (np.abs(wx - 1.48) < .025) & (np.abs(wy) < .75)
        mask |= transverse
    frame = np.where(mask, 28., 219.)
    if condition == 'mixed':
        shade = .78 + .16 * np.sin(wx * 3.1 + wy * 1.3)
        frame *= shade
        # A camera dropout is timed in simulation seconds, not position.
        if 1.36 <= t < 1.52:
            frame[:] = 170.
        if 2.1 <= t < 2.3:
            frame[38:57] = 195.
    # Fixed time-indexed sensor noise provides paired disturbances.
    rng = np.random.default_rng(np.random.SeedSequence([course.seed, step, 702]))
    frame += rng.normal(0., 2. if condition == 'clean' else 5., frame.shape)
    frame[_v <= 16] = 210.
    return np.ascontiguousarray(np.clip(frame, 0, 255).astype(np.uint8))


def run(scenario, condition, seed, method, capture=False, speed_cap=None):
    course = Course(scenario, seed)
    control = Controller(method)
    if method in ('spatial_slow','branchguard_slow'):
        speed_cap = .55
    rng = np.random.default_rng(seed + 901)
    x, y, heading = .35, float(course.target(.35) + rng.uniform(-.025, .025)), 0.
    speed, steer, ds, da = 0., 0., 0., 0.
    lag = rng.uniform(.07, .13)
    distance_scale, gyro_bias = rng.uniform(.985, 1.015), rng.uniform(-.012, .012)
    errors, steering_changes, records, frames = [], [], [], []
    statuses = dict.fromkeys(STATUS, 0)
    status, violation = 'timeout', 0.
    previous_command = 0.
    for step in range(500):
        t = step * DT
        frame = render(course, condition, (x, y, heading), t, step)
        noisy_ds = max(0., ds * distance_scale + rng.normal(0, .0008))
        noisy_da = da + gyro_bias * DT + rng.normal(0, .0015)
        out, obs = control.step(frame, course.route, noisy_ds, noisy_da, DT)
        statuses[STATUS[out.status]] += 1
        errors.append(float(y - course.target(x)))
        steering_changes.append(abs(out.steering - previous_command))
        previous_command = out.steering
        record = {'t': t, 'x': x, 'y': y, 'heading': heading, 'speed': speed,
                  'error': errors[-1], 'steering': out.steering, 'candidates': out.count,
                  'decision': STATUS[out.status], 'discrepancy': out.discrepancy}
        if capture:
            records.append(record)
            if step % 3 == 0:
                frames.append((frame, out, obs, record.copy()))
        target_speed = out.speed if speed_cap is None else min(out.speed, speed_cap)
        speed += (target_speed - speed) * (1. - math.exp(-DT / .12))
        steer += (out.steering - steer) * (1. - math.exp(-DT / lag))
        yaw_rate = speed / .26 * math.tan(steer)
        ds, da = speed * DT, yaw_rate * DT
        x += ds * math.cos(heading + da / 2.)
        y += ds * math.sin(heading + da / 2.)
        heading += da
        violation = violation + DT if abs(errors[-1]) > .28 else 0.
        if x > course.junction + .8 and course.route:
            target_error = abs(y - course.target(x))
            other_error = abs(y - course.branch(x, -course.route))
            if other_error + .12 < target_error:
                status = 'wrong_branch'
                break
        if violation >= .20:
            status = 'off_track'
            break
        if x >= course.length:
            status = 'complete'
            break
        if t > 4. and speed < .01 and statuses['stop'] > 50:
            status = 'stopped'
            break
    result = {'scenario': scenario, 'condition': condition, 'seed': seed, 'method': method,
              'status': status, 'time_s': (step + 1) * DT, 'progress_m': x,
              'rms_error_m': float(np.sqrt(np.mean(np.square(errors)))),
              'max_error_m': max(abs(e) for e in errors),
              'steering_tv_rad': sum(steering_changes),
              **{name + '_frames': count for name, count in statuses.items()}}
    return result, records, frames
