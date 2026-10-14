import ctypes as C
import os
from pathlib import Path as FilePath
import shutil
import subprocess

ROOT = FilePath(__file__).resolve().parents[1]
METHODS = {name: i for i, name in enumerate(
    ('nearest', 'greedy_route', 'spatial', 'temporal', 'branchguard', 'no_motion'))}
METHODS['spatial_slow'] = METHODS['spatial']
METHODS['branchguard_slow'] = METHODS['branchguard']
METHODS['no_prefix'] = 6
STATUS = ('stop', 'track', 'pending', 'hold')


class Point(C.Structure):
    _fields_ = [('y', C.c_float), ('quality', C.c_float)]


class Row(C.Structure):
    _fields_ = [('count', C.c_int), ('points', Point * 6)]


class Observation(C.Structure):
    _fields_ = [('rows', Row * 12)]


class Path(C.Structure):
    _fields_ = [('y', C.c_float * 12), ('cost', C.c_float),
                ('mask', C.c_uint), ('support', C.c_int)]


class State(C.Structure):
    _fields_ = [('previous', Path), ('pending', Path),
                ('missing_time', C.c_float), ('pending_time', C.c_float),
                ('has_previous', C.c_int), ('has_pending', C.c_int)]


class Output(C.Structure):
    _fields_ = [('paths', Path * 8), ('selected', Path),
                ('steering', C.c_float), ('speed', C.c_float),
                ('margin', C.c_float), ('discrepancy', C.c_float),
                ('count', C.c_int), ('status', C.c_int)]


def build():
    dst = ROOT / 'build' / ('branchguard.dll' if os.name == 'nt' else 'libbranchguard.so')
    dst.parent.mkdir(exist_ok=True)
    src = [ROOT / 'core' / f for f in ('branchguard.c', 'branchguard.h')]
    if dst.exists() and dst.stat().st_mtime > max(p.stat().st_mtime for p in src):
        return dst
    zig = os.environ.get('BRANCHGUARD_ZIG') or shutil.which('zig')
    if not zig:
        try:
            import ziglang
            zig = str(FilePath(next(iter(ziglang.__path__))) / ('zig.exe' if os.name == 'nt' else 'zig'))
        except ImportError:
            pass
    cc = [zig, 'cc'] if zig else [os.environ.get('CC') or shutil.which('cc') or shutil.which('gcc') or 'clang']
    args = ['-std=c99', '-O2', '-Wall', '-Wextra', '-Werror', '-shared', str(src[0]), '-o', str(dst)]
    if os.name != 'nt':
        args += ['-fPIC', '-lm']
    subprocess.run(cc + args, check=True)
    return dst


def load():
    lib = C.CDLL(str(build()))
    lib.bg_init.argtypes = [C.POINTER(State)]
    lib.bg_extract.argtypes = [C.POINTER(C.c_uint8), C.c_int, C.POINTER(Observation)]
    lib.bg_step.argtypes = [C.POINTER(State), C.POINTER(Observation), C.c_int,
                           C.c_float, C.c_float, C.c_float, C.c_int]
    lib.bg_step.restype = Output
    lib.bg_search.argtypes = [C.POINTER(Observation), C.POINTER(Path)]
    lib.bg_search.restype = C.c_int
    lib.bg_path_at.argtypes = [C.POINTER(Path), C.c_float]
    lib.bg_path_at.restype = C.c_float
    return lib


class Controller:
    def __init__(self, method='branchguard'):
        self.method = METHODS[method]
        self.lib = load()
        self.state = State()
        self.lib.bg_init(C.byref(self.state))

    def extract(self, frame):
        if frame.shape != (120, 188) or str(frame.dtype) != 'uint8' or not frame.flags.c_contiguous:
            raise ValueError('expected a contiguous uint8 120 x 188 frame')
        obs = Observation()
        self.lib.bg_extract(frame.ctypes.data_as(C.POINTER(C.c_uint8)), 188, C.byref(obs))
        return obs

    def step(self, frame, route=0, distance=0., yaw_change=0., dt=.04):
        obs = self.extract(frame)
        return self.step_observation(obs, route, distance, yaw_change, dt), obs

    def step_observation(self, obs, route=0, distance=0., yaw_change=0., dt=.04):
        return self.lib.bg_step(C.byref(self.state), C.byref(obs), route,
                                distance, yaw_change, dt, self.method)


SCAN = (112, 100, 88, 77, 67, 58, 50, 43, 37, 32, 28, 25)
DISTANCES = tuple(18. / (row - 12.) for row in SCAN)
