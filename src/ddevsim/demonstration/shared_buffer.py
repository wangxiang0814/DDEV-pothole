"""Minimal ctypes reader for the installed VS Shared Buffer API v2.

Indices and signatures are from Programs/vs_sb/vs_SharedBuffer.h. The buffer
handle is opaque, never a copied private C++ structure. Copy pixels while locked.
"""
import ctypes as c
import math
from pathlib import Path
import time

import numpy as np

DEFAULT_DLL = Path(r'F:\TruckSim2019\TruckSim2019.0_Prog\Programs\vs_sb\vs_sb_64.dll')


class FrameInfo(c.Structure):
    _fields_ = [('number', c.c_int32)] + [(n, c.c_double) for n in (
        'time_s', 'hfov_rad', 'vfov_rad', 'x', 'y', 'z', 'ax', 'ay', 'az', 'angle_rad')]


class SharedBuffer:
    def __init__(self, name, timeout_s=45, dll_path=DEFAULT_DLL):
        self.dll = c.CDLL(str(dll_path))
        self.dll.GetVsSharedBufferApi.argtypes = [c.c_int]
        self.dll.GetVsSharedBufferApi.restype = c.POINTER(c.c_void_p)
        self.api = self.dll.GetVsSharedBufferApi(2)
        if not self.api:
            raise RuntimeError('VS Shared Buffer API v2 unavailable')
        self.handle = None
        self.name = name
        deadline = time.monotonic() + timeout_s
        result = c.c_int()
        while time.monotonic() < deadline:
            self.handle = self.fn(17, c.c_void_p, c.c_wchar_p, c.c_int, c.POINTER(c.c_int))(
                name, 100, c.byref(result))
            if self.handle:
                return
            # Polling only waits for OS resources; it never determines simulation time.
            time.sleep(.05)
        raise RuntimeError('Shared buffer %s unavailable: %s' % (
            name, self.fn(9, c.c_wchar_p, c.c_int)(result.value)))

    def fn(self, index, result, *args):
        return c.CFUNCTYPE(result, *args)(self.api[index])

    def read(self, after_time=None, minimum_time=None):
        if self.fn(24, c.c_int, c.c_void_p, c.c_int)(self.handle, 1000) != 0:
            raise TimeoutError('shared buffer lock failed')
        try:
            if self.fn(26, c.c_int, c.c_void_p)(self.handle) != 1:
                return None
            info = FrameInfo()
            self.fn(29, None, c.c_void_p, c.POINTER(FrameInfo))(self.handle, c.byref(info))
            if info.number < 0:
                return None
            if not math.isfinite(info.time_s):
                raise ValueError('nonfinite camera timestamp')
            # Poll metadata cheaply. Do not copy five RGB arrays hundreds of
            # times per second while waiting for the next requested sample.
            if ((after_time is not None and info.time_s <= after_time+1e-9) or
                    (minimum_time is not None and info.time_s < minimum_time-1e-8)):
                return None
            w = self.fn(22, c.c_size_t, c.c_void_p)(self.handle)
            h = self.fn(23, c.c_size_t, c.c_void_p)(self.handle)
            types = self.fn(21, c.c_int, c.c_void_p)(self.handle)
            if not 0 < w <= 4096 or not 0 < h <= 4096 or not types & 1:
                raise ValueError('invalid RGB buffer format')
            ptr = self.fn(31, c.c_void_p, c.c_void_p, c.c_int, c.c_uint)(self.handle, 1, 0)
            if not ptr:
                raise ValueError('missing RGB pixels')
            raw = c.string_at(ptr, w*h*3)
        finally:
            self.fn(25, c.c_int, c.c_void_p)(self.handle)
        meta = {n: getattr(info, n) for n, _ in info._fields_}
        if not all(math.isfinite(v) for v in meta.values()):
            raise ValueError('nonfinite camera metadata')
        if not 0 < info.hfov_rad < math.pi or not 0 < info.vfov_rad < math.pi:
            raise ValueError('invalid native camera FOV')
        meta['intrinsics_K'] = [
            [w/(2*math.tan(info.hfov_rad/2)), 0, (w-1)/2],
            [0, h/(2*math.tan(info.vfov_rad/2)), (h-1)/2], [0, 0, 1]]
        meta['width'], meta['height'] = w, h
        # Native OpenGL rows start at the bottom. RGB remains RGB (not OpenCV BGR).
        rgb = np.frombuffer(raw, np.uint8).reshape(h, w, 3)[::-1].copy()
        return meta, rgb

    def close(self):
        if self.handle:
            handle = c.c_void_p(self.handle)
            self.fn(19, None, c.POINTER(c.c_void_p))(c.byref(handle))
            self.handle = None


def synchronized_frames(frames, last_time=-1):
    """Reject torn five-camera groups; never relabel timestamps."""
    if len(frames) != 5 or any(f is None for f in frames):
        return False
    times = [f[0]['time_s'] for f in frames]
    return (all(math.isfinite(t) for t in times) and
            max(times)-min(times) < 1e-9 and min(times) > last_time + 1e-9)
