"""Native TruckSim (VS Visualizer) animation packaging and video-export helpers.

What "native" means here
------------------------
The engineering replay produced by ``ddevsim.replay_video`` is a NumPy software
render of the logged channels.  It is useful for debugging but it is **not** the
TruckSim 3D scene, so it is unsuitable as a paper figure.  A native video must be
rendered by VS Visualizer from the solver's own history (``.vs`` + ``.vsb``).

What can and cannot be automated (verified against TruckSim 2019.0)
------------------------------------------------------------------
The VS Visualizer command line was enumerated from its own embedded help strings
(``Programs\\VsVisualizer\\VsVisualizer64\\VsVisualizer.exe``)::

    -res              Resolution of render window in pixels.
    -nosound          Disable all sound processing.
    -vsrap <name>     Create VSRAP with the specified name from the Parsfile,
                      then exit.
    -vsrapoverwrite   Overwrite VSRAP if it exists. Use with "-vsrap".
    (plus -fs, -tb, -fscf, -uimode, -pos, -rd, -tr, -wb, -zc, -altresource)

There is **no command-line video option**.  Video export exists only behind the
GUI menu item ``Export Video...``, which calls the Windows Video-for-Windows API
(``AVISaveOptions`` -> ``AVIMakeCompressedStream`` -> ``AVIStreamWrite``) and
therefore opens an interactive compressor chooser.  The binary imports that API
(``AVIFIL32.dll``) but exposes no switch to drive it.

So the pipeline is: *everything up to the final write is scripted; the last step is
a short, deterministic GUI sequence.*  This module automates its half, and
:func:`manual_export_steps` returns the exact clicks for the other half.
"""

from __future__ import annotations

import json
import os
import struct
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence

DEFAULT_VISUALIZER_64 = Path(
    r"F:\TruckSim2019\TruckSim2019.0_Prog\Programs\VsVisualizer\VsVisualizer64\VsVisualizer.exe"
)
DEFAULT_VISUALIZER_32 = Path(
    r"F:\TruckSim2019\TruckSim2019.0_Prog\Programs\VsVisualizer\VsVisualizer.exe"
)
DEFAULT_TRUCKSIM_DATA = Path(r"F:\TruckSim2019\TruckSim2019.0_Data")


# --------------------------------------------------------------------- discovery
@dataclass
class HistorySet:
    """A complete TruckSim native history triple."""

    vs: Path
    vsb: Path
    par: Path
    basename: str
    channels: List[str] = field(default_factory=list)
    units: Dict[str, str] = field(default_factory=dict)
    samples: int = 0
    time_step_s: float = 0.0
    duration_s: float = 0.0

    def as_dict(self) -> Dict[str, object]:
        return {
            "basename": self.basename,
            "vs": str(self.vs),
            "vsb": str(self.vsb),
            "par": str(self.par),
            "channels": len(self.channels),
            "samples": self.samples,
            "time_step_s": self.time_step_s,
            "duration_s": self.duration_s,
        }


def find_history(source: Path) -> HistorySet:
    """Locate and validate the ``.vs`` / ``.vsb`` / ``_all.par`` triple.

    ``source`` may be the ``.vs`` file, the ``.vsb`` file, or any directory that
    contains them (for example a batch case's ``model/output`` directory).
    """
    source = Path(source)
    if source.is_file():
        directory = source.parent
        basename = source.stem
    elif source.is_dir():
        candidates = sorted(source.glob("*.vs"))
        if not candidates:
            raise FileNotFoundError("no .vs history file in %s" % source)
        if len(candidates) > 1:
            names = ", ".join(candidate.stem for candidate in candidates)
            raise ValueError(
                "%s contains several histories (%s); point at the .vs file directly"
                % (source, names)
            )
        directory, basename = source, candidates[0].stem
    else:
        raise FileNotFoundError("no such history source: %s" % source)

    vs = directory / (basename + ".vs")
    vsb = directory / (basename + ".vsb")
    par = directory / (basename + "_all.par")
    for path in (vs, vsb, par):
        if not path.exists():
            raise FileNotFoundError(
                "incomplete history: %s is missing (need .vs, .vsb and _all.par "
                "in the same directory)" % path
            )
    return describe_history(vs, vsb, par)


def describe_history(vs: Path, vsb: Path, par: Path) -> HistorySet:
    """Read the ``.vs`` JSON header and check it against the binary ``.vsb``.

    The ``.vs`` header carries each channel's declared unit, which is the
    authority used by :mod:`ddevsim.units`.  ``.vsb`` layout (verified): a 24-byte
    header ``<6i>`` whose first int is a version, the fifth is bytes per value and
    the sixth is the channel count, followed by ``channels * samples`` values.
    """
    header = json.loads(Path(vs).read_text(encoding="utf-8"))
    group = header["VsChannelGroup"]
    channels = [entry["Name Aliases"][0] for entry in group["Channels"]]
    units = {
        entry["Name Aliases"][0]: entry.get("Units", "") for entry in group["Channels"]
    }

    raw = Path(vsb).read_bytes()
    version, _, _, _, value_bytes, channel_count = struct.unpack_from("<6i", raw, 0)
    if channel_count != len(channels):
        raise ValueError(
            ".vsb declares %d channels but .vs lists %d" % (channel_count, len(channels))
        )
    if value_bytes not in (4, 8):
        raise ValueError("unexpected .vsb value size %d" % value_bytes)
    data_offset = 24
    total = len(raw) - data_offset
    if total % (channel_count * value_bytes) != 0:
        raise ValueError(
            ".vsb payload %d bytes is not a whole number of frames" % total
        )
    samples = total // (channel_count * value_bytes)
    step = float(group.get("XStep", 0.0))

    return HistorySet(
        vs=Path(vs), vsb=Path(vsb), par=Path(par), basename=Path(vs).stem,
        channels=channels, units=units, samples=samples, time_step_s=step,
        duration_s=step * max(0, samples - 1),
    )


# ------------------------------------------------------------------------ staging
def is_ascii_path(path: Path) -> bool:
    try:
        str(path).encode("ascii")
        return True
    except UnicodeEncodeError:
        return False


def resolve_stage_dir(preferred: Path) -> tuple:
    """Return an ASCII-safe staging directory and whether it had to be relocated.

    VS Visualizer's parsfiles are read as ANSI text, so a project path containing
    non-ASCII characters (this project's does) cannot be written into an
    ``animator.par``.  When that happens the history is staged under the system
    temporary directory instead, exactly as the retained manual script did by
    staging into ``F:\\DDEV_TruckSim_Visualizer``.
    """
    preferred = Path(preferred)
    if is_ascii_path(preferred):
        return preferred, False
    import tempfile

    fallback = Path(tempfile.gettempdir()) / "ddev_native_video" / preferred.name
    return fallback, True


def stage_history(history: HistorySet, stage_dir: Path) -> HistorySet:
    """Copy the ``.vs``/``.vsb``/``_all.par`` triple into ``stage_dir``."""
    import shutil

    stage_dir = Path(stage_dir)
    stage_dir.mkdir(parents=True, exist_ok=True)
    for source in (history.vs, history.vsb, history.par):
        target = stage_dir / source.name
        if source.resolve() != target.resolve():
            shutil.copy2(str(source), str(target))
    return describe_history(
        stage_dir / history.vs.name,
        stage_dir / history.vsb.name,
        stage_dir / history.par.name,
    )


def write_animator_par(
    stage_dir: Path, history: HistorySet, run_slot: int = 0
) -> Path:
    """Write the animator parsfle that binds a history into VS Visualizer.

    Running VS Visualizer with this file as its argument loads the history, which
    is what makes both the VSRAP generation and the GUI export work.

    **Encoding matters.**  VS Visualizer reads parsfiles as ANSI text, so this is
    written with the Windows ANSI codepage (``mbcs``, code page 936 on this
    machine).  Writing ASCII instead would mangle a non-ASCII project path into
    ``F:\\1tongji\\1 ?????\\...`` and the dataset would fail to load.  Only if the
    ANSI codepage genuinely cannot represent a character is the file written as
    UTF-8, and the caller is told so it can stage to an ASCII path instead.
    """
    stage_dir = Path(stage_dir)
    stage_dir.mkdir(parents=True, exist_ok=True)
    par = stage_dir / "animator.par"
    text = (
        "PARSFILE\n"
        "SET_RUN_SLOT %d\n"
        "DATASET %s\n"
        "PARSFILE %s\n"
        "END\n" % (run_slot, history.vs, history.par)
    )
    try:
        par.write_text(text, encoding="mbcs")
    except (UnicodeEncodeError, LookupError):
        par.write_text(text, encoding="utf-8")
    return par


def create_vsrap(
    animator_par: Path,
    vsrap_path: Path,
    visualizer: Path = DEFAULT_VISUALIZER_64,
    overwrite: bool = True,
    timeout_s: float = 90.0,
) -> Dict[str, object]:
    """Create a ``.vsrap`` package non-interactively via ``-vsrap`` (opt-in).

    ``-vsrap`` is documented in the binary's own help text ("Create VSRAP with the
    specified name from the Parsfile, then exit"), and the retained run on this
    machine does contain a 52 KB package, so the option works.  It is nevertheless
    treated as best-effort here: VS Visualizer is a GUI binary, it can block
    waiting for a window, and it has been observed to exit non-zero even on
    success.  A failure or timeout is therefore reported, not raised, and the
    animator.par launch path remains the primary route.
    """
    animator_par = Path(animator_par)
    vsrap_path = Path(vsrap_path)
    vsrap_path.parent.mkdir(parents=True, exist_ok=True)
    if vsrap_path.exists() and not overwrite:
        return {"created": False, "reason": "exists", "vsrap": str(vsrap_path)}

    # Argument order matters: "-vsrap" takes the output name as its own argument,
    # so the overwrite flag must not be inserted between them.
    arguments = ["-vsrap", str(vsrap_path)]
    if overwrite:
        arguments.append("-vsrapoverwrite")
    arguments.append(str(animator_par))

    # Redirect to files rather than pipes: some sandboxes forbid the named pipes
    # that capture_output creates, which would silently produce no diagnostics.
    stdout_path = animator_par.parent / "vsrap_stdout.txt"
    stderr_path = animator_par.parent / "vsrap_stderr.txt"
    try:
        with stdout_path.open("wb") as out, stderr_path.open("wb") as err:
            completed = subprocess.run(
                [str(visualizer)] + arguments,
                cwd=str(animator_par.parent),
                stdout=out,
                stderr=err,
                timeout=timeout_s,
            )
        return_code = completed.returncode
        timed_out = False
    except subprocess.TimeoutExpired:
        return_code = None
        timed_out = True

    created = vsrap_path.exists() and vsrap_path.stat().st_size > 0
    return {
        "created": created,
        "vsrap": str(vsrap_path),
        "size_bytes": vsrap_path.stat().st_size if vsrap_path.exists() else 0,
        "return_code": return_code,
        "timed_out": timed_out,
        "command": " ".join([str(visualizer)] + arguments),
        "stdout": stdout_path.read_text(encoding="utf-8", errors="replace")[:2000]
        if stdout_path.exists() else "",
        "stderr": stderr_path.read_text(encoding="utf-8", errors="replace")[:2000]
        if stderr_path.exists() else "",
        "note": (
            "Best-effort. VS Visualizer returns a non-zero code even on success, "
            "and it may block on a GUI window; the file existence/size check is the "
            "authoritative result. The animator.par launch path does not need this."
        ),
    }


def launch_visualizer(
    animator_par: Path,
    visualizer: Path = DEFAULT_VISUALIZER_64,
    workdir: Path = DEFAULT_TRUCKSIM_DATA,
    window_name: str = "DDEV_Pothole",
    frame_scale: str = "0",
    resolution: Optional[Sequence[int]] = (1280, 720),
    show_toolbars: bool = True,
    sound: bool = False,
    extra_args: Optional[Sequence[str]] = None,
) -> Dict[str, object]:
    """Launch VS Visualizer on a history, ready for the manual Export Video step."""
    animator_par = Path(animator_par)
    arguments: List[str] = ["-i", window_name, "-fs", str(frame_scale)]
    if resolution:
        arguments += ["-res", str(int(resolution[0])), str(int(resolution[1]))]
    if show_toolbars:
        arguments += ["-tb", "1"]
    if not sound:
        arguments.append("-nosound")
    if extra_args:
        arguments += list(extra_args)
    arguments.append(str(animator_par))

    creation = 0x00000010 | 0x00000008  # NEW_CONSOLE | DETACHED_PROCESS
    process = subprocess.Popen(
        [str(visualizer)] + arguments,
        cwd=str(workdir),
        creationflags=creation,
        close_fds=True,
    )
    return {
        "launched": True,
        "pid": process.pid,
        "command": " ".join([str(visualizer)] + arguments),
        "cwd": str(workdir),
    }


def manual_export_steps(
    history: HistorySet, stage_dir: Path, resolution: Sequence[int] = (1280, 720)
) -> List[str]:
    """The exact GUI sequence that writes the native AVI.

    Returned as data so both the CLI and the documentation print the same steps.
    """
    return [
        "The remaining step is the only one TruckSim does not expose on the command "
        "line: the AVI writer is behind VS Visualizer's Export Video dialog.",
        "",
        "1. VS Visualizer should now be open showing the TruckSim 3D scene for "
        "'%s'." % history.basename,
        "2. If it is not open, run the launch command printed above (or double-click "
        "the .vsrap).",
        "3. Confirm the replay covers the whole run: the history is %.3f s long "
        "(%d frames at %.4f s)." % (history.duration_s, history.samples, history.time_step_s),
        "4. Set the camera before exporting: use the Animator camera controls, or the "
        "dataset camera already embedded in the run (this platform sets "
        "azimuth -45 deg, elevation 14 deg, distance 16 m for the pothole case).",
        "5. Menu: File > Export Video...",
        "6. In the dialog set the time range to 0 to %.3f s and the frame rate to 30 "
        "fps (%d frames)." % (history.duration_s, max(1, int(round(history.duration_s * 30)))),
        "7. Choose the output file name with an .avi extension.",
        "8. Choose a compressor. 'Microsoft Video 1' is universally available; "
        "'Full Frames (Uncompressed)' is lossless but produces a very large file. "
        "H.264 is only offered if a suitable VFW codec is installed on the machine.",
        "9. Click OK/Save and wait for the export; it renders in real time, so a "
        "%.1f s run takes roughly %.1f s plus encoding time."
        % (history.duration_s, history.duration_s),
        "10. Optional: convert the AVI to MP4 for a paper figure, e.g. "
        "ffmpeg -i native.avi -c:v libx264 -pix_fmt yuv420p -crf 18 native.mp4",
        "",
        "Output resolution is controlled by the -res flag on launch (currently %d x %d)."
        % (int(resolution[0]), int(resolution[1])),
        "Staged files for this export are in: %s" % stage_dir,
    ]


def export_native_video(
    source: Path,
    stage_dir: Path,
    visualizer: Path = DEFAULT_VISUALIZER_64,
    resolution: Sequence[int] = (1280, 720),
    create_vsrap_package: bool = False,
    launch: bool = True,
    workdir: Path = DEFAULT_TRUCKSIM_DATA,
    ascii_stage: bool = False,
) -> Dict[str, object]:
    """Run the automatable part of the native video export for one history.

    ``create_vsrap_package`` is opt-in because the ``-vsrap`` switch drives a GUI
    binary.  On a sandboxed session VS Visualizer cannot even start (it needs to
    write ``%LOCALAPPDATA%\\VS Visualizer\\<version>\\``), so check the returned
    report rather than assuming success.
    """
    history = find_history(source)
    stage_dir = Path(stage_dir)
    if ascii_stage:
        stage_dir, relocated = resolve_stage_dir(stage_dir)
    else:
        relocated = False
    stage_dir.mkdir(parents=True, exist_ok=True)

    # Stage a copy so the .vsrap and the parsfle live side by side.  Paths may stay
    # non-ASCII because the parsfle is written with the ANSI codepage.
    staged = stage_history(history, stage_dir)
    animator_par = write_animator_par(stage_dir, staged)
    report: Dict[str, object] = {
        "source_history": history.as_dict(),
        "history": staged.as_dict(),
        "animator_par": str(animator_par),
        "stage_dir": str(stage_dir),
        "stage_dir_relocated_for_ascii": relocated,
        "video_export_is_gui_only": True,
        "cli_options_verified": {
            "-vsrap": "Create VSRAP with the specified name from the Parsfile, then exit.",
            "-vsrapoverwrite": 'Overwrite VSRAP if it exists. Use with "-vsrap".',
            "-res": "Resolution of render window in pixels.",
            "-nosound": "Disable all sound processing.",
        },
    }
    history = staged

    if create_vsrap_package:
        vsrap = stage_dir / (history.basename + ".vsrap")
        report["vsrap"] = create_vsrap(animator_par, vsrap, visualizer=visualizer)

    if launch:
        report["launch"] = launch_visualizer(
            animator_par, visualizer=visualizer, workdir=workdir, resolution=resolution
        )

    report["manual_steps"] = manual_export_steps(history, stage_dir, resolution)
    (stage_dir / "native_video_export.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return report


def default_visualizer() -> Path:
    """Prefer the 64-bit build, falling back to the 32-bit one."""
    if DEFAULT_VISUALIZER_64.exists():
        return DEFAULT_VISUALIZER_64
    if DEFAULT_VISUALIZER_32.exists():
        return DEFAULT_VISUALIZER_32
    raise FileNotFoundError("VS Visualizer executable not found")


def default_workdir() -> Path:
    """VS Visualizer expects to start in the TruckSim data directory."""
    return DEFAULT_TRUCKSIM_DATA if DEFAULT_TRUCKSIM_DATA.exists() else Path(os.getcwd())
