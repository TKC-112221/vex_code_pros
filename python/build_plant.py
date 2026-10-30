"""Build python/plant.pyd from MockDrive and the drive-policy sources.

The extension is the physics the gymnasium env steps. It does not link the
V5 firmware. It also exports the Boomerang settle window
(SETTLE_LARGE_IN, SETTLE_LARGE_DEG, SETTLE_SMALL_S, SETTLE_LARGE_S).
optimize_tuning.py stops the clock on that window, so rebuild after
changing Tuning.h or plant_bind.cpp. Run this from anywhere:

    python python/build_plant.py
"""

import shutil
import subprocess
import sys
import sysconfig
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUT = Path(__file__).resolve().parent / "plant.pyd"

SOURCES = [
    ROOT / "python" / "plant_bind.cpp",
    ROOT / "src" / "host" / "MockDrive.cpp",
    ROOT / "src" / "host" / "FieldWindow.cpp",
    ROOT / "src" / "host" / "DeviceScreens.cpp",
    ROOT / "src" / "logic" / "GoalAligner.cpp",
    ROOT / "src" / "math" / "WallRange.cpp",
    ROOT / "src" / "control" / "DrivePolicy.cpp",
    ROOT / "src" / "control" / "DriftFollower.cpp",
    ROOT / "src" / "control" / "SegmentFollower.cpp",
    ROOT / "src" / "math" / "NeuralNetwork.cpp",
    ROOT / "src" / "math" / "ContactPhysics.cpp",
    ROOT / "src" / "auton" / "AutonCatalog.cpp",
    ROOT / "src" / "auton" / "MatchRoutines.cpp",
    ROOT / "src" / "auton" / "SkillsRoutine.cpp",
]


def includes():
    import pybind11

    return [
        f"-I{ROOT / 'include'}",
        f"-I{pybind11.get_include()}",
        f"-I{sysconfig.get_path('include')}",
    ]


def python_lib():
    version = f"python{sys.version_info.major}{sys.version_info.minor}"
    libs = Path(sys.base_prefix) / "libs"
    return libs, version


GNU_TARGET = "--target=x86_64-w64-windows-gnu"


def compiler():
    path = shutil.which("clang++")
    if path is None:
        raise SystemExit("clang++ is not on PATH.")
    return path


def require_gnu_stdlib(clang):
    # Windows clang++ defaults to the MSVC triple. The GNU triple still needs
    # a MinGW-w64 sysroot for libstdc++ and the Windows headers.
    with tempfile.TemporaryDirectory() as tmp:
        source = Path(tmp) / "probe.cpp"
        obj = Path(tmp) / "probe.o"
        source.write_text("#include <cstdint>\nint probe(){return 0;}\n", encoding="ascii")
        result = subprocess.run(
            [clang, GNU_TARGET, "-std=gnu++20", "-c", str(source), "-o", str(obj)],
            capture_output=True,
            text=True,
        )
    if result.returncode == 0:
        return
    detail = (result.stderr or result.stdout).strip()
    raise SystemExit(
        "clang++ cannot see a GNU C++ standard library.\n"
        "Install a MinGW-w64 toolchain (WinLibs or LLVM MinGW) so "
        f"{GNU_TARGET} can find <cstdint>.\n"
        + detail
    )


def command():
    clang = compiler()
    require_gnu_stdlib(clang)
    libs, version = python_lib()
    prefix = Path(sys.base_prefix)
    cmd = [
        clang,
        GNU_TARGET,
        "-shared",
        "-O2",
        "-std=gnu++20",
        "-D_USE_MATH_DEFINES",
        "-DNDEBUG",
        "-DPLANT_MODULE",
        *includes(),
        *[str(path) for path in SOURCES],
        f"-L{prefix}",
        f"-L{libs}",
        f"-l{version}",
        "-lpthread",
        "-luser32",
        "-lgdi32",
        "-o",
        str(OUT),
    ]
    return cmd, Path(clang).resolve().parent


def copy_runtime(compiler, bindir):
    # Python does not search PATH for a pyd's dependencies. These live beside it.
    dirs = [bindir]
    probe = subprocess.run(
        [compiler, GNU_TARGET, "-print-file-name=libstdc++-6.dll"],
        check=False,
        capture_output=True,
        text=True,
    )
    printed = Path(probe.stdout.strip()) if probe.returncode == 0 else None
    if printed is not None and printed.is_file():
        dirs.append(printed.parent)
    for name in ("libstdc++-6.dll", "libgcc_s_seh-1.dll", "libwinpthread-1.dll"):
        for directory in dirs:
            src = directory / name
            if src.is_file():
                try:
                    shutil.copy2(src, OUT.parent / name)
                except shutil.SameFileError:
                    pass
                break


def main():
    missing = [str(path) for path in SOURCES if not path.is_file()]
    if missing:
        raise SystemExit("missing sources:\n  " + "\n  ".join(missing))
    cmd, bindir = command()
    print(" ".join(cmd))
    subprocess.check_call(cmd, cwd=ROOT)
    copy_runtime(cmd[0], bindir)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
