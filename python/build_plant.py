"""Build python/plant.pyd from MockDrive and the drive-policy sources.

The extension is the physics the gymnasium env steps. It does not link the
V5 firmware. Run this from anywhere:

    python python/build_plant.py
"""

import shutil
import subprocess
import sys
import sysconfig
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUT = Path(__file__).resolve().parent / "plant.pyd"

SOURCES = [
    ROOT / "python" / "plant_bind.cpp",
    ROOT / "src" / "host" / "MockDrive.cpp",
    ROOT / "src" / "host" / "FieldWindow.cpp",
    ROOT / "src" / "host" / "DeviceScreens.cpp",
    ROOT / "src" / "math" / "WallRange.cpp",
    ROOT / "src" / "control" / "DrivePolicy.cpp",
    ROOT / "src" / "control" / "DriftFollower.cpp",
    ROOT / "src" / "math" / "NeuralNetwork.cpp",
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


def command():
    libs, version = python_lib()
    common = includes() + [str(path) for path in SOURCES]
    # clang-cl speaks the MSVC ABI, which is what official Windows CPython uses.
    return [
        "clang-cl",
        "/nologo",
        "/LD",
        "/std:c++20",
        "/O2",
        "/EHsc",
        "/D_USE_MATH_DEFINES",
        "/DNDEBUG",
        *common,
        "/link",
        f"/OUT:{OUT}",
        f"/LIBPATH:{libs}",
        f"{version}.lib",
        "user32.lib",
        "gdi32.lib",
    ]


def mingw_command():
    compiler = shutil.which("clang++")
    if compiler is None:
        raise SystemExit(
            "clang-cl could not find the MSVC standard library, and clang++ is not on PATH."
        )
    libs, version = python_lib()
    cmd = [
        compiler,
        "-shared",
        "-O2",
        "-std=gnu++20",
        "-D_USE_MATH_DEFINES",
        "-DNDEBUG",
        *includes(),
        *[str(path) for path in SOURCES],
        f"-L{libs}",
        f"-l{version}",
        "-luser32",
        "-lgdi32",
        "-lpthread",
        "-o",
        str(OUT),
    ]
    return cmd, Path(compiler).resolve().parent


def copy_runtime(bindir):
    # Python does not search PATH for a pyd's dependencies. These live beside it.
    for name in ("libstdc++-6.dll", "libgcc_s_seh-1.dll", "libwinpthread-1.dll"):
        src = bindir / name
        if src.is_file():
            shutil.copy2(src, OUT.parent / name)


def main():
    missing = [str(path) for path in SOURCES if not path.is_file()]
    if missing:
        raise SystemExit("missing sources:\n  " + "\n  ".join(missing))
    msvc = command()
    print(" ".join(msvc))
    try:
        subprocess.check_call(msvc, cwd=ROOT)
    except (OSError, subprocess.CalledProcessError):
        cmd, bindir = mingw_command()
        print(" ".join(cmd))
        subprocess.check_call(cmd, cwd=ROOT)
        copy_runtime(bindir)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    main()
