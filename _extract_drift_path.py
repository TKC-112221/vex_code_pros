from pathlib import Path

root = Path(r"d:\projects\VexCode\vex_code_pros")
src_path = root / "src/control/DriftCurve.cpp"
lines = src_path.read_text(encoding="utf-8").splitlines(keepends=True)


def find(prefix):
    hits = [i for i, line in enumerate(lines) if line.startswith(prefix)]
    if len(hits) != 1:
        raise SystemExit(f"{prefix!r} -> {hits}")
    return hits[0]


i_onaxis = find("constexpr double kOnAxisM")
i_volts = find("double voltsPerPower()")
i_circle = find("// Circle through the robot")
i_ns_end = find("} // namespace")

body = lines[i_onaxis:i_volts] + lines[i_circle:i_ns_end]


def prefix_inline(line):
    if line.startswith("constexpr double"):
        return line.replace("constexpr double", "inline constexpr double", 1)
    head = line.split("{")[0]
    if line.startswith(("void ", "double ", "Finish ", "Arc ", "GraphPath ", "Quintic ", "Momentum ")) and "(" in head:
        return "inline " + line
    return line


body = [prefix_inline(line) for line in body]
header = """#pragma once
/* ---------------------------------------------------------------------------
 * DriftPathFit  --  the DriftCurve shapes and the straight finish.
 *
 * Shared by DriftCurve, which refits every tick, and DriftFollower, which
 * samples a path once. The geometry matches the notes in DriftCurve.h.
 * ------------------------------------------------------------------------- */

#include "core/Specs.h"
#include "math/MathUtils.h"
#include <cmath>

namespace driftpath {

""" + "".join(body) + "} // namespace driftpath\n"

out = root / "include/control/DriftPathFit.h"
out.write_text(header, encoding="utf-8")

new_lines = lines[:i_onaxis] + lines[i_volts:i_circle] + lines[i_ns_end:]
text = "".join(new_lines)
old_inc = '#include "control/DriftCurve.h"\n'
new_inc = '#include "control/DriftCurve.h"\n#include "control/DriftPathFit.h"\n'
if old_inc not in text:
    raise SystemExit("include missing")
text = text.replace(old_inc, new_inc, 1)
needle = "namespace {\n"
if needle not in text:
    raise SystemExit("namespace missing")
text = text.replace(needle, "using namespace driftpath;\n\nnamespace {\n", 1)
src_path.write_text(text, encoding="utf-8")
print("header", out.stat().st_size)
print("markers", i_onaxis, i_volts, i_circle, i_ns_end)
