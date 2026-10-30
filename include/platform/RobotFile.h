#pragma once
/* ---------------------------------------------------------------------------
 * RobotFile  --  open a path the robot code already uses.
 * On the V5, "/usd/foo" is the microSD card. On the host build, that prefix
 * is removed so the same call opens a file in the working directory.
 * A missing card returns null. fopen on "/usd" waits inside VEXos for a
 * card that is not there, and initialize() calls this before any task runs.
 * ------------------------------------------------------------------------- */

#include <cstdio>

FILE* openRobotFile(const char* path, const char* mode);
