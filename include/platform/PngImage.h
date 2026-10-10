#pragma once
/* ---------------------------------------------------------------------------
 * PngImage  --  load one PNG into 8-bit RGBA.
 * Shared by the V5 image and the host build. Non-interlaced images only.
 * ------------------------------------------------------------------------- */

#include <cstdint>
#include <vector>

// False when the file is missing or the PNG is not a supported image.
// On success, rgba is width * height * 4 bytes, row-major, R,G,B,A.
bool loadPngRgba(const char* path, std::vector<std::uint8_t>& rgba, int& width, int& height);
