#include "platform/RobotFile.h"
#include <cstring>

#if !defined(ROBOT_HOST)
#include "pros/misc.h"
#endif

FILE* openRobotFile(const char* path, const char* mode) {
    if (path == nullptr || mode == nullptr) return nullptr;
    const char* open_path = path;
#if defined(ROBOT_HOST)
    constexpr char kUsdPrefix[] = "/usd/";
    constexpr std::size_t kUsdPrefixLen = sizeof(kUsdPrefix) - 1;
    if (std::strncmp(path, kUsdPrefix, kUsdPrefixLen) == 0)
        open_path = path + kUsdPrefixLen;
#else
    // Card-detect only. fopen mounts the volume and does not return while
    // the slot is empty, which would hold initialize() for the whole boot.
    const bool usd = std::strncmp(path, "/usd", 4) == 0 && (path[4] == '/' || path[4] == '\0');
    if (usd && pros::c::usd_is_installed() != 1) return nullptr;
#endif
#if defined(_MSC_VER)
    FILE* file = nullptr;
    if (fopen_s(&file, open_path, mode) != 0) return nullptr;
    return file;
#else
    return std::fopen(open_path, mode);
#endif
}
