#include "platform/RobotFile.h"
#include <cstring>

FILE* openRobotFile(const char* path, const char* mode) {
    const char* open_path = path;
#if defined(ROBOT_HOST)
    constexpr char kUsdPrefix[] = "/usd/";
    constexpr std::size_t kUsdPrefixLen = sizeof(kUsdPrefix) - 1;
    if (path != nullptr && std::strncmp(path, kUsdPrefix, kUsdPrefixLen) == 0)
        open_path = path + kUsdPrefixLen;
#endif
#if defined(_MSC_VER)
    FILE* file = nullptr;
    if (fopen_s(&file, open_path, mode) != 0) return nullptr;
    return file;
#else
    return std::fopen(open_path, mode);
#endif
}
