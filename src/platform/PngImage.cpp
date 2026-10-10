/* ---------------------------------------------------------------------------
 * PngImage.cpp  --  PNG loader for splash frames.
 * Non-interlaced gray, RGB, palette, gray+alpha, and RGBA. Bit depths follow
 * the PNG spec for each color type. Both builds decode the same bytes; only
 * the path prefix differs, via openRobotFile.
 * ------------------------------------------------------------------------- */
#include "platform/PngImage.h"
#include "platform/RobotFile.h"

#include <cstdint>
#include <cstring>
#include <vector>

namespace {

constexpr std::size_t kMaxFile = 8u * 1024u * 1024u;
constexpr int kMaxEdge = 1920;

constexpr int kLenBase[29] = {
    3, 4, 5, 6, 7, 8, 9, 10,
    11, 13, 15, 17, 19, 23, 27, 31,
    35, 43, 51, 59, 67, 83, 99, 115,
    131, 163, 195, 227, 258
};
constexpr int kLenExtra[29] = {
    0, 0, 0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 2, 2, 2, 2,
    3, 3, 3, 3, 4, 4, 4, 4,
    5, 5, 5, 5, 0
};
constexpr int kDistBase[30] = {
    1, 2, 3, 4, 5, 7, 9, 13,
    17, 25, 33, 49, 65, 97, 129, 193,
    257, 385, 513, 769, 1025, 1537, 2049, 3073,
    4097, 6145, 8193, 12289, 16385, 24577
};
constexpr int kDistExtra[30] = {
    0, 0, 0, 0, 1, 1, 2, 2,
    3, 3, 4, 4, 5, 5, 6, 6,
    7, 7, 8, 8, 9, 9, 10, 10,
    11, 11, 12, 12, 13, 13
};
constexpr int kClenOrder[19] = {
    16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15
};

struct BitReader {
    const std::uint8_t* p;
    const std::uint8_t* end;
    std::uint32_t acc = 0;
    int bits = 0;

    int get(int n) {
        while (bits < n) {
            if (p >= end) return -1;
            acc |= (std::uint32_t)(*p++) << bits;
            bits += 8;
        }
        int value = (int)(acc & ((1u << n) - 1u));
        acc >>= n;
        bits -= n;
        return value;
    }
};

struct Huff {
    int counts[16] = {};
    int symbols[288] = {};
    int first_code[16] = {};
    int first_index[16] = {};
    int max_len = 0;
};

bool buildHuff(Huff& table, const int* lengths, int count) {
    for (int i = 0; i < 16; ++i) table.counts[i] = 0;
    for (int i = 0; i < count; ++i) {
        int len = lengths[i];
        if (len < 0 || len > 15) return false;
        if (len > 0) table.counts[len] += 1;
    }
    int code = 0;
    int index = 0;
    table.max_len = 0;
    for (int len = 1; len <= 15; ++len) {
        code = (code + table.counts[len - 1]) << 1;
        table.first_code[len] = code;
        table.first_index[len] = index;
        index += table.counts[len];
        if (table.counts[len] > 0) table.max_len = len;
    }
    if (table.max_len > 0) {
        int past = table.first_code[table.max_len] + table.counts[table.max_len];
        if (past > (1 << table.max_len)) return false;
    }
    int next[16];
    for (int len = 0; len < 16; ++len) next[len] = table.first_index[len];
    for (int sym = 0; sym < count; ++sym) {
        int len = lengths[sym];
        if (len > 0) table.symbols[next[len]++] = sym;
    }
    return true;
}

int decodeHuff(BitReader& reader, const Huff& table) {
    if (table.max_len == 0) return -1;
    int code = 0;
    for (int len = 1; len <= table.max_len; ++len) {
        int bit = reader.get(1);
        if (bit < 0) return -1;
        code = (code << 1) | bit;
        int n = table.counts[len];
        if (n > 0 && code >= table.first_code[len] && code < table.first_code[len] + n)
            return table.symbols[table.first_index[len] + (code - table.first_code[len])];
    }
    return -1;
}

std::uint32_t readBe32(const std::uint8_t* p) {
    return ((std::uint32_t)p[0] << 24) | ((std::uint32_t)p[1] << 16) |
           ((std::uint32_t)p[2] << 8) | (std::uint32_t)p[3];
}

std::uint32_t adler32(const std::uint8_t* data, std::size_t n) {
    std::uint32_t s1 = 1;
    std::uint32_t s2 = 0;
    for (std::size_t i = 0; i < n; ++i) {
        s1 = (s1 + data[i]) % 65521u;
        s2 = (s2 + s1) % 65521u;
    }
    return (s2 << 16) | s1;
}

bool inflateCodes(BitReader& reader, const Huff& lit, const Huff& dist,
                  std::vector<std::uint8_t>& out, std::size_t expected) {
    for (;;) {
        int sym = decodeHuff(reader, lit);
        if (sym < 0 || sym > 285) return false;
        if (sym < 256) {
            if (out.size() >= expected) return false;
            out.push_back((std::uint8_t)sym);
            continue;
        }
        if (sym == 256) return true;
        int extra_n = kLenExtra[sym - 257];
        int extra = extra_n > 0 ? reader.get(extra_n) : 0;
        if (extra < 0) return false;
        int length = kLenBase[sym - 257] + extra;
        int dsym = decodeHuff(reader, dist);
        if (dsym < 0 || dsym > 29) return false;
        int dextra_n = kDistExtra[dsym];
        int dextra = dextra_n > 0 ? reader.get(dextra_n) : 0;
        if (dextra < 0) return false;
        int distance = kDistBase[dsym] + dextra;
        if (distance <= 0 || (std::size_t)distance > out.size()) return false;
        if (out.size() + (std::size_t)length > expected) return false;
        for (int i = 0; i < length; ++i)
            out.push_back(out[out.size() - (std::size_t)distance]);
    }
}

bool inflateBlock(BitReader& reader, std::vector<std::uint8_t>& out, std::size_t expected, bool& final_block) {
    int bfinal = reader.get(1);
    int btype = reader.get(2);
    if (bfinal < 0 || btype < 0) return false;
    final_block = bfinal != 0;

    if (btype == 0) {
        int drop = reader.bits & 7;
        if (drop > 0) {
            reader.acc >>= drop;
            reader.bits -= drop;
        }
        int lo = reader.get(8);
        int hi = reader.get(8);
        int nlo = reader.get(8);
        int nhi = reader.get(8);
        if (lo < 0 || hi < 0 || nlo < 0 || nhi < 0) return false;
        int len = lo | (hi << 8);
        int nlen = nlo | (nhi << 8);
        if ((len ^ nlen) != 0xFFFF) return false;
        if (reader.bits != 0) return false;
        if ((std::size_t)(reader.end - reader.p) < (std::size_t)len) return false;
        if (out.size() + (std::size_t)len > expected) return false;
        out.insert(out.end(), reader.p, reader.p + len);
        reader.p += len;
        return true;
    }

    Huff lit;
    Huff dist;
    if (btype == 1) {
        int lit_len[288];
        int dist_len[32];
        for (int i = 0; i <= 143; ++i) lit_len[i] = 8;
        for (int i = 144; i <= 255; ++i) lit_len[i] = 9;
        for (int i = 256; i <= 279; ++i) lit_len[i] = 7;
        for (int i = 280; i <= 287; ++i) lit_len[i] = 8;
        for (int i = 0; i < 32; ++i) dist_len[i] = 5;
        if (!buildHuff(lit, lit_len, 288) || !buildHuff(dist, dist_len, 32)) return false;
    } else if (btype == 2) {
        int hlit = reader.get(5);
        int hdist = reader.get(5);
        int hclen = reader.get(4);
        if (hlit < 0 || hdist < 0 || hclen < 0) return false;
        hlit += 257;
        hdist += 1;
        hclen += 4;
        if (hlit > 286 || hdist > 32 || hclen > 19) return false;

        int clen_len[19] = {};
        for (int i = 0; i < hclen; ++i) {
            int n = reader.get(3);
            if (n < 0) return false;
            clen_len[kClenOrder[i]] = n;
        }
        Huff clen;
        if (!buildHuff(clen, clen_len, 19)) return false;

        int lengths[318];
        for (int i = 0; i < 318; ++i) lengths[i] = 0;
        int filled = 0;
        int prev = -1;
        int need = hlit + hdist;
        while (filled < need) {
            int sym = decodeHuff(reader, clen);
            if (sym < 0 || sym > 18) return false;
            int rep = 1;
            int value = sym;
            if (sym == 16) {
                if (prev < 0) return false;
                int extra = reader.get(2);
                if (extra < 0) return false;
                rep = 3 + extra;
                value = prev;
            } else if (sym == 17) {
                int extra = reader.get(3);
                if (extra < 0) return false;
                rep = 3 + extra;
                value = 0;
            } else if (sym == 18) {
                int extra = reader.get(7);
                if (extra < 0) return false;
                rep = 11 + extra;
                value = 0;
            }
            if (filled + rep > need) return false;
            for (int i = 0; i < rep; ++i) lengths[filled++] = value;
            prev = value;
        }
        if (!buildHuff(lit, lengths, hlit) || !buildHuff(dist, lengths + hlit, hdist)) return false;
    } else {
        return false;
    }
    return inflateCodes(reader, lit, dist, out, expected);
}

bool inflateZlib(const std::uint8_t* src, std::size_t len,
                 std::vector<std::uint8_t>& out, std::size_t expected) {
    if (len < 6 || expected == 0) return false;
    unsigned cmf = src[0];
    unsigned flg = src[1];
    if ((cmf & 0x0fu) != 8u || (cmf >> 4) > 7u) return false;
    if (((cmf << 8) + flg) % 31u != 0u) return false;
    if ((flg & 0x20u) != 0u) return false;

    BitReader reader{src + 2, src + len, 0, 0};
    out.clear();
    out.reserve(expected);
    bool final_block = false;
    int guard = 0;
    while (!final_block) {
        if (++guard > 100000) return false;
        if (!inflateBlock(reader, out, expected, final_block)) return false;
    }
    if (out.size() != expected) return false;
    if (reader.bits >= 8) return false;
    if ((std::size_t)(reader.end - reader.p) != 4) return false;
    return readBe32(reader.p) == adler32(out.data(), out.size());
}

int paeth(int a, int b, int c) {
    int p = a + b - c;
    int pa = p > a ? p - a : a - p;
    int pb = p > b ? p - b : b - p;
    int pc = p > c ? p - c : c - p;
    if (pa <= pb && pa <= pc) return a;
    if (pb <= pc) return b;
    return c;
}

int channelsFor(int color) {
    if (color == 0) return 1;
    if (color == 2) return 3;
    if (color == 3) return 1;
    if (color == 4) return 2;
    if (color == 6) return 4;
    return 0;
}

bool depthOk(int color, int depth) {
    if (color == 0) return depth == 1 || depth == 2 || depth == 4 || depth == 8 || depth == 16;
    if (color == 2 || color == 4 || color == 6) return depth == 8 || depth == 16;
    if (color == 3) return depth == 1 || depth == 2 || depth == 4 || depth == 8;
    return false;
}

int sampleAt(const std::uint8_t* row, int depth, int channels, int x, int c) {
    if (depth == 16) {
        std::size_t i = (std::size_t)(x * channels + c) * 2u;
        return ((int)row[i] << 8) | (int)row[i + 1];
    }
    if (depth == 8) return (int)row[x * channels + c];
    int ppb = 8 / depth;
    int byte = (int)row[x / ppb];
    int shift = (ppb - 1 - (x % ppb)) * depth;
    return (byte >> shift) & ((1 << depth) - 1);
}

int to8(int sample, int depth) {
    if (depth >= 8) return sample >> (depth - 8);
    int maxv = (1 << depth) - 1;
    return (sample * 255) / maxv;
}

bool unfilter(const std::uint8_t* src, std::size_t src_n, int width, int height,
              int depth, int channels, std::vector<std::uint8_t>& raw) {
    int bpp = (depth * channels + 7) / 8;
    if (bpp < 1) return false;
    std::size_t rowbytes = ((std::size_t)width * (std::size_t)(depth * channels) + 7u) / 8u;
    if (src_n != (rowbytes + 1u) * (std::size_t)height) return false;
    raw.assign((std::size_t)height * rowbytes, 0);
    const std::uint8_t* prev = nullptr;
    for (int y = 0; y < height; ++y) {
        int filter = (int)(*src++);
        if (filter > 4) return false;
        std::uint8_t* row = raw.data() + (std::size_t)y * rowbytes;
        for (std::size_t x = 0; x < rowbytes; ++x) {
            int value = (int)(*src++);
            int a = x >= (std::size_t)bpp ? (int)row[x - (std::size_t)bpp] : 0;
            int b = prev != nullptr ? (int)prev[x] : 0;
            int c = (prev != nullptr && x >= (std::size_t)bpp) ? (int)prev[x - (std::size_t)bpp] : 0;
            if (filter == 1) value += a;
            else if (filter == 2) value += b;
            else if (filter == 3) value += (a + b) / 2;
            else if (filter == 4) value += paeth(a, b, c);
            row[x] = (std::uint8_t)(value & 0xff);
        }
        prev = row;
    }
    return true;
}

std::vector<std::uint8_t> readFile(const char* path) {
    FILE* file = openRobotFile(path, "rb");
    if (file == nullptr) return {};
    std::vector<std::uint8_t> data;
    std::uint8_t buf[4096];
    while (true) {
        std::size_t n = std::fread(buf, 1, sizeof(buf), file);
        if (n == 0) break;
        if (data.size() + n > kMaxFile) {
            std::fclose(file);
            return {};
        }
        data.insert(data.end(), buf, buf + n);
    }
    std::fclose(file);
    return data;
}

}  // namespace

bool loadPngRgba(const char* path, std::vector<std::uint8_t>& rgba, int& width, int& height) {
    rgba.clear();
    width = 0;
    height = 0;
    if (path == nullptr || path[0] == '\0') return false;

    std::vector<std::uint8_t> file = readFile(path);
    static const std::uint8_t kSig[8] = {137, 80, 78, 71, 13, 10, 26, 10};
    if (file.size() < 8 || std::memcmp(file.data(), kSig, 8) != 0) return false;

    int color = -1;
    int depth = 0;
    int interlace = 0;
    bool saw_ihdr = false;
    std::vector<std::uint8_t> idat;
    std::vector<std::uint8_t> plte;
    std::vector<std::uint8_t> trns;

    std::size_t off = 8;
    while (off + 12 <= file.size()) {
        std::uint32_t len = readBe32(file.data() + off);
        if ((std::size_t)len > file.size() - off - 12) return false;
        const std::uint8_t* type = file.data() + off + 4;
        const std::uint8_t* payload = file.data() + off + 8;
        off += 12u + (std::size_t)len;

        bool ihdr = std::memcmp(type, "IHDR", 4) == 0;
        bool plte_chunk = std::memcmp(type, "PLTE", 4) == 0;
        bool idat_chunk = std::memcmp(type, "IDAT", 4) == 0;
        bool iend = std::memcmp(type, "IEND", 4) == 0;
        bool trns_chunk = std::memcmp(type, "tRNS", 4) == 0;
        bool critical = (type[0] & 0x20) == 0;
        if (critical && !ihdr && !plte_chunk && !idat_chunk && !iend) return false;

        if (ihdr) {
            if (saw_ihdr || len != 13) return false;
            std::uint32_t w = readBe32(payload);
            std::uint32_t h = readBe32(payload + 4);
            if (w == 0 || h == 0 || w > (std::uint32_t)kMaxEdge || h > (std::uint32_t)kMaxEdge) return false;
            width = (int)w;
            height = (int)h;
            depth = (int)payload[8];
            color = (int)payload[9];
            if (payload[10] != 0 || payload[11] != 0) return false;
            interlace = (int)payload[12];
            saw_ihdr = true;
        } else if (plte_chunk) {
            if ((len % 3u) != 0u || len > 256u * 3u) return false;
            plte.assign(payload, payload + len);
        } else if (trns_chunk) {
            if (len > 256u) return false;
            trns.assign(payload, payload + len);
        } else if (idat_chunk) {
            if (idat.size() + (std::size_t)len > kMaxFile) return false;
            idat.insert(idat.end(), payload, payload + len);
        } else if (iend) {
            break;
        }
    }
    if (!saw_ihdr || idat.empty() || interlace != 0) return false;
    int channels = channelsFor(color);
    if (channels == 0 || !depthOk(color, depth)) return false;
    if (color == 3 && plte.empty()) return false;

    std::size_t rowbytes = ((std::size_t)width * (std::size_t)(depth * channels) + 7u) / 8u;
    std::size_t expected = (rowbytes + 1u) * (std::size_t)height;
    std::vector<std::uint8_t> inflated;
    if (!inflateZlib(idat.data(), idat.size(), inflated, expected)) return false;

    std::vector<std::uint8_t> raw;
    if (!unfilter(inflated.data(), inflated.size(), width, height, depth, channels, raw)) return false;

    rgba.assign((std::size_t)width * (std::size_t)height * 4u, 255);
    int palette_n = (int)(plte.size() / 3u);
    for (int y = 0; y < height; ++y) {
        const std::uint8_t* row = raw.data() + (std::size_t)y * rowbytes;
        for (int x = 0; x < width; ++x) {
            std::uint8_t* px = rgba.data() + ((std::size_t)y * (std::size_t)width + (std::size_t)x) * 4u;
            if (color == 3) {
                int index = sampleAt(row, depth, 1, x, 0);
                if (index < 0 || index >= palette_n) return false;
                px[0] = plte[(std::size_t)index * 3u];
                px[1] = plte[(std::size_t)index * 3u + 1u];
                px[2] = plte[(std::size_t)index * 3u + 2u];
                px[3] = (index < (int)trns.size()) ? trns[(std::size_t)index] : 255;
            } else if (color == 0) {
                int sample = sampleAt(row, depth, 1, x, 0);
                std::uint8_t v = (std::uint8_t)to8(sample, depth);
                px[0] = px[1] = px[2] = v;
                int key = (trns.size() >= 2) ? (((int)trns[0] << 8) | (int)trns[1]) : -1;
                px[3] = (sample == key) ? 0 : 255;
            } else if (color == 2) {
                int r = sampleAt(row, depth, 3, x, 0);
                int g = sampleAt(row, depth, 3, x, 1);
                int b = sampleAt(row, depth, 3, x, 2);
                px[0] = (std::uint8_t)to8(r, depth);
                px[1] = (std::uint8_t)to8(g, depth);
                px[2] = (std::uint8_t)to8(b, depth);
                bool keyed = false;
                if (trns.size() >= 6) {
                    int kr = ((int)trns[0] << 8) | (int)trns[1];
                    int kg = ((int)trns[2] << 8) | (int)trns[3];
                    int kb = ((int)trns[4] << 8) | (int)trns[5];
                    keyed = (r == kr && g == kg && b == kb);
                }
                px[3] = keyed ? 0 : 255;
            } else if (color == 4) {
                int sample = sampleAt(row, depth, 2, x, 0);
                int alpha = sampleAt(row, depth, 2, x, 1);
                std::uint8_t v = (std::uint8_t)to8(sample, depth);
                px[0] = px[1] = px[2] = v;
                px[3] = (std::uint8_t)to8(alpha, depth);
            } else {
                int r = sampleAt(row, depth, 4, x, 0);
                int g = sampleAt(row, depth, 4, x, 1);
                int b = sampleAt(row, depth, 4, x, 2);
                int a = sampleAt(row, depth, 4, x, 3);
                px[0] = (std::uint8_t)to8(r, depth);
                px[1] = (std::uint8_t)to8(g, depth);
                px[2] = (std::uint8_t)to8(b, depth);
                px[3] = (std::uint8_t)to8(a, depth);
            }
        }
    }
    return true;
}
