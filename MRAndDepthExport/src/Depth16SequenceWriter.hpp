// Depth16SequenceWriter.hpp
#pragma once
#include <k4a/k4a.hpp>

#include <cstdint>
#include <fstream>
#include <string>

class Depth16SequenceWriter {
public:
    enum : uint32_t { PIXFMT_DEPTH16_MM = 1 };

    Depth16SequenceWriter() = default;
    ~Depth16SequenceWriter() { close(); }

    bool open(const std::string& path, uint32_t width, uint32_t height, uint32_t pixfmt = PIXFMT_DEPTH16_MM) {
        close();
        ofs_.open(path, std::ios::binary | std::ios::trunc);
        if (!ofs_) return false;

        width_ = width; height_ = height; pixfmt_ = pixfmt; frame_count_ = 0;

        writeBytes("D16S", 4);        // magic
        writeU32(1);                  // version
        writeU32(width_);
        writeU32(height_);
        writeU32(pixfmt_);
        frame_count_pos_ = ofs_.tellp();
        writeU64(0);                  // frame_count placeholder
        for (int i = 0; i < 24; ++i) ofs_.put(0); // reserved (24 bytes)

        return bool(ofs_);
    }

    bool append(const k4a::image& depth) {
        if (!ofs_ || !depth) return false;
        if (depth.get_format() != K4A_IMAGE_FORMAT_DEPTH16) return false;
        if (depth.get_width_pixels() != int(width_) || depth.get_height_pixels() != int(height_)) return false;

        const int rowBytes = int(width_) * int(sizeof(uint16_t));
        const int stride = depth.get_stride_bytes();
        const uint8_t* base = depth.get_buffer();

        uint64_t ts_us = 0;
        try { ts_us = static_cast<uint64_t>(depth.get_device_timestamp().count()); }
        catch (...) {}

        writeU64(ts_us);
        writeU32(uint32_t(rowBytes * height_));
        writeU32(0); // reserved

        for (uint32_t y = 0; y < height_; ++y) {
            ofs_.write(reinterpret_cast<const char*>(base + y * stride), rowBytes);
        }
        if (!ofs_) return false;
        ++frame_count_;
        return true;
    }

    void close() {
        if (ofs_) {
            const auto endpos = ofs_.tellp();
            ofs_.seekp(frame_count_pos_);
            writeU64(frame_count_);
            ofs_.seekp(endpos);
            ofs_.flush();
            ofs_.close();
        }
        frame_count_ = 0;
        width_ = height_ = pixfmt_ = 0;
        frame_count_pos_ = {};
    }

private:
    std::ofstream ofs_;
    uint32_t width_ = 0, height_ = 0, pixfmt_ = 0;
    uint64_t frame_count_ = 0;
    std::streampos frame_count_pos_{};

    void writeBytes(const void* p, size_t n) { ofs_.write(reinterpret_cast<const char*>(p), n); }
    void writeU32(uint32_t v) { for (int i = 0; i < 4; ++i) ofs_.put(uint8_t(v >> (8 * i))); }
    void writeU64(uint64_t v) { for (int i = 0; i < 8; ++i) ofs_.put(uint8_t(v >> (8 * i))); }
};
