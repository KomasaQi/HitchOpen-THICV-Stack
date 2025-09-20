#ifndef CANLIB_BITBUFFER_H
#define CANLIB_BITBUFFER_H

#include <cstdint>
#include <cstddef>

namespace PixDriver
{

// ---------------- Bit工具 ----------------
constexpr inline int byteIndex(int pos) noexcept { return pos >> 3; }   // pos/8
constexpr inline int bitIndex(int pos) noexcept { return pos & 0x07; } // pos%8

inline int readBit(const uint8_t *buf, int pos)
{
    return (buf[byteIndex(pos)] >> bitIndex(pos)) & 0x01;
}

inline void setBit(uint8_t *buf, int pos)
{
    buf[byteIndex(pos)] |= (1u << bitIndex(pos));
}

inline void clearBit(uint8_t *buf, int pos)
{
    buf[byteIndex(pos)] &= ~(1u << bitIndex(pos));
}

inline void writeBit(uint8_t *buf, int pos, bool value)
{
    if (value)
        setBit(buf, pos);
    else
        clearBit(buf, pos);
}

// ---------------- 整数读写 ----------------
inline uint32_t readUInt(const uint8_t *buf, int offset, int length)
{
    uint32_t ret = 0;
    for (int i = length - 1; i >= 0; --i)
    {
        ret = (ret << 1) | readBit(buf, offset + i);
    }
    return ret;
}

inline int32_t readInt(const uint8_t *buf, int offset, int length)
{
    uint32_t raw = readUInt(buf, offset, length);
    // 符号扩展
    if (length < 32 && (raw & (1u << (length - 1))))
    {
        raw |= ~((1u << length) - 1);
    }
    return static_cast<int32_t>(raw);
}

inline void writeUInt(uint8_t *buf, int offset, int length, uint32_t value)
{
    for (int i = 0; i < length; ++i)
    {
        writeBit(buf, offset + i, (value >> i) & 1u);
    }
}

inline void writeInt(uint8_t *buf, int offset, int length, int32_t value)
{
    writeUInt(buf, offset, length, static_cast<uint32_t>(value));
}

} // namespace PixDriver

#endif // CANLIB_BITBUFFER_H
