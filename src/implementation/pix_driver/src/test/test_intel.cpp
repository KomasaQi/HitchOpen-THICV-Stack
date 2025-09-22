#include <iostream>
#include <iomanip>
#include <algorithm>
#include "../common.hpp"

using namespace PixDriver;

void printBuf(const uint8_t* buf, int len = 8)
{
    std::cout << "Buffer: ";
    for (int i = 0; i < len; ++i)
    {
        std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<int>(buf[i]) << " ";
    }
    std::cout << std::dec << "\n";
}

int main()
{
    uint8_t buf[8] = {0};

    std::cout << "=== Intel (Little Endian) Unsigned Test ===\n";
    {
        std::fill(std::begin(buf), std::end(buf), 0);

        // 写入一个12位无符号数 0xABC (十进制 2748)，起始bit=0
        writeUInt(buf, 0, 12, 0xABC);
        printBuf(buf);

        // 读出
        uint32_t val = readUInt(buf, 0, 12);
        std::cout << "Read value = 0x" << std::hex << val << std::dec << "\n";
    }

    std::cout << "\n=== Intel (Little Endian) Signed Test ===\n";
    {
        std::fill(std::begin(buf), std::end(buf), 0);

        // 写入 -5 到一个 6bit 信号
        writeInt(buf, 0, 6, -5);
        printBuf(buf);

        int32_t sval = readInt(buf, 0, 6);
        std::cout << "Read signed value = " << sval << "\n";
    }

    return 0;
}
