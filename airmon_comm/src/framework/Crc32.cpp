#include "Crc32.h"

static uint32_t s_crc32Table[256];
static bool s_bTableGenerated = false;

void GenerateCrc32Table()
{
    for (int i = 0; i < 256; ++i)
    {
        uint32_t crc = i;
        for (int j = 0; j < 8; ++j)
        {
            crc = crc & 1 ? (crc >> 1) ^ 0xEDB88320UL : crc >> 1;
        }
        s_crc32Table[i] = crc;
    }
    s_bTableGenerated = true;
}

uint32_t GetCrc32(uint8_t* buf, uint32_t len)
{
    if(!s_bTableGenerated)
    {
        GenerateCrc32Table();
    }

    uint32_t crc = 0xFFFFFFFFUL;
    while (len--)
    {
        crc = s_crc32Table[(crc ^ *buf++) & 0xFF] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFUL;
}