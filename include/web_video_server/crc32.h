#ifndef CRC32_H_
#define CRC32_H_

#include <stdint.h>

namespace web_video_server
{
uint32_t updateCrc32(uint32_t crc, const uint8_t* buf, uint32_t len);
uint32_t crc32(const uint8_t* buf, uint32_t len);
}

#endif
