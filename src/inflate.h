#ifndef INFLATE_H
#define INFLATE_H
#include <stdint.h>

/* Decode a raw DEFLATE stream. Returns the number of bytes written, or -1 on error. */
int inflate_raw(const uint8_t *src, uint32_t srclen, uint8_t *dst, uint32_t dstlen);

#endif
