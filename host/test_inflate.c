#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "../src/inflate.h"
extern const uint8_t pack_data[]; extern const uint32_t pack_size;
int main(void){
  uint16_t n = pack_data[4] | pack_data[5]<<8;
  const uint8_t *offs = pack_data+6, *raws = offs + 4*(n+1);
  uint8_t *buf = malloc(1<<20); int bad=0;
  for (int i=0;i<n;i++){
    uint32_t o0=*(uint32_t*)(offs+4*i), o1=*(uint32_t*)(offs+4*i+4), rs=*(uint32_t*)(raws+4*i);
    int r = inflate_raw(pack_data+o0, o1-o0, buf, rs);
    if (r != (int)rs) { printf("record %d: got %d expected %u\n", i, r, rs); bad++; }
  }
  printf("%d records, %d bad\n", n, bad);
  return bad;
}
