/* Minimal raw DEFLATE (RFC 1951) decoder, one-shot into a caller buffer. */
#include "inflate.h"

typedef struct {
  const uint8_t *src, *end;
  uint32_t bits;
  int nbits;
  uint8_t *out, *out_end, *out_start;
} St;

typedef struct {
  uint16_t count[16];
  uint16_t sym[288];
} Huff;

static int getbit(St *s) {
  if (!s->nbits) {
    s->bits = s->src < s->end ? *s->src++ : 0;
    s->nbits = 8;
  }
  int b = s->bits & 1;
  s->bits >>= 1;
  s->nbits--;
  return b;
}

static uint32_t getbits(St *s, int n) {
  uint32_t v = 0;
  for (int i = 0; i < n; i++) v |= (uint32_t)getbit(s) << i;
  return v;
}

static void build(Huff *h, const uint8_t *len, int n) {
  uint16_t offs[16];
  for (int i = 0; i < 16; i++) h->count[i] = 0;
  for (int i = 0; i < n; i++) h->count[len[i]]++;
  h->count[0] = 0;
  offs[1] = 0;
  for (int i = 1; i < 15; i++) offs[i + 1] = offs[i] + h->count[i];
  for (int i = 0; i < n; i++)
    if (len[i]) h->sym[offs[len[i]]++] = i;
}

static int decode(St *s, const Huff *h) {
  int code = 0, first = 0, index = 0;
  for (int l = 1; l < 16; l++) {
    code |= getbit(s);
    int c = h->count[l];
    if (code - c < first) return h->sym[index + (code - first)];
    index += c;
    first += c;
    first <<= 1;
    code <<= 1;
  }
  return -1;
}

static const uint16_t lbase[29] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31,
                                   35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258};
static const uint8_t lext[29] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2,
                                 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
static const uint16_t dbase[30] = {1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129,
                                   193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097,
                                   6145, 8193, 12289, 16385, 24577};
static const uint8_t dext[30] = {0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6,
                                 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13};

static int block(St *s, const Huff *lh, const Huff *dh) {
  for (;;) {
    int sym = decode(s, lh);
    if (sym < 0) return -1;
    if (sym < 256) {
      if (s->out >= s->out_end) return -1;
      *s->out++ = sym;
    } else if (sym == 256) {
      return 0;
    } else {
      sym -= 257;
      if (sym >= 29) return -1;
      int len = lbase[sym] + getbits(s, lext[sym]);
      int ds = decode(s, dh);
      if (ds < 0 || ds >= 30) return -1;
      int dist = dbase[ds] + getbits(s, dext[ds]);
      if (dist > s->out - s->out_start || s->out + len > s->out_end) return -1;
      uint8_t *p = s->out - dist;
      while (len--) *s->out++ = *p++;
    }
  }
}

int inflate_raw(const uint8_t *src, uint32_t srclen, uint8_t *dst, uint32_t dstlen) {
  St s;
  Huff lh, dh;
  uint8_t lens[320];
  s.src = src;
  s.end = src + srclen;
  s.nbits = 0;
  s.bits = 0;
  s.out = s.out_start = dst;
  s.out_end = dst + dstlen;
  int last;
  do {
    last = getbit(&s);
    int type = getbits(&s, 2);
    if (type == 0) {
      s.nbits = 0;
      if (s.src + 4 > s.end) return -1;
      uint16_t len = s.src[0] | s.src[1] << 8;
      s.src += 4;
      if (s.src + len > s.end || s.out + len > s.out_end) return -1;
      while (len--) *s.out++ = *s.src++;
    } else if (type == 1) {
      int i = 0;
      for (; i < 144; i++) lens[i] = 8;
      for (; i < 256; i++) lens[i] = 9;
      for (; i < 280; i++) lens[i] = 7;
      for (; i < 288; i++) lens[i] = 8;
      build(&lh, lens, 288);
      for (i = 0; i < 30; i++) lens[i] = 5;
      build(&dh, lens, 30);
      if (block(&s, &lh, &dh)) return -1;
    } else if (type == 2) {
      static const uint8_t ord[19] = {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};
      int nlen = getbits(&s, 5) + 257, ndist = getbits(&s, 5) + 1, ncode = getbits(&s, 4) + 4;
      uint8_t cl[19] = {0};
      for (int i = 0; i < ncode; i++) cl[ord[i]] = getbits(&s, 3);
      build(&lh, cl, 19);
      int i = 0;
      while (i < nlen + ndist) {
        int sym = decode(&s, &lh);
        if (sym < 0) return -1;
        if (sym < 16) {
          lens[i++] = sym;
        } else {
          int rep, val = 0;
          if (sym == 16) {
            if (!i) return -1;
            val = lens[i - 1];
            rep = 3 + getbits(&s, 2);
          } else if (sym == 17) {
            rep = 3 + getbits(&s, 3);
          } else {
            rep = 11 + getbits(&s, 7);
          }
          if (i + rep > nlen + ndist) return -1;
          while (rep--) lens[i++] = val;
        }
      }
      build(&lh, lens, nlen);
      build(&dh, lens + nlen, ndist);
      if (block(&s, &lh, &dh)) return -1;
    } else {
      return -1;
    }
  } while (!last);
  return (int)(s.out - s.out_start);
}
