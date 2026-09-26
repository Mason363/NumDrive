"""Reader for Fancade game files (format version 31) and the stock prefab list."""
import os
import struct
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
SOURCE = os.path.join(HERE, 'source')
GAME_FILE = os.environ.get('FANCADE_GAME', os.path.join(SOURCE, '5F084A0BCE06B710'))
STOCK_FILE = os.path.join(SOURCE, 'stockPrefabs.fcpl')

# Fancade colour palette, index 0 = empty voxel
PALETTE = [(0, 0, 0), (29, 29, 40), (63, 63, 80), (101, 103, 121), (144, 147, 164), (191, 194, 205),
           (255, 255, 255), (148, 83, 94), (192, 109, 128), (225, 153, 152), (253, 181, 168),
           (255, 219, 197), (255, 243, 242), (192, 54, 80), (255, 74, 104), (255, 154, 154),
           (201, 80, 64), (255, 112, 83), (255, 168, 129), (232, 168, 0), (255, 213, 0),
           (255, 255, 122), (0, 136, 76), (66, 195, 79), (181, 255, 110), (0, 108, 218),
           (0, 142, 255), (0, 193, 255), (104, 83, 162), (140, 111, 217), (176, 138, 255),
           (235, 112, 171), (255, 149, 206), (255, 185, 232)]

NORMAL, PHYSICS, SCRIPT, LEVEL = 0, 1, 2, 3


class Reader:
    def __init__(self, b):
        self.b = b
        self.p = 0

    def u8(self):
        v = self.b[self.p]
        self.p += 1
        return v

    def u16(self):
        v = struct.unpack_from('<H', self.b, self.p)[0]
        self.p += 2
        return v

    def u32(self):
        v = struct.unpack_from('<I', self.b, self.p)[0]
        self.p += 4
        return v

    def i32(self):
        v = struct.unpack_from('<i', self.b, self.p)[0]
        self.p += 4
        return v

    def f32(self):
        v = struct.unpack_from('<f', self.b, self.p)[0]
        self.p += 4
        return v

    def string(self):
        n = self.u8()
        v = self.b[self.p:self.p + n].decode('utf-8', 'replace')
        self.p += n
        return v

    def raw(self, n):
        v = self.b[self.p:self.p + n]
        self.p += n
        return v


class Prefab:
    """One raw prefab segment as stored in the file."""

    def voxel(self, x, y, z, f):
        return self.vox[f * 512 + x + y * 8 + z * 64]

    def solid(self, x, y, z):
        i = x + y * 8 + z * 64
        v = self.vox
        return any(v[f * 512 + i] & 0x7f for f in range(6))

    def block(self, x, y, z):
        sx, sy, sz = self.size
        return self.blocks[x + y * sx + z * sx * sy]


def read_prefab(r):
    h0 = r.u8()
    h1 = r.u8()
    p = Prefab()
    has_conn = h0 & 1
    has_set = (h0 >> 1) & 1
    has_blocks = (h0 >> 2) & 1
    has_vox = (h0 >> 3) & 1
    p.in_group = (h0 >> 4) & 1
    has_coll = (h0 >> 5) & 1
    has_bg = h1 & 1
    has_d2 = (h1 >> 1) & 1
    has_d1 = (h1 >> 2) & 1
    has_name = (h1 >> 3) & 1
    has_type = (h1 >> 4) & 1
    p.type = r.u8() if has_type else NORMAL
    p.name = r.string() if has_name else 'New Block'
    p.data1 = r.u8() if has_d1 else 0
    p.data2 = r.u32() if has_d2 else 0
    p.bg = r.u8() if has_bg else 27
    p.collider = r.u8() if has_coll else 1
    p.group = None
    p.gpos = (0, 0, 0)
    if p.in_group:
        p.group = r.u16()
        p.gpos = (r.u8(), r.u8(), r.u8())
    p.vox = r.raw(3072) if has_vox else None
    p.size = (0, 0, 0)
    p.blocks = []
    if has_blocks:
        p.size = (r.u16(), r.u16(), r.u16())
        n = p.size[0] * p.size[1] * p.size[2]
        p.blocks_off = r.p
        p.blocks = list(struct.unpack_from('<%dH' % n, r.b, r.p))
        r.p += 2 * n
    p.settings = []
    if has_set:
        for _ in range(r.u16()):
            idx = r.u8()
            t = r.u8()
            pos = (r.u16(), r.u16(), r.u16())
            if t == 1:
                v = r.u8()
            elif t == 2:
                v = r.u16()
            elif t == 3:
                v = r.i32()
            elif t == 4:
                v = r.f32()
            elif t == 5:
                v = (r.f32(), r.f32(), r.f32())
            else:
                v = r.string()
            p.settings.append((idx, t, pos, v))
    p.conns = []
    p.conns_off = None
    if has_conn:
        p.conns_off = r.p + 2
        for _ in range(r.u16()):
            a = [r.u16() for _ in range(12)]
            p.conns.append((tuple(a[0:3]), tuple(a[3:6]), tuple(a[6:9]), tuple(a[9:12])))
    return p


class Game:
    def __init__(self, game_file=GAME_FILE, stock_file=STOCK_FILE):
        d = zlib.decompress(open(game_file, 'rb').read())
        r = Reader(d)
        self.version = r.u16()
        self.title = r.string()
        self.author = r.string()
        self.description = r.string()
        self.id_offset = r.u16()
        self.prefabs = [read_prefab(r) for _ in range(r.u16())]
        assert r.p == len(d)
        s = Reader(open(stock_file, 'rb').read())
        n = s.u32()
        off = s.u16()
        assert off == 0
        self.stock = [read_prefab(s) for _ in range(n)]
        assert len(self.stock) == self.id_offset
        for i, p in enumerate(self.stock):
            p.id = i
        for i, p in enumerate(self.prefabs):
            p.id = i + self.id_offset

    def seg(self, id):
        if id < self.id_offset:
            return self.stock[id]
        return self.prefabs[id - self.id_offset]

    def levels(self):
        return [p for p in self.prefabs if p.type == LEVEL]
