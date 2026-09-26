"""Build the NumDrive data pack from the original Drive Mad game file.

Usage: python3 pack.py [--fast] [out.c]
"""
import struct
import sys
import zlib

import ops
from deflate import raw_deflate, zop
from fancade import Game, LEVEL, PHYSICS, SCRIPT
from model import BlockLib, build_objects, grid_cells, DIRS, vidx
from scriptc import Compiler, custom_terminals

MAGIC = b'NDMP'
REF_NONE = 0xFFFF


class W:
    def __init__(self):
        self.b = bytearray()

    def u8(self, v):
        self.b += struct.pack('<B', v)

    def u16(self, v):
        self.b += struct.pack('<H', v)

    def i16(self, v):
        self.b += struct.pack('<h', v)

    def u32(self, v):
        self.b += struct.pack('<I', v)

    def f32(self, v):
        self.b += struct.pack('<f', v)

    def s(self, text):
        e = text.encode('utf-8')
        self.u8(len(e))
        self.b += e


class Packer:
    def __init__(self, fast=False):
        self.fast = fast
        self.g = Game()
        self.lib = BlockLib(self.g)
        self.comp = Compiler(self.g, self.lib, build_objects)
        self.block_index = {}   # segment id -> library index
        self.records = []       # raw record bytes
        self.prog_record = {}   # prefab id -> record index

    # ---------------------------------------------------------------- blocks
    def blk(self, id):
        if id not in self.block_index:
            self.block_index[id] = len(self.block_index)
            assert len(self.block_index) <= 255
        return self.block_index[id]

    GROUP = 16

    def block_library(self):
        ids = sorted(self.block_index, key=lambda i: self.block_index[i])
        groups = []
        for g0 in range(0, len(ids), self.GROUP):
            w = W()
            for id in ids[g0:g0 + self.GROUP]:
                self.write_block(w, id)
            groups.append(bytes(w.b))
        return groups

    def write_block(self, w, id):
        if True:
            info = self.lib.get(id)
            s = self.g.seg(id)
            flags = min(s.collider, 2) | ((1 if s.type == PHYSICS else 0) << 2)
            w.u8(flags)
            w.u8(info.ncomp)
            bits = bytearray(64)
            for i in range(512):
                if info.solid[i]:
                    bits[i >> 3] |= 1 << (i & 7)
            w.b += bits
            if info.ncomp > 1:
                nib = bytearray(256)
                for i in range(512):
                    c = max(info.comp[i], 0)
                    nib[i >> 1] |= (c & 15) << ((i & 1) * 4)
                w.b += nib
            boxes = info.boxes()
            assert len(boxes) < 256
            w.u8(len(boxes))
            for (c, x0, y0, z0, x1, y1, z1) in boxes:
                v = c | x0 << 3 | y0 << 6 | z0 << 9 | (x1 - 1) << 12 | (y1 - 1) << 15 | (z1 - 1) << 18
                w.b += struct.pack('<I', v)[:3]
            # glue masks: per component, per face, 8x8 bits of solid sticky boundary voxels
            for c in range(info.ncomp):
                for f in range(6):
                    d = DIRS[f]
                    axis = [k for k in range(3) if d[k]][0]
                    layer = 7 if d[axis] > 0 else 0
                    ua, va = [k for k in range(3) if k != axis]
                    m = 0
                    for w_ in range(8):
                        for u in range(8):
                            p_ = [0, 0, 0]
                            p_[axis] = layer; p_[ua] = u; p_[va] = w_
                            i = vidx(*p_)
                            if info.solid[i] and info.comp[i] == c and info.sticky(p_[0], p_[1], p_[2], f):
                                m |= 1 << (u + w_ * 8)
                    w.b += struct.pack('<Q', m)
            full = 0
            for f in range(6):
                if info.full[f]:
                    full |= 1 << f
            w.u8(full)
            for f in range(6):
                q = info.quads(f)
                assert len(q) < 256
                w.u8(len(q))
                for (layer, u, v_, du, dv, col, c, boundary) in q:
                    x = layer | u << 3 | v_ << 6 | (du - 1) << 9 | (dv - 1) << 12 | col << 15 | c << 21
                    w.b += struct.pack('<I', x)[:3]

    # -------------------------------------------------------------- programs
    def write_grid(self, w, pf):
        """Sparse grid of object cells (non-script voxel blocks), RLE in z, y, x order.
        The runtime rebuilds objects with the same glue rules and ordering as model.build_objects."""
        cells = grid_cells(self.g, pf)
        if not cells:
            w.u16(0xFFFF)
            return
        xs = [c[0] for c in cells]; ys = [c[1] for c in cells]; zs = [c[2] for c in cells]
        x0, x1, y0, y1, z0, z1 = min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)
        w.u16(x0); w.u16(y0); w.u16(z0)
        w.u16(x1 - x0 + 1); w.u16(y1 - y0 + 1); w.u16(z1 - z0 + 1)
        seq = []
        for z in range(z0, z1 + 1):
            for y in range(y0, y1 + 1):
                for x in range(x0, x1 + 1):
                    id = cells.get((x, y, z))
                    seq.append(0 if id is None else self.blk(id) + 1)
        i = 0
        n = len(seq)
        while i < n:
            j = i
            while j < n and seq[j] == 0 and j - i < 255:
                j += 1
            w.u8(j - i)
            i = j
            k = i
            while k < n and seq[k] != 0 and k - i < 255:
                k += 1
            w.u8(k - i)
            for t in range(i, k):
                w.u8(seq[t] - 1)
            i = k

    def ref(self, r, selfmap):
        if r is None:
            return REF_NONE
        kind = r[0]
        if kind == 'node':
            assert r[1] < 2048 and r[2] < 4
            return r[1] | r[2] << 11
        if kind == 'in':
            return 1 << 13 | r[1]
        if kind == 'self':
            if r[1] not in selfmap:
                selfmap[r[1]] = len(selfmap)
            return 2 << 13 | selfmap[r[1]]
        if kind == 'obj':
            if r[1] is None:
                return REF_NONE
            return 3 << 13 | r[1]
        raise ValueError(r)

    def program_record(self, pid):
        if pid in self.prog_record:
            return self.prog_record[pid]
        prog = self.comp.program(pid)
        # make sure children get records first
        for n in prog.nodes:
            if n.op == ops.OP_CUSTOM:
                self.program_record(n.custom_prog)
        idx = len(self.records)
        self.records.append(None)
        self.prog_record[pid] = idx
        w = W()
        selfmap = {}
        body = W()
        for n in prog.nodes:
            body.u8(n.op)
            if n.op == ops.OP_CUSTOM:
                body.u16(self.prog_record[n.custom_prog])
                body.u16(n.pos[0]); body.u16(n.pos[1]); body.u16(n.pos[2])
                body.u8(len(n.ins))
                for r in n.ins:
                    body.u16(self.ref(r, selfmap))
                continue
            for r in n.ins:
                body.u16(self.ref(r, selfmap))
            for tg in n.execs:
                body.u8(len(tg))
                for t in tg:
                    body.u16(t)
            body.u8(len(n.after))
            for t in n.after:
                body.u16(t)
            d = n.data
            if d:
                if d[0] == 'num':
                    body.f32(d[1])
                elif d[0] in ('vec', 'rot'):
                    for x in d[1]:
                        body.f32(x)
                elif d[0] == 'var':
                    body.u16(d[2] | (0x8000 if d[1] else 0))
                elif d[0] == 'byte':
                    body.u8(d[1])
            elif n.block_id in (36,):
                body.f32(0.0)
            elif n.block_id in (38, 42):
                body.f32(0.0); body.f32(0.0); body.f32(0.0)
            elif n.block_id in (46, 48, 50, 52, 54, 56, 428, 430, 432, 434, 436, 438):
                raise ValueError('variable without data')
            elif n.block_id in (252, 256, 588, 268, 592, 264):
                body.u8(0)
        outs = W()
        omap = dict(prog.outputs)
        for k in range(len(prog.outputs_decl)):
            outs.u16(self.ref(omap.get(k), selfmap))
        is_level = prog.pf.type == LEVEL
        w.u8(1 if is_level else 0)
        w.u16(len(prog.nodes))
        w.u16(len(prog.entries))
        for e in prog.entries:
            w.u16(e)
        locs = sorted(prog.locals.items(), key=lambda kv: kv[1])
        w.u16(len(locs))
        for (name, t), i in locs:
            w.u8(t)
        w.u8(len(prog.inputs))
        w.u8(len(prog.outputs_decl))
        w.b += outs.b
        w.u8(len(selfmap))
        for v, i in sorted(selfmap.items(), key=lambda kv: kv[1]):
            w.u8(v[0]); w.u8(v[1]); w.u8(v[2])
        if not is_level:
            self.write_grid(w, prog.pf)
        else:
            w.u16(0xFFFF)
        w.b += body.b
        prog.selfmap = selfmap
        self.records[idx] = bytes(w.b)
        return idx

    # ----------------------------------------------------------------- levels
    def level_record(self, L):
        g = self.g
        lv = g.prefabs[L]
        prog = self.comp.program(lv.id)
        self.program_record(lv.id)
        w = W()
        w.s(lv.name)
        w.u16(lv.size[0]); w.u16(lv.size[1]); w.u16(lv.size[2])
        w.u8(lv.bg)
        w.u16(self.prog_record[lv.id])
        self.write_grid(w, lv)
        w.u16(len(prog.objects))
        nlevel = len(prog.objects)
        return bytes(w.b)

    def build(self):
        # record 0: directory (filled last)
        self.records = [None]
        levels = []
        for L in range(200):
            rec = self.level_record(L)
            levels.append(len(self.records))
            self.records.append(rec)
        groups = self.block_library()
        first_block = len(self.records)
        self.records += groups
        d = W()
        d.u16(len(self.block_index))
        d.u8(self.GROUP)
        d.u16(first_block)
        d.u16(len(self.comp.globals))
        for (name, t), i in sorted(self.comp.globals.items(), key=lambda kv: kv[1]):
            d.u8(t)
        d.u16(len(levels))
        for i, r in enumerate(levels):
            d.u16(r)
            d.s(self.g.prefabs[i].name)
        # global variable names needed by the engine (index lookup)
        wanted = ['$Win', '$Lose', '$You', '$Cam', '$Offset']
        d.u8(len(wanted))
        for name in wanted:
            idx = [i for (n, t), i in self.comp.globals.items() if n == name]
            d.s(name)
            d.u8(len(idx))
            for i in idx:
                d.u16(i)
        self.records[0] = bytes(d.b)
        return self.records

    def serialize(self):
        comp = []
        for r in self.records:
            c = raw_deflate(r) if self.fast else zop(r)
            comp.append(c)
        out = W()
        out.b += MAGIC
        out.u16(len(self.records))
        hdr = 4 + 2 + 4 * (len(self.records) + 1) + 4 * len(self.records)
        off = hdr
        for c in comp:
            out.u32(off)
            off += len(c)
        out.u32(off)
        for r in self.records:
            out.u32(len(r))
        for c in comp:
            out.b += c
        return bytes(out.b)


def main():
    fast = '--fast' in sys.argv
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    out = args[0] if args else '../src/pack.c'
    p = Packer(fast)
    p.build()
    data = p.serialize()
    raw = sum(len(r) for r in p.records)
    print('records', len(p.records), 'raw', raw, 'packed', len(data), 'blocks', len(p.block_index))
    with open(out.replace('.c', '.bin'), 'wb') as f:
        f.write(data)
    with open(out, 'w') as f:
        f.write('/* Generated by tools/pack.py from the original Drive Mad game data. */\n')
        f.write('#include <stdint.h>\n')
        f.write('const uint32_t pack_size = %d;\n' % len(data))
        f.write('const uint8_t pack_data[%d] __attribute__((aligned(4))) = {\n' % len(data))
        for i in range(0, len(data), 24):
            f.write(','.join(str(b) for b in data[i:i + 24]) + ',\n')
        f.write('};\n')
    with open(out.replace('pack.c', 'ops.h'), 'w') as f:
        f.write(ops.c_header())


if __name__ == '__main__':
    main()
