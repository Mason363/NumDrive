import os
"""Build the NumDrive data pack from the original Drive Mad game file.

Usage: python3 pack.py [--fast] [out.c]
"""
import struct
import sys
import zlib

import ops
from deflate import raw_deflate, zop
from fancade import Game, LEVEL, PHYSICS, SCRIPT

# levels left out: parts turning out of the 2D plane (115, 144, 158, 173), too heavy to draw (128), or no
# win found by the host win search
SKIP = {115, 144, 158, 173, 128,
        16, 19, 25, 35, 42, 45, 48, 53, 56, 58, 59, 64, 66, 67, 69, 81, 85, 93, 97, 103, 104, 112, 122, 127,
        132, 137, 141, 146, 150, 154, 155, 169, 172, 180, 183, 190, 196, 198}
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
        self.children = {}      # program record -> child program records
        self.node_tables = {}   # program record -> (header length, node offsets, slots, slot count)

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
            # bit 3: stock block (its position counts its whole cell)
            flags = min(s.collider, 2) | ((1 if s.type == PHYSICS else 0) << 2) | ((1 if id < self.g.id_offset else 0) << 3)
            w.u8(flags)
            w.u8(info.ncomp)
            # per component voxel bounds (voxel units)
            st = [[8, 8, 8, 0, 0, 0] for _ in range(info.ncomp)]
            for i in range(512):
                if not info.solid[i]:
                    continue
                x, y, z = i & 7, (i >> 3) & 7, i >> 6
                c = max(info.comp[i], 0) & 15 if info.ncomp > 1 else 0
                b = st[c]
                b[0] = min(b[0], x); b[1] = min(b[1], y); b[2] = min(b[2], z)
                b[3] = max(b[3], x); b[4] = max(b[4], y); b[5] = max(b[5], z)
            for bb in st:
                w.b += bytes(bb)
            boxes = info.boxes()
            assert len(boxes) < 256
            w.u8(len(boxes))
            for (c, x0, y0, z0, x1, y1, z1) in boxes:
                v = c | x0 << 3 | y0 << 6 | z0 << 9 | (x1 - 1) << 12 | (y1 - 1) << 15 | (z1 - 1) << 18
                w.b += struct.pack('<I', v)[:3]
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

    @staticmethod
    def varint(w, v):
        while v >= 0x80:
            w.u8((v & 0x7F) | 0x80)
            v >>= 7
        w.u8(v)

    def write_objects(self, w, prog):
        """Objects of a grid, already grouped (same glue rules and order as the original).
        u16 count, varint part count per object, then all part blocks (u8), then all part key deltas
        (varint, key = x<<20 | y<<13 | z<<3 | component, parts of an object sorted by key)."""
        w.u16(len(prog.objects))
        blocks, deltas = W(), W()
        for o in prog.objects:
            parts = sorted((c[0] << 20 | c[1] << 13 | c[2] << 3 | k, self.blk(id)) for (c, id, k) in o.parts)
            self.varint(w, len(parts))
            prev = 0
            for key, b in parts:
                blocks.u8(b)
                self.varint(deltas, key - prev)
                prev = key
        w.b += blocks.b + deltas.b

    def custom_objects(self, body, prog, n):
        """object indices (in the parent grid) of a custom block's anchor and self references"""
        child = self.comp.program(n.custom_prog)
        bx, by, bz = n.pos
        a = self.comp.object_at(prog, (bx, by, bz), (4, 4, 4))
        body.u16(0xFFFF if a is None else a)
        body.u8(len(child.selfmap))
        for v, i in sorted(child.selfmap.items(), key=lambda kv: kv[1]):
            cell = (bx + v[0] // 8, by + v[1] // 8, bz + v[2] // 8)
            o = self.comp.object_at(prog, cell, (v[0] % 8, v[1] % 8, v[2] % 8))
            body.u16(0xFFFF if o is None else o)

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
        self.children[idx] = [self.prog_record[n.custom_prog] for n in prog.nodes if n.op == ops.OP_CUSTOM]
        w = W()
        selfmap = {}
        body = W()
        offs, slots, nslots = [], [], 0
        for n in prog.nodes:
            offs.append(len(body.b))
            if n.op != ops.OP_CUSTOM:
                o = ops.OPS[ops.SUPPORTED[n.op]]
                if o['active'] and o['outs']:
                    slots.append(nslots)
                    nslots += len(o['outs'])
                else:
                    slots.append(0xFFFF)
            else:
                slots.append(0xFFFF)
            body.u8(n.op)
            if n.op == ops.OP_CUSTOM:
                body.u16(self.prog_record[n.custom_prog])
                body.u16(n.pos[0]); body.u16(n.pos[1]); body.u16(n.pos[2])
                body.u8(len(n.ins))
                for r in n.ins:
                    body.u16(self.ref(r, selfmap))
                self.custom_objects(body, prog, n)
                continue
            for r in n.ins:
                body.u16(self.ref(r, selfmap))
            # several wires from one output run in block order (node indices follow it)
            for tg in n.execs:
                body.u8(len(tg))
                for t in sorted(tg):
                    body.u16(t)
            body.u8(len(n.after))
            for t in sorted(n.after):
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
        # bits 1-4: height of the script block's model centre in 1/16 (its Get Position without object)
        # (custom blocks with scripts use the standard script block model: 3/16)
        yc = 3
        if prog.pf.type == SCRIPT and prog.pf.vox is not None:
            ys = [y for z in range(8) for y in range(8) for x in range(8) if prog.pf.solid(x, y, z)]
            if ys:
                yc = min(ys) + max(ys) + 1
        w.u8((1 if is_level else 0) | yc << 1)
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
            self.write_objects(w, prog)
        else:
            w.u16(0)
        self.node_tables[idx] = (len(w.b), offs, slots, nslots)
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
        # capacities measured by the host harness (objects, physics bodies, joints)
        oc, bc, jc = self.caps.get(L, (600, 64, 64))
        w.u16(oc); w.u16(bc); w.u16(jc)
        self.write_objects(w, prog)
        return bytes(w.b)

    def reachable(self, rec):
        """program records reachable from a program record (itself and custom block children)"""
        out, todo = set(), [rec]
        while todo:
            r = todo.pop()
            if r in out:
                continue
            out.add(r)
            todo += self.children.get(r, [])
        return out

    def load_caps(self):
        """caps.txt: 'level obj bodies joints' peak counts from host/caps.sh; add safety margins"""
        self.caps = {}
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'caps.txt')
        if not os.path.exists(path):
            return
        for line in open(path):
            f = line.split()
            if len(f) != 4:
                continue
            L, o, b, j = map(int, f)
            self.caps[L] = (o + max(16, o // 8), b + max(4, b // 8), j + max(4, j // 8))

    def build(self):
        self.load_caps()
        # record 0: directory (filled last)
        self.records = [None]
        levels = []
        self.uses = {}
        self.kept = [L for L in range(200) if L not in SKIP]
        for L in self.kept:
            before = set(self.prog_record.values())
            rec = self.level_record(L)
            levels.append(len(self.records))
            self.records.append(rec)
            lv = self.g.prefabs[L]
            for r in self.reachable(self.prog_record[lv.id]):
                self.uses[r] = self.uses.get(r, 0) + 1
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
        for L, r in zip(self.kept, levels):
            d.u16(r)
            d.s(self.g.prefabs[L].name)
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

    def with_tables(self, i):
        """a program record read in place gets its node offset and output slot tables (bit 7 of byte 0)"""
        r = self.records[i]
        hl, offs, slots, nslots = self.node_tables[i]
        t = W()
        t.u16(nslots)
        if (hl + 2) & 1:
            t.u8(0)
        for o in offs:
            t.u16(o)
        for sl in slots:
            t.u16(sl)
        return bytes([r[0] | 0x80]) + r[1:hl] + bytes(t.b) + r[hl:]

    def serialize(self):
        # records read in place from flash (no RAM copy): the directory and programs used by most levels
        self.raw = {0} | {r for r, n in self.uses.items() if n >= 100}
        for i in self.raw:
            if i in self.node_tables:
                self.records[i] = self.with_tables(i)
        comp = []
        for i, r in enumerate(self.records):
            if i in self.raw:
                comp.append(r)
                continue
            c = raw_deflate(r) if self.fast else zop(r)
            comp.append(c)
        out = W()
        out.b += MAGIC
        out.u16(len(self.records))
        hdr = 4 + 2 + 4 * (len(self.records) + 1) + 4 * len(self.records)
        # every record starts 4-byte aligned (tables in records read in place are accessed directly)
        comp = [c + bytes(-len(c) & 3) for c in comp]
        start = (hdr + 3) & ~3
        off = start
        for c in comp:
            out.u32(off)
            off += len(c)
        out.u32(off)
        for i, r in enumerate(self.records):
            out.u32(len(r) | (0x80000000 if i in self.raw else 0))
        out.b += bytes(start - hdr)
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
