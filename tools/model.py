"""Voxel block analysis and object formation (Fancade rules)."""
from fancade import NORMAL, PHYSICS, SCRIPT, LEVEL

# voxel face order in Fancade files: +X -X +Y -Y +Z -Z
DIRS = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]


def vidx(x, y, z):
    return x + y * 8 + z * 64


class BlockInfo:
    """Derived data for one voxel block (segment)."""

    def __init__(self, seg):
        self.seg = seg
        v = seg.vox
        self.solid = [any(v[f * 512 + i] & 0x7f for f in range(6)) for i in range(512)]
        self._components()
        self._full_faces()

    def color(self, x, y, z, f):
        return self.seg.vox[f * 512 + vidx(x, y, z)] & 0x7f

    def sticky(self, x, y, z, f):
        return not (self.seg.vox[f * 512 + vidx(x, y, z)] & 0x80)

    def _components(self):
        parent = list(range(512))

        def find(a):
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            return a
        for z in range(8):
            for y in range(8):
                for x in range(8):
                    i = vidx(x, y, z)
                    if not self.solid[i]:
                        continue
                    for axis in range(3):
                        q = [x, y, z]
                        q[axis] += 1
                        if q[axis] > 7:
                            continue
                        j = vidx(*q)
                        if self.solid[j] and self.sticky(x, y, z, axis * 2) and self.sticky(q[0], q[1], q[2], axis * 2 + 1):
                            parent[find(i)] = find(j)
        roots = {}
        self.comp = [-1] * 512
        for i in range(512):
            if self.solid[i]:
                r = find(i)
                if r not in roots:
                    roots[r] = len(roots)
                self.comp[i] = roots[r]
        self.ncomp = len(roots)
        self.comp_count = [0] * self.ncomp
        self.comp_sum = [[0.0, 0.0, 0.0] for _ in range(self.ncomp)]
        self.comp_min = [[8, 8, 8] for _ in range(self.ncomp)]
        self.comp_max = [[-1, -1, -1] for _ in range(self.ncomp)]
        for i in range(512):
            c = self.comp[i]
            if c < 0:
                continue
            p = (i % 8, (i // 8) % 8, i // 64)
            self.comp_count[c] += 1
            for k in range(3):
                self.comp_sum[c][k] += p[k] + 0.5
                self.comp_min[c][k] = min(self.comp_min[c][k], p[k])
                self.comp_max[c][k] = max(self.comp_max[c][k], p[k])

    def _full_faces(self):
        self.full = []
        for f, d in enumerate(DIRS):
            axis = [k for k in range(3) if d[k]][0]
            layer = 7 if d[axis] > 0 else 0
            ok = True
            for u in range(8):
                for w in range(8):
                    p = [0, 0, 0]
                    p[axis] = layer
                    a, b = [k for k in range(3) if k != axis]
                    p[a] = u
                    p[b] = w
                    if not self.solid[vidx(*p)]:
                        ok = False
            self.full.append(ok)

    def boxes(self):
        """Greedy merge solid voxels of each component into boxes: list of (comp, x0,y0,z0,x1,y1,z1) (exclusive max)."""
        used = [False] * 512
        out = []
        for z in range(8):
            for y in range(8):
                for x in range(8):
                    i = vidx(x, y, z)
                    if not self.solid[i] or used[i]:
                        continue
                    c = self.comp[i]

                    def ok(xx, yy, zz):
                        j = vidx(xx, yy, zz)
                        return self.solid[j] and not used[j] and self.comp[j] == c
                    x1 = x + 1
                    while x1 < 8 and ok(x1, y, z):
                        x1 += 1
                    y1 = y + 1
                    while y1 < 8 and all(ok(xx, y1, z) for xx in range(x, x1)):
                        y1 += 1
                    z1 = z + 1
                    while z1 < 8 and all(ok(xx, yy, z1) for xx in range(x, x1) for yy in range(y, y1)):
                        z1 += 1
                    for zz in range(z, z1):
                        for yy in range(y, y1):
                            for xx in range(x, x1):
                                used[vidx(xx, yy, zz)] = True
                    out.append((c, x, y, z, x1, y1, z1))
        return out

    def quads(self, f):
        """Greedy mesh exposed faces for direction f within the block.
        Returns list of (layer, u, v, du, dv, color, comp, boundary)."""
        d = DIRS[f]
        axis = [k for k in range(3) if d[k]][0]
        ua, va = [k for k in range(3) if k != axis]
        out = []
        for layer in range(8):
            grid = {}
            for u in range(8):
                for w in range(8):
                    p = [0, 0, 0]
                    p[axis] = layer
                    p[ua] = u
                    p[va] = w
                    i = vidx(*p)
                    if not self.solid[i]:
                        continue
                    col = self.color(p[0], p[1], p[2], f)
                    if not col:
                        continue
                    q = list(p)
                    q[axis] += d[axis]
                    if 0 <= q[axis] < 8 and self.solid[vidx(*q)]:
                        continue
                    grid[(u, w)] = (col, self.comp[i])
            used = set()
            for w in range(8):
                for u in range(8):
                    if (u, w) not in grid or (u, w) in used:
                        continue
                    key = grid[(u, w)]
                    du = 1
                    while (u + du, w) in grid and grid[(u + du, w)] == key and (u + du, w) not in used:
                        du += 1
                    dv = 1
                    while all((u + k, w + dv) in grid and grid[(u + k, w + dv)] == key and (u + k, w + dv) not in used for k in range(du)):
                        dv += 1
                    for a in range(du):
                        for b in range(dv):
                            used.add((u + a, w + b))
                    boundary = (layer == 7 and d[axis] > 0) or (layer == 0 and d[axis] < 0)
                    out.append((layer, u, w, du, dv, key[0], key[1], boundary))
        return out


class BlockLib:
    def __init__(self, game):
        self.game = game
        self.info = {}

    def get(self, id):
        if id not in self.info:
            self.info[id] = BlockInfo(self.game.seg(id))
        return self.info[id]


def grid_cells(game, pf):
    """Non-script voxel cells in a prefab grid: {cell: id}."""
    sx, sy, sz = pf.size
    out = {}
    for z in range(sz):
        for y in range(sy):
            for x in range(sx):
                id = pf.blocks[x + y * sx + z * sx * sy]
                if not id:
                    continue
                s = game.seg(id)
                if s.type == SCRIPT or not s.vox:
                    continue
                out[(x, y, z)] = id
    return out


class Obj:
    def __init__(self):
        self.parts = []  # (cell, id, comp)

    def is_physics(self, game):
        return any(game.seg(id).type == PHYSICS for (_, id, _) in self.parts)


def build_objects(game, lib, pf):
    """Group voxel components of a grid into Fancade objects.
    Returns (objects, part_obj) where part_obj maps (cell, comp) -> object index."""
    cells = grid_cells(game, pf)
    nodes = []
    for c, id in cells.items():
        for k in range(lib.get(id).ncomp):
            nodes.append((c, k))
    parent = {n: n for n in nodes}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    for c, id in cells.items():
        A = lib.get(id)
        for axis in range(3):
            n = list(c)
            n[axis] += 1
            n = tuple(n)
            if n not in cells:
                continue
            B = lib.get(cells[n])
            ua, va = [k for k in range(3) if k != axis]
            for u in range(8):
                for w in range(8):
                    pa = [0, 0, 0]
                    pa[axis] = 7
                    pa[ua] = u
                    pa[va] = w
                    pb = list(pa)
                    pb[axis] = 0
                    ia = vidx(*pa)
                    ib = vidx(*pb)
                    if A.solid[ia] and B.solid[ib] and A.sticky(pa[0], pa[1], pa[2], axis * 2) and B.sticky(pb[0], pb[1], pb[2], axis * 2 + 1):
                        ra = find((c, A.comp[ia]))
                        rb = find((n, B.comp[ib]))
                        if ra != rb:
                            parent[rb] = ra
    groups = {}
    for nd in nodes:
        groups.setdefault(find(nd), []).append(nd)
    # deterministic order: by first node in (z desc, y desc, x asc) parse order
    def order_key(nd):
        (x, y, z), k = nd
        return (-z, -y, x, k)
    objs = []
    part_obj = {}
    for root, members in sorted(groups.items(), key=lambda kv: min(order_key(m) for m in kv[1])):
        o = Obj()
        for (c, k) in sorted(members, key=lambda m: (m[0][0], m[0][1], m[0][2], m[1])):
            o.parts.append((c, cells[c], k))
            part_obj[(c, k)] = len(objs)
        objs.append(o)
    return objs, part_obj
