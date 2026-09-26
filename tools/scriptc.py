"""Compile Fancade script grids into a compact node program for the VM."""
from blockdefs import DEFS
from fancade import SCRIPT, LEVEL
import ops

OUTSIDE = 32769
VAR_GET = {46: 0, 48: 1, 50: 2, 52: 3, 54: 4, 56: 5}
VAR_SET = {428: 0, 430: 1, 432: 2, 434: 3, 436: 4, 438: 5}
SIG = {'Float': 0, 'Vec3': 1, 'Rot': 2, 'Bool': 3, 'Obj': 4, 'Con': 5, 'Void': 6}
TERM_SETTING = {7: ('Void', None), 8: ('Float', True), 9: ('Float', False), 10: ('Vec3', True), 11: ('Vec3', False),
                12: ('Rot', True), 13: ('Rot', False), 14: ('Bool', True), 15: ('Bool', False),
                16: ('Obj', True), 17: ('Obj', False), 18: ('Con', True), 19: ('Con', False)}
IGNORED = {15}  # Comment


class CompileError(Exception):
    pass


def custom_terminals(pf):
    """Declared terminals of a custom prefab: list of (voxel pos, signal, is_input), top-first order."""
    out = []
    for (idx, t, pos, v) in pf.settings:
        if t >= 7 and max(pos) < 256 and t in TERM_SETTING:
            sig, is_in = TERM_SETTING[t]
            if is_in is None:
                raise CompileError('void terminal on custom prefab %s' % pf.name)
            out.append((pos, sig, is_in, v))
    out.sort(key=lambda t: (-t[0][2], t[0][0], t[0][1]))
    return out


class Node:
    def __init__(self, op, pos, block_id):
        self.op = op
        self.pos = pos
        self.block_id = block_id
        self.ins = []      # refs
        self.execs = []    # branch targets (node indices or None), op order
        self.after = None  # next statement
        self.data = None
        self.has_before = False
        self.custom_prog = None


class Program:
    def __init__(self, pf):
        self.pf = pf
        self.nodes = []
        self.entries = []
        self.locals = {}      # (name, type) -> index
        self.templates = []   # object indices in the prefab grid (for non-level prefabs)
        self.outputs = []     # for custom prefabs: declared output terminals -> ref
        self.inputs = []      # declared input terminals


class Compiler:
    def __init__(self, game, lib, build_objects):
        self.game = game
        self.lib = lib
        self.build_objects = build_objects
        self.programs = {}     # prefab id -> Program
        self.globals = {}      # (name, type) -> index
        self.warnings = set()

    def gvar(self, name, t):
        key = (name, t)
        if key not in self.globals:
            self.globals[key] = len(self.globals)
        return self.globals[key]

    def program(self, pid):
        if pid not in self.programs:
            self.programs[pid] = None  # recursion guard
            self.programs[pid] = self.compile(self.game.seg(pid))
        return self.programs[pid]

    def compile(self, pf):
        g = self.game
        prog = Program(pf)
        sx, sy, sz = pf.size
        settings = {}
        for (idx, t, pos, v) in pf.settings:
            settings.setdefault(tuple(pos), []).append((idx, t, v))
        # block instances (origin cells)
        inst = {}
        for z in range(sz):
            for y in range(sy):
                for x in range(sx):
                    id = pf.blocks[x + y * sx + z * sx * sy]
                    if not id:
                        continue
                    s = g.seg(id)
                    if s.in_group and s.gpos != (0, 0, 0):
                        continue
                    inst[(x, y, z)] = id
        # objects inside this grid (templates / level objects)
        objs, part_obj = self.build_objects(g, self.lib, pf)
        prog.objects = objs
        prog.part_obj = part_obj
        # terminal tables
        tpos = {}   # world voxel -> list of (block pos, term name, dir, sig)
        node_of = {}
        order = sorted(inst, key=lambda p: (-p[2], p[0], -p[1]))
        for pos in order:
            id = inst[pos]
            s = g.seg(id)
            if id in ops.OPS:
                n = Node(ops.OPS[id]['index'], pos, id)
                node_of[pos] = len(prog.nodes)
                prog.nodes.append(n)
                for t in DEFS[id][3]:
                    if not t:
                        continue
                    w = (pos[0] * 8 + t[3][0], pos[1] * 8 + t[3][1], pos[2] * 8 + t[3][2])
                    nm = t[2] if t[2] in ('After', 'Before') else ('V', tuple(t[3]))
                    tpos.setdefault(w, []).append((pos, nm, t[1], t[0]))
            elif id < g.id_offset:
                if s.type == SCRIPT and id not in IGNORED and id in DEFS:
                    self.warnings.add('unsupported block %s' % DEFS[id][0])
            elif s.blocks and s.size != (0, 0, 0):
                # custom prefab with inner grid: script block or scripted object
                child = self.program(id)
                if child is None:
                    raise CompileError('recursive prefab %s' % s.name)
                n = Node(ops.OP_CUSTOM, pos, id)
                n.custom_prog = id
                node_of[pos] = len(prog.nodes)
                prog.nodes.append(n)
                for (tp, sig, is_in, name) in custom_terminals(s):
                    w = (pos[0] * 8 + tp[0], pos[1] * 8 + tp[1], pos[2] * 8 + tp[2])
                    tpos.setdefault(w, []).append((pos, ('T', tp), 'In' if is_in else 'Out', sig))
        prog.inputs = [t for t in custom_terminals(pf) if t[2]]
        prog.outputs_decl = [t for t in custom_terminals(pf) if not t[2]]

        # connections: explicit + implicit adjacency
        edges = []  # (src pos, src term, dst pos, dst term)

        def term_at(pos, voxel):
            w = (pos[0] * 8 + voxel[0], pos[1] * 8 + voxel[1], pos[2] * 8 + voxel[2])
            for (p, nm, d, sig) in tpos.get(w, []):
                if p == pos:
                    return nm, d, sig
            return None
        for (fp, tp, fo, to) in pf.conns:
            src = ('outside', fo) if fp[0] == OUTSIDE else None
            dst = ('outside', to) if tp[0] == OUTSIDE else None
            if src is None:
                tt = term_at(fp, fo) if fp in node_of else None
                src = (fp, tt[0]) if tt else ('object', fp, fo)
            if dst is None:
                tt = term_at(tp, to) if tp in node_of else None
                if tt is None:
                    self.warnings.add('connection into non-terminal in %s' % pf.name)
                    continue
                dst = (tp, tt[0])
            edges.append((src, dst))
        for w, lst in tpos.items():
            for (p, nm, d, sig) in lst:
                if d != 'Out':
                    continue
                if sig == 'Void' and nm == 'After':
                    for (p2, n2, d2, s2) in tpos.get((w[0], w[1], w[2] - 2), []):
                        if n2 == 'Before':
                            edges.append(((p, nm), (p2, n2)))
                else:
                    for (p2, n2, d2, s2) in tpos.get((w[0] + 2, w[1], w[2]), []):
                        if d2 == 'In' and (s2 == 'Void') == (sig == 'Void') and n2 != 'Before':
                            edges.append(((p, nm), (p2, n2)))
        edges = list(dict.fromkeys(edges))

        # resolve refs
        def src_ref(src):
            if src[0] == 'outside':
                voxel = src[1]
                for k, t in enumerate(prog.inputs):
                    if tuple(t[0]) == tuple(voxel):
                        return ('in', k)
                return ('self', tuple(voxel))
            if src[0] == 'object':
                cell, voxel = src[1], src[2]
                # find the object component at that voxel
                c = (cell[0] + voxel[0] // 8, cell[1] + voxel[1] // 8, cell[2] + voxel[2] // 8)
                lv = (voxel[0] % 8, voxel[1] % 8, voxel[2] % 8)
                return ('obj', self.object_at(prog, c, lv))
            pos, term = src
            ni = node_of[pos]
            n = prog.nodes[ni]
            if n.op == ops.OP_CUSTOM:
                child = self.programs[n.custom_prog]
                outs = [t for t in custom_terminals(g.seg(n.custom_prog)) if not t[2]]
                for k, t in enumerate(outs):
                    if ('T', t[0]) == term:
                        return ('node', ni, k)
                raise CompileError('bad custom output')
            outs = ops.OPS[n.block_id]['outs']
            for k, t in enumerate(outs):
                if ('V', tuple(t[3])) == term:
                    return ('node', ni, k)
            raise CompileError('output %s not found on %s' % (term, DEFS[n.block_id][0]))

        in_edges = {}
        exec_edges = {}
        for (src, dst) in edges:
            if dst[0] == 'outside':
                # value output of this prefab
                for k, t in enumerate(prog.outputs_decl):
                    if tuple(t[0]) == tuple(dst[1]):
                        prog.outputs.append((k, src_ref(src)))
                continue
            dpos, dterm = dst
            dn = prog.nodes[node_of[dpos]]
            if dterm == 'Before':
                if src[0] == 'outside':
                    raise CompileError('void input from outside')
                exec_edges.setdefault((src[0], src[1]), []).append(node_of[dpos])
                dn.has_before = True
                continue
            in_edges.setdefault((dpos, dterm), []).append(src)
        # fill node inputs / execs
        for ni, n in enumerate(prog.nodes):
            if n.op == ops.OP_CUSTOM:
                for (tp, sig, is_in, name) in custom_terminals(g.seg(n.custom_prog)):
                    if is_in:
                        srcs = in_edges.get((n.pos, ('T', tp)), [])
                        n.ins.append(src_ref(srcs[0]) if srcs else None)
                continue
            o = ops.OPS[n.block_id]
            for t in o['ins']:
                srcs = in_edges.get((n.pos, ('V', tuple(t[3]))), [])
                if len(srcs) > 1:
                    self.warnings.add('multiple sources into %s.%s' % (o['name'], t[2]))
                n.ins.append(src_ref(srcs[0]) if srcs else None)
            for t in o['execs']:
                tg = exec_edges.get((n.pos, ('V', tuple(t[3]))), [])
                n.execs.append(tg)
            nx = exec_edges.get((n.pos, 'After'), [])
            n.after = nx
            n.data = self.node_data(prog, n, settings.get(n.pos, []))
        # entries: active statements without Before connection, parse order
        for ni, n in enumerate(prog.nodes):
            if n.op != ops.OP_CUSTOM and ops.OPS[n.block_id]['active'] and not n.has_before:
                prog.entries.append(ni)
        # templates: every object inside a non-level prefab grid
        prog.env_children = [ni for ni, n in enumerate(prog.nodes) if n.op == ops.OP_CUSTOM]
        return prog

    def object_at(self, prog, cell, voxel):
        c = cell
        cands = [(oi, pid, k) for (pc, k), oi in prog.part_obj.items() if pc == c for pid in [self.cell_id(prog, pc)]]
        i = voxel[0] + voxel[1] * 8 + voxel[2] * 64
        for (oi, pid, k) in cands:
            if self.lib.get(pid).comp[i] == k:
                return oi
        if cands:
            return min(cands)[0]
        self.warnings.add('object reference to empty cell in %s' % prog.pf.name)
        return None

    def cell_id(self, prog, c):
        sx, sy, sz = prog.pf.size
        return prog.pf.blocks[c[0] + c[1] * sx + c[2] * sx * sy]

    def node_data(self, prog, n, sets):
        id = n.block_id
        if id == 36:
            return ('num', next((v for (i, t, v) in sets if t == 4), 0.0))
        if id == 38:
            return ('vec', next((v for (i, t, v) in sets if t == 5), (0.0, 0.0, 0.0)))
        if id == 42:
            return ('rot', next((v for (i, t, v) in sets if t == 5), (0.0, 0.0, 0.0)))
        if id in VAR_GET or id in VAR_SET:
            name = next((v for (i, t, v) in sets if t == 6), '')
            vt = VAR_GET[id] if id in VAR_GET else VAR_SET[id]
            if name.startswith('$') or name.startswith('!'):
                return ('var', 1, self.gvar(name, vt), name)
            key = (name, vt)
            if key not in prog.locals:
                prog.locals[key] = len(prog.locals)
            return ('var', 0, prog.locals[key], name)
        if id in (252, 256):  # win / lose delay
            return ('byte', next((v for (i, t, v) in sets if t == 1), 0))
        if id == 588:  # button type
            return ('byte', next((v for (i, t, v) in sets if t == 1), 0))
        if id == 268:  # camera perspective flag
            return ('byte', next((v for (i, t, v) in sets if t == 1), 0))
        if id == 592:
            return ('byte', next((v for (i, t, v) in sets if t == 1), 0))
        if id == 264:  # play sound: sound id
            return ('byte', next((v for (i, t, v) in sets if t == 1), 0))
        if sets and id not in (15,):
            known = {t for (i, t, v) in sets}
            if known - {6}:
                self.warnings.add('settings ignored on %s: %s' % (DEFS[id][0], sets))
        return None
