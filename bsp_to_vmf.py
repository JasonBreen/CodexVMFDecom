#!/usr/bin/env python3
"""
Basic Source BSP -> VMF decompiler.

Goals:
- Preserve brush geometry from dbrush/dbrushside/planes.
- Preserve texture assignment from texinfo/texdata/string tables.
- Preserve point entities from the entity lump.

Limitations:
- Displacements, overlays, and many game-specific features are not reconstructed.
- Brush entities are emitted with no solids (world geometry is exported).
"""

from __future__ import annotations

import argparse
import lzma
import math
import re
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

LUMP_ENTITIES = 0
LUMP_PLANES = 1
LUMP_TEXDATA = 2
LUMP_TEXINFO = 6
LUMP_TEXDATA_STRING_DATA = 43
LUMP_TEXDATA_STRING_TABLE = 44
LUMP_BRUSHES = 18
LUMP_BRUSHSIDES = 19

HEADER_LUMPS = 64
HEADER_FMT = "<4sI" + "4i" * HEADER_LUMPS + "I"

MAX_COORD = 32768.0
EPS = 1e-5


@dataclass(frozen=True)
class LumpInfo:
    offset: int
    length: int
    version: int
    fourcc: int


@dataclass(frozen=True)
class Plane:
    normal: Tuple[float, float, float]
    dist: float
    ptype: int


@dataclass(frozen=True)
class TexData:
    reflectivity: Tuple[float, float, float]
    name_id: int
    width: int
    height: int
    view_width: int
    view_height: int


@dataclass(frozen=True)
class TexInfo:
    tex_s: Tuple[float, float, float, float]
    tex_t: Tuple[float, float, float, float]
    lightmap_s: Tuple[float, float, float, float]
    lightmap_t: Tuple[float, float, float, float]
    flags: int
    texdata: int


@dataclass(frozen=True)
class Brush:
    first_side: int
    num_sides: int
    contents: int


@dataclass(frozen=True)
class BrushSide:
    plane_num: int
    texinfo: int
    dispinfo: int
    bevel: int
    thin: int


def vec_add(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float, float]:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vec_sub(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vec_scale(v: Sequence[float], s: float) -> Tuple[float, float, float]:
    return (v[0] * s, v[1] * s, v[2] * s)


def dot(a: Sequence[float], b: Sequence[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a: Sequence[float], b: Sequence[float]) -> Tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def length(v: Sequence[float]) -> float:
    return math.sqrt(dot(v, v))


def normalize(v: Sequence[float]) -> Tuple[float, float, float]:
    l = length(v)
    if l < EPS:
        return (0.0, 0.0, 1.0)
    return (v[0] / l, v[1] / l, v[2] / l)


def almost_equal(a: Sequence[float], b: Sequence[float], eps: float = 1e-3) -> bool:
    return abs(a[0] - b[0]) < eps and abs(a[1] - b[1]) < eps and abs(a[2] - b[2]) < eps


class BspParser:
    def __init__(self, path: Path):
        self.path = path
        self.data = path.read_bytes()
        self.lumps: List[LumpInfo] = []
        self.version: int = 0
        self._parse_header()

    def _parse_header(self) -> None:
        header_size = struct.calcsize(HEADER_FMT)
        if len(self.data) < header_size:
            raise ValueError("File too small to be a Source BSP")

        unpacked = struct.unpack_from(HEADER_FMT, self.data, 0)
        ident = unpacked[0]
        self.version = unpacked[1]
        if ident != b"VBSP":
            raise ValueError(f"Unsupported BSP ident: {ident!r}")

        self.lumps = []
        idx = 2
        for _ in range(HEADER_LUMPS):
            offset, length_, version, fourcc = unpacked[idx : idx + 4]
            self.lumps.append(LumpInfo(offset, length_, version, fourcc))
            idx += 4

    def _decompress_lzma_lump(self, data: bytes) -> bytes:
        # Valve LZMA lump header (see Source SDK 2013 public/bspfile.h):
        # 4 bytes "LZMA", int actualSize, int lzmaSize, 5 bytes properties.
        if len(data) < 17 or data[:4] != b"LZMA":
            return data

        actual_size, lzma_size = struct.unpack_from("<ii", data, 4)
        props = data[12:17]
        payload = data[17 : 17 + max(0, lzma_size)]
        if len(payload) != max(0, lzma_size):
            raise ValueError("Invalid LZMA lump payload length")

        prop0 = props[0]
        lc = prop0 % 9
        rest = prop0 // 9
        lp = rest % 5
        pb = rest // 5
        dict_size = int.from_bytes(props[1:5], "little", signed=False)

        filters = [
            {
                "id": lzma.FILTER_LZMA1,
                "dict_size": max(4096, dict_size),
                "lc": lc,
                "lp": lp,
                "pb": pb,
            }
        ]

        out = lzma.decompress(payload, format=lzma.FORMAT_RAW, filters=filters)
        if actual_size >= 0 and len(out) != actual_size:
            raise ValueError("LZMA lump decompressed size mismatch")
        return out

    def read_lump(self, lump_id: int) -> bytes:
        info = self.lumps[lump_id]
        if info.offset < 0 or info.length < 0 or info.offset + info.length > len(self.data):
            raise ValueError(f"Invalid lump bounds for lump {lump_id}")
        raw = self.data[info.offset : info.offset + info.length]
        if raw[:4] == b"LZMA":
            return self._decompress_lzma_lump(raw)
        return raw


def parse_planes(data: bytes) -> List[Plane]:
    fmt = "<4fi"
    sz = struct.calcsize(fmt)
    planes: List[Plane] = []
    for i in range(0, len(data), sz):
        nx, ny, nz, dist, ptype = struct.unpack_from(fmt, data, i)
        planes.append(Plane((nx, ny, nz), dist, ptype))
    return planes


def parse_texdata(data: bytes) -> List[TexData]:
    fmt = "<3f5i"
    sz = struct.calcsize(fmt)
    out: List[TexData] = []
    for i in range(0, len(data), sz):
        rx, ry, rz, name_id, w, h, vw, vh = struct.unpack_from(fmt, data, i)
        out.append(TexData((rx, ry, rz), name_id, w, h, vw, vh))
    return out


def parse_texinfo(data: bytes) -> List[TexInfo]:
    fmt = "<8f8fii"
    sz = struct.calcsize(fmt)
    out: List[TexInfo] = []
    for i in range(0, len(data), sz):
        vals = struct.unpack_from(fmt, data, i)
        out.append(
            TexInfo(
                vals[0:4],
                vals[4:8],
                vals[8:12],
                vals[12:16],
                vals[16],
                vals[17],
            )
        )
    return out


def parse_string_table(data: bytes) -> List[int]:
    out = []
    for i in range(0, len(data), 4):
        (ofs,) = struct.unpack_from("<i", data, i)
        out.append(ofs)
    return out


def parse_brushes(data: bytes) -> List[Brush]:
    fmt = "<iii"
    sz = struct.calcsize(fmt)
    out: List[Brush] = []
    for i in range(0, len(data), sz):
        first, count, contents = struct.unpack_from(fmt, data, i)
        out.append(Brush(first, count, contents))
    return out


def parse_brushsides(data: bytes) -> List[BrushSide]:
    # Source SDK 2013 dbrushside_t is 8 bytes: unsigned short, short, short, short.
    # Some later branches add fields. Detect by stride from lump length.
    if len(data) % 8 == 0:
        fmt = "<Hhhh"
    elif len(data) % 6 == 0:
        # Older/non-standard variants seen in some branches/tools.
        fmt = "<HhHbb"
    else:
        raise ValueError(f"Unsupported brushside lump size {len(data)}")

    sz = struct.calcsize(fmt)
    out: List[BrushSide] = []
    for i in range(0, len(data), sz):
        vals = struct.unpack_from(fmt, data, i)
        if sz == 8:
            plane_num, texinfo, dispinfo, bevel = vals
            thin = 0
        else:
            plane_num, texinfo, dispinfo, bevel, thin = vals
        out.append(BrushSide(plane_num, texinfo, dispinfo, bevel, thin))
    return out


def parse_entities(data: bytes) -> List[Dict[str, str]]:
    txt = data.decode("utf-8", errors="replace")
    blocks = re.findall(r"\{[^{}]*\}", txt, flags=re.DOTALL)
    entities: List[Dict[str, str]] = []
    pair_re = re.compile(r'"([^"]*)"\s+"([^"]*)"')
    for block in blocks:
        ent: Dict[str, str] = {}
        for k, v in pair_re.findall(block):
            ent[k] = v
        if ent:
            entities.append(ent)
    return entities


def make_base_winding(normal: Sequence[float], dist: float) -> List[Tuple[float, float, float]]:
    n = normalize(normal)
    org = vec_scale(n, dist)

    if abs(n[2]) > 0.999:
        vup = (1.0, 0.0, 0.0)
    else:
        vup = (0.0, 0.0, 1.0)

    vright = normalize(cross(vup, n))
    vup = normalize(cross(n, vright))

    vright = vec_scale(vright, MAX_COORD)
    vup = vec_scale(vup, MAX_COORD)

    return [
        vec_add(vec_sub(org, vright), vup),
        vec_add(vec_add(org, vright), vup),
        vec_sub(vec_add(org, vright), vup),
        vec_sub(vec_sub(org, vright), vup),
    ]


def clip_polygon(poly: List[Tuple[float, float, float]], normal: Sequence[float], dist: float) -> List[Tuple[float, float, float]]:
    if not poly:
        return []

    out: List[Tuple[float, float, float]] = []
    prev = poly[-1]
    prev_d = dot(prev, normal) - dist

    for cur in poly:
        cur_d = dot(cur, normal) - dist
        prev_in = prev_d <= EPS
        cur_in = cur_d <= EPS

        if prev_in and cur_in:
            out.append(cur)
        elif prev_in and not cur_in:
            t = prev_d / (prev_d - cur_d)
            inter = vec_add(prev, vec_scale(vec_sub(cur, prev), t))
            out.append(inter)
        elif not prev_in and cur_in:
            t = prev_d / (prev_d - cur_d)
            inter = vec_add(prev, vec_scale(vec_sub(cur, prev), t))
            out.append(inter)
            out.append(cur)

        prev = cur
        prev_d = cur_d

    # Deduplicate adjacent points
    dedup: List[Tuple[float, float, float]] = []
    for p in out:
        if not dedup or not almost_equal(dedup[-1], p):
            dedup.append(p)
    if len(dedup) > 2 and almost_equal(dedup[0], dedup[-1]):
        dedup.pop()
    return dedup


def build_side_winding(
    side_index: int,
    sides: List[BrushSide],
    planes: List[Plane],
    brush_side_indices: Iterable[int],
) -> List[Tuple[float, float, float]]:
    side = sides[side_index]
    plane = planes[side.plane_num]
    poly = make_base_winding(plane.normal, plane.dist)

    for other_idx in brush_side_indices:
        if other_idx == side_index:
            continue
        other = sides[other_idx]
        oplane = planes[other.plane_num]
        # Keep points behind the inward-facing plane.
        poly = clip_polygon(poly, oplane.normal, oplane.dist)
        if len(poly) < 3:
            return []

    return poly


def choose_three_points(poly: List[Tuple[float, float, float]], normal: Sequence[float]) -> Tuple[Tuple[float, float, float], ...]:
    if len(poly) < 3:
        raise ValueError("Need at least 3 points")

    p0 = poly[0]
    p1 = poly[1]
    best_i = 2
    best_area = 0.0
    for i in range(2, len(poly)):
        area = length(cross(vec_sub(poly[i], p0), vec_sub(p1, p0)))
        if area > best_area:
            best_area = area
            best_i = i
    p2 = poly[best_i]

    c = cross(vec_sub(p1, p0), vec_sub(p2, p0))
    if dot(c, normal) < 0:
        return (p0, p2, p1)
    return (p0, p1, p2)


def fmt_pt(p: Sequence[float]) -> str:
    return f"({p[0]:.6f} {p[1]:.6f} {p[2]:.6f})"


def fmt_axis(v: Sequence[float], shift: float, scale: float = 0.25) -> str:
    return f"[{v[0]:.6f} {v[1]:.6f} {v[2]:.6f} {shift:.6f}] {scale:.6f}"


def decompile(bsp_path: Path, vmf_path: Path) -> None:
    parser = BspParser(bsp_path)

    planes = parse_planes(parser.read_lump(LUMP_PLANES))
    texdata = parse_texdata(parser.read_lump(LUMP_TEXDATA))
    texinfo = parse_texinfo(parser.read_lump(LUMP_TEXINFO))
    str_data = parser.read_lump(LUMP_TEXDATA_STRING_DATA)
    str_tbl = parse_string_table(parser.read_lump(LUMP_TEXDATA_STRING_TABLE))
    brushes = parse_brushes(parser.read_lump(LUMP_BRUSHES))
    brushsides = parse_brushsides(parser.read_lump(LUMP_BRUSHSIDES))
    entities = parse_entities(parser.read_lump(LUMP_ENTITIES))

    tex_names: List[str] = []
    for ofs in str_tbl:
        if ofs < 0 or ofs >= len(str_data):
            tex_names.append("TOOLS/TOOLSNODRAW")
            continue
        end = str_data.find(b"\x00", ofs)
        if end == -1:
            end = len(str_data)
        tex_names.append(str_data[ofs:end].decode("utf-8", errors="replace"))

    cubemap_fixup_re = re.compile(r"^maps/[^/]+/(.+?)(?:_-?\d+_-?\d+_-?\d+)?$", re.IGNORECASE)

    def normalize_material(name: str) -> str:
        n = name.strip().replace("\\", "/")
        m = cubemap_fixup_re.match(n)
        if m:
            n = m.group(1)
        return n or "TOOLS/TOOLSNODRAW"

    def get_material(side: BrushSide) -> str:
        if side.texinfo < 0 or side.texinfo >= len(texinfo):
            return "TOOLS/TOOLSNODRAW"
        ti = texinfo[side.texinfo]
        if ti.texdata < 0 or ti.texdata >= len(texdata):
            return "TOOLS/TOOLSNODRAW"
        td = texdata[ti.texdata]
        if td.name_id < 0 or td.name_id >= len(tex_names):
            return "TOOLS/TOOLSNODRAW"
        return normalize_material(tex_names[td.name_id])

    lines: List[str] = []
    lines.append("versioninfo")
    lines.append("{")
    lines.append('\t"editorversion" "400"')
    lines.append('\t"editorbuild" "0"')
    lines.append('\t"mapversion" "1"')
    lines.append('\t"formatversion" "100"')
    lines.append('\t"prefab" "0"')
    lines.append("}")
    lines.append("visgroups")
    lines.append("{")
    lines.append("}")
    lines.append("viewsettings")
    lines.append("{")
    lines.append('\t"bSnapToGrid" "1"')
    lines.append('\t"bShowGrid" "1"')
    lines.append('\t"bShowLogicalGrid" "0"')
    lines.append('\t"nGridSpacing" "64"')
    lines.append("}")

    next_id = 1

    worldspawn = next((e for e in entities if e.get("classname") == "worldspawn"), {"classname": "worldspawn"})

    lines.append("world")
    lines.append("{")
    lines.append(f'\t"id" "{next_id}"')
    next_id += 1
    for k, v in worldspawn.items():
        if k == "classname" or k == "id":
            continue
        lines.append(f'\t"{k}" "{v}"')
    lines.append('\t"classname" "worldspawn"')

    for brush in brushes:
        if brush.num_sides < 4:
            continue
        brush_indices = list(range(brush.first_side, brush.first_side + brush.num_sides))
        if brush_indices[-1] >= len(brushsides):
            continue

        lines.append("\tsolid")
        lines.append("\t{")
        lines.append(f'\t\t"id" "{next_id}"')
        next_id += 1

        side_count = 0
        for side_idx in brush_indices:
            side = brushsides[side_idx]
            if side.bevel:
                continue
            if side.plane_num >= len(planes):
                continue

            poly = build_side_winding(side_idx, brushsides, planes, brush_indices)
            if len(poly) < 3:
                continue

            p0, p1, p2 = choose_three_points(poly, planes[side.plane_num].normal)
            mat = get_material(side)
            ti = texinfo[side.texinfo] if 0 <= side.texinfo < len(texinfo) else None
            uaxis = fmt_axis(ti.tex_s[:3], ti.tex_s[3]) if ti else "[1 0 0 0] 0.25"
            vaxis = fmt_axis(ti.tex_t[:3], ti.tex_t[3]) if ti else "[0 1 0 0] 0.25"

            lines.append("\t\tside")
            lines.append("\t\t{")
            lines.append(f'\t\t\t"id" "{next_id}"')
            next_id += 1
            lines.append(f'\t\t\t"plane" "{fmt_pt(p0)} {fmt_pt(p1)} {fmt_pt(p2)}"')
            lines.append(f'\t\t\t"material" "{mat}"')
            lines.append(f'\t\t\t"uaxis" "{uaxis}"')
            lines.append(f'\t\t\t"vaxis" "{vaxis}"')
            lines.append('\t\t\t"rotation" "0"')
            lines.append('\t\t\t"lightmapscale" "16"')
            lines.append('\t\t\t"smoothing_groups" "0"')
            lines.append("\t\t}")
            side_count += 1

        if side_count == 0:
            lines.pop()
            lines.pop()
            continue

        lines.append("\t\teditor")
        lines.append("\t\t{")
        lines.append('\t\t\t"color" "0 255 0"')
        lines.append('\t\t\t"visgroupshown" "1"')
        lines.append('\t\t\t"visgroupautoshown" "1"')
        lines.append("\t\t}")
        lines.append("\t}")

    lines.append("}")

    for ent in entities:
        classname = ent.get("classname", "")
        if classname in {"worldspawn", ""}:
            continue
        lines.append("entity")
        lines.append("{")
        lines.append(f'\t"id" "{next_id}"')
        next_id += 1
        for k, v in ent.items():
            if k == "id":
                continue
            lines.append(f'\t"{k}" "{v}"')
        lines.append("\teditor")
        lines.append("\t{")
        lines.append('\t\t"color" "220 30 220"')
        lines.append('\t\t"visgroupshown" "1"')
        lines.append('\t\t"visgroupautoshown" "1"')
        lines.append("\t}")
        lines.append("}")

    lines.append("cameras")
    lines.append("{")
    lines.append('\t"activecamera" "-1"')
    lines.append("}")
    lines.append("cordons")
    lines.append("{")
    lines.append('\t"active" "0"')
    lines.append("}")

    vmf_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser(description="Basic Source BSP to raw VMF decompiler")
    ap.add_argument("bsp", type=Path, help="Input .bsp path")
    ap.add_argument("vmf", type=Path, nargs="?", help="Output .vmf path (default: <bsp>.vmf)")
    args = ap.parse_args()

    bsp_path = args.bsp
    if args.vmf:
        vmf_path = args.vmf
    else:
        vmf_path = bsp_path.with_suffix(".vmf")

    decompile(bsp_path, vmf_path)
    print(f"Wrote VMF: {vmf_path}")


if __name__ == "__main__":
    main()
