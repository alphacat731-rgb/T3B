#!/usr/bin/env python3
"""T3B - tiny terminal 3D model viewer.

Native loaders: OBJ, GLTF 2.0, GLB, ASCII/Binary STL, ASCII PLY.
Other formats (including FBX) can be imported through the Assimp CLI when
assimp is installed.

Rendering is CPU-only. Braille is the default because a 2x4 dot cell gives
much finer lines than one character per screen pixel.
"""
from __future__ import annotations

import argparse
import base64
import binascii
import curses
import json
import locale
import math
import os
import struct
import subprocess
import sys
import tempfile
import time
import urllib.parse
from pathlib import Path

Vec3 = tuple[float, float, float]
Face = tuple[int, int, int]
RASTER_W = 0

BRAILLE_DOTS = {
    (0, 0): 0x01, (0, 1): 0x02, (0, 2): 0x04, (1, 0): 0x08,
    (1, 1): 0x10, (1, 2): 0x20, (0, 3): 0x40, (1, 3): 0x80,
}


class T3BError(Exception):
    pass


class Mesh:
    def __init__(self, vertices: list[Vec3], faces: list[Face], name: str = "model") -> None:
        if not vertices:
            raise T3BError("The model contains no vertices.")
        self.vertices = vertices
        self.faces = faces
        self.name = name
        self.edges = self._make_edges(faces)
        self.center, self.radius = self._bounds(vertices)
        cx, cy, cz = self.center
        self.render_vertices = [(x - cx, y - cy, z - cz) for x, y, z in vertices]
        self.edge_scores = self._score_edges(vertices, faces, self.edges)
        self._edges_by_score = sorted(
            range(len(self.edges)),
            key=self.edge_scores.__getitem__,
            reverse=True,
        )

    @staticmethod
    def _make_edges(faces: list[Face]) -> list[tuple[int, int]]:
        edges: set[tuple[int, int]] = set()
        for a, b, c in faces:
            for u, v in ((a, b), (b, c), (c, a)):
                if u != v:
                    edges.add((min(u, v), max(u, v)))
        return list(edges)

    @staticmethod
    def _score_edges(
        vertices: list[Vec3],
        faces: list[Face],
        edges: list[tuple[int, int]],
    ) -> list[float]:
        if not edges:
            return []

        edge_faces: dict[tuple[int, int], list[int]] = {}
        for fi, (a, b, c) in enumerate(faces):
            for u, v in ((a, b), (b, c), (c, a)):
                if u == v:
                    continue
                key = (min(u, v), max(u, v))
                edge_faces.setdefault(key, []).append(fi)

        normals: list[Vec3] = []
        for a, b, c in faces:
            ax, ay, az = vertices[a]
            bx, by, bz = vertices[b]
            cx, cy, cz = vertices[c]
            ux, uy, uz = bx - ax, by - ay, bz - az
            vx, vy, vz = cx - ax, cy - ay, cz - az
            nx, ny, nz = (
                uy * vz - uz * vy,
                uz * vx - ux * vz,
                ux * vy - uy * vx,
            )
            length = math.sqrt(nx * nx + ny * ny + nz * nz)
            if length > 1e-12:
                normals.append((nx / length, ny / length, nz / length))
            else:
                normals.append((0.0, 0.0, 0.0))

        scores: list[float] = []
        for edge in edges:
            a, b = edge
            length = math.dist(vertices[a], vertices[b])
            attached = edge_faces.get(edge, [])
            structural = 1.0
            if len(attached) == 1:
                structural += 2.5  # open/border edge
            elif len(attached) >= 2:
                n1, n2 = normals[attached[0]], normals[attached[1]]
                dot = max(-1.0, min(1.0, n1[0] * n2[0] + n1[1] * n2[1] + n1[2] * n2[2]))
                crease = 1.0 - abs(dot)
                structural += crease * 1.75
            scores.append(length * structural)

        return scores

    @staticmethod
    def _bounds(vertices: list[Vec3]) -> tuple[Vec3, float]:
        min_x = min(v[0] for v in vertices); max_x = max(v[0] for v in vertices)
        min_y = min(v[1] for v in vertices); max_y = max(v[1] for v in vertices)
        min_z = min(v[2] for v in vertices); max_z = max(v[2] for v in vertices)
        center = (
            (min_x + max_x) * 0.5,
            (min_y + max_y) * 0.5,
            (min_z + max_z) * 0.5,
        )
        radius = max(
            math.dist(center, (min_x, min_y, min_z)),
            math.dist(center, (max_x, max_y, max_z)),
            1e-6,
        )
        return center, radius

    def limited_edges(self, limit: int) -> list[tuple[int, int]]:
        if limit <= 0 or len(self.edges) <= limit:
            return self.edges
        if limit < 256:
            return [self.edges[i] for i in self._edges_by_score[:limit]]

        # Keep the strongest structural edges, then add a uniformly sampled
        # cross-section of the remaining mesh so dense models retain detail.
        structural_count = max(1, int(limit * 0.72))
        selected = set(self._edges_by_score[:structural_count])

        remaining = limit - len(selected)
        if remaining > 0:
            step = len(self.edges) / remaining
            for i in range(remaining):
                selected.add(min(len(self.edges) - 1, int(i * step)))
                if len(selected) >= limit:
                    break

        return [self.edges[i] for i in sorted(selected)[:limit]]


def triangulate(indices: list[int]) -> list[Face]:
    return [
        (indices[0], indices[i], indices[i + 1])
        for i in range(1, len(indices) - 1)
    ] if len(indices) >= 3 else []


def parse_obj(path: Path) -> Mesh:
    vertices: list[Vec3] = []
    faces: list[Face] = []
    with path.open("r", encoding="utf-8", errors="replace") as f:
        for lineno, raw in enumerate(f, 1):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            p = line.split()
            if p[0] == "v" and len(p) >= 4:
                try:
                    vertices.append((float(p[1]), float(p[2]), float(p[3])))
                except ValueError as exc:
                    raise T3BError(f"OBJ line {lineno}: invalid vertex") from exc
            elif p[0] == "f" and len(p) >= 4:
                idxs = []
                for token in p[1:]:
                    raw_idx = token.split("/", 1)[0]
                    if not raw_idx:
                        continue
                    idx = int(raw_idx)
                    idx = idx - 1 if idx > 0 else len(vertices) + idx
                    if idx < 0 or idx >= len(vertices):
                        raise T3BError(f"OBJ line {lineno}: face index out of range")
                    idxs.append(idx)
                faces.extend(triangulate(idxs))
    return Mesh(vertices, faces, path.name)


def read_accessor(doc: dict, buffers: list[bytes], index: int) -> list[object]:
    acc = doc["accessors"][index]
    count = int(acc["count"])
    comp_type = int(acc["componentType"])
    type_name = acc["type"]
    components = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}.get(type_name)
    if components is None:
        raise T3BError(f"GLTF accessor type not supported: {type_name}")

    formats = {
        5120: ("b", 1), 5121: ("B", 1), 5122: ("h", 2),
        5123: ("H", 2), 5125: ("I", 4), 5126: ("f", 4),
    }
    if comp_type not in formats:
        raise T3BError(f"GLTF component type not supported: {comp_type}")
    fmt, size = formats[comp_type]
    view_index = acc.get("bufferView")
    if view_index is None:
        raise T3BError("Sparse/implicit GLTF accessors are not supported.")
    view = doc["bufferViews"][view_index]
    buf_index = int(view.get("buffer", 0))
    if buf_index >= len(buffers):
        raise T3BError("GLTF buffer index is out of range.")
    blob = buffers[buf_index]
    stride = int(view.get("byteStride", components * size))
    base = int(view.get("byteOffset", 0)) + int(acc.get("byteOffset", 0))
    width = components * size
    unpack_fmt = "<" + fmt * components

    out: list[object] = []
    for i in range(count):
        off = base + i * stride
        if off + width > len(blob):
            raise T3BError("GLTF accessor points outside its buffer.")
        vals = struct.unpack_from(unpack_fmt, blob, off)
        out.append(vals[0] if type_name == "SCALAR" else vals)
    return out


def load_glb(path: Path) -> tuple[dict, list[bytes]]:
    data = path.read_bytes()
    if len(data) < 12 or data[:4] != b"glTF":
        raise T3BError("Not a valid GLB file.")
    version, total_len = struct.unpack_from("<II", data, 4)
    if version != 2:
        raise T3BError("Only glTF 2.0 GLB files are supported.")
    if total_len > len(data):
        raise T3BError("GLB declares a length larger than the file.")

    json_chunk = None
    bins: list[bytes] = []
    pos = 12
    while pos + 8 <= total_len:
        length, kind = struct.unpack_from("<II", data, pos)
        pos += 8
        if pos + length > len(data):
            raise T3BError("GLB chunk exceeds the file.")
        chunk = data[pos:pos + length]
        pos += length
        if kind == 0x4E4F534A:
            json_chunk = chunk
        elif kind == 0x004E4942:
            bins.append(chunk)

    if json_chunk is None:
        raise T3BError("GLB has no JSON chunk.")
    return json.loads(json_chunk.decode("utf-8-sig").rstrip("\x00 \t\r\n")), bins


def load_gltf_document(path: Path) -> tuple[dict, list[bytes]]:
    if path.suffix.lower() == ".glb":
        doc, bins = load_glb(path)
        buffers: list[bytes] = []
        bin_i = 0
        for buf in doc.get("buffers", []):
            uri = buf.get("uri")
            if uri:
                if not uri.startswith("data:"):
                    raise T3BError("External GLB buffers are not supported.")
                try:
                    buffers.append(base64.b64decode(uri.split(",", 1)[1]))
                except (ValueError, binascii.Error) as exc:
                    raise T3BError("Invalid embedded GLB buffer.") from exc
            else:
                if bin_i >= len(bins):
                    raise T3BError("GLB is missing a BIN chunk.")
                buffers.append(bins[bin_i])
                bin_i += 1
        if not buffers and bins:
            buffers = bins
        return doc, buffers

    doc = json.loads(path.read_text(encoding="utf-8"))
    buffers = []
    for buf in doc.get("buffers", []):
        uri = buf.get("uri")
        if not uri:
            raise T3BError("GLTF buffer has no URI; use GLB for self-contained assets.")
        if uri.startswith("data:"):
            try:
                buffers.append(base64.b64decode(uri.split(",", 1)[1]))
            except (ValueError, binascii.Error) as exc:
                raise T3BError("Invalid embedded GLTF buffer.") from exc
        else:
            buffers.append((path.parent / urllib.parse.unquote(uri)).resolve().read_bytes())
    return doc, buffers


def parse_gltf(path: Path) -> Mesh:
    doc, buffers = load_gltf_document(path)
    vertices: list[Vec3] = []
    faces: list[Face] = []

    for mesh in doc.get("meshes", []):
        for prim in mesh.get("primitives", []):
            mode = int(prim.get("mode", 4))
            if mode not in (4, 5, 6):
                continue
            pos = prim.get("attributes", {}).get("POSITION")
            if pos is None:
                continue

            positions = read_accessor(doc, buffers, int(pos))
            base = len(vertices)
            vertices.extend((float(p[0]), float(p[1]), float(p[2])) for p in positions)

            if "indices" in prim:
                indices = [int(i) for i in read_accessor(doc, buffers, int(prim["indices"]))]
            else:
                indices = list(range(len(positions)))

            if mode == 4:
                local = [
                    (indices[i], indices[i + 1], indices[i + 2])
                    for i in range(0, len(indices) - 2, 3)
                ]
            elif mode == 5:
                local = []
                for i in range(len(indices) - 2):
                    a, b, c = indices[i], indices[i + 1], indices[i + 2]
                    local.append((a, c, b) if i & 1 else (a, b, c))
            else:
                local = (
                    [(indices[0], indices[i], indices[i + 1]) for i in range(1, len(indices) - 1)]
                    if indices else []
                )

            for a, b, c in local:
                if min(a, b, c) < 0 or max(a, b, c) >= len(positions):
                    raise T3BError("GLTF primitive contains an invalid index.")
                faces.append((a + base, b + base, c + base))

    if not vertices:
        raise T3BError("GLTF contains no POSITION geometry.")
    return Mesh(vertices, faces, path.name)


def parse_stl(path: Path) -> Mesh:
    data = path.read_bytes()
    if len(data) >= 84:
        count = struct.unpack_from("<I", data, 80)[0]
        if 84 + count * 50 == len(data):
            vertices: list[Vec3] = []
            faces: list[Face] = []
            for i in range(count):
                off = 84 + i * 50
                tri = struct.unpack_from("<9f", data, off + 12)
                base = len(vertices)
                vertices.extend(((tri[0], tri[1], tri[2]),
                                 (tri[3], tri[4], tri[5]),
                                 (tri[6], tri[7], tri[8])))
                faces.append((base, base + 1, base + 2))
            return Mesh(vertices, faces, path.name)

    vertices = []
    faces = []
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        current: list[Vec3] = []
        for raw in f:
            p = raw.strip().split()
            if len(p) >= 4 and p[0].lower() == "vertex":
                try:
                    current.append((float(p[1]), float(p[2]), float(p[3])))
                except ValueError:
                    current = []
                    continue
                if len(current) == 3:
                    base = len(vertices)
                    vertices.extend(current)
                    faces.append((base, base + 1, base + 2))
                    current = []
    if not vertices:
        raise T3BError("STL contains no triangles.")
    return Mesh(vertices, faces, path.name)


def parse_ply(path: Path) -> Mesh:
    with path.open("r", encoding="utf-8", errors="replace") as f:
        header: list[str] = []
        while True:
            line = f.readline()
            if not line:
                raise T3BError("PLY header is incomplete.")
            header.append(line.strip())
            if line.strip() == "end_header":
                break

        if not header or header[0] != "ply":
            raise T3BError("Not a PLY file.")
        if any("format binary" in x for x in header):
            raise T3BError("Binary PLY needs Assimp in this build.")

        v_count = f_count = 0
        for line in header:
            p = line.split()
            if len(p) >= 3 and p[0] == "element" and p[1] == "vertex":
                v_count = int(p[2])
            elif len(p) >= 3 and p[0] == "element" and p[1] == "face":
                f_count = int(p[2])

        vertices = []
        for _ in range(v_count):
            p = f.readline().split()
            if len(p) < 3:
                raise T3BError("PLY vertex row is incomplete.")
            vertices.append((float(p[0]), float(p[1]), float(p[2])))

        faces = []
        for _ in range(f_count):
            p = f.readline().split()
            if not p:
                continue
            n = int(p[0])
            faces.extend(triangulate([int(x) for x in p[1:1 + n]]))
    return Mesh(vertices, faces, path.name)


def find_assimp() -> str | None:
    for candidate in ("assimp", "assimp_cmd"):
        for directory in os.environ.get("PATH", "").split(os.pathsep):
            if directory:
                exe = Path(directory) / candidate
                if exe.is_file() and os.access(exe, os.X_OK):
                    return str(exe)
    return None


def load_via_assimp(path: Path) -> Mesh:
    assimp = find_assimp()
    if assimp is None:
        raise T3BError(
            f"{path.suffix.lower()} needs Assimp. Install it with "
            "sudo apt install assimp-utils for FBX and other formats."
        )

    with tempfile.TemporaryDirectory(prefix="t3b-") as tmp:
        out = Path(tmp) / "converted.obj"
        proc = subprocess.run(
            [assimp, "export", str(path), str(out)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=120,
        )
        if proc.returncode != 0 or not out.exists():
            detail = proc.stdout.strip().splitlines()[-1] if proc.stdout.strip() else "unknown Assimp error"
            raise T3BError(f"Assimp could not import {path.name}: {detail}")
        mesh = parse_obj(out)
        mesh.name = path.name
        return mesh


def load_model(path: Path) -> Mesh:
    if not path.exists():
        raise T3BError(f"File not found: {path}")
    ext = path.suffix.lower()
    try:
        if ext == ".obj":
            return parse_obj(path)
        if ext in (".glb", ".gltf"):
            return parse_gltf(path)
        if ext == ".stl":
            return parse_stl(path)
        if ext == ".ply":
            return parse_ply(path)
        return load_via_assimp(path)
    except T3BError:
        raise
    except (OSError, ValueError, KeyError, json.JSONDecodeError, struct.error) as exc:
        raise T3BError(f"Could not load {path.name}: {exc}") from exc


def rotation_matrix(yaw: float, pitch: float, roll: float) -> tuple[float, ...]:
    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)

    # Same transform order as the original viewer: yaw -> pitch -> roll.
    return (
        cr * cy + sr * sp * sy, -sr * cp, -cr * sy + sr * sp * cy,
        sr * cy - cr * sp * sy,  cr * cp, -sr * sy - cr * sp * cy,
        cp * sy,                  sp,        cp * cy,
    )


def project(
    v: Vec3,
    matrix: tuple[float, ...],
    cam_dist: float,
    out_w: int,
    out_h: int,
    fov_deg: float,
    aspect: float,
) -> tuple[float, float, float] | None:
    x0, y0, z0 = v
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = matrix
    x = m00 * x0 + m01 * y0 + m02 * z0
    y = m10 * x0 + m11 * y0 + m12 * z0
    z = m20 * x0 + m21 * y0 + m22 * z0 + cam_dist

    if z <= 0.04:
        return None

    focal = 1.0 / math.tan(math.radians(fov_deg) * 0.5)
    ndc_x = (x * focal / z) / max(aspect, 0.001)
    ndc_y = y * focal / z
    px = (ndc_x * 0.5 + 0.5) * out_w
    py = (0.5 - ndc_y * 0.5) * out_h
    return px, py, z


def raster_line(
    points: list[int],
    depths: list[float],
    x0: float,
    y0: float,
    z0: float,
    x1: float,
    y1: float,
    z1: float,
) -> None:
    dx, dy = x1 - x0, y1 - y0
    steps = max(1, int(max(abs(dx), abs(dy))))
    inv = 1.0 / steps
    for i in range(steps + 1):
        t = i * inv
        x, y = int(x0 + dx * t), int(y0 + dy * t)
        if x < 0 or y < 0 or x >= RASTER_W:
            continue
        idx = y * RASTER_W + x
        if idx < 0 or idx >= len(points):
            continue
        z = z0 + (z1 - z0) * t
        if z < depths[idx] - 0.01:
            points[idx] = 1
            depths[idx] = z


def render_wire(
    mesh: Mesh,
    yaw: float,
    pitch: float,
    roll: float,
    cam_dist: float,
    view_w: int,
    view_h: int,
    fov: float,
    ascii_mode: bool,
    edge_limit: int,
    pan_x: float = 0.0,
    pan_y: float = 0.0,
) -> list[str]:
    global RASTER_W
    rw, rh = (view_w, view_h) if ascii_mode else (view_w * 2, view_h * 4)
    RASTER_W = rw

    points = bytearray(rw * rh)
    depths = [float("inf")] * (rw * rh)

    matrix = rotation_matrix(yaw, pitch, roll)
    # Braille's 2x4 dot cell is already close to the physical aspect ratio
    # of a normal terminal cell. ASCII mode needs an extra vertical correction.
    aspect = (rw / max(rh, 1)) if not ascii_mode else (rw / max(rh * 2.0, 1.0))
    projected = [
        project(v, matrix, cam_dist, rw, rh, fov, aspect)
        for v in mesh.render_vertices
    ]

    pan_px = (pan_x / max(mesh.radius, 1e-6)) * rw * 0.5
    pan_py = (pan_y / max(mesh.radius, 1e-6)) * rh * 0.5
    if pan_x or pan_y:
        projected = [
            None if p is None else (p[0] + pan_px, p[1] - pan_py, p[2])
            for p in projected
        ]

    for a, b in mesh.limited_edges(edge_limit):
        pa, pb = projected[a], projected[b]
        if pa is None or pb is None:
            continue
        raster_line(points, depths, pa[0], pa[1], pa[2], pb[0], pb[1], pb[2])

    if ascii_mode:
        return [
            "".join("." if points[y * rw + x] else " " for x in range(rw))
            for y in range(rh)
        ]

    rows: list[str] = []
    for cy in range(view_h):
        row: list[str] = []
        base_y = cy * 4 * rw
        for cx in range(view_w):
            mask = 0
            x0 = cx * 2
            for py, dy in enumerate((0, 1, 2, 3)):
                base = base_y + dy * rw + x0
                if points[base]:
                    mask |= BRAILLE_DOTS[(0, py)]
                if x0 + 1 < rw and points[base + 1]:
                    mask |= BRAILLE_DOTS[(1, py)]
            row.append(chr(0x2800 + mask) if mask else " ")
        rows.append("".join(row))
    return rows


def demo_mesh() -> Mesh:
    v = [
        (-1, -1, -1), (1, -1, -1), (1, 1, -1), (-1, 1, -1),
        (-1, -1, 1), (1, -1, 1), (1, 1, 1), (-1, 1, 1),
    ]
    f = [
        (0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6),
        (0, 4, 5), (0, 5, 1), (3, 2, 6), (3, 6, 7),
        (0, 3, 7), (0, 7, 4), (1, 5, 6), (1, 6, 2),
    ]
    return Mesh(v, f, "demo cube")


class Viewer:
    QUALITY_PRESETS = {
        "LOW": 0.28,
        "MED": 0.50,
        "HIGH": 1.00,
        "AUTO": 1.00,
    }

    def __init__(
        self,
        model: Mesh,
        ascii_mode: bool,
        fps: int,
        edge_limit: int,
        fov: float,
        quality: str = "AUTO",
    ):
        self.model = model
        self.ascii_mode = ascii_mode
        self.max_fps = fps
        self.base_edge_limit = max(500, edge_limit)
        self.min_auto_edges = min(self.base_edge_limit, 1800)
        self.max_auto_edges = self.base_edge_limit
        self.active_edge_limit = (
            min(self.base_edge_limit, 6500)
            if quality.upper() == "AUTO"
            else self.base_edge_limit
        )
        self.quality = quality.upper()
        self.fov = max(25.0, min(110.0, fov))
        self.yaw, self.pitch, self.roll = 0.45, -0.25, 0.0
        self.fit_distance = self._calculate_fit_distance()
        self.zoom = 1.0
        self.pan_x = self.pan_y = 0.0

        self.auto_rotate = False
        self.help = False
        self.theme = 0
        self.running = True

        self.last_fps = 0.0
        self.render_ms = 0.0
        self.frame_count = 0
        self.fps_stamp = time.monotonic()
        self.quality_stamp = self.fps_stamp
        self.quality_good_time = 0.0

        self.dirty = True
        self.cached_rows: list[str] = []
        self.cached_size = (0, 0)
        self.status = "READY"

        self.target_render_fps = 12.0

    def _calculate_fit_distance(self) -> float:
        half_fov = math.radians(self.fov) * 0.5
        fit = self.model.radius / max(math.tan(half_fov), 0.05)
        return max(self.model.radius * 1.10, fit * 1.12)

    def mark_dirty(self, reason: str = "UPDATED") -> None:
        self.dirty = True
        self.status = reason

    def reset(self) -> None:
        self.yaw, self.pitch, self.roll = 0.45, -0.25, 0.0
        self.zoom = 1.0
        self.pan_x = self.pan_y = 0.0
        self.mark_dirty("RESET")

    def cycle_quality(self) -> None:
        order = ("AUTO", "HIGH", "MED", "LOW")
        self.quality = order[(order.index(self.quality) + 1) % len(order)]
        if self.quality == "AUTO":
            self.active_edge_limit = min(self.base_edge_limit, 6500)
        else:
            self.active_edge_limit = max(
                500,
                int(self.base_edge_limit * self.QUALITY_PRESETS[self.quality]),
            )
        self.mark_dirty(f"QUALITY {self.quality}")

    def key(self, ch: int) -> None:
        changed = False

        if ch in (27, ord("q"), ord("Q")):
            self.running = False
            return
        if ch == curses.KEY_LEFT:
            self.yaw -= 0.08; changed = True
        elif ch == curses.KEY_RIGHT:
            self.yaw += 0.08; changed = True
        elif ch == curses.KEY_UP:
            self.pitch = max(-1.50, self.pitch - 0.08); changed = True
        elif ch == curses.KEY_DOWN:
            self.pitch = min(1.50, self.pitch + 0.08); changed = True
        elif ch in (ord("a"), ord("A")):
            self.pan_x -= 0.08 * self.model.radius; changed = True
        elif ch in (ord("d"), ord("D")):
            self.pan_x += 0.08 * self.model.radius; changed = True
        elif ch in (ord("w"), ord("W")):
            self.zoom = max(0.45, self.zoom * 0.92); changed = True
        elif ch in (ord("s"), ord("S")):
            self.zoom = min(8.0, self.zoom * 1.09); changed = True
        elif ch in (ord("z"), ord("Z")):
            self.roll -= 0.10; changed = True
        elif ch in (ord("c"), ord("C")):
            self.roll += 0.10; changed = True
        elif ch == ord(" "):
            self.auto_rotate = not self.auto_rotate
            self.mark_dirty("AUTO ROTATE" if self.auto_rotate else "PAUSED")
        elif ch in (ord("r"), ord("R")):
            self.reset()
            return
        elif ch == ord("1"):
            self.ascii_mode = not self.ascii_mode; changed = True
        elif ch == ord("2"):
            self.theme = (self.theme + 1) % 4; changed = True
        elif ch in (ord("k"), ord("K")):
            self.cycle_quality()
            return
        elif ch in (ord("h"), ord("H"), ord("?")):
            self.help = not self.help
            self.mark_dirty("HELP" if self.help else "READY")
            return

        if changed:
            self.mark_dirty("MOVING" if not self.auto_rotate else "AUTO")

    def init_colors(self) -> None:
        if not curses.has_colors():
            return
        curses.start_color()
        curses.use_default_colors()
        for pair, color in enumerate(
            (curses.COLOR_GREEN, curses.COLOR_WHITE, curses.COLOR_CYAN, curses.COLOR_YELLOW),
            1,
        ):
            curses.init_pair(pair, color, -1)

    def color_attr(self, pair_offset: int = 1, bold: bool = False) -> int:
        attr = curses.A_BOLD if bold else curses.A_NORMAL
        if curses.has_colors():
            attr |= curses.color_pair(pair_offset + self.theme)
        return attr

    def update_adaptive_quality(self, now: float) -> None:
        if self.quality != "AUTO" or now - self.quality_stamp < 1.25:
            return
        self.quality_stamp = now

        if self.last_fps < self.target_render_fps - 1.5:
            new_limit = max(self.min_auto_edges, int(self.active_edge_limit * 0.78))
            if new_limit < self.active_edge_limit:
                self.active_edge_limit = new_limit
                self.mark_dirty(f"AUTO {new_limit:,} EDGES")
            self.quality_good_time = 0.0
        elif self.last_fps > self.target_render_fps + 3.0:
            self.quality_good_time += 1.25
            if self.quality_good_time >= 3.0:
                new_limit = min(self.max_auto_edges, int(self.active_edge_limit * 1.15) + 1)
                if new_limit > self.active_edge_limit:
                    self.active_edge_limit = new_limit
                    self.mark_dirty(f"AUTO {new_limit:,} EDGES")
                self.quality_good_time = 0.0
        else:
            self.quality_good_time = 0.0

    def draw_help(self, stdscr: "curses.window", h: int, w: int) -> None:
        stdscr.erase()
        title = " T3B  /  HELP "
        try:
            stdscr.addnstr(0, 0, title.ljust(w), max(0, w - 1), self.color_attr(bold=True))
        except curses.error:
            pass

        lines = [
            "ARROWS  rotate        W/S  zoom",
            "A/D     pan           Z/C  roll",
            "SPACE   auto-rotate   K    quality: AUTO/HIGH/MED/LOW",
            "1       Braille/ASCII 2    colour theme",
            "R       reset + refit  H/?  toggle help",
            "Q/ESC   quit",
            "",
            "BRAILLE mode uses a 2x4 terminal dot grid for the finest wireframe.",
            "AUTO quality changes the edge budget to hold a practical FPS on small CPUs.",
            "",
            f"Model: {self.model.name}",
            f"Vertices: {len(self.model.vertices):,}",
            f"Triangles: {len(self.model.faces):,}",
            f"Topology edges: {len(self.model.edges):,}",
        ]

        for y, line in enumerate(lines, 2):
            if y >= h - 1:
                break
            try:
                stdscr.addnstr(y, 2, line, max(0, w - 3), self.color_attr())
            except curses.error:
                pass

    def render_frame(self, h: int, w: int) -> list[str]:
        view_h = max(5, h - 3)
        cam_dist = self.fit_distance * self.zoom
        started = time.monotonic()

        rows = render_wire(
            self.model,
            self.yaw,
            self.pitch,
            self.roll,
            cam_dist,
            w,
            view_h,
            self.fov,
            self.ascii_mode,
            self.active_edge_limit,
            self.pan_x,
            self.pan_y,
        )

        self.render_ms = (time.monotonic() - started) * 1000.0
        self.cached_rows = rows
        self.cached_size = (w, view_h)
        self.dirty = False
        return rows

    def draw(self, stdscr: "curses.window", h: int, w: int) -> None:
        stdscr.erase()
        view_h = max(5, h - 3)

        if self.dirty or self.cached_size != (w, view_h):
            self.render_frame(h, w)

        header = (
            f" T3B  //  {self.model.name} "
            f" //  {len(self.model.vertices):,}V {len(self.model.faces):,}F "
            f" //  {self.last_fps:4.1f} FPS  {self.render_ms:6.1f}ms "
        )
        quality = (
            f" MODE:{'ASCII' if self.ascii_mode else 'BRAILLE'}"
            f"  QUALITY:{self.quality}"
            f"  EDGES:{self.active_edge_limit:,}/{len(self.model.edges):,}"
            f"  {'AUTO' if self.auto_rotate else self.status}"
        )

        if h >= 12:
            try:
                stdscr.addnstr(0, 0, header.ljust(w), max(0, w - 1), self.color_attr(bold=True))
            except curses.error:
                pass

        for y, row in enumerate(self.cached_rows[:view_h], 1):
            try:
                stdscr.addnstr(y, 0, row, max(0, w - 1), self.color_attr())
            except curses.error:
                pass

        try:
            stdscr.addnstr(h - 2, 0, quality.ljust(w), max(0, w - 1), self.color_attr(bold=True))
            controls = " ARROWS rotate  W/S zoom  A/D pan  Z/C roll  SPACE auto  K quality  1 mode  H help  Q quit "
            stdscr.addnstr(h - 1, 0, controls.ljust(w), max(0, w - 1), self.color_attr(bold=True))
        except curses.error:
            pass

    def run(self, stdscr: "curses.window") -> None:
        self.init_colors()
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.keypad(True)

        frame_time = 1.0 / self.max_fps
        previous = time.monotonic()

        while self.running:
            loop_start = time.monotonic()
            h, w = stdscr.getmaxyx()

            while True:
                ch = stdscr.getch()
                if ch == -1:
                    break
                self.key(ch)

            now = time.monotonic()
            dt = min(0.1, now - previous)
            previous = now

            if self.auto_rotate and not self.help:
                self.yaw += dt * 0.85
                self.dirty = True

            self.update_adaptive_quality(now)

            if h < 10 or w < 42:
                stdscr.erase()
                msg = "T3B needs at least 42x10 terminal cells."
                try:
                    stdscr.addnstr(0, 0, msg, max(0, w - 1), self.color_attr(bold=True))
                except curses.error:
                    pass
                stdscr.refresh()
                time.sleep(0.1)
                continue

            if self.help:
                self.draw_help(stdscr, h, w)
            else:
                self.draw(stdscr, h, w)

            stdscr.refresh()

            elapsed = time.monotonic() - loop_start
            if frame_time > elapsed:
                time.sleep(frame_time - elapsed)

            self.frame_count += 1
            if now - self.fps_stamp >= 0.5:
                self.last_fps = self.frame_count / (now - self.fps_stamp)
                self.frame_count = 0
                self.fps_stamp = now



def main() -> int:
    locale.setlocale(locale.LC_ALL, "")
    p = argparse.ArgumentParser(description="T3B - lightweight terminal 3D model viewer")
    p.add_argument("model", nargs="?", type=Path, help="3D model file. Omit for the built-in cube.")
    p.add_argument("--ascii", action="store_true", help="Use classic one-character ASCII dots instead of Braille.")
    p.add_argument("--fps", type=int, default=30, choices=range(5, 61), metavar="5-60")
    p.add_argument("--edges", type=int, default=14000, help="Maximum wireframe edges available to the quality system.")
    p.add_argument(
        "--quality",
        choices=("auto", "high", "med", "low"),
        default="auto",
        help="Wireframe detail policy (default: auto).",
    )
    p.add_argument("--fov", type=float, default=70.0, help="Vertical field of view in degrees.")
    args = p.parse_args()

    try:
        mesh = load_model(args.model) if args.model else demo_mesh()
        viewer = Viewer(mesh, args.ascii, args.fps, args.edges, args.fov, args.quality.upper())
        curses.wrapper(viewer.run)
        return 0
    except KeyboardInterrupt:
        return 130
    except T3BError as exc:
        print(f"T3B: {exc}", file=sys.stderr)
        return 2
    except curses.error as exc:
        print(f"T3B: terminal error: {exc}", file=sys.stderr)
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
