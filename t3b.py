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

    @staticmethod
    def _make_edges(faces: list[Face]) -> list[tuple[int, int]]:
        edges: set[tuple[int, int]] = set()
        for a, b, c in faces:
            for u, v in ((a, b), (b, c), (c, a)):
                if u != v:
                    edges.add((min(u, v), max(u, v)))
        return list(edges)

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
        step = len(self.edges) / limit
        return [self.edges[min(len(self.edges) - 1, int(i * step))] for i in range(limit)]


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


def rotate(v: Vec3, yaw: float, pitch: float, roll: float) -> Vec3:
    x, y, z = v
    cy, sy = math.cos(yaw), math.sin(yaw)
    x, z = x * cy - z * sy, x * sy + z * cy
    cp, sp = math.cos(pitch), math.sin(pitch)
    y, z = y * cp - z * sp, y * sp + z * cp
    cr, sr = math.cos(roll), math.sin(roll)
    x, y = x * cr - y * sr, x * sr + y * cr
    return x, y, z


def project(
    v: Vec3,
    yaw: float,
    pitch: float,
    roll: float,
    cam_dist: float,
    out_w: int,
    out_h: int,
    fov_deg: float,
) -> tuple[float, float, float] | None:
    x, y, z = rotate(v, yaw, pitch, roll)
    z += cam_dist
    if z <= 0.04:
        return None

    cell_height = 2.0
    physical_h = max(1.0, out_h * cell_height)
    aspect = out_w / physical_h
    focal = 1.0 / math.tan(math.radians(fov_deg) * 0.5)

    ndc_x = (x * focal / z) / max(aspect, 0.001)
    ndc_y = y * focal / z
    px = (ndc_x * 0.5 + 0.5) * out_w
    py = ((0.5 - ndc_y * 0.5) * physical_h) / cell_height
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
        if x < 0 or y < 0:
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
    points = [0] * (rw * rh)
    depths = [float("inf")] * (rw * rh)

    projected = [project(v, yaw, pitch, roll, cam_dist, rw, rh, fov) for v in mesh.vertices]
    pan_px = (pan_x / max(mesh.radius, 1e-6)) * rw * 0.5
    pan_py = (pan_y / max(mesh.radius, 1e-6)) * rh * 0.5
    if pan_x or pan_y:
        projected = [
            None if p is None else (p[0] + pan_px, p[1] - pan_py, p[2])
            for p in projected
        ]
    for a, b in mesh.limited_edges(edge_limit):
        pa, pb = projected[a], projected[b]
        if pa is not None and pb is not None:
            raster_line(points, depths, pa[0], pa[1], pa[2], pb[0], pb[1], pb[2])

    if ascii_mode:
        return [
            "".join("." if points[y * rw + x] else " " for x in range(rw))
            for y in range(rh)
        ]

    rows: list[str] = []
    for cy in range(view_h):
        row: list[str] = []
        for cx in range(view_w):
            mask = 0
            for py in range(4):
                base = (cy * 4 + py) * rw + cx * 2
                if points[base]:
                    mask |= BRAILLE_DOTS[(0, py)]
                if base + 1 < len(points) and points[base + 1]:
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
    def __init__(self, model: Mesh, ascii_mode: bool, fps: int, edge_limit: int, fov: float):
        self.model = model
        self.ascii_mode = ascii_mode
        self.max_fps = fps
        self.edge_limit = max(100, edge_limit)
        self.fov = max(25.0, min(110.0, fov))
        self.yaw, self.pitch, self.roll = 0.45, -0.25, 0.0
        self.zoom = 3.0
        self.pan_x = self.pan_y = 0.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.auto_rotate = False
        self.help = False
        self.theme = 0
        self.last_fps = 0.0
        self.frame_count = 0
        self.fps_stamp = time.monotonic()
        self.running = True

    def reset(self) -> None:
        self.yaw, self.pitch, self.roll = 0.45, -0.25, 0.0
        self.zoom = 3.0

    def key(self, ch: int) -> None:
        if ch in (27, ord("q"), ord("Q")):
            self.running = False
        elif ch == curses.KEY_LEFT:
            self.yaw -= 0.08
        elif ch == curses.KEY_RIGHT:
            self.yaw += 0.08
        elif ch == curses.KEY_UP:
            self.pitch = max(-1.50, self.pitch - 0.08)
        elif ch == curses.KEY_DOWN:
            self.pitch = min(1.50, self.pitch + 0.08)
        elif ch in (ord("a"), ord("A")):
            self.pan_x -= 0.08 * self.model.radius
        elif ch in (ord("d"), ord("D")):
            self.pan_x += 0.08 * self.model.radius
        elif ch in (ord("w"), ord("W")):
            self.zoom = max(1.1, self.zoom * 0.92)
        elif ch in (ord("s"), ord("S")):
            self.zoom = min(25.0, self.zoom * 1.09)
        elif ch in (ord("z"), ord("Z")):
            self.roll -= 0.10
        elif ch in (ord("c"), ord("C")):
            self.roll += 0.10
        elif ch == ord(" "):
            self.auto_rotate = not self.auto_rotate
        elif ch in (ord("r"), ord("R")):
            self.reset()
        elif ch == ord("1"):
            self.ascii_mode = not self.ascii_mode
        elif ch == ord("2"):
            self.theme = (self.theme + 1) % 4
        elif ch in (ord("h"), ord("H"), ord("?")):
            self.help = not self.help

    def init_colors(self) -> None:
        if not curses.has_colors():
            return
        curses.start_color()
        curses.use_default_colors()
        for pair, color in enumerate(
            (curses.COLOR_GREEN, curses.COLOR_WHITE, curses.COLOR_CYAN, curses.COLOR_YELLOW), 1
        ):
            curses.init_pair(pair, color, -1)

    def run(self, stdscr: "curses.window") -> None:
        self.init_colors()
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.keypad(True)
        frame_time = 1.0 / self.max_fps
        previous = time.monotonic()

        while self.running:
            start = time.monotonic()
            h, w = stdscr.getmaxyx()
            while True:
                ch = stdscr.getch()
                if ch == -1:
                    break
                self.key(ch)

            now = time.monotonic()
            dt = min(0.1, now - previous)
            previous = now
            if self.auto_rotate:
                self.yaw += dt * 0.85

            if h < 8 or w < 30:
                stdscr.erase()
                try:
                    stdscr.addnstr(0, 0, "T3B needs at least 30x8 terminal cells. Resize me.", max(0, w - 1))
                except curses.error:
                    pass
            elif self.help:
                self.draw_help(stdscr, h, w)
            else:
                self.draw(stdscr, h, w)

            stdscr.refresh()
            elapsed = time.monotonic() - start
            if frame_time > elapsed:
                time.sleep(frame_time - elapsed)

            self.frame_count += 1
            if now - self.fps_stamp >= 0.5:
                self.last_fps = self.frame_count / (now - self.fps_stamp)
                self.frame_count = 0
                self.fps_stamp = now

    def draw_help(self, stdscr: "curses.window", h: int, w: int) -> None:
        stdscr.erase()
        lines = [
            "T3B — TERMINAL 3D VIEWER",
            "",
            "Arrow keys   rotate model",
            "W / S        zoom in / out",
            "Z / C        roll",
            "SPACE        auto-rotate",
            "R            reset view",
            "1            toggle Braille / ASCII",
            "2            cycle terminal colour",
            "H / ?        this help",
            "Q / ESC      quit",
            "",
            f"Model: {self.model.name}",
            f"Vertices: {len(self.model.vertices):,}",
            f"Triangles: {len(self.model.faces):,}",
            f"Edges: {len(self.model.edges):,} (showing up to {self.edge_limit:,})",
        ]
        attr = curses.A_BOLD | (curses.color_pair(1 + self.theme) if curses.has_colors() else 0)
        for y, line in enumerate(lines):
            if y >= h:
                break
            try:
                stdscr.addnstr(y, 0, line, max(0, w - 1), attr if y == 0 else 0)
            except curses.error:
                pass

    def draw(self, stdscr: "curses.window", h: int, w: int) -> None:
        stdscr.erase()
        view_h = h - 2
        cam_dist = max(1.25, self.zoom) * self.model.radius
        rows = render_wire(
            self.model, self.yaw, self.pitch, self.roll, cam_dist,
            w, view_h, self.fov, self.ascii_mode, self.edge_limit,
            self.pan_x, self.pan_y,
        )
        attr = curses.A_NORMAL | (curses.color_pair(1 + self.theme) if curses.has_colors() else 0)
        for y, row in enumerate(rows):
            try:
                stdscr.addnstr(y, 0, row, max(0, w - 1), attr)
            except curses.error:
                pass

        mode = "ASCII" if self.ascii_mode else "BRAILLE"
        auto = "AUTO" if self.auto_rotate else "MANUAL"
        hud = (
            f" T3B | {self.model.name} | {len(self.model.vertices):,}V "
            f"{len(self.model.faces):,}F {len(self.model.edges):,}E | "
            f"{self.last_fps:4.1f} FPS | {mode} | {auto} "
        )
        controls = " ARROWS rotate  W/S zoom  Z/C roll  SPACE auto  1 mode  2 colour  H help  Q quit "
        hud_attr = curses.A_BOLD | (curses.color_pair(1 + self.theme) if curses.has_colors() else 0)
        try:
            stdscr.addnstr(h - 2, 0, hud.ljust(w), max(0, w - 1), hud_attr)
            stdscr.addnstr(h - 1, 0, controls.ljust(w), max(0, w - 1), hud_attr)
        except curses.error:
            pass


def main() -> int:
    locale.setlocale(locale.LC_ALL, "")
    p = argparse.ArgumentParser(description="T3B - lightweight terminal 3D model viewer")
    p.add_argument("model", nargs="?", type=Path, help="3D model file. Omit for the built-in cube.")
    p.add_argument("--ascii", action="store_true", help="Use classic one-character ASCII dots instead of Braille.")
    p.add_argument("--fps", type=int, default=30, choices=range(5, 61), metavar="5-60")
    p.add_argument("--edges", type=int, default=18000, help="Maximum visible wireframe edges.")
    p.add_argument("--fov", type=float, default=70.0, help="Vertical field of view in degrees.")
    args = p.parse_args()

    try:
        mesh = load_model(args.model) if args.model else demo_mesh()
        viewer = Viewer(mesh, args.ascii, args.fps, args.edges, args.fov)
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
