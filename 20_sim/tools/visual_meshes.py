"""Mesh helpers for the CAD look in Viam's 3D scene (Phase V, decision #30). numpy only, no Viam.

Parts are (vertices mm Nx3, faces Mx3 int, rgb 0..1). Files are written in METRES, as the 3D scene
expects for both arm models (GLB) and world state store meshes (PLY):
  - write_glb: one GLB with one primitive per part (own colour). No normals, so the scene shades flat.
  - write_ply: one binary PLY, colour per vertex (parts merged).
"""

import json
import math
import struct

import numpy as np


# ---- primitives (MuJoCo sizes, mm) --------------------------------------------------------------

def box(half):
    hx, hy, hz = half
    v = np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float)
    f = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                  [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
    return v, f


def cylinder(r, half_h, n=32):
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    ring = np.c_[r * np.cos(a), r * np.sin(a)]
    v = np.vstack([np.c_[ring, np.full(n, -half_h)], np.c_[ring, np.full(n, half_h)], [[0, 0, -half_h], [0, 0, half_h]]])
    f = []
    for i in range(n):
        j = (i + 1) % n
        f += [[i, j, n + j], [i, n + j, n + i], [2 * n, j, i], [2 * n + 1, n + i, n + j]]
    return v, np.array(f)


def sphere(r, n=16):
    v, f = [], []
    for i in range(n + 1):
        th = math.pi * i / n
        for k in range(2 * n):
            ph = math.pi * k / n
            v.append([r * math.sin(th) * math.cos(ph), r * math.sin(th) * math.sin(ph), r * math.cos(th)])
    w = 2 * n
    for i in range(n):
        for k in range(w):
            a, b = i * w + k, i * w + (k + 1) % w
            f += [[a, a + w, b + w], [a, b + w, b]]
    return np.array(v), np.array(f)


def capsule(r, half_h, n=16):
    v, f = sphere(r, n)
    v = v.copy()
    v[:, 2] += np.where(v[:, 2] >= 0, half_h, -half_h)
    return v, f


def transform(v, t44):
    return (np.c_[v, np.ones(len(v))] @ t44.T)[:, :3]


# ---- writers ------------------------------------------------------------------------------------

def write_glb(parts, path):
    """parts: [(verts_mm, faces, rgb)] -> GLB in metres, one mesh, one primitive + material per part."""
    bin_chunks, views, accessors, prims, materials = [], [], [], [], []
    offset = 0

    def add(data, target, comp, kind, count, extra=None):
        nonlocal offset
        raw = data.tobytes()
        raw += b"\0" * ((4 - len(raw) % 4) % 4)
        views.append({"buffer": 0, "byteOffset": offset, "byteLength": len(data.tobytes()), "target": target})
        bin_chunks.append(raw)
        offset += len(raw)
        acc = {"bufferView": len(views) - 1, "componentType": comp, "count": count, "type": kind}
        acc.update(extra or {})
        accessors.append(acc)
        return len(accessors) - 1

    for verts, faces, rgb in parts:
        p = (np.asarray(verts, float) / 1000.0).astype(np.float32)
        i = np.asarray(faces, np.uint32).reshape(-1)
        pos = add(p, 34962, 5126, "VEC3", len(p), {"min": p.min(axis=0).tolist(), "max": p.max(axis=0).tolist()})
        idx = add(i, 34963, 5125, "SCALAR", len(i))
        materials.append({"pbrMetallicRoughness": {"baseColorFactor": [*map(float, rgb), 1.0],
                                                   "metallicFactor": 0.1, "roughnessFactor": 0.6},
                          "doubleSided": True})
        prims.append({"attributes": {"POSITION": pos}, "indices": idx, "material": len(materials) - 1})

    blob = b"".join(bin_chunks)
    gltf = {"asset": {"version": "2.0", "generator": "viam_lab build_viam_files.py"},
            "scene": 0, "scenes": [{"nodes": [0]}], "nodes": [{"mesh": 0}],
            "meshes": [{"primitives": prims}], "materials": materials,
            "buffers": [{"byteLength": len(blob)}], "bufferViews": views, "accessors": accessors}
    js = json.dumps(gltf, separators=(",", ":")).encode()
    js += b" " * ((4 - len(js) % 4) % 4)
    out = struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(js) + 8 + len(blob))
    out += struct.pack("<II", len(js), 0x4E4F534A) + js
    out += struct.pack("<II", len(blob), 0x004E4942) + blob
    path.write_bytes(out)
    return len(out)


def write_ply(parts, path):
    """parts: [(verts_mm, faces, rgb)] -> one binary PLY in metres, colour per vertex."""
    vs, fs, cs, base = [], [], [], 0
    for verts, faces, rgb in parts:
        vs.append(np.asarray(verts, float) / 1000.0)
        fs.append(np.asarray(faces, int) + base)
        cs.append(np.tile(np.clip(np.round(np.asarray(rgb) * 255), 0, 255).astype(np.uint8), (len(verts), 1)))
        base += len(verts)
    v, f, c = np.vstack(vs), np.vstack(fs), np.vstack(cs)
    head = (f"ply\nformat binary_little_endian 1.0\nelement vertex {len(v)}\n"
            "property float x\nproperty float y\nproperty float z\n"
            "property uchar red\nproperty uchar green\nproperty uchar blue\n"
            f"element face {len(f)}\nproperty list uchar int vertex_indices\nend_header\n").encode()
    vert = np.zeros(len(v), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("r", "u1"), ("g", "u1"), ("b", "u1")])
    vert["x"], vert["y"], vert["z"] = v[:, 0], v[:, 1], v[:, 2]
    vert["r"], vert["g"], vert["b"] = c[:, 0], c[:, 1], c[:, 2]
    face = np.zeros(len(f), dtype=[("n", "u1"), ("i", "<i4", (3,))])
    face["n"], face["i"] = 3, f
    data = head + vert.tobytes() + face.tobytes()
    path.write_bytes(data)
    return len(data)
