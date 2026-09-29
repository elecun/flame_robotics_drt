#!/usr/bin/env python3
"""Export a URDF's visual geometry at its zero joint configuration to STL."""

import argparse
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


def values(text, count):
    return np.array([float(x) for x in (text or "").split()] or [0.] * count)


def transform(origin):
    xyz = values(origin.get("xyz") if origin is not None else "", 3)
    roll, pitch, yaw = values(origin.get("rpy") if origin is not None else "", 3)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = rotation, xyz
    return result


def box(size):
    x, y, z = np.array(size) / 2
    vertices = np.array([[a, b, c] for a in (-x, x) for b in (-y, y) for c in (-z, z)])
    faces = ((0, 4, 6), (0, 6, 2), (1, 3, 7), (1, 7, 5), (0, 1, 5), (0, 5, 4),
             (2, 6, 7), (2, 7, 3), (0, 2, 3), (0, 3, 1), (4, 5, 7), (4, 7, 6))
    return vertices[np.array(faces)]


def cylinder(radius, length, sides=32):
    angles = np.arange(sides) * (2 * math.pi / sides)
    ring = np.column_stack((radius * np.cos(angles), radius * np.sin(angles)))
    vertices = np.vstack((np.column_stack((ring, np.full(sides, -length / 2))),
                          np.column_stack((ring, np.full(sides, length / 2))),
                          [[0, 0, -length / 2], [0, 0, length / 2]]))
    faces = []
    for i in range(sides):
        j = (i + 1) % sides
        faces += [(i, j, sides + j), (i, sides + j, sides + i),
                  (2 * sides, j, i), (2 * sides + 1, sides + i, sides + j)]
    return vertices[np.array(faces)]


def stl_triangles(path):
    data = path.read_bytes()
    if len(data) >= 84 and len(data) == 84 + 50 * struct.unpack_from("<I", data, 80)[0]:
        count = struct.unpack_from("<I", data, 80)[0]
        triangles = np.empty((count, 3, 3), dtype=float)
        for i in range(count):
            triangles[i] = np.array(struct.unpack_from("<9f", data, 84 + i * 50)).reshape(3, 3)
        return triangles
    rows = []
    for line in data.decode("utf-8", errors="ignore").splitlines():
        fields = line.split()
        if len(fields) == 4 and fields[0].lower() == "vertex":
            rows.append([float(v) for v in fields[1:]])
    if len(rows) % 3:
        raise ValueError(f"Invalid ASCII STL: {path}")
    return np.array(rows, dtype=float).reshape(-1, 3, 3)


def apply(matrix, triangles):
    points = triangles.reshape(-1, 3)
    return (points @ matrix[:3, :3].T + matrix[:3, 3]).reshape(-1, 3, 3)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("urdf", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--scale", type=float, default=1.0,
                        help="Uniform scale factor for exported coordinates (default: 1).")
    args = parser.parse_args()
    root = ET.parse(args.urdf).getroot()
    joints = {j.find("child").get("link"): (j.find("parent").get("link"), transform(j.find("origin")))
              for j in root.findall("joint")}
    child_links = set(joints)
    root_links = [link.get("name") for link in root.findall("link")
                  if link.get("name") not in child_links]
    if len(root_links) != 1:
        raise ValueError(f"Expected one URDF root link, found: {root_links}")
    poses = {root_links[0]: np.eye(4)}
    while len(poses) <= len(joints):
        progressed = False
        for child, (parent, local) in joints.items():
            if child not in poses and parent in poses:
                poses[child] = poses[parent] @ local
                progressed = True
        if not progressed:
            break
    all_triangles = []
    for link in root.findall("link"):
        link_pose = poses.get(link.get("name"), np.eye(4))
        for visual in link.findall("visual"):
            geometry = visual.find("geometry")
            local = transform(visual.find("origin"))
            shape = geometry.find("box")
            if shape is not None:
                triangles = box(values(shape.get("size"), 3))
            else:
                shape = geometry.find("cylinder")
                if shape is not None:
                    triangles = cylinder(float(shape.get("radius")), float(shape.get("length")))
                else:
                    shape = geometry.find("mesh")
                    if shape is None:
                        continue
                    triangles = stl_triangles(args.urdf.parent / shape.get("filename"))
                    triangles *= values(shape.get("scale"), 3)
            all_triangles.append(apply(link_pose @ local, triangles))
    triangles = np.concatenate(all_triangles)
    triangles *= args.scale
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    valid = lengths > 1e-12
    triangles, normals, lengths = triangles[valid], normals[valid], lengths[valid]
    normals /= lengths[:, None]
    with args.output.open("wb") as stream:
        stream.write(b"URDF visual geometry (zero joint configuration)".ljust(80, b" "))
        stream.write(struct.pack("<I", len(triangles)))
        for normal, triangle in zip(normals, triangles):
            stream.write(struct.pack("<12fH", *normal, *triangle.ravel(), 0))
    print(f"Wrote {len(triangles):,} triangles to {args.output}")


if __name__ == "__main__":
    main()
