"""
將 Gmsh 2.2 格式 (.msh) 轉為 TetMeshJson 格式 (.json)
用法: python convert_msh_to_json.py liver3-HD.msh liver3-HD.json [--scale 5.0]

TetMeshJson 格式:
{
  "verts": [x0,y0,z0, x1,y1,z1, ...],      # flat float array
  "tetIds": [a,b,c,d, ...],                  # flat int array (0-based)
  "tetSurfaceTriIds": [a,b,c, ...]           # flat int array (boundary faces)
}
"""
import sys
import json
import argparse
from collections import defaultdict


def parse_msh(filepath):
    """Parse Gmsh 2.2 ASCII format."""
    with open(filepath, 'r') as f:
        lines = f.readlines()

    # Strip \r\n
    lines = [l.strip() for l in lines]

    nodes = {}
    elements = []

    i = 0
    while i < len(lines):
        line = lines[i]

        if line == '$Nodes':
            i += 1
            num_nodes = int(lines[i])
            i += 1
            for _ in range(num_nodes):
                parts = lines[i].split()
                nid = int(parts[0])
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                nodes[nid] = (x, y, z)
                i += 1
            # skip $EndNodes
            i += 1
            continue

        if line == '$Elements':
            i += 1
            num_elems = int(lines[i])
            i += 1
            for _ in range(num_elems):
                parts = lines[i].split()
                eid = int(parts[0])
                etype = int(parts[1])
                num_tags = int(parts[2])
                # node indices start after tags
                node_start = 3 + num_tags
                node_ids = [int(parts[j]) for j in range(node_start, len(parts))]
                elements.append((etype, node_ids))
                i += 1
            # skip $EndElements
            i += 1
            continue

        i += 1

    return nodes, elements


def extract_boundary_faces(tet_indices, num_verts):
    """Extract boundary triangles (faces appearing exactly once)."""
    face_count = defaultdict(int)
    face_order = {}

    # Each tet has 4 faces
    tet_faces = [(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)]

    num_tets = len(tet_indices) // 4
    for t in range(num_tets):
        base = t * 4
        tv = [tet_indices[base], tet_indices[base+1],
              tet_indices[base+2], tet_indices[base+3]]

        for face in tet_faces:
            f = [tv[face[0]], tv[face[1]], tv[face[2]]]
            sorted_f = tuple(sorted(f))
            face_count[sorted_f] += 1
            if sorted_f not in face_order:
                face_order[sorted_f] = f

    boundary = []
    for key, count in face_count.items():
        if count == 1:
            f = face_order[key]
            boundary.extend(f)

    return boundary


def main():
    parser = argparse.ArgumentParser(description='Convert Gmsh .msh to TetMeshJson .json')
    parser.add_argument('input', help='Input .msh file')
    parser.add_argument('output', help='Output .json file')
    parser.add_argument('--scale', type=float, default=1.0,
                        help='Scale factor for vertex coordinates (default: 1.0)')
    parser.add_argument('--center', action='store_true',
                        help='Center the mesh at origin before scaling')
    args = parser.parse_args()

    print(f"Parsing {args.input}...")
    nodes, elements = parse_msh(args.input)

    # Build vertex mapping (Gmsh node IDs are 1-based, may have gaps)
    sorted_nids = sorted(nodes.keys())
    nid_to_idx = {nid: idx for idx, nid in enumerate(sorted_nids)}
    num_verts = len(sorted_nids)

    # Flatten vertices
    verts = []
    for nid in sorted_nids:
        x, y, z = nodes[nid]
        verts.extend([x, y, z])

    # Center mesh if requested
    if args.center:
        cx = sum(verts[i] for i in range(0, len(verts), 3)) / num_verts
        cy = sum(verts[i] for i in range(1, len(verts), 3)) / num_verts
        cz = sum(verts[i] for i in range(2, len(verts), 3)) / num_verts
        for i in range(num_verts):
            verts[i*3]   -= cx
            verts[i*3+1] -= cy
            verts[i*3+2] -= cz
        print(f"Centered mesh (was at {cx:.3f}, {cy:.3f}, {cz:.3f})")

    # Apply scale
    if args.scale != 1.0:
        verts = [v * args.scale for v in verts]
        print(f"Applied scale: {args.scale}")

    # Extract tetrahedra (type 4 in Gmsh)
    tet_ids = []
    num_tets = 0
    for etype, node_ids in elements:
        if etype == 4:  # 4-node tetrahedron
            for nid in node_ids:
                tet_ids.append(nid_to_idx[nid])
            num_tets += 1

    print(f"Vertices: {num_verts}")
    print(f"Tetrahedra: {num_tets}")

    # Extract boundary faces
    surface_tris = extract_boundary_faces(tet_ids, num_verts)
    num_surface_tris = len(surface_tris) // 3
    print(f"Boundary triangles: {num_surface_tris}")

    # Compute bounding box
    xs = verts[0::3]
    ys = verts[1::3]
    zs = verts[2::3]
    print(f"Bounding box:")
    print(f"  X: [{min(xs):.3f}, {max(xs):.3f}]")
    print(f"  Y: [{min(ys):.3f}, {max(ys):.3f}]")
    print(f"  Z: [{min(zs):.3f}, {max(zs):.3f}]")

    # Build JSON
    data = {
        "verts": verts,
        "tetIds": tet_ids,
        "tetSurfaceTriIds": surface_tris,
    }

    print(f"Writing {args.output}...")
    with open(args.output, 'w') as f:
        json.dump(data, f, separators=(',', ':'))

    file_size = len(json.dumps(data, separators=(',', ':')))
    print(f"Done! File size: {file_size / 1024:.1f} KB")


if __name__ == '__main__':
    main()
