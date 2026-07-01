import gmsh
import numpy as np
import json
import os
import sys
import argparse
from collections import defaultdict
from itertools import combinations


def analyze_mesh(input_path: str) -> None:
    gmsh.initialize()
    gmsh.open(input_path)

    print("\n" + "=" * 60)
    print(f"MESH ANALYSIS: {input_path}")
    print("=" * 60)

    entities = gmsh.model.getEntities()
    dims = {}
    for dim, tag in entities:
        dims.setdefault(dim, []).append(tag)

    dim_names = {0: "Points", 1: "Curves", 2: "Surfaces", 3: "Volumes"}
    print(f"\nEntities: {len(entities)}")
    for dim in sorted(dims.keys()):
        print(f"  {dim_names.get(dim, f'Dim {dim}')}: {len(dims[dim])}")

    physical_groups = gmsh.model.getPhysicalGroups()
    print(f"\nPhysical Groups: {len(physical_groups)}")
    for dim, ptag in physical_groups:
        name = gmsh.model.getPhysicalName(dim, ptag)
        ents = gmsh.model.getEntitiesForPhysicalGroup(dim, ptag)
        print(f"  '{name}' (phys_tag={ptag}, dim={dim}): {len(ents)} entities")

    print(f"\nMesh:")
    print(f"  Nodes: {len(gmsh.model.mesh.getNodes()[0])}")
    total = 0
    for etype in gmsh.model.mesh.getElementTypes():
        ename = gmsh.model.mesh.getElementProperties(etype)[0]
        n = len(gmsh.model.mesh.getElementsByType(etype)[0])
        total += n
        print(f"  {ename}: {n}")
    print(f"  Total: {total}")

    views = gmsh.view.getTags()
    print(f"\nViews: {len(views)}")
    for vt in views:
        vname = gmsh.option.getString(f'View[{gmsh.view.getIndex(vt)}].Name')
        print(f"  View {vt}: {vname}")

    print("=" * 60 + "\n")
    gmsh.finalize()


def extract_boundary_selection_from_groups(
        input_path: str,
        boundary_output: str,
        selections: list,
        is_2d: bool = False,
        output_mesh: str = None,
        output_groups: list = None,
        skip_tags: list = None) -> None:
    """Extract interface boundary selection from a physical-groups .msh file.

    For each selection S of group names, finds all interior mesh faces (edges
    in 2D) that lie on the boundary of S using the boundary-of-S rule: a face
    between adjacent tag-sets A and B is selected iff
        S ⊆ (A ∪ B)  AND  NOT (S ⊆ (A ∩ B))
    i.e. S's coverage spans the face but is not present on both sides.

    Faces on the outer mesh boundary (only one adjacent tet) are excluded.
    Tets sharing a node-set across multiple groups (WMTK's write_msh_groups
    duplicate-tet pattern) are merged into a single logical tet whose tag-set
    is the union of those groups, so interface faces aren't dropped as
    non-manifold.

    If output_mesh + output_groups are provided, a stripped mesh is written
    containing only the listed groups, with nodes renumbered to be consecutive.
    The boundary selection node IDs are consistent with this stripped mesh.

    Output format (0-indexed node IDs):
        {id} v1 v2 v3    (3D facets)
        {id} v1 v2       (2D edges)

    Args:
        input_path:    Path to _groups.msh.
        boundary_output: Output boundary selection .txt path.
        selections:    List of {"groups": [str, ...], "id": int}, each with 2+ groups.
        is_2d:         True for 2D meshes (Triangle / edge boundaries).
        output_mesh:   Optional path for stripped output .msh.
        output_groups: Required with output_mesh. List of {"group": str, "id": int}.
        skip_tags:     Group names to skip if not referenced in any selection or
                       output_group (default: ["ambient"]).
    """
    if not selections:
        print("Warning: no selections specified, nothing to do.")
        return
    if output_mesh and not output_groups:
        raise ValueError("output_groups is required when output_mesh is specified")

    skip_tags = set(skip_tags) if skip_tags else {"ambient"}

    mesh_dim = 2 if is_2d else 3
    target_elem_name = "Triangle" if is_2d else "Tetra"
    nodes_per_face = 2 if is_2d else 3
    boundary_type = "edges" if is_2d else "facets"

    # All group names we actually need to load
    needed_groups = set(g for sel in selections for g in sel["groups"])
    if output_groups:
        needed_groups |= {og["group"] for og in output_groups}

    # ------------------------------------------------------------------ #
    # 1. Load mesh from gmsh
    # ------------------------------------------------------------------ #
    gmsh.initialize()
    gmsh.open(input_path)

    physical_groups = gmsh.model.getPhysicalGroups(mesh_dim)
    name_to_entities = {}
    for dim, ptag in physical_groups:
        name = gmsh.model.getPhysicalName(dim, ptag)
        name_to_entities[name] = list(gmsh.model.getEntitiesForPhysicalGroup(dim, ptag))

    unused_skipped = [n for n in name_to_entities if n in skip_tags and n not in needed_groups]
    if unused_skipped:
        print(f"  Skipping unused groups: {unused_skipped}")
    print(f"  Loading groups: {sorted(needed_groups & set(name_to_entities))}")

    # Find volume element type
    elem_type = None
    for etype in gmsh.model.mesh.getElementTypes():
        if target_elem_name in gmsh.model.mesh.getElementProperties(etype)[0]:
            elem_type = etype
            break
    if elem_type is None:
        print(f"Error: no {target_elem_name} elements found")
        gmsh.finalize()
        return
    nodes_per_elem = gmsh.model.mesh.getElementProperties(elem_type)[3]

    # Load all node coords
    node_tags_all, coords_flat, _ = gmsh.model.mesh.getNodes()
    all_coords = np.array(coords_flat, dtype=float).reshape(-1, 3)
    orig_node_id_to_row = {int(t): i for i, t in enumerate(node_tags_all)}

    # Load elements per group, build tet_tag → tag_set and tet_tag → nodes
    tet_to_tags = defaultdict(set)   # elem_tag → set of group names
    tet_to_nodes = {}                # elem_tag → array of orig node tags
    group_to_etags = {}              # group name → elem_tag array (for mesh writing)
    group_to_ntags = {}              # group name → Nx(nodes_per_elem) node array

    for name in needed_groups:
        if name not in name_to_entities:
            print(f"  Warning: group '{name}' not found in mesh")
            continue
        etag_rows, ntag_rows = [], []
        for entity in name_to_entities[name]:
            try:
                etags, ntags = gmsh.model.mesh.getElementsByType(elem_type, entity)
            except Exception:
                continue
            if len(etags) == 0:
                continue
            etag_rows.append(np.array(etags, dtype=np.int64))
            ntag_rows.append(np.array(ntags, dtype=np.int64).reshape(-1, nodes_per_elem))

        if not etag_rows:
            print(f"  Group '{name}': no elements found")
            continue

        etags_arr = np.concatenate(etag_rows)
        ntags_arr = np.vstack(ntag_rows)
        group_to_etags[name] = etags_arr
        group_to_ntags[name] = ntags_arr
        for i, et in enumerate(etags_arr):
            et = int(et)
            tet_to_tags[et].add(name)
            tet_to_nodes[et] = ntags_arr[i]
        print(f"  Group '{name}': {len(etags_arr)} elements")

    gmsh.finalize()

    # ------------------------------------------------------------------ #
    # 2. Merge duplicate-tet rows (same sorted node-set in different groups)
    #    into logical tets carrying the union of group names. Then build
    #    face → [adjacent logical-tet keys] map.
    #
    #    logical_key : tuple of sorted orig node IDs (one per logical tet)
    #    face_key    : frozenset of orig node IDs (size nodes_per_face)
    #    face_repr   : ordered tuple from first tet seen (stable output order)
    # ------------------------------------------------------------------ #
    logical_tags: dict[tuple, set] = defaultdict(set)
    logical_nodes: dict[tuple, list] = {}
    for et, nodes in tet_to_nodes.items():
        nodes_list = [int(v) for v in nodes.tolist()]
        lkey = tuple(sorted(nodes_list))
        logical_tags[lkey] |= tet_to_tags[et]
        if lkey not in logical_nodes:
            logical_nodes[lkey] = nodes_list

    face_to_tets = defaultdict(list)   # face_key → [logical_key, ...]
    face_repr    = {}                  # face_key → ordered tuple of orig node IDs

    for lkey, nodes_list in logical_nodes.items():
        for face_nodes in combinations(nodes_list, nodes_per_face):
            key = frozenset(face_nodes)
            face_to_tets[key].append(lkey)
            if key not in face_repr:
                face_repr[key] = tuple(face_nodes)

    interior_faces = {k: v for k, v in face_to_tets.items() if len(v) == 2}
    print(f"  Interior faces: {len(interior_faces)}, boundary faces (skipped): "
          f"{sum(1 for v in face_to_tets.values() if len(v) == 1)}")

    # ------------------------------------------------------------------ #
    # 3. Node renumbering for output mesh
    #    Keep only nodes used by output_groups tets, sorted for determinism.
    # ------------------------------------------------------------------ #
    keep_group_names = ({og["group"] for og in output_groups}
                        if output_groups else set(group_to_ntags.keys()))

    used_orig_tags = set()
    for name in keep_group_names:
        if name in group_to_ntags:
            used_orig_tags.update(group_to_ntags[name].flatten().tolist())

    sorted_orig_tags = sorted(used_orig_tags)
    orig_to_new = {orig: new1 for new1, orig in enumerate(sorted_orig_tags, start=1)}
    print(f"  Nodes: {len(node_tags_all)} total → {len(sorted_orig_tags)} kept")

    # ------------------------------------------------------------------ #
    # 4. Write stripped output mesh (optional)
    # ------------------------------------------------------------------ #
    if output_mesh:
        gmsh.initialize()
        gmsh.model.add("stripped")

        # All kept nodes in one global entity to avoid duplicate-node issues
        # when groups share interface nodes
        NODE_ENTITY = 0
        gmsh.model.addDiscreteEntity(0, NODE_ENTITY)
        new_ids   = [orig_to_new[o] for o in sorted_orig_tags]
        new_coords = [c for o in sorted_orig_tags
                      for c in all_coords[orig_node_id_to_row[o]].tolist()]
        gmsh.model.mesh.addNodes(0, NODE_ENTITY, new_ids, new_coords)
        print(f"  Added {len(new_ids)} nodes to output mesh")

        for og in output_groups:
            name, phys_id = og["group"], og["id"]
            if name not in group_to_etags:
                print(f"  Warning: output group '{name}' has no elements, skipping")
                continue
            etags = group_to_etags[name]
            ntags = group_to_ntags[name]
            gmsh.model.addDiscreteEntity(mesh_dim, phys_id)
            renumbered = np.vectorize(orig_to_new.__getitem__)(ntags).astype(np.int64)
            gmsh.model.mesh.addElementsByType(
                phys_id, elem_type, etags.tolist(), renumbered.flatten().tolist())
            gmsh.model.addPhysicalGroup(mesh_dim, [phys_id], phys_id)
            gmsh.model.setPhysicalName(mesh_dim, phys_id, name)
            print(f"  Output mesh: '{name}' → phys_tag={phys_id}, {len(etags)} elements")

        gmsh.option.setNumber("Mesh.SaveAll", 1)
        gmsh.write(output_mesh)
        gmsh.finalize()
        print(f"  Wrote stripped mesh: {output_mesh}")

    # ------------------------------------------------------------------ #
    # 5. Extract interface faces per selection and write boundary file
    # ------------------------------------------------------------------ #
    output_lines = []

    for sel in selections:
        out_id   = sel["id"]
        sel_tags = sel["groups"]
        S        = frozenset(sel_tags)

        interface_keys = set()
        for face_key, (a_key, b_key) in (
                (k, (v[0], v[1])) for k, v in interior_faces.items()):
            A = logical_tags[a_key]
            B = logical_tags[b_key]
            if S <= (A | B) and not (S <= (A & B)):
                interface_keys.add(face_key)

        desc = " ∩ ".join(f"'{g}'" for g in sel_tags)
        print(f"  Selection id={out_id} ({desc}): {len(interface_keys)} {boundary_type}")

        for key in interface_keys:
            nodes_orig = face_repr[key]
            # Map to 0-indexed new IDs; skip face if any node not in kept set
            try:
                verts = [orig_to_new[v] - 1 for v in nodes_orig]
            except KeyError:
                continue
            if is_2d:
                output_lines.append(f"{out_id} {verts[0]} {verts[1]}\n")
            else:
                output_lines.append(f"{out_id} {verts[0]} {verts[1]} {verts[2]}\n")

    if not output_lines:
        print(f"\nWarning: no boundary {boundary_type} to write.")
        return

    with open(boundary_output, "w") as f:
        f.writelines(output_lines)
    print(f"\nWrote {len(output_lines)} {boundary_type} to: {boundary_output}")

    # Write ID map alongside boundary output
    map_path = os.path.splitext(boundary_output)[0] + "_id_map.txt"
    lines = []
    if output_groups:
        lines.append("# Body IDs  (polyfem materials[].id  ←  mesh physical group tag)\n")
        for og in output_groups:
            lines.append(f"  {og['id']:>4}  ←  '{og['group']}'\n")
        lines.append("\n")
    lines.append("# Surface selection IDs  (polyfem boundary_conditions[].id  ←  groups)\n")
    for sel in selections:
        desc = " ∩ ".join(f"'{g}'" for g in sel["groups"])
        lines.append(f"  {sel['id']:>4}  ←  {desc}\n")
    with open(map_path, "w") as f:
        f.writelines(lines)
    print(f"Wrote ID map to: {map_path}")


# ------------------------------------------------------------------ #
# CLI
# ------------------------------------------------------------------ #
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Extract interface boundary selection from WMTK _groups.msh files.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example JSON config:
{
  "input": "m3_groups.msh",
  "boundary_output": "m3_bdry_selection.txt",
  "output_mesh": "m3_sim.msh",
  "output_groups": [
    {"group": "tag_0", "id": 1},
    {"group": "tag_1", "id": 2}
  ],
  "selections": [
    {"groups": ["tag_0", "ambient"], "id": 1},
    {"groups": ["tag_1", "ambient"], "id": 2},
    {"groups": ["tag_0", "tag_1"],   "id": 3}
  ],
  "skip_tags": ["ambient"],
  "is_2d": false,
  "analyze": false
}

Each selection needs 2+ groups. The extracted faces are interior mesh faces
where adjacent tets carry complementary subsets of the selection's tag set.
Outer mesh boundary faces (with only one adjacent tet) are always excluded.
""")
    parser.add_argument("-j", "--json", required=True, help="JSON config file")
    args = parser.parse_args()

    if not os.path.exists(args.json):
        print(f"Error: config not found: {args.json}", file=sys.stderr)
        sys.exit(1)

    json_dir = os.path.dirname(os.path.abspath(args.json))
    with open(args.json) as f:
        config = json.load(f)

    VALID_KEYS = {"input", "boundary_output", "output_mesh", "output_groups",
                  "selections", "skip_tags", "is_2d", "analyze"}
    unknown = set(config) - VALID_KEYS
    if unknown:
        print(f"Error: unknown field(s): {sorted(unknown)}", file=sys.stderr)
        sys.exit(1)

    if "input" not in config:
        print("Error: 'input' required", file=sys.stderr); sys.exit(1)
    if "selections" not in config or not config["selections"]:
        print("Error: 'selections' required and non-empty", file=sys.stderr); sys.exit(1)
    if "output_mesh" in config and "output_groups" not in config:
        print("Error: 'output_groups' required with 'output_mesh'", file=sys.stderr)
        sys.exit(1)

    for i, sel in enumerate(config["selections"]):
        if "groups" not in sel or "id" not in sel:
            print(f"Error: selections[{i}] needs 'groups' and 'id'", file=sys.stderr)
            sys.exit(1)
        if not isinstance(sel["id"], int):
            print(f"Error: selections[{i}].id must be int", file=sys.stderr); sys.exit(1)
        if len(sel["groups"]) < 2:
            print(f"Error: selections[{i}].groups needs at least 2 groups", file=sys.stderr)
            sys.exit(1)

    input_path = config["input"]
    if not os.path.isabs(input_path):
        input_path = os.path.join(json_dir, input_path)
    if not input_path.endswith(".msh") or not os.path.exists(input_path):
        print(f"Error: .msh not found: {input_path}", file=sys.stderr); sys.exit(1)

    is_2d = config.get("is_2d", False)

    boundary_output = config.get("boundary_output")
    if boundary_output and not os.path.isabs(boundary_output):
        boundary_output = os.path.join(json_dir, boundary_output)
    if boundary_output is None:
        boundary_output = os.path.splitext(input_path)[0] + "_bdry_selection.txt"

    output_mesh = config.get("output_mesh")
    if output_mesh and not os.path.isabs(output_mesh):
        output_mesh = os.path.join(json_dir, output_mesh)

    print("Configuration:")
    print(f"  Input:            {input_path}")
    print(f"  Boundary output:  {boundary_output}")
    if output_mesh:
        print(f"  Output mesh:      {output_mesh}")
        for og in config["output_groups"]:
            print(f"    '{og['group']}' → phys_tag={og['id']}")
    print(f"  Skip tags:        {config.get('skip_tags', ['ambient'])}")
    print(f"  Mesh dimension:   {'2D' if is_2d else '3D'}")
    print(f"  Selections:")
    for sel in config["selections"]:
        print(f"    id={sel['id']}  groups={sel['groups']}")

    if config.get("analyze", False):
        analyze_mesh(input_path)

    print("\n" + "=" * 60)
    print("EXTRACTING BOUNDARY SELECTION")
    print("=" * 60 + "\n")
    extract_boundary_selection_from_groups(
        input_path=input_path,
        boundary_output=boundary_output,
        selections=config["selections"],
        is_2d=is_2d,
        output_mesh=output_mesh,
        output_groups=config.get("output_groups"),
        skip_tags=config.get("skip_tags", ["ambient"]),
    )
    print("\n" + "=" * 60 + "\n")
