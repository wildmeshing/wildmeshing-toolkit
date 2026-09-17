# Prismatic mesh input

Run from the repository root:

```sh
cmake --build build --target wmtk_app -j 4
./build/app/wmtk_app -j mydata/prismatic_mesh.json
```

The JSON `input` and `output` paths are relative to the JSON file's directory. Example:

```json
{
  "application": "prismatic_mesh",
  "input": "deliverable/cube/cube_2_construction_correspondence.vtu",
  "output": "resultmesh.vtu"
}
```

This initial stage loads a tetrahedral construction mesh and its correspondence, then keeps
only the offset band tetrahedra (`offset_tet_tags == 1`) and writes them to `output`.
It does not yet construct prisms. A filename without an extension gets `.vtu` appended.
For this example, the output is `mydata/resultmesh.vtu`.
`load_prismatic_mesh(path)` returns a
`PrismaticMeshInput` containing WMTK `TetMesh` connectivity, vertex coordinates,
tetrahedra, and the following required VTU fields:

| Field | Association | Meaning |
| --- | --- | --- |
| `labels` | PointData | 0 = other, 1 = input surface, 2 = offset surface |
| `vid` | PointData | Unique original vertex ID |
| `corr_input_vid` | PointData | Original input vertex ID for every offset vertex; -1 elsewhere |
| `tag_0` | CellData | Input volume membership (0/1) |
| `offset_tag` | CellData | Offset band membership (0/1) |

`offset_tag` is intentional: `offset` is zero in the construction export.
On loading, VTU `labels` becomes `vertex_tags` (1 = input, 2 = offset, -1 = other),
and VTU `offset_tag` becomes `offset_tet_tags` (1 = band, -1 = other).
The source VTU uses 0 for these other vertices/cells; it is converted to -1 in memory.
`keep_offset_band(input)` filters tetrahedra and cell attributes together, preserving their
order and tag values, and rebuilds the WMTK mesh. Vertex arrays, IDs, tags and correspondence
remain unchanged; vertices unused by the band are inactive in the rebuilt mesh.
The component entry point in `prismatic_mesh.cpp` handles configuration and loading, then
calls `prism_main(input)` in `prism_main.cpp`, where the algorithm pipeline continues.
`write_vtu.cpp` exports only active vertices and retained tetrahedra. Output connectivity
uses compact row indices; `vid` and `corr_input_vid` preserve the original source IDs.
The output fields are `labels` (1 = input, 2 = offset, -1 = other), `vid`,
`corr_input_vid`, `tag_0` and `offset_tag` (1 = band, -1 = other).
The loader accepts both the construction file's 0 and the output's -1 for other elements.
`corr_input_vertex` resolves source IDs to mesh row indices, and
`input_to_offset_vertices[input_row]` contains all corresponding offset rows.
Several offset vertices may correspond to one input vertex. Positions and attributes
are owned by the returned input object alongside `TetMesh`; these arrays describe the
loaded mesh and are not automatically updated by later topology operations.

Supported VTU: a single UnstructuredGrid piece with four-node tetrahedra, ASCII or
uncompressed inline base64 arrays, UInt32/UInt64 binary headers and either byte order.
A missing byte_order defaults to LittleEndian for the existing paraviewo exports.
Compressed, appended, multi-piece and mixed-cell files are rejected explicitly.
Missing fields, invalid indices and invalid correspondence also cause an error.

Tests:

```sh
cmake --build build --target wmtk_test_prismatic_mesh -j 4
ctest --test-dir build -R '^wmtk_test_prismatic_mesh$' --output-on-failure
```
