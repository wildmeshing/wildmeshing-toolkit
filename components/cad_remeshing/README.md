# CAD Remeshing

The `cad_remeshing` component remeshes the triangle mesh of a CAD model to a target edge length while preserving the structure of the CAD model: its patches (faces), feature curves (edges), and corners (vertices).

The remeshing runs `max_iterations` rounds of edge splits, edge collapses, edge swaps, and vertex smoothing. Edges longer than 4/3 of the target length are split, and edges shorter than 4/5 of it are collapsed. Swaps improve the vertex valences.

The CAD structure is preserved as follows:

- **Corners** never move. An edge between two corners is never collapsed.
- **Feature curves** remain chains of mesh edges. Feature edges are never swapped, and a vertex on a curve can only be collapsed along that curve. Splitting a feature edge creates a new vertex on the same curve. Vertices on curves are not smoothed.
- **Patches**: every output face carries the patch ID of the input face it descends from.

Every operation is rejected if it creates a degenerate triangle or moves the surface outside an envelope around the input mesh. A vertex on a feature curve must also stay within an envelope around the input curve it lies on (one envelope per curve, so a vertex cannot drift onto a neighbouring curve near a corner). Smoothed vertices are projected back onto the input surface.

The envelope thickness is `eps` if set, else `eps_rel` times the bounding box diagonal if set, else (the default) half the average edge length of the input.

## Input

The component requires two input files:

- **`input`**: the triangle mesh, as `.msh` or any format libigl can read (e.g., `.obj`, `.off`, `.stl`, `.ply`).
- **`patches`**: a JSON file describing the CAD structure on that mesh. All IDs refer to the vertices and faces of `input`.
  - `fid2patch`: the patch ID of each face. It must contain every face exactly once.
  - `edge2seg`: the feature edges and their curve ID. Keys are the two vertex IDs of the edge, separated by a comma, and every key must be an edge of the mesh. The feature edges must form chains: a vertex with feature edges that is not a corner must have exactly two of them, i.e., every curve ends at a corner and curves only meet at corners.
  - `corner_vids`: maps each CAD corner ID to its vertex ID.

All three keys are required, and the IDs are validated against the mesh; an invalid file is reported as an error.

```json
{
  "fid2patch": { "0": 0, "1": 0, "2": 3, "...": "..." },
  "edge2seg": { "17,52": 4, "52,90": 4, "...": "..." },
  "corner_vids": { "0": 17, "1": 233, "...": "..." }
}
```

Relative paths of input files are resolved against the directory of the parameter JSON file.

### Converting `fused.pkl`

`scripts/convert_pkl_to_wmtk.py` converts a `fused.pkl` file into the mesh (`mesh.msh`), the patches file (`features.json`), and a parameter JSON (`remesh.json`) in the given output directory. It requires `numpy`, `meshio`, and `libigl`.

```bash
python scripts/convert_pkl_to_wmtk.py <output_dir> [--fused <fused.pkl>] [--len_rel <length>]
```

## Output

- **`<output>.vtu`**: the remeshed surface.
  - Vertex fields: `is_feature`, `corner_id` (CAD corner ID + 1, or -1 if the vertex is not a corner), `target_edge_length`, and `valence`.
  - Face fields: `patch_id` and `quality` (1 for an equilateral triangle, 0 for a degenerate one).
- **`<output>_edges.vtu`**: the feature edges as line elements, with the cell field `curve_id`.
- **`report`** (optional): a JSON file with mesh statistics before and after remeshing, as well as the runtime.

Output paths are relative to the working directory.

## Example

A minimal parameter JSON:

```json
{
  "application": "cad_remeshing",
  "input": "mesh.msh",
  "patches": "features.json",
  "output": "remeshed",
  "report": "remeshed_report.json"
}
```

Run it with:

```bash
wmtk_app -j remesh.json
```

The most important optional parameters are listed below. See [wmtk/components/cad_remeshing/cad_remeshing_spec.json](cad_remeshing_spec.json) for all of them.

| Parameter         | Default | Description                                                                                   |
| ----------------- | ------- | --------------------------------------------------------------------------------------------- |
| `length_rel`      | `0.05`  | Target edge length relative to the bounding box diagonal.                                     |
| `length_abs`      | `-1`    | Absolute target edge length. Overrides `length_rel` if positive.                              |
| `length_factor`   | `-1`    | If positive, use a per-patch target length: the longest edge of each patch times this factor. |
| `max_iterations`  | `3`     | Number of split, collapse, swap, and smooth rounds.                                           |
| `freeze_boundary` | `false` | Keep the vertices on the open boundary of the mesh fixed.                                     |
| `num_threads`     | `0`     | Number of threads. `0` runs sequentially.                                                     |
