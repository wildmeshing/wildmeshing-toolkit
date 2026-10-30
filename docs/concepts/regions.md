# Regions

A mesh with several materials, organs or parts carries them as *regions*: named sets of
top-dimensional cells (tetrahedra in 3D, triangles in 2D). A cell may belong to several regions,
where they overlap, or to none, in the background. simwild builds such meshes and edits them;
triwild tags its triangles by input; topological_offset and manifold_extraction select regions to
work on. All of them name regions the same way, and select them with the same small expression
language.

## Tags

On a mesh that leaves or enters a component, regions are the `tags` attribute of the
top-dimensional cells: a UInt8 matrix with one column per region, labelled with the region's name,
and a 1 where the cell belongs to the region. A cell in no region is the background; components
write it as a column of its own, labelled `ambient`. In Python
([in-memory I/O](../in-memory-io.md)):

```python
mesh["cells"]["tetrahedron"]["attributes"] = {
    "tags": {
        "values": np.array([[1, 1], [1, 0]], dtype=np.uint8),
        "labels": ["region_a", "region_b"],
    },
}
```

In a `.msh` file, each region is a physical group. Inside a component, each cell holds the set of
the ids of its regions, and the mesh maps names to ids.

Regions come from:

- **simwild on surfaces or curves**: one region per input, named by
  [`input_names`](../reference/simwild.md#input_names) (otherwise `tag_0`, `tag_1`, ...). A cell
  belongs to an input's region if it is inside that input: by default the input's winding number
  at the cell's centroid decides. For surfaces,
  [`tag_from_winding_number`](../reference/simwild.md#tag_from_winding_number) off propagates
  membership across the arrangement instead, from the outside of the box, flipping each time a
  face of the input is crossed; every input must then be closed. [`split_connected_components`](../reference/simwild.md#split_connected_components)
  makes one region per connected component of a file.
- **triwild**: one region per input curve, with the triangles inside it by winding number.
- **A tagged mesh** given as input (a `.msh`, or a mesh in memory): its columns are its regions.

## Expressions

Options that select cells take a boolean expression over region names:

| Expression | Selects the cells |
|---|---|
| `a` | in region `a` |
| `a & b` | in both `a` and `b` (their overlap) |
| `a | b` | in `a`, in `b`, or in both |
| `!a` | not in `a` |
| `(a | b) & !c` | in `a` or `b`, and not in `c` |
| `_` | in no region |
| `!_` | in at least one region |

`!` binds tightest, then `&`, then `|`: `a | b & c` is `a | (b & c)`. A name is made of letters,
digits, `_` and `-`. In a selection, a name the mesh does not have is reported with a warning and selects nothing;
in `replace_tags_out` and `tag_priority`, a new name creates the region.
The parser is in
[`components/simwild/wmtk/components/simwild/expression_parser/`](https://github.com/wildmeshing/wildmeshing-toolkit/blob/main/components/simwild/wmtk/components/simwild/expression_parser/Parser.cpp).

The background depends on the component. simwild reads the `ambient` column as no region at all,
so its background is `_`. topological_offset and manifold_extraction give the background a region
of its own, named `ambient`, so there it is selected by that name.

## How components use them

| Component | Option | Takes |
|---|---|---|
| simwild, remeshing | [`sizing_field`](../reference/simwild.md#sizing_field) | a list of `{"tags": expression, "length": ...}` (or `length_rel`): a target edge length per selection; each cell takes the last entry that selects it, and each vertex the smallest length of its cells |
| simwild, remeshing | [`quality_field`](../reference/simwild.md#quality_field) | a list of `{"tags": expression, "quality": ...}`: a target energy per selection; where several apply, the last wins |
| simwild, `resolve_overlaps` | [`resolve_overlaps_tags`](../reference/simwild.md#resolve_overlaps_tags) | pairs of expressions; each overlap is filled with the nearest of the two |
| simwild, `replace_tags` | [`replace_tags_in`](../reference/simwild.md#replace_tags_in), [`replace_tags_out`](../reference/simwild.md#replace_tags_out) | `&` only: the regions of each input expression are replaced by those of the output one |
| simwild, `fill_holes_topo`, `tight_seal_topo`, `keep_lcc` | [`fill_holes_tags`](../reference/simwild.md#fill_holes_tags), [`tight_seal_tag_sets`](../reference/simwild.md#tight_seal_tag_sets), [`keep_lcc_tags`](../reference/simwild.md#keep_lcc_tags) | `&` only: sets of regions |
| simwild, `tag_priority` | [`tag_priority`](../reference/simwild.md#tag_priority) | region names, highest priority first |
| topological_offset | [`offset_selection`](../reference/topological_offset.md#offset_selection) | an expression: the cells to offset; a single name selects single-body mode |
| manifold_extraction | [`tag_selection`](../reference/manifold_extraction.md#tag_selection) | an expression: the cells whose boundary is made manifold |

In simwild the surface the optimization tracks is the interface between regions: the faces
whose two cells do not belong to the same regions. It is held in the
[envelope](robust-geometry.md#envelopes) like tetwild's tracked surface, and it is what
`history_output` records (see [Histories](histories.md)).

## See also

- The [simwild tutorial](../tutorials/simwild.md) and the [regions workflow](../tutorials/workflows/regions.md).
- The [topological offset](../tutorials/topological_offset.md) and
  [manifold extraction](../tutorials/manifold_extraction.md) tutorials.
- [In-memory I/O](../in-memory-io.md) for the full description of mesh dictionaries and `tags`.
