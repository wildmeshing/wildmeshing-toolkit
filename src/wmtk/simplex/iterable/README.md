# Simplex Iterables

#### Top Dimension Cofaces

To get all top dimension cofaces of a simplex, there are 4 different types of navigation, depending on the depth (top simplex dimension - simplex dimension) of the simplex:

1. depth 0 - no navigation, the top dimension coface is the simplex itself (e.g. a tetrahedron in a TetMesh)
2. depth 1 - 2-sides look-up (e.g. the triangles around an edge in a TriMesh)
3. depth 2 - circulation (e.g. the triangles around a vertex in a TriMesh)
4. depth 3 - breadth first search (the tetrahedra around a vertex in a TetMesh)

Note that there exists exactly one "depth 3" case.

#### Other Iterables

All other iterables (except for the `FacesIterable` which is not properly implemented yet) are using a `TopDimensionCofacesIterable::Iterator` internally to navigate around the simplex.
