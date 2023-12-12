# Code Structure and Manual

The wildmeshing toolkit provides a set of algorithms and corresponding data structure for supporting the creation, editing, and optimization of meshes in 2D and 3D. It is designed to support algorithms requiring to store data on a mesh whose connectivity is changing: for example in adaptive refinement, shape optimization, or multi-material mesh optimization.

## Key Concepts

1. *Attributes store per-simplex data.* This includes the connectivity `Attribute.hpp`.
2. *Tuples.* A tuple `Tuple.hpp` identifies a position on the mesh, and it is the only way to access attributes (and thus navigate). Indices of simplices are purposedly hiddern and not accessible to the public API.
3. *Checkpointing.* The library supports checkpointing to implement convenient operation rollback at any point during an operation.
4. *Invariants.* Mesh invariants `Invariant.hpp` (such as non-inversion) are automatically checked after each operation and the operation is rolled back if it does not satisfy the invariants.
5. *Topological Navigation.* The library directly supports topological navigation using a tuple, and standard topological operators (star, link, order, etc) in a dimension independent manner `SimplexCollection.hpp`. Efficient replacement for the most common operations is also supported to improve performance for common operations.
6. *Multi Mesh.* The library supports embedding of lower-dimensional manifold meshes `MultiMeshManager.hpp`, and provides a mechanism to update them as local operations are applied to either the background mesh or the embedded ones.
7. *Optimization.* The library can automatically minimize, through a combination of smooth (vertex relocation) and discrete (split,collapse,swap) operations, a differentiable energy specified on any of its simplices `Function.hpp`.

## Repository Structure

The main classes in the toolkit are:

1. Mesh `Mesh.hpp`. It stores a set of attributes and allows navigation and execution of operations. The types of meshes supported are: points (`PointMesh.hpp`), edges (`EdgeMesh.hpp`), triangles (`TriMesh.hpp`) and tetrahedra (`TetMesh.hpp`).
2. Tuple `Tuple.hpp`. A tuple is a ordered set of simplices, containing one simplex for each dimension. It allows navigation by using switch operations, see 
[Representing geometric structures in d dimensions: topology and order](https://dl.acm.org/doi/10.1145/73833.73858).
3. Simplex `Simplex.hpp`. A simplex uniquely identifies a simplex in a mesh, and it is internally implemented as a primitive type and a tuple. A collection of simplexes can be stored in a `SimplexCollection.hpp`. The operations that return collection of simplexes are: `boundary.hpp`, `closed_star.hpp`, `link.hpp`, `open_star`.
4. An accessor `Accessor.hpp` allows to access the attributes of the mesh. There are multiple accessors that provide different access levels: read-only (`ConstAccessor.hpp`) and read-write (`MutableAccessor.hpp`)

### attribute --- Management of Mesh Attributes, Caching, and Checkpoints

A mesh attribute `MeshAttributes.h` stores an attribute of a fixed type (char,long,double,Rational) and of a fixed lenght. The attribute can be attached to a simplex of a desired dimension dim.
A mesh has a set of default attributes, hidden and not accessible, that store the mesh connectivity (see for example `TriMesh.hpp`). Access to attributes is possible only through a Tuple `Tuple.hpp`.

TODO: Explain how to start a checkpoint

TODO: Explain how to load and 

### autogen --- Automatic Generation of Connectivity Tables

This folder stores a set of scripts and functions supporting the local navigation inside a simplex.

TODO: describe how the data structure works

### function

### invariants

### io

### multimesh

### operations

### simplex

### utils

