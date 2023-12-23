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

## Data Structure

The toolkit implements an abstract interface `Mesh.hpp`, which supports simplicial meshes of dimension 0 to 3. The navigation is dimension agnostic and based on a `Tuple.hpp`. We currently implement this interface for 4 simplicial mesh types: `PointMesh.hpp`, `EdgeMesh.hpp`, `TriangleMesh.hpp`, and `TetMesh.hpp`. 

Internally, the data structure is stored as:
- A vector of simplices for each dimension storing the simplex attributes (note that each simplex has thus a unique global index, which is its position in this vector)
- A set of standard attributes storing the connectivity.

For connectivity, we store the complete relationship between each top simplex of dimension d and both its neighbours connected through a d-1 face and its faces (for a triangle mesh, TT, TE, TV), and one representative for the inverse relationships (for a triangle mesh, every vertex stores the index of one of its incident triangles, and every edge also stores an index of one of the incident triangles). Note that we use only indices internally, but we **do not expose** this information on purpose to avoid the temptation to use indices in user code (which might be invalidated by local operations).

The data structure is not allowed to dynamically resize during local operations: it is responsability of the user to reserve enough memory before attempting to modify a mesh with local operations. Unused memory can be reclaimed by using the function TODO.

### Tuple

A `wmtk::Tuple` is internally represented as a single global index to its top simplex, and a collection of local indices identifying the local faces (for a triangle mesh, the tuple points to the triangle, and it has two local indices to identify the vertex and edge). Tuple implements switch operations that allows to navigate the mesh by changing one of the pointed simplexes: the navigation is performed by either accessing an adjency topological relation to move to a different top simplex, or by changing the local indices. The change in local indexes is tabulated `local_switch_tuple.hpp`, making the navigation efficient. The table is generated with the scripts in the autogen folder.

## Repository Structure

The main classes in the toolkit are:

1. Mesh `Mesh.hpp`. It stores a set of attributes and allows navigation and execution of operations. The types of meshes supported are: points (`PointMesh.hpp`), edges (`EdgeMesh.hpp`), triangles (`TriMesh.hpp`) and tetrahedra (`TetMesh.hpp`).
2. Tuple `Tuple.hpp`. A tuple is a ordered set of simplices, containing one simplex for each dimension. It allows navigation by using switch operations, see 
[Representing geometric structures in d dimensions: topology and order](https://dl.acm.org/doi/10.1145/73833.73858).
3. Simplex `Simplex.hpp`. A simplex uniquely identifies a simplex in a mesh, and it is internally implemented as a primitive type and a tuple. A collection of simplexes can be stored in a `SimplexCollection.hpp`. The operations that return collection of simplexes are: `boundary.hpp`, `closed_star.hpp`, `link.hpp`, `open_star.hpp`.
4. An accessor `Accessor.hpp` allows to access the attributes of the mesh. There are multiple accessors that provide different access levels: read-only (`ConstAccessor.hpp`) and read-write (`MutableAccessor.hpp`)

### attribute --- Management of Mesh Attributes, Caching, and Checkpoints

The attribute management system has the following functionalities:
* Support for per-simplex attributes
* Automatic serialization
* Checkpointing to restore the state of the mesh after a failed operation
* Safe access to attributes only through a tuple (no index access is exposed)

The key concepts are:
* **Handles**. They represent pointers to an entity. There are two of them, one for each attribute (AttributeHandle) and one for the Scope (in database terms a transaction, AttributeScopeHandle). Handles can be obtained from the mesh class after an attribute is registered using the register_attribute method. 
* **Accessors**. They enable safe access to an attribute. To obtain one you can convert an handle to an accessor using the function get_accessor of the mesh. There are two types of them, const and mutable. The mutable one is automatically casted to a const one if necessary. The scope (transaction) is handled by the AttributeManager class. *Attributes*. Attributes are indexed by type (i.e double/char/long/Rational), simplex dimension, attribute id, and finally an additional index for vector type. This nested hierarchy is represented by the following nested classes: AttributeManager (type), MeshAttributes (simplex dimension), Attribute (attribute index), and vector<T> (final index). 

#### Public interface description:
1. Register a new attribute. Mesh->register_attribute. This function returns a handle.
2. Two standard ways to create an accessor:
  * Mesh->create_accessor. Converts the handle into an accessor
  * Handle.create_accessor. Creates a handle with respect to the accessor
3. Access the attribute using a Tuple.

#### Private API development details:

The inclusion tree for storing attributes is: AttributeManager (type), MeshAttributes (simplex dimension), Attribute (simplex index), and vector<T> (final index). The actual data is stored in Attribute as a flatten vector.

The inheritance for accessing attributes is:
AccessorBase -> CachingAccessor -> TupleAccessor -> ConstAccessor -> MutableAccessor
The first three are private, the last two are public. AccessorBase provides basic index-based access. CachingAccessor adds a layer on top supporting the use of an intermediate cache to store the changes temporarily. Tuple accessor hides the index access in favor of using a tuple. Finally const and mutable are the two user-facing classes providing read-only or read-write access.

The scoping mechanics is complex but entirely hidden by the Accessor interface. Every attribute stores a stack of scopes `PerThreadAttributeScopeStacks<T>`. A `PerThreadAttributeScopeStacks` is a wrapper for a `AttributeScopeStack`. An `AttributeScopeStack` is a stack of changes not yet applied to a Attribute. Each change is an `AttributeScope`, which inherits from an `AttributeCache` to provide the functionality to support a stack. The `AttributeCache` stores an `AttributeCacheData`, which stores the actual diff of the modifications done on the attribute.

### autogen --- Automatic Generation of Connectivity Tables

This folder stores a set of scripts that generate navigation tables for the local navigation inside a simplex.

### function

TODO: Refactor in progress

### invariants

A `wmtk::Invariant` is a condition that can be test before `wmtk::Invariant::before` or after `wmtk::Invariant::after` an operation. If the condition is not satisfied, the operation is either not executed, or rolled back. An invariant can be applied to a simplex of any dimension, and should not rely on the before and after functions being called in order. Example of common invariants are checking for a triangle inversion `wmtk::TriangleInersionInvariant` or limiting the size of an edge `wmtk::MinEdgeLengthInvariant`. Invariants are associated with operations at runtime: each operation stores a collection of invariants `wmtk::TupleOperation::invariants`.

### io

TODO: Explain how to load and save a mesh, and specify the hdf5 format

### multimesh

TODO

### operations

TODO:

TODO: refactor in progress

### optimization

TODO: refactor in progress

### simplex

This folder contains a set of functions to extract and process simplicial meshes. A `wmtk::Simplex` is internally represented as a `wmtk::Tuple` and the simplex type `wmtk::PrimitiveType`. The functions supported are: `closed_star.hpp`, `cofaces_single_dimension.hpp`, `faces.hpp`, `link.hpp`, `open_star.hpp`, `top_dimension_cofaces.hpp`. We refer to [P.L. Homeomorphic Manifolds are Equivalent by Elementary Shellingst
](https://core.ac.uk/download/pdf/82717779.pdf) for the formal definition. These functions also have an iterable version (i.e. `closed_star_iterable.hpp`), which should be used when possible, as it avoids unnecessary memory allocations.

### utils

Contains the logger (`Logger.hpp`), the rational number type (`Rational.hpp`) and additional minor utilities.


