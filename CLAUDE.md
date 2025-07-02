# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Wildmeshing Toolkit is a C++ framework for mesh editing algorithms using declarative specifications. **The primary focus is on the bijective_map components** which provide specialized applications for bijective parameterization and mesh mapping with interactive visualization tools.

## Build System & Commands

### Building the Project
```bash
# Standard CMake build process
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make

# Dependencies: requires gmp and Perl (for hdf5)
# Install gmp via homebrew on macOS: brew install gmp
```

### Bijective Map Applications (Primary Focus)
The `bijective_map/` directory contains the main applications:

- **Bijective Map App**: `./build/bijective_map/bijective_map_app` - Interactive 2D bijective mapping with OpenGL visualization
- **Bijective Map Tet App**: `./build/bijective_map/bijective_map_app_tet` - 3D tetrahedral mesh bijective mapping

#### Key Bijective Map Features:
- Point tracking in tetrahedral meshes
- Line/curve tracking through mesh operations
- Surface tracking and embedding
- Integration with Interactive And Robust Mesh Booleans library
- VTU file export utilities for visualization

### Standard Application Commands
The toolkit also includes mesh editing applications in `applications/`:

- **Shortest Edge Collapse**: `./build/applications/shortest_edge_collapse_app input output [OPTIONS]`
- **Isotropic Remeshing**: `./build/applications/isotropic_remeshing/main_3d input output [OPTIONS]` 
- **Delaunay**: `./build/applications/delaunay_app input output [OPTIONS]`
- **Tetwild**: `./build/applications/tetwild_app -i input -o output [OPTIONS]`
- **Marching**: `./build/applications/marching_app input output [OPTIONS]`

### Test Data
Test data is available from: https://drive.google.com/drive/folders/1jFdQ77E2_n3EJF5_bPOOMEOxF4dyctjN
Extract the `wmtk-data-package.zip` to the `data/` directory.

## Code Architecture

### Bijective Map Components (Primary Focus)

**Core Bijective Map Files**:
- `bijective_map_app.cpp` - 2D interactive bijective mapping application
- `bijective_map_app_tet.cpp` - 3D tetrahedral mesh bijective mapping
- `track_operations_tet.hpp/cpp` - Point/curve tracking in tetrahedral meshes
- `track_point_app.cpp` - Point tracking application
- `track_line_app.cpp` - Line/curve tracking application
- `FindPointTetMesh.cpp/hpp` - Utilities for locating points in tetrahedral meshes
- `vtu_utils.cpp/hpp` - VTU file export utilities for ParaView visualization

**Key Bijective Map Data Structures**:
- `query_point_tet` - Point representation with barycentric coordinates in tets
- `query_segment_tet` - Line segment representation in tets
- `query_curve_tet` - Curve representation as connected segments
- `query_triangle_tet` - Triangle representation in tets
- `query_surface_tet` - Surface representation as connected triangles

**Interactive And Robust Mesh Booleans Integration**:
- Located in `bijective_map/InteractiveAndRobustMeshBooleans/`
- Provides robust boolean operations for mesh processing
- Used for complex geometry operations in bijective mapping

## Bijective Map Detailed Documentation

### Overview
The bijective map module provides specialized applications for creating and managing bijective mappings between meshes. This is critical for applications requiring one-to-one correspondence between mesh elements while preserving geometric properties.

### Current Implementation Status
⚠️ **Note**: The bijective map implementation is currently incomplete and requires further development. The existing code provides a foundation but needs completion of core algorithms and functionality.

### Architecture

#### Core Applications
1. **bijective_map_app** (`bijective_map_app.cpp`)
   - Interactive 2D bijective mapping application
   - Uses OpenGL for real-time visualization
   - Integrates with libigl for mesh operations
   - Status: 🔄 **Incomplete - needs algorithm implementation**

2. **bijective_map_app_tet** (`bijective_map_app_tet.cpp`)
   - 3D tetrahedral mesh bijective mapping
   - Handles point tracking through tetrahedral mesh operations
   - Reads/writes vertex and tetrahedron data from CSV files
   - Status: 🔄 **Incomplete - needs core mapping algorithms**

#### Point and Curve Tracking System
**Core Data Structures**:
```cpp
struct query_point_tet {
    int64_t t_id;           // Tetrahedral element ID
    Eigen::Vector4d bc;     // Barycentric coordinates
    Eigen::Vector4i tv_ids; // Tet vertex IDs
};

struct query_segment_tet {
    int64_t t_id;           // Tetrahedral element ID
    Eigen::Vector4d bcs[2]; // Barycentric coords for endpoints
    Eigen::Vector4i tv_ids; // Tet vertex IDs
};

struct query_curve_tet {
    std::vector<query_segment_tet> segments;
    std::vector<int> next_segment_ids;
};
```

**Tracking Functions** (defined in `track_operations_tet.hpp`):
- `barycentric_to_world_tet()` - Convert barycentric to world coordinates
- `world_to_barycentric_tet()` - Convert world to barycentric coordinates
- `track_point_one_operation_tet()` - Track point through single mesh operation
- Status: 🔄 **Incomplete - needs robust tracking algorithms**

#### Point Location Utilities
**FindPointTetMesh** (`FindPointTetMesh.hpp/cpp`):
- `findTetContainingPoint()` - Traditional barycentric coordinate-based point location
- `findTetContainingPointRational()` - High-precision rational version
- `findTetContainingPointOrient3d()` - **NEW**: Numerically stable version using `wmtk::utils::wmtk_orient3d`
- Status: ✅ **Enhanced with robust geometric predicates**

**Key Improvements in Orient3D Version**:
- Uses exact geometric predicates via `wmtk::utils::wmtk_orient3d` for robust point-in-tetrahedron testing
- Eliminates numerical precision issues in traditional barycentric coordinate methods
- Tests point containment by checking orientation consistency across all four tetrahedral faces
- Provides error reporting when points are not contained in any tetrahedron
- Maintains compatibility with existing barycentric coordinate computation for return values

#### Mesh Boolean Integration
- Uses InteractiveAndRobustMeshBooleans library (external dependency)
- Provides robust boolean operations for complex geometry
- Fetched from: https://github.com/zlyfunction/InteractiveAndRobustMeshBooleans.git
- Status: ✅ **Integrated but needs adaptation for bijective use cases**

#### Visualization and Export
**VTU Utils** (`vtu_utils.hpp/cpp`):
- `write_triangle_mesh_to_vtu()` - Export triangle meshes to VTU format
- `write_point_mesh_to_vtu()` - Export point clouds to VTU format
- Enables visualization in ParaView
- Status: 🔄 **Basic functionality exists, needs enhancement**

### Required Development Tasks

#### High Priority (Core Algorithms)
1. **Bijective Mapping Algorithm Implementation**
   - Implement core bijective mapping between 2D/3D meshes
   - Ensure one-to-one correspondence preservation
   - Add distortion metrics and optimization

2. **Point Tracking Robustness**
   - Complete `track_point_one_operation_tet()` implementation
   - Handle edge cases in barycentric coordinate updates
   - Add validation for tracking accuracy
   - ✅ **COMPLETED**: Enhanced point location with `findTetContainingPointOrient3d()` using robust geometric predicates

3. **Curve and Surface Tracking**
   - Implement robust curve tracking through mesh operations
   - Add surface embedding and tracking capabilities
   - Handle topology changes during operations

#### Medium Priority (Features)
1. **Interactive Visualization**
   - Complete OpenGL visualization in `bijective_map_app`
   - Add real-time manipulation tools
   - Implement user interface for parameter adjustment

2. **File I/O Enhancement**
   - Add support for standard mesh formats (OBJ, STL, PLY)
   - Implement JSON-based configuration system
   - Add progress tracking and logging

#### Low Priority (Optimization)
1. **Performance Optimization**
   - Parallelize tracking operations using TBB
   - Optimize data structures for large meshes
   - Add memory management improvements

### Dependencies
- **Required**: wmtk::toolkit, CLI11, igl::core, nlohmann_json, TBB
- **Optional**: cinolib (for enhanced tetrahedral operations)
- **External**: InteractiveAndRobustMeshBooleans

### Usage Patterns
```cpp
// Point location examples
Eigen::MatrixXd V; // Vertex matrix (n x 3)
Eigen::MatrixXi T; // Tetrahedron matrix (m x 4)
Eigen::Vector3d query_point; // Point to locate

// Traditional method (may have numerical issues)
auto [tet_id, bary_coords] = findTetContainingPoint(V, T, query_point);

// Robust method using orient3d (recommended)
auto [tet_id_robust, bary_coords_robust] = findTetContainingPointOrient3d(V, T, query_point);

// Point tracking example (to be completed)
std::vector<query_point_tet> query_points;
// Initialize points...
track_point_one_operation_tet(operation_log, query_points);

// VTU export example
vtu_utils::write_triangle_mesh_to_vtu(V, F, "output.vtu");
```

### Development Guidelines for Bijective Map
1. **Maintain Robustness**: All operations must preserve bijective properties
2. **Handle Edge Cases**: Account for degenerate meshes and boundary conditions
3. **Performance**: Optimize for real-time applications where possible
4. **Testing**: Implement comprehensive unit tests for all tracking functions
5. **Documentation**: Document all data structures and algorithms thoroughly

### Core Toolkit Components

**Mesh Types**: The toolkit supports multiple mesh types through inheritance:
- `PointMesh` (0D)
- `EdgeMesh` (1D) 
- `TriMesh` (2D)
- `TetMesh` (3D)

All mesh classes inherit from the base `Mesh` class and use CRTP pattern via `MeshCRTP`.

**Key Directories**:
- `src/wmtk/` - Core mesh classes and operations
- `src/wmtk/operations/` - Mesh editing operations (collapse, split, swap, smoothing)
- `src/wmtk/invariants/` - Invariant checking system for operation validation
- `src/wmtk/attribute/` - Attribute management system with caching
- `src/wmtk/multimesh/` - Multi-mesh management for hierarchical meshes
- `src/wmtk/autogen/` - Auto-generated topology tables and navigation
- `src/wmtk/simplex/` - Simplex operations and iterators
- `src/wmtk/function/` - Energy functions and optimization

### Operation System

Operations follow a consistent pattern:
1. **Before Operation**: Validate preconditions
2. **Execute**: Perform topology changes  
3. **After Operation**: Update attributes and validate invariants
4. **Rollback**: Automatic rollback if invariants fail

**Core Operations**:
- `EdgeCollapse` - Merge two vertices by collapsing an edge
- `EdgeSplit` - Split an edge by inserting a new vertex
- Swapping operations (2-3, 3-2, 4-4 swaps in 3D)
- Smoothing operations for mesh quality improvement

### Attribute System

Attributes are strongly typed and cached for performance:
- `AttributeHandle<T>` for typed access
- `CachingAccessor` for high-performance batch operations
- Automatic attribute updates during operations
- Support for per-vertex, per-edge, per-face, and per-tetrahedron attributes

### Invariant System

Invariants ensure operation validity:
- `EnvelopeInvariant` - Prevents vertices from moving outside geometric envelope
- `LinkCondition` - Ensures topological validity
- Energy-based invariants for mesh quality
- Boundary preservation invariants

## Development Guidelines

### Code Organization
- Operations are implemented as classes inheriting from `Operation`
- Invariants inherit from `Invariant` base class
- Use the scheduler system for operation queuing and parallelization
- Follow existing patterns for attribute management

### Common Patterns
- Use `Tuple` for mesh element references (vertex/edge/face/tet)
- Access attributes through `Accessor` classes for performance
- Implement invariants as separate classes for modularity
- Use the auto-generated navigation functions for mesh traversal

### Testing
- Unit tests are in `tests/` directory
- Build tests with `make` after CMake configuration
- Integration tests use data from the `data/` directory

## Key Dependencies

- **Eigen**: Linear algebra and matrix operations
- **TBB**: Parallel execution and threading
- **GMP**: Arbitrary precision arithmetic
- **Fast Envelope**: Geometric envelope containment checks
- **Polysolve**: Numerical optimization
- **ParaView**: Mesh visualization output
- **HDF5**: Data serialization and caching