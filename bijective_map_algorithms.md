# Bijective Mapping Algorithms on Tetrahedral Meshes

## Overview

This document provides a comprehensive description of the bijective mapping algorithms implemented in the wildmeshing-toolkit for tetrahedral meshes. The algorithms enable tracking of geometric entities (points, curves, surfaces) through mesh modification operations while preserving bijective correspondence.

## Core Data Structures

### Barycentric Coordinate System

The foundation of our tracking system uses barycentric coordinates to represent positions within tetrahedral elements:

```cpp
struct query_point_tet {
    int64_t t_id;           // Tetrahedral element ID
    Eigen::Vector4d bc;     // Barycentric coordinates (sum = 1.0)
    Eigen::Vector4i tv_ids; // Tet vertex IDs [v0, v1, v2, v3]
};
```

**Key Properties:**
- Barycentric coordinates (α, β, γ, δ) satisfy: α + β + γ + δ = 1.0
- Point position: P = α·V₀ + β·V₁ + γ·V₂ + δ·V₃
- Interior points: all coordinates > 0
- Boundary points: at least one coordinate = 0

### Extended Data Structures

```cpp
struct query_segment_tet {
    int64_t t_id;              // Host tetrahedral element
    Eigen::Vector4d bcs[2];    // Barycentric coords for endpoints
    Eigen::Vector4i tv_ids;    // Tet vertex IDs
};

struct query_curve_tet {
    std::vector<query_segment_tet> segments;    // Connected segments
    std::vector<int> next_segment_ids;          // Connectivity information
};

struct query_triangle_tet {
    int64_t t_id;              // Host tetrahedral element
    Eigen::Vector4d bcs[3];    // Barycentric coords for triangle vertices
    Eigen::Vector4i tv_ids;    // Tet vertex IDs
};

struct query_surface_tet {
    std::vector<query_triangle_tet> triangles;  // Surface triangulation
    // TODO: Add triangle connectivity information
};
```

## Core Algorithm Framework

### 1. Coordinate Conversion Functions

#### Barycentric to World Coordinates
```cpp
Eigen::Vector3d barycentric_to_world_tet(
    const Eigen::Vector4d& bc,
    const Eigen::Matrix<double, 4, 3>& tet_vertices)
{
    return bc[0] * tet_vertices.row(0) + 
           bc[1] * tet_vertices.row(1) + 
           bc[2] * tet_vertices.row(2) + 
           bc[3] * tet_vertices.row(3);
}
```

#### World to Barycentric Coordinates
Uses the `findTetContainingPoint()` function to:
1. Locate the tetrahedral element containing the world point
2. Compute barycentric coordinates within that element
3. Return (-1, invalid) if point is outside the mesh

### 2. Operation Tracking Pipeline

The tracking system processes mesh operations through a standardized pipeline:

```
Input: Operation Log (JSON) + Query Objects
     ↓
Operation Type Detection
     ↓
┌─────────────────┬─────────────────┐
│ MeshConsolidate │ Local Operation │
└─────────────────┴─────────────────┘
     ↓                       ↓
ID Remapping            Geometric Update
     ↓                       ↓
Output: Updated Query Objects
```

## Point Tracking Algorithm

### Algorithm Flow

#### 1. **Operation Classification**
```cpp
void track_point_one_operation_tet(
    const json& operation_log,
    std::vector<query_point_tet>& query_points,
    bool do_forward, bool use_rational)
```

**Input Processing:**
- Parse operation type from JSON log
- Extract before/after mesh states (vertices, tetrahedra, ID mappings)
- Determine processing direction (forward/backward)

#### 2. **Consolidation Operations**
For `MeshConsolidate` operations (mesh cleanup, ID reassignment):

```cpp
void handle_consolidate_tet(
    const std::vector<int64_t>& tet_ids_maps,
    const std::vector<int64_t>& vertex_ids_maps,
    std::vector<query_point_tet>& query_points,
    bool forward)
```

**Forward Mapping (old → new IDs):**
```
For each query_point qp:
    new_tet_id = find_index(tet_ids_maps, qp.t_id)
    For each vertex i in qp.tv_ids:
        new_vertex_id = find_index(vertex_ids_maps, qp.tv_ids[i])
```

**Backward Mapping (new → old IDs):**
```
For each query_point qp:
    old_tet_id = tet_ids_maps[qp.t_id]
    For each vertex i in qp.tv_ids:
        old_vertex_id = vertex_ids_maps[qp.tv_ids[i]]
```

#### 3. **Local Geometric Operations**
For operations that modify mesh topology (split, collapse, swap):

```cpp
void handle_local_mapping_tet(
    V_before, T_before, id_map_before, v_id_map_before,
    V_after, T_after, id_map_after, v_id_map_after,
    query_points)
```

**Detailed Algorithm:**

1. **Geometric Reconstruction:**
   ```cpp
   // Reconstruct world position from current barycentric coordinates
   Eigen::Vector3d world_pos = Eigen::Vector3d::Zero();
   for (int i = 0; i < 4; i++) {
       int global_vertex_id = qp.tv_ids[i];
       int local_vertex_index = find_in_map(v_id_map_after, global_vertex_id);
       world_pos += qp.bc[i] * V_after.row(local_vertex_index);
   }
   ```

2. **Target Mesh Localization:**
   ```cpp
   auto [new_tet_id, new_bc] = findTetContainingPoint(V_before, T_before, world_pos);
   if (new_tet_id == -1) {
       // Point outside mesh - handle error
       mark_as_invalid(qp);
       continue;
   }
   ```

3. **Query Point Update:**
   ```cpp
   qp.t_id = id_map_before[new_tet_id];
   for (int i = 0; i < 4; i++) {
       qp.tv_ids[i] = v_id_map_before[T_before(new_tet_id, i)];
   }
   qp.bc = new_bc;
   ```

#### 4. **Batch Processing**
```cpp
void track_point_tet(
    path operation_logs_dir,
    std::vector<query_point_tet>& query_points,
    bool do_forward, bool use_rational)
```

**Sequential Operation Processing:**
1. Enumerate all operation log files in directory
2. Sort by operation sequence number
3. Process in forward or backward order based on `do_forward` flag
4. Apply each operation sequentially to maintain causality

## Curve Tracking Algorithm

### Data Structure and Connectivity

Curves are represented as sequences of connected segments:
```cpp
struct query_curve_tet {
    std::vector<query_segment_tet> segments;
    std::vector<int> next_segment_ids;  // -1 indicates end of curve
};
```

### Curve Sampling Strategy

#### Tetrahedral Face Sampling
```cpp
auto mid_point_on_face = [](int face_id) -> Eigen::Vector4d {
    switch(face_id) {
        case 0: return {1/3, 1/3, 1/3, 0};   // Face [v0,v1,v2]
        case 1: return {1/3, 1/3, 0, 1/3};   // Face [v0,v1,v3]
        case 2: return {0, 1/3, 1/3, 1/3};   // Face [v1,v2,v3]
        case 3: return {1/3, 0, 1/3, 1/3};   // Face [v0,v2,v3]
    }
};
```

#### Curve Generation Algorithm
```cpp
1. Select random starting tetrahedron
2. Choose random face for segment start
3. Choose different face for segment end
4. Create segment with midpoints of selected faces
5. Move to adjacent tetrahedron through second face
6. Repeat until desired curve length or boundary reached
7. Update connectivity information in next_segment_ids
```

### Curve Tracking Process

The tracking algorithm parallels point tracking but operates on segment collections:

1. **Segment-wise Processing:** Each segment tracked independently using point tracking logic
2. **Connectivity Preservation:** Maintain adjacency relationships through ID remapping
3. **Topology Validation:** Ensure curve remains connected after operations

## Surface Tracking Algorithm

### Surface Representation

Surfaces are embedded as collections of triangles within tetrahedral elements:
```cpp
struct query_surface_tet {
    std::vector<query_triangle_tet> triangles;
    // Future: Add triangle adjacency information
};
```

### Surface Sampling Strategies

#### 1. **Sub-surface Sampling** (Conservative)
```cpp
query_surface_tet sample_query_surface_sub_surface(
    const Eigen::MatrixXd& V, const Eigen::MatrixXi& T)
```

**Algorithm:**
1. Start with tetrahedron 0
2. Add all 4 faces as triangles using vertex barycentric coordinates
3. Use breadth-first traversal to find adjacent tetrahedra
4. Limit sampling to prevent excessive surface complexity
5. Use adjacency test: check for shared edges between tetrahedra

**Triangle Creation:**
```cpp
for each face_id in [0,1,2,3]:
    triangle.bcs[0] = unit_vector(face_vertices[face_id][0])  // (1,0,0,0) etc.
    triangle.bcs[1] = unit_vector(face_vertices[face_id][1])
    triangle.bcs[2] = unit_vector(face_vertices[face_id][2])
```

#### 2. **Large Triangle Sampling** (using Boolean Operations)
```cpp
query_surface_tet sample_query_surface_large_triangle(
    const Eigen::MatrixXd& V, const Eigen::MatrixXi& T)
```

**Algorithm:**
1. **Surface Extraction:** Convert tetrahedra to boundary triangulation
2. **Random Triangle Generation:** Sample 3 random points within tetrahedra
3. **Boolean Arrangement:** Use InteractiveAndRobustMeshBooleans library
4. **Intersection Computation:** Find arrangement of input surface with random triangle
5. **Result Extraction:** Extract intersected triangles and compute barycentric embeddings

**Detailed Steps:**
```cpp
// Convert tetrahedral mesh to surface triangulation
for each tetrahedron t in T:
    for each face f in [0,1,2,3]:
        add_triangle(tet_vertices[f], label=t.id)

// Sample random triangle
for i in [0,1,2]:
    random_tet = select_random_tetrahedron()
    random_bc = generate_random_barycentric_coordinates()
    sampled_points[i] = interpolate(random_tet, random_bc)

// Perform boolean arrangement
FastTrimesh tm = create_arrangement(surface_triangles, sampled_triangle)
extract_result_triangles(tm, labels, output_triangles)

// Convert back to barycentric representation
for each output_triangle ot:
    containing_tet = find_containing_tetrahedron(ot.vertices)
    for each vertex v in ot:
        barycentric_coords[v] = world_to_barycentric(v, containing_tet)
```

### Surface Tracking Process

#### 1. **Triangle-wise Tracking**
```cpp
void handle_local_mapping_tet_surface(...)
```

Each triangle in the surface is tracked using the same geometric reconstruction approach as point tracking:

```cpp
for each triangle in surface.triangles:
    for each vertex v in [0,1,2]:
        world_pos[v] = barycentric_to_world(triangle.bcs[v], triangle.tv_ids)
        new_location[v] = world_to_barycentric(world_pos[v], target_mesh)
    update_triangle_representation(triangle, new_location)
```

#### 2. **Manifold Validation**
```cpp
// Post-tracking validation
std::map<edge, int> edge_counts;
for each triangle in tracked_surface:
    for each edge in triangle:
        edge_counts[edge]++

// Check manifold property
for each edge in edge_counts:
    if edge_counts[edge] > 2:
        report_non_manifold_edge(edge)
```

## Algorithm Robustness and Error Handling

### 1. **Geometric Degeneracies**
- **Near-zero barycentric coordinates:** Clamp to exactly 0.0 if |bc| < 1e-15
- **Point outside mesh:** Mark as invalid (-1 tet_id) and continue processing
- **Degenerate tetrahedra:** Skip processing for elements with near-zero volume

### 2. **Topological Consistency**
- **ID mapping validation:** Ensure all referenced vertices/tetrahedra exist in target mesh
- **Connectivity preservation:** Maintain curve and surface adjacency relationships
- **Boundary handling:** Special treatment for entities near mesh boundaries

### 3. **Numerical Stability**
- **Coordinate normalization:** Ensure barycentric coordinates sum to 1.0
- **Precision management:** Use consistent tolerance values across all computations
- **Overflow prevention:** Validate array indices before access

## Performance Optimizations

### 1. **Parallel Processing**
```cpp
// Point tracking parallelization
igl::parallel_for(query_points.size(), [&](int id) {
    track_single_point(query_points[id], operation_data);
});
```

### 2. **Spatial Data Structures**
- **Octree acceleration:** For point location in large meshes
- **BVH optimization:** Hierarchical bounding volumes for intersection tests
- **Caching strategies:** Reuse geometric computations where possible

### 3. **Memory Management**
- **In-place updates:** Modify query objects directly to reduce memory allocation
- **Batch processing:** Group operations to improve cache locality
- **Sparse representations:** Use efficient data structures for large but sparse operations

## Applications and Use Cases

### 1. **Mesh Evolution Tracking**
- Track material points through adaptive remeshing
- Preserve feature curves during topology optimization
- Maintain surface boundaries in deformation processes

### 2. **Multi-physics Simulations**
- Track embedded sensors through mesh adaptation
- Preserve contact surfaces in collision detection
- Maintain material interfaces in multi-material problems

### 3. **Geometric Processing**
- Feature preservation in mesh simplification
- Texture coordinate maintenance through remeshing
- Boundary condition transfer between mesh hierarchies

## Implementation Status and Future Work

### Current Implementation
- ✅ **Point tracking:** Fully implemented with consolidation and local mapping
- ✅ **Curve tracking:** Basic implementation with face-based sampling
- ✅ **Surface tracking:** Two sampling strategies implemented
- ✅ **File I/O:** VTU export for visualization, JSON serialization

### Required Improvements
- 🔄 **Robustness:** Better handling of degenerate cases and boundary conditions
- 🔄 **Performance:** Parallelization of curve and surface tracking
- 🔄 **Validation:** Comprehensive testing with complex operation sequences
- 🔄 **Documentation:** API documentation and usage examples

### Future Extensions
- 📋 **Higher-order elements:** Support for quadratic tetrahedra
- 📋 **Adaptive sampling:** Dynamic refinement based on local geometry
- 📋 **Error metrics:** Quantitative assessment of tracking accuracy
- 📋 **GPU acceleration:** CUDA implementation for large-scale problems