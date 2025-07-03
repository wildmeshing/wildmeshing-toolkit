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

#### Point Location using Orient3D + Rational Arithmetic

**Robust Point-in-Tetrahedron Test**:
For point $\mathbf{p}$ and tetrahedron $T = \{\mathbf{v}_0, \mathbf{v}_1, \mathbf{v}_2, \mathbf{v}_3\}$:

$$\text{orient3d}(\mathbf{v}_i, \mathbf{v}_j, \mathbf{v}_k, \mathbf{p}) \geq 0 \quad \forall \text{ faces } (i,j,k)$$

Using exact rational arithmetic ensures numerical stability. The algorithm:
1. Compute orientation for each of the 4 tetrahedral faces
2. Point is inside if all orientations have consistent sign
3. Rational arithmetic eliminates floating-point errors
4. Compute barycentric coordinates using Cramer's rule with rational determinants

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
   // Use robust point location method for numerical stability
   auto [new_tet_id, new_bc] = findTetContainingPointOrient3d(V_before, T_before, world_pos);
   if (new_tet_id == -1) {
       // Point outside mesh - error message already printed by findTetContainingPointOrient3d
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

### Curve Tracking Algorithm

#### Segment Transformation
For each segment with endpoints in barycentric coordinates, track through mesh operations by reconstructing world coordinates and relocating in target mesh.

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

### Surface Tracking with Arrangement Labels

#### Boolean Arrangement Method
Surfaces are tracked using mesh arrangement where each triangle carries a label indicating its source tetrahedron. During tracking:

1. **Label Preservation**: Each triangle maintains its original tetrahedron ID through the arrangement process
2. **Intersection Handling**: When surfaces intersect, new triangles inherit labels from their geometric location
3. **Barycentric Reconstruction**: Use arrangement labels to determine containing tetrahedra for barycentric coordinate computation

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
Each edge must appear in at most 2 triangles to maintain manifold property.

## Algorithm Robustness and Error Handling

### 1. **Geometric Degeneracies**
- **Point outside mesh:** Mark as invalid (-1 tet_id) and continue processing
- **Degenerate tetrahedra:** Skip processing for elements with near-zero volume

### 2. **Topological Consistency**
- **ID mapping validation:** Ensure all referenced vertices/tetrahedra exist in target mesh
- **Connectivity preservation:** Maintain curve and surface adjacency relationships
- **Boundary handling:** Special treatment for entities near mesh boundaries

### 3. **Numerical Stability**
- **Exact predicates:** Use `wmtk::utils::wmtk_orient3d` with rational arithmetic
- **Determinant computation:** Rational Cramer's rule for barycentric coordinates
- **Overflow prevention:** Validate array indices before access

## Performance Optimizations

### Performance Optimizations
- **Parallel processing:** Independent tracking of multiple entities
- **Caching:** Reuse geometric computations
- **In-place updates:** Modify query objects directly

## Applications
- Material point tracking through adaptive remeshing
- Feature preservation in mesh operations
- Multi-material interface maintenance

