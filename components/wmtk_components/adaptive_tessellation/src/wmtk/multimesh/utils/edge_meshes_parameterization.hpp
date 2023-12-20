#pragma once
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/Simplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
using namespace wmtk;
namespace wmtk::components::adaptive_tessellation::multimesh::utils {

std::pair<Tuple, Tuple> get_ends_of_edge_mesh(const EdgeMesh& edge_mesh);

// the edge mesh is parameterized by the variable t.
// t = 0 at one end of the edge mesh, and t = sum_edge_mesh_arclength at the other end
void parameterize_edge_mesh(
    EdgeMesh& edge_mesh,
    const TriMesh& uv_mesh,
    MeshAttributeHandle<double>& t_handle,
    const MeshAttributeHandle<double>& uv_handle);

// two sibling edge meshes are parameterized by the same variable t.
void parameterize_seam_edge_meshes(
    EdgeMesh& edge_mesh1,
    EdgeMesh& edge_mesh2,
    const TriMesh& uv_mesh,
    MeshAttributeHandle<double>& t1_handle,
    MeshAttributeHandle<double>& t2_handle,
    MeshAttributeHandle<double>& uv_handle);
} // namespace wmtk::components::adaptive_tessellation::multimesh::utils