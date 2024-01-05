#pragma once
#include <map>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/components/adaptive_tessellation/image/Image.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk::components::adaptive_tessellation::operations::internal {
using namespace wmtk::simplex;
class ATData
{
    std::shared_ptr<TriMesh> m_uv_mesh_ptr;
    std::shared_ptr<TriMesh> m_position_mesh_ptr;
    std::vector<std::shared_ptr<Mesh>> m_edge_mesh_ptrs;
    std::map<Mesh*, Mesh*> m_sibling_meshes_map;
    std::array<image::Image, 3>& m_images;

public:
    // Tnvariants are dependant on the input mesh where the operation is defined one (interior
    // edge op input_m = uv_mesh, boundary edge op input_m = position_mesh) The invariant that
    // is shared among the operations besides the base invariants is the no-triangle-inversion
    // of the uv_mesh

    // handle to vertex uv coordinates used for the uv non-inversion invariants
    wmtk::attribute::MeshAttributeHandle m_uv_handle;
    wmtk::attribute::MeshAttributeHandle m_uv_edge_length_handle;

    // Scheduler m_scheduler;

    ATData(
        std::shared_ptr<TriMesh> uv_mesh,
        std::shared_ptr<TriMesh> position_mesh,
        std::vector<std::shared_ptr<Mesh>> edge_mesh_ptrs,
        std::map<Mesh*, Mesh*> sibling_meshes_map,
        std::array<image::Image, 3>& images);
    ATData(
        std::shared_ptr<TriMesh> uv_mesh,
        std::shared_ptr<TriMesh> position_mesh,
        std::array<image::Image, 3>& images);

    ATData(std::shared_ptr<TriMesh> uv_mesh, std::array<image::Image, 3>& images);

    wmtk::attribute::MeshAttributeHandle& uv_handle();
    TriMesh& uv_mesh() const;
    TriMesh& position_mesh() const;
    std::shared_ptr<Mesh> edge_mesh_i_ptr(int64_t i) const;
    int64_t num_edge_meshes() const;
    Mesh* sibling_edge_mesh_ptr(Mesh* my_edge_mesh_ptr);
    Simplex sibling_edge(Mesh* my_edge_mesh_ptr, const Simplex& s);
    const std::array<image::Image, 3>& images() const;
};
} // namespace wmtk::components::adaptive_tessellation::operations::internal