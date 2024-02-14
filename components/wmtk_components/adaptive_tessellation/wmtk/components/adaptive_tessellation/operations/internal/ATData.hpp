#pragma once
#include <map>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/PerTriangleAnalyticalIntegral.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/PerTriangleTextureIntegralAccuracyFunction.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/PositionMapAMIPS.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/IntegralBase.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/ThreeChannelPositionMapEvaluator.hpp>
#include <wmtk/components/adaptive_tessellation/image/Image.hpp>
#include <wmtk/components/adaptive_tessellation/image/Sampling.hpp>
#include <wmtk/function/PerSimplexFunction.hpp>
#include <wmtk/function/simplex/TriangleAMIPS.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>
#include <wmtk/simplex/Simplex.hpp>


namespace wmtk::components::operations::internal {
using namespace wmtk::simplex;
class ATData
{
    std::shared_ptr<Mesh> m_position_mesh_ptr;
    std::shared_ptr<Mesh> m_uv_mesh_ptr;

    std::vector<std::shared_ptr<Mesh>> m_edge_mesh_ptrs;
    std::map<Mesh*, Mesh*> m_sibling_meshes_map;
    std::array<std::shared_ptr<image::Image>, 3> m_images = {{nullptr, nullptr, nullptr}};
    std::array<std::shared_ptr<image::Sampling>, 3> m_funcs = {{nullptr, nullptr, nullptr}};

public:
    // Tnvariants are dependant on the input mesh where the operation is defined one (interior
    // edge op input_m = uv_mesh, boundary edge op input_m = position_mesh) The invariant that
    // is shared among the operations besides the base invariants is the no-triangle-inversion
    // of the uv_mesh

    // handle to vertex uv coordinates used for the uv non-inversion invariants
    wmtk::attribute::MeshAttributeHandle m_uv_handle;
    wmtk::attribute::MeshAttributeHandle m_uvmesh_xyz_handle;
    wmtk::attribute::MeshAttributeHandle m_distance_error_handle;
    wmtk::attribute::MeshAttributeHandle m_sum_error_handle;
    wmtk::attribute::MeshAttributeHandle m_barrier_energy_handle;
    wmtk::attribute::MeshAttributeHandle m_amips_error_handle;

    wmtk::attribute::MeshAttributeHandle m_3d_edge_length_handle;
    // wmtk::attribute::MeshAttributeHandle m_edge_priority_handle;

    // ATData(
    //     std::shared_ptr<Mesh> uv_mesh,
    //     std::shared_ptr<Mesh> position_mesh,
    //     std::vector<std::shared_ptr<Mesh>> edge_mesh_ptrs,
    //     std::map<Mesh*, Mesh*> sibling_meshes_map,
    //     std::array<std::shared_ptr<image::Image>, 3>& images);
    // ATData(
    //     std::shared_ptr<Mesh> uv_mesh,
    //     std::shared_ptr<Mesh> position_mesh,
    //     std::array<std::shared_ptr<image::Image>, 3>& images);

    ATData(
        std::shared_ptr<Mesh> position_mesh_ptr,
        std::shared_ptr<Mesh> uv_mesh_ptr,
        std::array<std::shared_ptr<image::Image>, 3>& images);
    ATData(
        std::shared_ptr<Mesh> position_mesh_ptr,
        std::shared_ptr<Mesh> uv_mesh_ptr,
        const std::filesystem::path& position_path,
        const std::filesystem::path& normal_path,
        const std::filesystem::path& height_path);
    ATData(
        std::shared_ptr<Mesh> position_mesh_ptr,
        std::shared_ptr<Mesh> uv_mesh,
        std::array<std::shared_ptr<image::Sampling>, 3>& funcs);


    void initialize_handles();
    wmtk::attribute::MeshAttributeHandle uv_handle();
    Mesh& uv_mesh() const;
    Mesh& position_mesh() const;
    std::shared_ptr<Mesh> uv_mesh_ptr() const;
    std::shared_ptr<Mesh> position_mesh_ptr() const;
    std::shared_ptr<Mesh> edge_mesh_i_ptr(int64_t i) const;
    int64_t num_edge_meshes() const;
    Mesh* sibling_edge_mesh_ptr(Mesh* my_edge_mesh_ptr);
    Simplex sibling_edge(Mesh* my_edge_mesh_ptr, const Simplex& s);
    const std::array<std::shared_ptr<image::Image>, 3>& images() const;
    const std::array<std::shared_ptr<image::Sampling>, 3>& funcs() const;

    void _debug_sampling(
        wmtk::components::function::utils::ThreeChannelPositionMapEvaluator& image_sampling,
        wmtk::components::function::utils::ThreeChannelPositionMapEvaluator& func_eval) const;
};
} // namespace wmtk::components::operations::internal