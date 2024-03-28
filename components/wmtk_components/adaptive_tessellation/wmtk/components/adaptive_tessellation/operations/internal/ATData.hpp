#pragma once
#include <map>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/PerTriangleAnalyticalIntegral.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/PerTriangleTextureIntegralAccuracyFunction.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/PositionMapAMIPS.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/IntegralBasedAvgDistance.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/ThreeChannelPositionMapEvaluator.hpp>
#include <wmtk/components/adaptive_tessellation/function/utils/Triangle2DTo3DMapping.hpp>
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
    wmtk::attribute::MeshAttributeHandle m_pmesh_xyz_handle;
    wmtk::attribute::MeshAttributeHandle m_distance_error_handle;
    wmtk::attribute::MeshAttributeHandle m_curved_edge_length_handle;
    wmtk::attribute::MeshAttributeHandle m_face_rgb_state_handle;
    wmtk::attribute::MeshAttributeHandle m_edge_rgb_state_handle;
    wmtk::attribute::MeshAttributeHandle m_edge_todo_handle;

    std::shared_ptr<wmtk::components::function::utils::ThreeChannelPositionMapEvaluator>
        m_evaluator_ptr;

    std::shared_ptr<wmtk::components::function::utils::Triangle2DTo3DMapping> m_mapping_ptr;

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

    // default constructor
    ATData() = default;

    /**
     * @brief Construct a ATData with images for position mapping. The distance to limit surface is
     * set to max distance of every triangle if max_distance is true. This requires the image
     * sampler to be bilinear. Otherwise it uses the average distance of every triangle
     *
     * @param position_mesh_ptr
     * @param uv_mesh_ptr
     * @param images
     * @param max_distance
     */
    ATData(
        std::shared_ptr<Mesh> position_mesh_ptr,
        std::shared_ptr<Mesh> uv_mesh_ptr,
        std::array<std::shared_ptr<image::Image>, 3>& images,
        bool max_distance);

    /**
     * @brief Construct a new ATData object by loading images from the given paths.
     *
     * @param position_mesh_ptr
     * @param uv_mesh_ptr
     * @param position_path
     * @param normal_path
     * @param height_path
     * @param max_distance
     */
    ATData(
        std::shared_ptr<Mesh> position_mesh_ptr,
        std::shared_ptr<Mesh> uv_mesh_ptr,
        const std::filesystem::path& position_path,
        const std::filesystem::path& normal_path,
        const std::filesystem::path& height_path,
        bool max_distance);


    /**
     * @brief Construct a ATData with analytical functions for position mapping. The distance to
     * limit surface is set to average distance of every triangle
     *
     * @param position_mesh_ptr
     * @param uv_mesh
     * @param funcs
     */
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
    const std::shared_ptr<wmtk::components::function::utils::ThreeChannelPositionMapEvaluator>&
    evaluator_ptr() const;
    const std::shared_ptr<wmtk::components::function::utils::Triangle2DTo3DMapping>& mapping_ptr()
        const;
    void _debug_sampling(
        wmtk::components::function::utils::ThreeChannelPositionMapEvaluator& image_sampling,
        wmtk::components::function::utils::ThreeChannelPositionMapEvaluator& func_eval) const;
};
} // namespace wmtk::components::operations::internal