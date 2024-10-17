#include "periodic_optimization.hpp"

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/utils/Logger.hpp>

#include "internal/PeriodicOptimizationOptions.hpp"
#include "internal/periodic_optimization.hpp"

#include <wmtk/components/multimesh_from_tag/internal/MultiMeshFromTag.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>


namespace wmtk::components {

void periodic_optimization(const utils::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    PeriodicOptimizationOptions options = j.get<PeriodicOptimizationOptions>();

    std::shared_ptr<Mesh> periodic_mesh = cache.read_mesh(options.periodic_mesh);
    assert(periodic_mesh->top_simplex_type() == PrimitiveType::Tetrahedron);

    std::shared_ptr<Mesh> position_mesh = cache.read_mesh(options.position_mesh);
    assert(position_mesh->top_simplex_type() == PrimitiveType::Tetrahedron);

    assert(position_mesh->is_from_same_multi_mesh_structure(*periodic_mesh));

    // register surface child mesh to position mesh
    auto surface_handle =
        position_mesh->register_attribute<int64_t>("surface", PrimitiveType::Triangle, 1);
    auto surface_accessor = position_mesh->create_accessor<int64_t>(surface_handle);

    for (const auto& f : position_mesh->get_all(PrimitiveType::Triangle)) {
        surface_accessor.scalar_attribute(f) =
            position_mesh->is_boundary(PrimitiveType::Triangle, f) ? 1 : 0;
    }

    internal::MultiMeshFromTag getchild(*position_mesh, surface_handle, 1);
    getchild.compute_substructure_mesh();

    std::shared_ptr<Mesh> surface_mesh = position_mesh->get_child_meshes().back();

    getchild.remove_soup();

    assert(position_mesh->get_child_meshes().size() == 1);
    assert(periodic_mesh->get_child_meshes().size() == 1);

    // propagate position
    auto position_handle =
        position_mesh->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    auto surface_position_handle =
        surface_mesh->register_attribute<double>("vertices", PrimitiveType::Vertex, 3);

    auto propagate_to_child_position =
        [](const Eigen::MatrixX<double>& P) -> Eigen::VectorX<double> { return P; };

    auto update_child_positon =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            surface_position_handle,
            position_handle,
            propagate_to_child_position);

    update_child_positon->run_on_all();

    wmtk::logger().info("registered surface mesh to position mesh");

    std::vector<attribute::MeshAttributeHandle> pass_through_attributes;
    pass_through_attributes.push_back(surface_handle);

    wmtk::logger().info("start optimization");

    periodic_optimization(
        *periodic_mesh,
        *position_mesh,
        *surface_mesh,
        options.target_edge_length,
        options.target_max_amips,
        options.passes,
        options.envelope_size,
        options.intermediate_output,
        pass_through_attributes,
        paths.output_dir,
        options.output);

    wmtk::logger().info("finished optimization");

    cache.write_mesh(*periodic_mesh, options.output);
}

} // namespace wmtk::components