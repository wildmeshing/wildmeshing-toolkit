#include "isotropic_remeshing.hpp"

#include <wmtk/TriMesh.hpp>
#include <wmtk/components/base/get_attributes.hpp>
#include <wmtk/utils/Logger.hpp>

#include "internal/IsotropicRemeshing.hpp"
#include "internal/IsotropicRemeshingOptions.hpp"

namespace wmtk::components {
// compute the length relative to the bounding box diagonal
double relative_to_absolute_length(
    const TriMesh& mesh,
    const attribute::MeshAttributeHandle& pos_handle,
    const double length_rel)
{
    auto pos = mesh.create_const_accessor(pos_handle.as<double>());

    Eigen::Vector3d p_max;
    p_max.setConstant(std::numeric_limits<double>::lowest());
    Eigen::Vector3d p_min;
    p_max.setConstant(std::numeric_limits<double>::max());

    for (const Tuple& v : mesh.get_all(PrimitiveType::Vertex)) {
        const Eigen::Vector3d p = pos.const_vector_attribute(v);
        p_max[0] = std::max(p_max[0], p[0]);
        p_max[1] = std::max(p_max[1], p[1]);
        p_max[2] = std::max(p_max[2], p[2]);
        p_min[0] = std::min(p_min[0], p[0]);
        p_min[1] = std::min(p_min[1], p[1]);
        p_min[2] = std::min(p_min[2], p[2]);
    }

    const double diag_length = (p_max - p_min).norm();

    return length_rel / diag_length;
}


void isotropic_remeshing(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    IsotropicRemeshingOptions options = j.get<IsotropicRemeshingOptions>();

    // input
    std::shared_ptr<Mesh> mesh_in = cache.read_mesh(options.input);

    if (mesh_in->top_simplex_type() != PrimitiveType::Face) {
        log_and_throw_error("Info works only for triangle meshes: {}", mesh_in->top_simplex_type());
    }

    TriMesh& mesh = static_cast<TriMesh&>(*mesh_in);

    auto pos_handle =
        mesh.get_attribute_handle<double>(options.attributes.position, PrimitiveType::Vertex);

    auto pass_through_attributes = base::get_attributes(mesh, options.pass_through);

    if (options.length_abs < 0) {
        if (options.length_rel < 0) {
            throw std::runtime_error("Either absolute or relative length must be set!");
        }
        options.length_abs = relative_to_absolute_length(mesh, pos_handle, options.length_rel);
    }

    // clear attributes
    std::vector<attribute::MeshAttributeHandle> keeps = pass_through_attributes;
    keeps.emplace_back(pos_handle);
    mesh.clear_attributes(keeps);

    // gather handles again as they were invalidated by clear_attributes
    pos_handle =
        mesh.get_attribute_handle<double>(options.attributes.position, PrimitiveType::Vertex);
    pass_through_attributes = base::get_attributes(mesh, options.pass_through);


    IsotropicRemeshing isotropicRemeshing(
        mesh,
        pos_handle,
        pass_through_attributes,
        options.length_abs,
        options.lock_boundary,
        false,
        false,
        true,
        true,
        true,
        true,
        false);
    isotropicRemeshing.remeshing(options.iterations);

    // output
    cache.write_mesh(mesh, options.output);
}
} // namespace wmtk::components
