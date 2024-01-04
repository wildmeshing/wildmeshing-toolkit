#include "isotropic_remeshing.hpp"

#include <wmtk/TriMesh.hpp>

#include <wmtk/utils/Logger.hpp>

#include "internal/IsotropicRemeshing.hpp"
#include "internal/IsotropicRemeshingOptions.hpp"

namespace wmtk::components {
// compute the length relative to the bounding box diagonal
double relative_to_absolute_length(const TriMesh& mesh, const double length_rel)
{
    auto pos_handle = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
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

void isotropic_remeshing(const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    IsotropicRemeshingOptions options = j.get<IsotropicRemeshingOptions>();

    // input
    std::shared_ptr<Mesh> mesh_in = cache.read_mesh(options.input);

    if (mesh_in->top_simplex_type() != PrimitiveType::Face) {
        log_and_throw_error("Info works only for triangle meshes: {}", mesh_in->top_simplex_type());
    }

    TriMesh& mesh = static_cast<TriMesh&>(*mesh_in);


    if (options.length_abs < 0) {
        if (options.length_rel < 0) {
            throw std::runtime_error("Either absolute or relative length must be set!");
        }
        options.length_abs = relative_to_absolute_length(mesh, options.length_rel);
    }

    IsotropicRemeshing isotropicRemeshing(
        mesh,
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
