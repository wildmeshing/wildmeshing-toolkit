#include "isotropic_remeshing.hpp"

#include <wmtk/TriMesh.hpp>
#include <wmtk/components/base/get_attributes.hpp>
#include <wmtk/utils/Logger.hpp>

#include <Eigen/Geometry>
#include "internal/IsotropicRemeshing.hpp"
#include "internal/IsotropicRemeshingOptions.hpp"

namespace wmtk::components {
// compute the length relative to the bounding box diagonal
double relative_to_absolute_length(
    const attribute::MeshAttributeHandle& pos_handle,
    const double length_rel)
{
    auto pos = pos_handle.mesh().create_const_accessor<double>(pos_handle);
    const auto vertices = pos_handle.mesh().get_all(PrimitiveType::Vertex);
    Eigen::AlignedBox<double, Eigen::Dynamic> bbox(pos.dimension());


    for (const auto& v : vertices) {
        bbox.extend(pos.const_vector_attribute(v));
    }

    const double diag_length = bbox.sizes().norm();

    return length_rel * diag_length;
}


void isotropic_remeshing(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    IsotropicRemeshingOptions options = j.get<IsotropicRemeshingOptions>();

    std::shared_ptr<Mesh> mesh_in = cache.read_mesh(options.input);


    auto pos_handles = base::get_attributes(cache, *mesh_in, options.attributes.position);
    assert(pos_handles.size() == 1);
    auto pos_handle = pos_handles.front();

    if (pos_handle.mesh().top_simplex_type() != PrimitiveType::Face) {
        log_and_throw_error(
            "isotropic remeshing works only for triangle meshes: {}",
            mesh_in->top_simplex_type());
    }

    auto pass_through_attributes = base::get_attributes(cache, *mesh_in, options.pass_through);
    auto other_positions =
        base::get_attributes(cache, *mesh_in, options.attributes.other_positions);

    if (options.length_abs < 0) {
        if (options.length_rel < 0) {
            throw std::runtime_error("Either absolute or relative length must be set!");
        }
        options.length_abs = relative_to_absolute_length(pos_handle, options.length_rel);
    }

    // clear attributes
    std::vector<attribute::MeshAttributeHandle> keeps = pass_through_attributes;
    keeps.emplace_back(pos_handle);
    keeps.insert(keeps.end(), other_positions.begin(), other_positions.end());

    // TODO: brig me back!
    // mesh_in->clear_attributes(keeps);

    // gather handles again as they were invalidated by clear_attributes
    // pos_handles = base::get_attributes(cache, *mesh_in, options.attributes.position);
    // assert(pos_handles.size() == 1);
    // pos_handle = pos_handles.front();
    // pass_through_attributes = base::get_attributes(cache, *mesh_in, options.pass_through);

    std::optional<attribute::MeshAttributeHandle> position_for_inversion;

    if (!options.attributes.inversion_position.empty()) {
        auto tmp = base::get_attributes(cache, *mesh_in, options.attributes.inversion_position);
        assert(tmp.size() == 1);
        position_for_inversion = tmp.front();
    }

    other_positions = base::get_attributes(cache, *mesh_in, options.attributes.other_positions);


    spdlog::info("About to call internal!");
    internal::isotropic_remeshing(
        pos_handle,
        pass_through_attributes,
        options.length_abs,
        options.lock_boundary,
        options.use_for_periodic,
        options.iterations,
        other_positions,
        options.attributes.update_other_positions,
        position_for_inversion);

    spdlog::info("About to write!");
    // output
    cache.write_mesh(*mesh_in, options.output);
    spdlog::info("Finished writing!");
}
} // namespace wmtk::components
