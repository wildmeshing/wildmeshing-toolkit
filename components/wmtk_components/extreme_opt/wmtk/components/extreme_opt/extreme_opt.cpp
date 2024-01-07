#include "extreme_opt.hpp"

#include <wmtk/TriMesh.hpp>
#include <wmtk/components/base/get_attributes.hpp>
#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>
#include <wmtk/utils/SeamlessConstraints.hpp>

#include <wmtk/utils/Logger.hpp>

#include "internal/ExtremeOpt.hpp"
#include "internal/ExtremeOptOptions.hpp"

namespace wmtk::components {


void extreme_opt(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    ExtremeOptOptions options = j.get<ExtremeOptOptions>();

    // input
    std::shared_ptr<Mesh> seamed_mesh_ptr = cache.read_mesh(options.mesh_name + "_seamed");
    TriMesh& seamed_mesh = static_cast<TriMesh&>(*seamed_mesh_ptr);
    std::shared_ptr<TriMesh> cut_mesh_ptr =
        std::static_pointer_cast<TriMesh>(cache.read_mesh(options.mesh_name + "_cut"));

    auto child_map = multimesh::same_simplex_dimension_bijection(seamed_mesh, *cut_mesh_ptr);
    seamed_mesh.register_child_mesh(cut_mesh_ptr, child_map);

    /*
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


        ExtremeOpt extreme_opt(
            mesh_name,
            seamed_mesh,
            length_abs,
            options.lock_boundary,
            options.do_split,
            options.do_collapse,
            options.collapse_optimize_E_max,
            options.do_swap,
            options.swap_optimize_E_max,
            options.do_smooth,
            options.debug_output);
        extreme_opt.remeshing(options.iterations);

        // output
        cache.write_mesh(mesh, options.output);
        */
}
} // namespace wmtk::components
