#include "extreme_opt.hpp"

#include <wmtk/TriMesh.hpp>
#include <wmtk/components/base/get_attributes.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/SeamlessConstraints.hpp>

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


    // get target length for remeshing ON CUT MESH
    double length_rel = options.length_rel;
    double length_abs = 0;
    {
        auto pos_handle =
            cut_mesh_ptr->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
        auto pos = cut_mesh_ptr->create_const_accessor(pos_handle.as<double>());

        Eigen::Vector2d p_max;
        p_max.setConstant(std::numeric_limits<double>::lowest());
        Eigen::Vector2d p_min;
        p_min.setConstant(std::numeric_limits<double>::max());

        for (const Tuple& v : cut_mesh_ptr->get_all(PrimitiveType::Vertex)) {
            const Eigen::Vector2d p = pos.const_vector_attribute(v);
            p_max[0] = std::max(p_max[0], p[0]);
            p_max[1] = std::max(p_max[1], p[1]);
            p_min[0] = std::min(p_min[0], p[0]);
            p_min[1] = std::min(p_min[1], p[1]);
        }
        const double diag_length = (p_max - p_min).norm();
        length_abs = diag_length * length_rel;
        std::cout << "length_abs: " << length_abs << std::endl;
    }

    if (!wmtk::utils::check_constraints(
            seamed_mesh,
            *cut_mesh_ptr,
            cut_mesh_ptr->get_attribute_handle_typed<double>("vertices", PrimitiveType::Vertex))) {
        std::cout << "Input does not satisfying the seamless constraints!" << std::endl;
        return;
    };

    ExtremeOpt extreme_opt(
        options.mesh_name,
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
    std::cout << "finish remeshing" << std::endl;
    std::cout << "to check constraints" << std::endl;
    wmtk::utils::check_constraints(
        seamed_mesh,
        *cut_mesh_ptr,
        cut_mesh_ptr->get_attribute_handle_typed<double>("vertices", PrimitiveType::Vertex));

    // TODO: Figure out if it is possible to write with cache?
    // write final result to file
    io::ParaviewWriter writer(
        "extreme_opt_" + options.mesh_name + "seamed_final",
        "vertices",
        seamed_mesh,
        true,
        true,
        true,
        false);
    seamed_mesh.serialize(writer);
    io::ParaviewWriter writer2(
        "extreme_opt_" + options.mesh_name + "cut_final",
        "vertices",
        *cut_mesh_ptr,
        true,
        true,
        true,
        false);
    cut_mesh_ptr->serialize(writer2);

    return;
    /*
        // clear attributes
        std::vector<attribute::MeshAttributeHandle> keeps = pass_through_attributes;
        keeps.emplace_back(pos_handle);
        mesh.clear_attributes(keeps);

        // gather handles again as they were invalidated by clear_attributes
        pos_handle =
            mesh.get_attribute_handle<double>(options.attributes.position, PrimitiveType::Vertex);
        pass_through_attributes = base::get_attributes(mesh, options.pass_through);

        // output
        cache.write_mesh(mesh, options.output);
        */
}
} // namespace wmtk::components
