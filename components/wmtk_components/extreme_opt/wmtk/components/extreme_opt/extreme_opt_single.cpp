#include "extreme_opt_single.hpp"

#include <wmtk/TriMesh.hpp>
#include <wmtk/components/base/get_attributes.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/SeamlessConstraints.hpp>

#include "internal/ExtremeOpt.hpp"
#include "internal/ExtremeOptOptions.hpp"
#include "internal/ExtremeOptSingle.hpp"

namespace wmtk::components {


void extreme_opt_single(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    ExtremeOptOptions options = j.get<ExtremeOptOptions>();

    // input
    std::shared_ptr<Mesh> seamed_mesh_ptr = cache.read_mesh(options.mesh_name + "_seamed");
    std::shared_ptr<TriMesh> cut_mesh_ptr =
        std::static_pointer_cast<TriMesh>(cache.read_mesh(options.mesh_name + "_cut"));

    auto ref_coordinates_handle =
        cut_mesh_ptr->register_attribute<double>("ref_coordinates", wmtk::PrimitiveType::Vertex, 3);
    auto ref_coordinates_acc = cut_mesh_ptr->create_accessor(ref_coordinates_handle.as<double>());

    auto seamed_pos_handle =
        seamed_mesh_ptr->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto seamed_pos = seamed_mesh_ptr->create_const_accessor(seamed_pos_handle.as<double>());

    // TODO: add attribute to cut mesh
    for (const Tuple& v : cut_mesh_ptr->get_all(PrimitiveType::Vertex)) {
        const Eigen::Vector3d p = seamed_pos.const_vector_attribute(v);
        ref_coordinates_acc.vector_attribute(v) = p;
    }

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


    ExtremeOptSingle extreme_opt_single(
        options.mesh_name,
        *cut_mesh_ptr,
        length_abs,
        options.do_split,
        options.do_collapse,
        options.do_swap,
        options.do_smooth,
        options.debug_output,
        options.debug_dir);

    // extreme_opt_single.remeshing(options.iterations);
    // TODO: test on amimps energy
    extreme_opt_single.remeshing_amips(options.iterations);
    std::cout << "finish remeshing" << std::endl;


    io::ParaviewWriter writer(
        "extreme_opt_single_" + options.mesh_name + "_3d_final",
        "ref_coordinates",
        *cut_mesh_ptr,
        true,
        true,
        true,
        false);
    cut_mesh_ptr->serialize(writer);

    io::ParaviewWriter writer2(
        "extreme_opt_single_" + options.mesh_name + "_uv_final",
        "vertices",
        *cut_mesh_ptr,
        true,
        true,
        true,
        false);
    cut_mesh_ptr->serialize(writer2);

    return;
}
} // namespace wmtk::components
