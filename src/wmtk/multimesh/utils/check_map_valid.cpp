
#include <wmtk/Mesh.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleInspector.hpp>
namespace wmtk::multimesh::utils {
bool check_child_maps_valid(const Mesh& m)
{
    bool ok = true;
    for (const auto& [cptr, attr] : m.m_multi_mesh_manager.m_children) {
        const auto& child = *cptr;
        auto map_accessor = m.create_const_accessor(attr);

        for (int64_t j = 0; j < child.top_cell_dimension(); ++j) {
            wmtk::PrimitiveType prim_type = wmtk::PrimitiveType(j);

            for (const auto& pt : m.get_all(prim_type)) {
                wmtk::simplex::Simplex s(m, prim_type, pt);

                auto tups = simplex::top_dimension_cofaces_tuples(m, s);

                for (const auto& source_tuple : tups) {
                    const auto [source_mesh_base_tuple, target_mesh_base_tuple] =
                        multimesh::utils::read_tuple_map_attribute(map_accessor, source_tuple);
                    if (source_mesh_base_tuple.is_null() || target_mesh_base_tuple.is_null()) {
                        if (source_mesh_base_tuple.is_null() && target_mesh_base_tuple.is_null()) {
                            ok = false;
                            wmtk::logger().error(
                                "Map from parent {} to child {} on tuple {} (dim {}) fails on  {} "
                                "or "
                                "{} null",
                                fmt::join(m.absolute_multi_mesh_id(), ","),
                                fmt::join(child.absolute_multi_mesh_id(), ","),
                                j,
                                wmtk::utils::TupleInspector::as_string(pt),
                                wmtk::utils::TupleInspector::as_string(source_mesh_base_tuple),
                                wmtk::utils::TupleInspector::as_string(target_mesh_base_tuple)

                            );
                        }
                    } else if (!child.is_valid_slow(target_mesh_base_tuple)) {
                        wmtk::logger().error(
                            "Map from parent {} to child {} on tuple {} (dim {}) fails on  {} -> "
                            "{}",
                            fmt::join(m.absolute_multi_mesh_id(), ","),
                            fmt::join(child.absolute_multi_mesh_id(), ","),
                            j,
                            wmtk::utils::TupleInspector::as_string(pt),
                            wmtk::utils::TupleInspector::as_string(source_mesh_base_tuple),
                            wmtk::utils::TupleInspector::as_string(target_mesh_base_tuple)

                        );
                        ok = false;
                    }
                }
            }
        }
    }
    return ok;
}

bool check_parent_map_valid(const Mesh& m)
{
    bool ok = true;
    const auto& parent_ptr = m.m_multi_mesh_manager.m_parent;
    if (parent_ptr == nullptr) {
        return true;
    }
    const auto& attr = m.m_multi_mesh_manager.map_to_parent_handle;
    const auto& parent = *parent_ptr;
    auto map_accessor = m.create_const_accessor(attr);

    wmtk::PrimitiveType prim_type = m.top_simplex_type();

    for (const auto& source_tuple : m.get_all(prim_type)) {
        const auto [source_mesh_base_tuple, target_mesh_base_tuple] =
            multimesh::utils::read_tuple_map_attribute(map_accessor, source_tuple);
        if (source_mesh_base_tuple.is_null() || target_mesh_base_tuple.is_null()) {
            wmtk::logger().error(
                "Map from child {} to parent {} on tuple {} (dim {}) has null entry {} -> "
                "{}",
                fmt::join(m.absolute_multi_mesh_id(), ","),
                fmt::join(parent.absolute_multi_mesh_id(), ","),
                m.top_cell_dimension(),
                wmtk::utils::TupleInspector::as_string(source_tuple),
                wmtk::utils::TupleInspector::as_string(source_mesh_base_tuple),
                wmtk::utils::TupleInspector::as_string(target_mesh_base_tuple)

            );
            ok = false;
        } else if (!parent.is_valid_slow(target_mesh_base_tuple)) {
            wmtk::logger().error(
                "Map from child {} to parent {} on tuple {} (dim {}) fails on  {} -> "
                "{}",
                fmt::join(m.absolute_multi_mesh_id(), ","),
                fmt::join(parent.absolute_multi_mesh_id(), ","),
                m.top_cell_dimension(),
                wmtk::utils::TupleInspector::as_string(source_tuple),
                wmtk::utils::TupleInspector::as_string(source_mesh_base_tuple),
                wmtk::utils::TupleInspector::as_string(target_mesh_base_tuple)

            );
            ok = false;
        }
    }
    return ok;
}
} // namespace wmtk::multimesh::utils
