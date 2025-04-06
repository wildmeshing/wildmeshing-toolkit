
#include "MapValidator.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/simplex/utils/SimplexComparisons.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/primitive_range.hpp>
namespace wmtk::multimesh::utils {

MapValidator::MapValidator(const Mesh& m)
    : m_mesh(m)
{}
bool MapValidator::check_all() const
{
    bool ok = true;
    ok &= check_parent_map_attribute_valid();
    ok &= check_child_map_attributes_valid();
    if (!ok) {
        return ok;
    }
    ok &= check_switch_homomorphism();

    return ok;
}

bool MapValidator::check_child_map_attributes_valid() const
{
    bool ok = true;
    for (const auto& [cptr, attr] : m_mesh.m_multi_mesh_manager.m_children) {
        const auto& child = *cptr;
        auto map_accessor = m_mesh.create_const_accessor(attr);

        for (int64_t j = 0; j < child.top_cell_dimension(); ++j) {
            wmtk::PrimitiveType prim_type = wmtk::PrimitiveType(j);

            for (const auto& pt : m_mesh.get_all(prim_type)) {
                wmtk::simplex::Simplex s(m_mesh, prim_type, pt);

                auto tups = simplex::top_dimension_cofaces_tuples(m_mesh, s);

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
                                fmt::join(m_mesh.absolute_multi_mesh_id(), ","),
                                fmt::join(child.absolute_multi_mesh_id(), ","),
                                j,
                                pt.as_string(),
                                source_mesh_base_tuple.as_string(),
                                target_mesh_base_tuple.as_string()

                            );
                        }
                    } else if (!child.is_valid(target_mesh_base_tuple)) {
                        wmtk::logger().error(
                            "Map from parent {} to child {} on tuple {} (dim {}) fails on  {} -> "
                            "{}",
                            fmt::join(m_mesh.absolute_multi_mesh_id(), ","),
                            fmt::join(child.absolute_multi_mesh_id(), ","),
                            j,
                            pt.as_string(),
                            source_mesh_base_tuple.as_string(),
                            target_mesh_base_tuple.as_string()

                        );
                        ok = false;
                    }
                }
            }
        }
    }
    return ok;
}

bool MapValidator::check_parent_map_attribute_valid() const
{
    bool ok = true;
    const auto& parent_ptr = m_mesh.m_multi_mesh_manager.m_parent;
    if (parent_ptr == nullptr) {
        return true;
    }
    const auto& attr = m_mesh.m_multi_mesh_manager.map_to_parent_handle;
    const auto& parent = *parent_ptr;
    auto map_accessor = m_mesh.create_const_accessor(attr);

    wmtk::PrimitiveType prim_type = m_mesh.top_simplex_type();

    for (const auto& source_tuple : m_mesh.get_all(prim_type)) {
        const auto [source_mesh_base_tuple, target_mesh_base_tuple] =
            multimesh::utils::read_tuple_map_attribute(map_accessor, source_tuple);
        if (source_mesh_base_tuple.is_null() || target_mesh_base_tuple.is_null()) {
            wmtk::logger().error(
                "Map from child {} to parent {} on tuple {} (dim {}) has null entry {} -> "
                "{}",
                fmt::join(m_mesh.absolute_multi_mesh_id(), ","),
                fmt::join(parent.absolute_multi_mesh_id(), ","),
                m_mesh.top_cell_dimension(),
                source_tuple.as_string(),
                source_mesh_base_tuple.as_string(),
                target_mesh_base_tuple.as_string()

            );
            ok = false;
        } else if (!parent.is_valid(target_mesh_base_tuple)) {
            wmtk::logger().error(
                "Map from child {} to parent {} on tuple {} (dim {}) fails on  {} -> "
                "{}",
                fmt::join(m_mesh.absolute_multi_mesh_id(), ","),
                fmt::join(parent.absolute_multi_mesh_id(), ","),
                m_mesh.top_cell_dimension(),
                source_tuple.as_string(),
                source_mesh_base_tuple.as_string(),
                target_mesh_base_tuple.as_string()

            );
            ok = false;
        }
    }
    return ok;
}


bool MapValidator::check_switch_homomorphism() const
{
    bool ok = true;
    for (const auto& cptr : m_mesh.get_child_meshes()) {
        ok &= check_child_switch_homomorphism(*cptr);
    }
    return ok;
}
bool MapValidator::check_child_switch_homomorphism(const Mesh& child) const
{
    assert(child.m_multi_mesh_manager.m_parent == &m_mesh);
    bool ok = true;
    for (PrimitiveType pt : wmtk::utils::primitive_below(child.top_simplex_type())) {
        auto tups = child.get_all(pt);
        for (const wmtk::Tuple& t : tups) {
            wmtk::simplex::Simplex s(child, pt, t);

            wmtk::Tuple parent_tuple = child.map_to_parent_tuple(s);
            for (PrimitiveType spt : wmtk::utils::primitive_below(child.top_simplex_type())) {
                // skip switches over boundaries
                if (spt == pt &&
                    child.is_boundary(wmtk::PrimitiveType(child.top_cell_dimension() - 1), t)) {
                    continue;
                }

                wmtk::simplex::Simplex switched_simplex(child, pt, m_mesh.switch_tuple(t, pt));
                wmtk::Tuple switch_map = child.map_to_parent_tuple(switched_simplex);
                wmtk::Tuple map_switch = m_mesh.switch_tuple(parent_tuple, pt);
                for (PrimitiveType my_pt : wmtk::utils::primitive_below(child.top_simplex_type())) {
                    bool worked = wmtk::simplex::utils::SimplexComparisons::equal(
                        m_mesh,
                        my_pt,
                        switch_map,
                        map_switch);
                    if (!worked) {
                        wmtk::logger().error(
                            "Map from child {0} to parent {1} on tuple {2} (dim {3}) fails on  "
                            "switch(map({2}),{4}) = {5} !=  "
                            "map(switch({2},{4})) = {6} (dim {7} id {8} != {9})",
                            fmt::join(child.absolute_multi_mesh_id(), ","), // 0
                            fmt::join(m_mesh.absolute_multi_mesh_id(), ","), // 1
                            t.as_string(), // 2
                            child.top_cell_dimension(), // 3
                            primitive_type_name(pt), // 4
                            map_switch.as_string(), // 5
                            switch_map.as_string(), // 6
                            primitive_type_name(my_pt), // 7
                            m_mesh.id(map_switch, my_pt), // 8
                            m_mesh.id(switch_map, my_pt) // 9
                        );
                        ok = false;
                    }
                }
            }
        }
    }
    return ok;
}


} // namespace wmtk::multimesh::utils
