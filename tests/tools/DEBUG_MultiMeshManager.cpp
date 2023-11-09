#include "DEBUG_MultiMeshManager.hpp"
#include <catch2/catch_test_macros.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
#include <wmtk/utils/TupleInspector.hpp>
#include "DEBUG_Mesh.hpp"


namespace wmtk::tests {
void DEBUG_MultiMeshManager::check_child_mesh_valid(const Mesh& my_mesh, const Mesh& child_mesh)
    const
{
    // TODO: implement this
}


void DEBUG_MultiMeshManager::check_map_valid(const Mesh& my_mesh) const
{
    for (long index = 0; index < long(children().size()); ++index) {
        const auto& child_data = children()[index];
        REQUIRE(bool(child_data.mesh));
        REQUIRE(child_data.mesh->absolute_multi_mesh_id().front() == index);
        check_child_map_valid(my_mesh, child_data);
    }
}
void DEBUG_MultiMeshManager::check_child_map_valid(const Mesh& my_mesh, const ChildData& child_data)
    const
{
    const Mesh& child_mesh = *child_data.mesh;
    const auto parent_to_child_handle = child_data.map_handle;
    PrimitiveType map_type = child_mesh.top_simplex_type();

    const std::string c_to_p_name = child_to_parent_map_attribute_name();

    REQUIRE(child_mesh.has_attribute<long>(c_to_p_name, map_type));
    auto child_to_parent_handle = child_mesh.get_attribute_handle<long>(c_to_p_name, map_type);
    auto child_cell_flag_accessor = child_mesh.get_flag_accessor(map_type);

    auto all_child_tuples = child_mesh.get_all(map_type);

    for (const Tuple& child_tuple : all_child_tuples) {
        spdlog::info(
            "[{} -> {}] Checking child tuple {}",
            fmt::join(absolute_id(), ","),
            fmt::join(child_mesh.absolute_multi_mesh_id(), ","),
            wmtk::utils::TupleInspector::as_string(child_tuple));
        // 1. test if all maps in child_mesh exisits
        auto [child_tuple_from_child, parent_tuple_from_child] =
            multimesh::utils::read_tuple_map_attribute_slow(
                child_mesh,
                child_to_parent_handle,
                child_tuple);

        // 2. test if tuples in maps are valid (and up_to_date)
        {
            spdlog::info(
                "[{} -> {}] Checking asserts from child {} {} (input tuple was {})",
                fmt::join(absolute_id(), ","),
                fmt::join(child_mesh.absolute_multi_mesh_id(), ","),
                wmtk::utils::TupleInspector::as_string(child_tuple_from_child),
                wmtk::utils::TupleInspector::as_string(child_tuple_from_child),
                wmtk::utils::TupleInspector::as_string(child_tuple));
            assert(child_mesh.is_valid_slow(child_tuple_from_child));
            CHECK(child_mesh.is_valid_slow(child_tuple_from_child));
            CHECK(my_mesh.is_valid_slow(parent_tuple_from_child));
        }

        // 3. test if map is symmetric
        {
            auto [parent_tuple_from_parent, child_tuple_from_parent] =
                multimesh::utils::read_tuple_map_attribute_slow(
                    my_mesh,
                    parent_to_child_handle,
                    parent_tuple_from_child);
            spdlog::info(
                "[{} -> {}] Checking asserts from child {} {}",
                fmt::join(absolute_id(), ","),
                fmt::join(child_mesh.absolute_multi_mesh_id(), ","),
                wmtk::utils::TupleInspector::as_string(parent_tuple_from_parent),
                wmtk::utils::TupleInspector::as_string(child_tuple_from_parent));

            CHECK(
                (child_tuple_from_child == child_tuple_from_parent &&
                 parent_tuple_from_child == parent_tuple_from_parent));
        }

        // 4. test switch_top_simplex operation
        // for 4, current code support only mapping between triangle meshes
        if (map_type == PrimitiveType::Face && my_mesh.top_simplex_type() == PrimitiveType::Face) {
            Tuple cur_child_tuple = child_tuple_from_child;
            Tuple cur_parent_tuple = parent_tuple_from_child;

            auto child_to_parent_accessor =
                child_mesh.create_const_accessor(child_to_parent_handle);
            for (int i = 0; i < 3; i++) {
                if (!child_mesh.is_boundary(cur_child_tuple, PrimitiveType::Edge)) {
                    REQUIRE(!my_mesh.is_boundary(cur_parent_tuple, PrimitiveType::Edge));

                    Tuple child_tuple_opp = child_mesh.switch_face(cur_child_tuple);
                    Tuple parent_tuple_opp = my_mesh.switch_face(cur_parent_tuple);
                    CHECK(
                        parent_tuple_opp == map_tuple_between_meshes(
                                                child_mesh,
                                                my_mesh,
                                                child_to_parent_accessor,
                                                child_tuple_opp));
                }
                cur_child_tuple = child_mesh.switch_edge(child_mesh.switch_vertex(cur_child_tuple));
                cur_parent_tuple = my_mesh.switch_edge(my_mesh.switch_vertex(cur_parent_tuple));
            }
        } else if (
            map_type == PrimitiveType::Edge && my_mesh.top_simplex_type() == PrimitiveType::Face) {
            if (!my_mesh.is_boundary(parent_tuple_from_child, PrimitiveType::Edge)) {
                auto parent_to_child_accessor =
                    my_mesh.create_const_accessor(parent_to_child_handle);
                const Tuple parent_tuple_opp = my_mesh.switch_face(parent_tuple_from_child);
                CHECK(
                    child_tuple_from_child == map_tuple_between_meshes(
                                                  my_mesh,
                                                  child_mesh,
                                                  parent_to_child_accessor,
                                                  parent_tuple_opp));
            }
        } else {
            // TODO: implement other cases
            continue;
        }
    }
}
} // namespace wmtk::tests
