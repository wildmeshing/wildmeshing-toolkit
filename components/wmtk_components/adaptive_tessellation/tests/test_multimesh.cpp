#include <cassert>
#include <catch2/catch_test_macros.hpp>
#include <set>
#include <tools/DEBUG_EdgeMesh.hpp>
#include <tools/DEBUG_TriMesh.hpp>
#include <tools/DEBUG_Tuple.hpp>
#include <tools/EdgeMesh_examples.hpp>
#include <tools/TriMesh_examples.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/io/utils/paraview_write_child_meshes.hpp>
#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>
#include <wmtk/multimesh/utils/create_tag.hpp>
#include <wmtk/multimesh/utils/edge_meshes_parameterization.hpp>
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>
#include "wmtk/multimesh/utils/find_critical_points.hpp"
#include "wmtk/multimesh/utils/map_sibling_edge_meshes.hpp"


using namespace wmtk;
using namespace wmtk::tests;
using namespace wmtk::simplex;
using namespace wmtk::components::adaptive_tessellation::multimesh::utils;
TEST_CASE("edge mesh registration")
{
    DEBUG_TriMesh position_mesh = two_neighbors();
    std::shared_ptr<DEBUG_TriMesh> uv_mesh_ptr =
        std::make_shared<DEBUG_TriMesh>(two_neighbors_cut_on_edge01());

    auto& uv_mesh = *uv_mesh_ptr;

    auto uv_mesh_map = wmtk::multimesh::same_simplex_dimension_bijection(position_mesh, uv_mesh);
    position_mesh.register_child_mesh(uv_mesh_ptr, uv_mesh_map);

    Simplex seam = Simplex::edge(position_mesh.edge_tuple_from_vids(0, 1));
    auto uv_seam = position_mesh.map(uv_mesh, seam);
    REQUIRE(uv_seam.size() == 2);

    std::set<int64_t> critical_vids = {0, 1, 2, 5, 6};
    auto tags = wmtk::multimesh::utils::create_tags(uv_mesh, critical_vids);
    REQUIRE(tags.size() == 5);
    std::vector<std::shared_ptr<Mesh>> edge_meshes;
    for (int64_t tag : tags) {
        edge_meshes.emplace_back(wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
            uv_mesh,
            "edge_tag",
            tag,
            PrimitiveType::Edge));
    }
    std::map<Mesh*, Mesh*> sibling_edge_meshes = map_sibling_edge_meshes(edge_meshes);
    REQUIRE(sibling_edge_meshes.size() == 2);
    // Iterate through the keys using an iterator
    for (auto it = sibling_edge_meshes.begin(); it != sibling_edge_meshes.end(); ++it) {
        Mesh* sibling0 = it->first;
        Mesh* sibling1 = it->second;

        REQUIRE(position_mesh.map(*sibling0, seam).size() == 1);
        REQUIRE(position_mesh.map(*sibling1, seam).size() == 1);
    }
}


TEST_CASE("find_critical_point")
{
    DEBUG_TriMesh position_mesh = sewed_at_seam_position_mesh_with_position();
    std::shared_ptr<DEBUG_TriMesh> uv_mesh_ptr =
        std::make_shared<DEBUG_TriMesh>(cutup_uv_mesh_with_position());

    auto& uv_mesh = *uv_mesh_ptr;

    auto uv_mesh_map = wmtk::multimesh::same_simplex_dimension_bijection(position_mesh, uv_mesh);
    position_mesh.register_child_mesh(uv_mesh_ptr, uv_mesh_map);

    Tuple v = position_mesh.edge_tuple_between_v1_v2(2, 3, 3);
    auto uv_v = position_mesh.map(uv_mesh, Simplex::vertex(v));

    REQUIRE(uv_v.size() == 2);

    std::set<Tuple> critical_points = find_critical_points(uv_mesh, position_mesh);

    REQUIRE(critical_points.size() == 17);
    std::set<int64_t> critical_vids;
    for (const Tuple& t : critical_points) {
        critical_vids.insert(uv_mesh.id(t, PrimitiveType::Vertex));
    }
    REQUIRE(critical_vids.size() == 17);
    std::set<int64_t> expected_critical_vids =
        {0, 1, 3, 4, 7, 5, 10, 8, 9, 11, 13, 16, 15, 14, 17, 19, 18};
    REQUIRE(critical_vids == expected_critical_vids);
}

TEST_CASE("edge_curves_parametrization")
{
    DEBUG_TriMesh position_mesh = sewed_at_seam_position_mesh_with_position();
    std::shared_ptr<DEBUG_TriMesh> uv_mesh_ptr =
        std::make_shared<DEBUG_TriMesh>(cutup_uv_mesh_with_position());

    auto& uv_mesh = *uv_mesh_ptr;

    auto uv_mesh_map = wmtk::multimesh::same_simplex_dimension_bijection(position_mesh, uv_mesh);
    position_mesh.register_child_mesh(uv_mesh_ptr, uv_mesh_map);

    std::set<Tuple> critical_points = find_critical_points(uv_mesh, position_mesh);
    auto tags = wmtk::multimesh::utils::create_tags(uv_mesh, critical_points);
    REQUIRE(tags.size() == 17);
    std::vector<std::shared_ptr<Mesh>> edge_meshes;
    for (int64_t tag : tags) {
        edge_meshes.emplace_back(wmtk::multimesh::utils::extract_and_register_child_mesh_from_tag(
            uv_mesh,
            "edge_tag",
            tag,
            PrimitiveType::Edge));
    }
    std::map<Mesh*, Mesh*> sibling_edge_meshes = map_sibling_edge_meshes(edge_meshes);
    parameterize_all_edge_meshes(uv_mesh, edge_meshes, sibling_edge_meshes);
    for (auto edge_mesh : edge_meshes) {
        REQUIRE(edge_mesh->has_attribute<double>("t", PrimitiveType::Vertex));
        wmtk::attribute::TypedAttributeHandle<double> t_handle =
            edge_mesh->get_attribute_handle<double>("t", PrimitiveType::Vertex).as<double>();
        wmtk::attribute::ConstAccessor t_accessor = edge_mesh->create_const_accessor(t_handle);
        Mesh* sibling_edge_mesh = sibling_edge_meshes[edge_mesh.get()];
        if (sibling_edge_mesh != nullptr) {
            wmtk::attribute::TypedAttributeHandle<double> sibling_t_handle =
                sibling_edge_mesh->get_attribute_handle<double>("t", PrimitiveType::Vertex)
                    .as<double>();
            wmtk::attribute::ConstAccessor sibling_t_accessor =
                sibling_edge_mesh->create_const_accessor(sibling_t_handle);
            for (Tuple& v : edge_mesh->get_all(PrimitiveType::Vertex)) {
                Tuple sibling_v = map_single_tuple(
                    reinterpret_cast<EdgeMesh&>(*edge_mesh),
                    reinterpret_cast<EdgeMesh&>(*sibling_edge_mesh),
                    v,
                    PrimitiveType::Vertex);
                REQUIRE(
                    t_accessor.const_scalar_attribute(v) ==
                    sibling_t_accessor.const_scalar_attribute(sibling_v));
            }
        }

        for (Tuple& v : edge_mesh.get()->get_all(PrimitiveType::Vertex)) {
            REQUIRE(t_accessor.const_scalar_attribute(v) >= 0);
        }
    }

    // output the edge meshes
    // for (int64_t i = 0; i < edge_meshes.size(); i++) {
    //     auto edge_mesh = edge_meshes[i];

    //     wmtk::io::utils::write_child_meshes(
    //         "edge_mesh_" + std::to_string(i),
    //         edge_mesh.get(),
    //         uv_mesh_ptr.get(),
    //         uv_mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex));
    // }
}