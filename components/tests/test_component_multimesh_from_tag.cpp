#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <tools/DEBUG_TetMesh.hpp>
#include <tools/DEBUG_TriMesh.hpp>
#include <tools/TetMesh_examples.hpp>
#include <tools/TriMesh_examples.hpp>
#include <wmtk/components/multimesh_from_tag/internal/MultiMeshFromTag.hpp>
#include <wmtk/components/multimesh_from_tag/internal/MultiMeshFromTagOptions.hpp>
#include <wmtk/components/multimesh_from_tag/multimesh_from_tag.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include <wmtk/utils/primitive_range.hpp>

using json = nlohmann::json;
using namespace wmtk;
using namespace tests;
using namespace tests_3d;
using namespace components;
using namespace internal;

TEST_CASE("multimesh_from_tag_tri_tri", "[components][multimesh][multimesh_from_tag]")
{
    auto mesh_in = tests::disk(6);
    DEBUG_TriMesh& m = static_cast<DEBUG_TriMesh&>(*mesh_in);

    auto tag_handle = m.register_attribute<int64_t>("tag", PrimitiveType::Triangle, 1);
    int64_t tag_value = 1;

    int64_t n_faces = -1;
    int64_t n_edges = -1;
    int64_t n_vertices = -1;
    simplex::SimplexCollection non_manifold_root_simplices(m);

    auto tag_acc = m.create_accessor<int64_t>(tag_handle);

    SECTION("one_component")
    {
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 1, 2)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 2, 3)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 3, 4)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 4, 5)) = tag_value;
        n_faces = 4;
        n_edges = 9;
        n_vertices = 6;
    }
    SECTION("two_components")
    {
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 1, 2)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 2, 3)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 3, 4)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 5, 6)) = tag_value;
        n_faces = 4;
        n_edges = 10;
        n_vertices = 8;
        non_manifold_root_simplices.add(PrimitiveType::Vertex, m.vertex_tuple_from_id(0));
    }
    SECTION("three_components")
    {
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 1, 2)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 3, 4)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 5, 6)) = tag_value;
        n_faces = 3;
        n_edges = 9;
        n_vertices = 9;
        non_manifold_root_simplices.add(PrimitiveType::Vertex, m.vertex_tuple_from_id(0));
    }

    MultiMeshFromTag mmft(m, tag_handle, tag_value);

    mmft.compute_substructure_mesh();

    REQUIRE(m.get_child_meshes().size() == 2);
    REQUIRE(m.is_multi_mesh_root());

    mmft.remove_soup();
    CHECK(m.get_child_meshes().size() == 1);
    REQUIRE(m.is_multi_mesh_root());

    auto substructure_mesh_ptr = mmft.substructure();
    Mesh& sub_mesh = *substructure_mesh_ptr;

    CHECK(sub_mesh.top_simplex_type() == PrimitiveType::Triangle);
    CHECK(sub_mesh.get_all(PrimitiveType::Triangle).size() == n_faces);
    CHECK(sub_mesh.get_all(PrimitiveType::Edge).size() == n_edges);
    CHECK(sub_mesh.get_all(PrimitiveType::Vertex).size() == n_vertices);

    non_manifold_root_simplices.sort_and_clean();
    for (const PrimitiveType pt : utils::primitive_below(m.top_simplex_type())) {
        for (const Tuple& t : m.get_all(pt)) {
            const simplex::Simplex s(pt, t);
            if (non_manifold_root_simplices.contains(s)) {
                CHECK_FALSE(mmft.is_root_simplex_manifold(s));
            } else {
                CHECK(mmft.is_root_simplex_manifold(s));
            }
        }
    }
}

TEST_CASE("multimesh_from_tag_tri_edge", "[components][multimesh][multimesh_from_tag]")
{
    auto mesh_in = tests::disk(6);
    DEBUG_TriMesh& m = static_cast<DEBUG_TriMesh&>(*mesh_in);

    auto tag_handle = m.register_attribute<int64_t>("tag", PrimitiveType::Edge, 1);
    int64_t tag_value = 1;

    int64_t n_edges = -1;
    int64_t n_vertices = -1;
    simplex::SimplexCollection non_manifold_root_simplices(m);

    auto tag_acc = m.create_accessor<int64_t>(tag_handle);

    SECTION("one_component")
    {
        tag_acc.scalar_attribute(m.edge_tuple_from_vids(0, 1)) = tag_value;
        tag_acc.scalar_attribute(m.edge_tuple_from_vids(1, 2)) = tag_value;
        tag_acc.scalar_attribute(m.edge_tuple_from_vids(2, 3)) = tag_value;
        n_edges = 3;
        n_vertices = 4;
    }
    SECTION("two_components")
    {
        tag_acc.scalar_attribute(m.edge_tuple_from_vids(0, 1)) = tag_value;
        tag_acc.scalar_attribute(m.edge_tuple_from_vids(0, 4)) = tag_value;
        tag_acc.scalar_attribute(m.edge_tuple_from_vids(5, 6)) = tag_value;
        n_edges = 3;
        n_vertices = 5;
    }
    SECTION("three_components")
    {
        tag_acc.scalar_attribute(m.edge_tuple_from_vids(0, 1)) = tag_value;
        tag_acc.scalar_attribute(m.edge_tuple_from_vids(0, 4)) = tag_value;
        tag_acc.scalar_attribute(m.edge_tuple_from_vids(5, 0)) = tag_value;
        non_manifold_root_simplices.add(PrimitiveType::Vertex, m.vertex_tuple_from_id(0));
        n_edges = 3;
        n_vertices = 6;
    }
    SECTION("two_closed_loops")
    {
        tag_acc.scalar_attribute(m.edge_tuple_from_vids(0, 1)) = tag_value;
        tag_acc.scalar_attribute(m.edge_tuple_from_vids(1, 2)) = tag_value;
        tag_acc.scalar_attribute(m.edge_tuple_from_vids(2, 0)) = tag_value;
        tag_acc.scalar_attribute(m.edge_tuple_from_vids(0, 4)) = tag_value;
        tag_acc.scalar_attribute(m.edge_tuple_from_vids(4, 5)) = tag_value;
        tag_acc.scalar_attribute(m.edge_tuple_from_vids(5, 0)) = tag_value;
        non_manifold_root_simplices.add(PrimitiveType::Vertex, m.vertex_tuple_from_id(0));
        n_edges = 6;
        n_vertices = 8;
    }

    MultiMeshFromTag mmft(m, tag_handle, tag_value);

    mmft.compute_substructure_mesh();

    REQUIRE(m.get_child_meshes().size() == 2);
    REQUIRE(m.is_multi_mesh_root());

    mmft.remove_soup();
    CHECK(m.get_child_meshes().size() == 1);
    REQUIRE(m.is_multi_mesh_root());

    auto substructure_mesh_ptr = mmft.substructure();
    Mesh& sub_mesh = *substructure_mesh_ptr;

    CHECK(sub_mesh.top_simplex_type() == PrimitiveType::Edge);
    CHECK(sub_mesh.get_all(PrimitiveType::Edge).size() == n_edges);
    CHECK(sub_mesh.get_all(PrimitiveType::Vertex).size() == n_vertices);

    non_manifold_root_simplices.sort_and_clean();
    for (const PrimitiveType pt : utils::primitive_below(m.top_simplex_type())) {
        for (const Tuple& t : m.get_all(pt)) {
            const simplex::Simplex s(pt, t);
            if (non_manifold_root_simplices.contains(s)) {
                CHECK_FALSE(mmft.is_root_simplex_manifold(s));
            } else {
                CHECK(mmft.is_root_simplex_manifold(s));
            }
        }
    }
}

TEST_CASE("multimesh_from_tag_tri_point", "[components][multimesh][multimesh_from_tag]")
{
    auto mesh_in = tests::disk(6);
    DEBUG_TriMesh& m = static_cast<DEBUG_TriMesh&>(*mesh_in);

    auto tag_handle = m.register_attribute<int64_t>("tag", PrimitiveType::Vertex, 1);
    int64_t tag_value = 1;


    auto tag_acc = m.create_accessor<int64_t>(tag_handle);

    auto vertices = m.get_all(PrimitiveType::Vertex);
    for (size_t i = 0; i < 4; ++i) {
        tag_acc.scalar_attribute(vertices[i]) = tag_value;
    }

    MultiMeshFromTag mmft(m, tag_handle, tag_value);

    mmft.compute_substructure_mesh();

    REQUIRE(m.get_child_meshes().size() == 2);
    REQUIRE(m.is_multi_mesh_root());

    mmft.remove_soup();
    CHECK(m.get_child_meshes().size() == 1);
    REQUIRE(m.is_multi_mesh_root());

    auto substructure_mesh_ptr = mmft.substructure();
    Mesh& sub_mesh = *substructure_mesh_ptr;

    CHECK(sub_mesh.top_simplex_type() == PrimitiveType::Vertex);
    CHECK(sub_mesh.get_all(PrimitiveType::Vertex).size() == 4);

    for (const PrimitiveType pt : utils::primitive_below(m.top_simplex_type())) {
        for (const Tuple& t : m.get_all(pt)) {
            const simplex::Simplex s(pt, t);
            CHECK(mmft.is_root_simplex_manifold(s));
        }
    }
}

TEST_CASE("multimesh_from_tag_tet_tet", "[components][multimesh][multimesh_from_tag]")
{
    DEBUG_TetMesh m = six_cycle_tets();

    auto tag_handle = m.register_attribute<int64_t>("tag", PrimitiveType::Tetrahedron, 1);
    int64_t tag_value = 1;

    int64_t n_tets = -1;
    int64_t n_faces = -1;
    int64_t n_edges = -1;
    int64_t n_vertices = -1;
    simplex::SimplexCollection non_manifold_root_simplices(m);

    auto tag_acc = m.create_accessor<int64_t>(tag_handle);

    SECTION("one_component")
    {
        tag_acc.scalar_attribute(m.tet_tuple_from_vids(0, 1, 2, 3)) = tag_value;
        tag_acc.scalar_attribute(m.tet_tuple_from_vids(0, 2, 3, 4)) = tag_value;
        tag_acc.scalar_attribute(m.tet_tuple_from_vids(2, 3, 4, 5)) = tag_value;
        n_tets = 3;
        n_faces = 10;
        n_edges = 12;
        n_vertices = 6;
    }
    SECTION("two_components")
    {
        tag_acc.scalar_attribute(m.tet_tuple_from_vids(0, 1, 2, 3)) = tag_value;
        tag_acc.scalar_attribute(m.tet_tuple_from_vids(0, 2, 3, 4)) = tag_value;
        tag_acc.scalar_attribute(m.tet_tuple_from_vids(2, 3, 4, 5)) = tag_value;
        tag_acc.scalar_attribute(m.tet_tuple_from_vids(2, 3, 6, 7)) = tag_value;
        n_tets = 4;
        n_faces = 14;
        n_edges = 18;
        n_vertices = 10;
        non_manifold_root_simplices.add(PrimitiveType::Vertex, m.vertex_tuple_from_id(2));
        non_manifold_root_simplices.add(PrimitiveType::Vertex, m.vertex_tuple_from_id(3));
        non_manifold_root_simplices.add(PrimitiveType::Edge, m.edge_tuple_from_vids(2, 3));
    }
    SECTION("three_components")
    {
        tag_acc.scalar_attribute(m.tet_tuple_from_vids(0, 1, 2, 3)) = tag_value;
        tag_acc.scalar_attribute(m.tet_tuple_from_vids(2, 3, 4, 5)) = tag_value;
        tag_acc.scalar_attribute(m.tet_tuple_from_vids(2, 3, 6, 7)) = tag_value;
        n_tets = 3;
        n_faces = 12;
        n_edges = 18;
        n_vertices = 12;

        non_manifold_root_simplices.add(PrimitiveType::Vertex, m.vertex_tuple_from_id(2));
        non_manifold_root_simplices.add(PrimitiveType::Vertex, m.vertex_tuple_from_id(3));
        non_manifold_root_simplices.add(PrimitiveType::Edge, m.edge_tuple_from_vids(2, 3));
    }

    MultiMeshFromTag mmft(m, tag_handle, tag_value);

    mmft.compute_substructure_mesh();

    REQUIRE(m.get_child_meshes().size() == 2);
    REQUIRE(m.is_multi_mesh_root());

    mmft.remove_soup();
    CHECK(m.get_child_meshes().size() == 1);
    REQUIRE(m.is_multi_mesh_root());

    auto substructure_mesh_ptr = mmft.substructure();
    Mesh& sub_mesh = *substructure_mesh_ptr;

    CHECK(sub_mesh.top_simplex_type() == PrimitiveType::Tetrahedron);
    CHECK(sub_mesh.get_all(PrimitiveType::Tetrahedron).size() == n_tets);
    CHECK(sub_mesh.get_all(PrimitiveType::Triangle).size() == n_faces);
    CHECK(sub_mesh.get_all(PrimitiveType::Edge).size() == n_edges);
    CHECK(sub_mesh.get_all(PrimitiveType::Vertex).size() == n_vertices);

    non_manifold_root_simplices.sort_and_clean();
    for (const PrimitiveType pt : utils::primitive_below(m.top_simplex_type())) {
        for (const Tuple& t : m.get_all(pt)) {
            const simplex::Simplex s(pt, t);
            if (non_manifold_root_simplices.contains(s)) {
                CHECK_FALSE(mmft.is_root_simplex_manifold(s));
            } else {
                CHECK(mmft.is_root_simplex_manifold(s));
            }
        }
    }
}

TEST_CASE("multimesh_from_tag_tet_tri", "[components][multimesh][multimesh_from_tag]")
{
    DEBUG_TetMesh m = six_cycle_tets();

    auto tag_handle = m.register_attribute<int64_t>("tag", PrimitiveType::Triangle, 1);
    int64_t tag_value = 1;

    int64_t n_faces = -1;
    int64_t n_edges = -1;
    int64_t n_vertices = -1;
    simplex::SimplexCollection non_manifold_root_simplices(m);

    auto tag_acc = m.create_accessor<int64_t>(tag_handle);

    SECTION("one_component")
    {
        tag_acc.scalar_attribute(m.face_tuple_from_vids(1, 2, 3)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 1, 2)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 3, 5)) = tag_value;
        n_faces = 3;
        n_edges = 7;
        n_vertices = 5;
    }
    SECTION("non_manifold_vertex")
    {
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 1, 2)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 2, 4)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(1, 2, 6)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 5, 7)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(3, 5, 7)) = tag_value;
        n_faces = 5;
        n_edges = 12;
        n_vertices = 9;
        non_manifold_root_simplices.add(PrimitiveType::Vertex, m.vertex_tuple_from_id(2));
    }
    SECTION("non_manifold_vertex_2")
    {
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 1, 2)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(1, 2, 3)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 2, 3)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 1, 3)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 5, 7)) = tag_value;
        n_faces = 5;
        n_edges = 9;
        n_vertices = 7;
        non_manifold_root_simplices.add(PrimitiveType::Vertex, m.vertex_tuple_from_id(2));
    }
    SECTION("non_manifold_edge")
    {
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 1, 2)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(1, 2, 3)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 2, 3)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 1, 3)) = tag_value;

        tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 3, 5)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(3, 5, 7)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 5, 7)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(2, 3, 7)) = tag_value;
        n_faces = 8;
        n_edges = 14;
        n_vertices = 8;
        non_manifold_root_simplices.add(PrimitiveType::Vertex, m.vertex_tuple_from_id(2));
        non_manifold_root_simplices.add(PrimitiveType::Vertex, m.vertex_tuple_from_id(3));
        non_manifold_root_simplices.add(PrimitiveType::Edge, m.edge_tuple_from_vids(2, 3));
    }

    MultiMeshFromTag mmft(m, tag_handle, tag_value);

    mmft.compute_substructure_mesh();

    REQUIRE(m.get_child_meshes().size() == 2);
    REQUIRE(m.is_multi_mesh_root());

    mmft.remove_soup();
    CHECK(m.get_child_meshes().size() == 1);
    REQUIRE(m.is_multi_mesh_root());

    auto substructure_mesh_ptr = mmft.substructure();
    Mesh& sub_mesh = *substructure_mesh_ptr;

    CHECK(sub_mesh.top_simplex_type() == PrimitiveType::Triangle);
    CHECK(sub_mesh.get_all(PrimitiveType::Triangle).size() == n_faces);
    CHECK(sub_mesh.get_all(PrimitiveType::Edge).size() == n_edges);
    CHECK(sub_mesh.get_all(PrimitiveType::Vertex).size() == n_vertices);

    non_manifold_root_simplices.sort_and_clean();
    for (const PrimitiveType pt : utils::primitive_below(m.top_simplex_type())) {
        for (const Tuple& t : m.get_all(pt)) {
            const simplex::Simplex s(pt, t);
            if (non_manifold_root_simplices.contains(s)) {
                CHECK_FALSE(mmft.is_root_simplex_manifold(s));
            } else {
                CHECK(mmft.is_root_simplex_manifold(s));
            }
        }
    }
}

TEST_CASE("multimesh_from_tag_tri_visualization", "[components][multimesh][multimesh_from_tag][.]")
{
    DEBUG_TriMesh m = tests::edge_region_with_position();

    auto tag_handle = m.register_attribute<int64_t>("tag", PrimitiveType::Triangle, 1);
    int64_t tag_value = 1;

    int64_t n_faces = -1;
    int64_t n_edges = -1;
    int64_t n_vertices = -1;

    auto tag_acc = m.create_accessor<int64_t>(tag_handle);

    tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 3, 4)) = tag_value;
    tag_acc.scalar_attribute(m.face_tuple_from_vids(3, 7, 4)) = tag_value;
    tag_acc.scalar_attribute(m.face_tuple_from_vids(4, 5, 1)) = tag_value;
    tag_acc.scalar_attribute(m.face_tuple_from_vids(5, 6, 2)) = tag_value;
    n_faces = 4;
    n_edges = 11;
    n_vertices = 10;

    MultiMeshFromTag mmft(m, tag_handle, tag_value);

    mmft.compute_substructure_mesh();

    CHECK(m.get_child_meshes().size() == 2);
    CHECK(m.is_multi_mesh_root());

    auto substructure_mesh_ptr = mmft.substructure();
    Mesh& sub_mesh = *substructure_mesh_ptr;

    CHECK(sub_mesh.top_simplex_type() == PrimitiveType::Triangle);
    CHECK(sub_mesh.get_all(PrimitiveType::Triangle).size() == n_faces);
    CHECK(sub_mesh.get_all(PrimitiveType::Edge).size() == n_edges);
    CHECK(sub_mesh.get_all(PrimitiveType::Vertex).size() == n_vertices);


    auto root_pos_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto subs_pos_handle =
        sub_mesh.register_attribute<double>("vertices", PrimitiveType::Vertex, 3);

    auto propagate_to_child_position = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        return P;
    };

    auto pos_transfer =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            subs_pos_handle,
            root_pos_handle,
            propagate_to_child_position);
    pos_transfer->run_on_all();

    ParaviewWriter writer("child_mesh", "vertices", sub_mesh, false, false, true, false);
    sub_mesh.serialize(writer);
    ParaviewWriter writer2("root_mesh", "vertices", m, false, false, true, false);
    m.serialize(writer2);
}

TEST_CASE("multimesh_from_tag", "[components][multimesh][multimesh_from_tag]")
{
    io::Cache cache("wmtk_cache", ".");
    {
        auto mesh_in = tests::disk(6);
        DEBUG_TriMesh& m = static_cast<DEBUG_TriMesh&>(*mesh_in);

        auto tag_handle = m.register_attribute<int64_t>("tag", PrimitiveType::Triangle, 1);
        auto tag_acc = m.create_accessor<int64_t>(tag_handle);
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 1, 2)) = 1;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 2, 3)) = 1;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 3, 4)) = 1;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 5, 6)) = 1;

        cache.write_mesh(*mesh_in, "input_mesh");
    }

    json o = R"(
        {
            "input": "input_mesh",
            "output": "output_mesh",
            "substructure_label": "tag",
            "substructure_value": 1,
            "pass_through": []
        }
        )"_json;

    CHECK_NOTHROW(multimesh_from_tag(base::Paths(), o, cache));
}

// TODO add tests for tet_edge and tet_point
// TODO add hour glass test (non-manifold vertex) in tet_tri