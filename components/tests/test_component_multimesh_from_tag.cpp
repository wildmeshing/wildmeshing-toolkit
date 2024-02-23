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
    }
    SECTION("three_components")
    {
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 1, 2)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 3, 4)) = tag_value;
        tag_acc.scalar_attribute(m.face_tuple_from_vids(0, 5, 6)) = tag_value;
        n_faces = 3;
        n_edges = 9;
        n_vertices = 9;
    }

    MultiMeshFromTag mmft(m, tag_handle, tag_value);

    REQUIRE(m.get_child_meshes().size() == 1);
    REQUIRE(m.is_multi_mesh_root());
    std::shared_ptr<Mesh> child_ptr = m.get_child_meshes()[0];

    mmft.compute_substructure_ids();

    const Eigen::MatrixX<int64_t> FV = mmft.get_new_id_matrix(PrimitiveType::Vertex);
    CHECK(FV.rows() == n_faces);
    CHECK(FV.cols() == 3);

    const Eigen::MatrixX<int64_t> FE = mmft.get_new_id_matrix(PrimitiveType::Edge);
    CHECK(FE.rows() == n_faces);
    CHECK(FE.cols() == 3);

    const VectorXl VF = mmft.get_new_top_coface_vector(PrimitiveType::Vertex);
    CHECK(VF.size() == n_vertices);
    const VectorXl EF = mmft.get_new_top_coface_vector(PrimitiveType::Edge);
    CHECK(EF.size() == n_edges);
}

TEST_CASE("multimesh_from_tag_tri_edge", "[components][multimesh][multimesh_from_tag]")
{
    auto mesh_in = tests::disk(6);
    DEBUG_TriMesh& m = static_cast<DEBUG_TriMesh&>(*mesh_in);

    auto tag_handle = m.register_attribute<int64_t>("tag", PrimitiveType::Edge, 1);
    int64_t tag_value = 1;

    int64_t n_edges = -1;
    int64_t n_vertices = -1;

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
        n_edges = 6;
        n_vertices = 8;
    }

    MultiMeshFromTag mmft(m, tag_handle, tag_value);
    mmft.compute_substructure_ids();

    const Eigen::MatrixX<int64_t> EV = mmft.get_new_id_matrix(PrimitiveType::Vertex);
    CHECK(EV.rows() == n_edges);
    CHECK(EV.cols() == 2);

    const VectorXl VE = mmft.get_new_top_coface_vector(PrimitiveType::Vertex);
    CHECK(VE.size() == n_vertices);
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
    mmft.compute_substructure_ids();

    const auto child = mmft.substructure_soup();

    CHECK(child->top_simplex_type() == PrimitiveType::Vertex);
    CHECK(child->get_all(PrimitiveType::Vertex).size() == 4);
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
    }

    MultiMeshFromTag mmft(m, tag_handle, tag_value);

    REQUIRE(m.get_child_meshes().size() == 1);
    REQUIRE(m.is_multi_mesh_root());

    mmft.compute_substructure_ids();

    const Eigen::MatrixX<int64_t> TV = mmft.get_new_id_matrix(PrimitiveType::Vertex);
    CHECK(TV.rows() == n_tets);
    CHECK(TV.cols() == 4);

    const Eigen::MatrixX<int64_t> TE = mmft.get_new_id_matrix(PrimitiveType::Edge);
    CHECK(TE.rows() == n_tets);
    CHECK(TE.cols() == 6);

    const Eigen::MatrixX<int64_t> TF = mmft.get_new_id_matrix(PrimitiveType::Triangle);
    CHECK(TF.rows() == n_tets);
    CHECK(TF.cols() == 4);

    const VectorXl VT = mmft.get_new_top_coface_vector(PrimitiveType::Vertex);
    CHECK(VT.size() == n_vertices);
    const VectorXl ET = mmft.get_new_top_coface_vector(PrimitiveType::Edge);
    CHECK(ET.size() == n_edges);
    const VectorXl FT = mmft.get_new_top_coface_vector(PrimitiveType::Triangle);
    CHECK(FT.size() == n_faces);

    TetMesh substructure_mesh;
    substructure_mesh.initialize(TV, TE, TF, mmft.adjacency_matrix(), VT, ET, FT);
}

TEST_CASE("multimesh_from_tag_tet_tri", "[components][multimesh][multimesh_from_tag]")
{
    DEBUG_TetMesh m = six_cycle_tets();

    auto tag_handle = m.register_attribute<int64_t>("tag", PrimitiveType::Triangle, 1);
    int64_t tag_value = 1;

    int64_t n_faces = -1;
    int64_t n_edges = -1;
    int64_t n_vertices = -1;

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
    }


    MultiMeshFromTag mmft(m, tag_handle, tag_value);

    REQUIRE(m.get_child_meshes().size() == 1);
    REQUIRE(m.is_multi_mesh_root());

    mmft.compute_substructure_ids();

    const Eigen::MatrixX<int64_t> FV = mmft.get_new_id_matrix(PrimitiveType::Vertex);
    CHECK(FV.rows() == n_faces);
    CHECK(FV.cols() == 3);

    const Eigen::MatrixX<int64_t> FE = mmft.get_new_id_matrix(PrimitiveType::Edge);
    CHECK(FE.rows() == n_faces);
    CHECK(FE.cols() == 3);

    const VectorXl VF = mmft.get_new_top_coface_vector(PrimitiveType::Vertex);
    CHECK(VF.size() == n_vertices);
    const VectorXl EF = mmft.get_new_top_coface_vector(PrimitiveType::Edge);
    CHECK(EF.size() == n_edges);
}

TEST_CASE("multimesh_from_tag_tri_visualization", "[components][multimesh][multimesh_from_tag]")
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

    REQUIRE(m.get_child_meshes().size() == 1);
    REQUIRE(m.is_multi_mesh_root());

    mmft.compute_substructure_ids();

    const Eigen::MatrixX<int64_t> FV = mmft.get_new_id_matrix(PrimitiveType::Vertex);
    CHECK(FV.rows() == n_faces);
    CHECK(FV.cols() == 3);

    const Eigen::MatrixX<int64_t> FE = mmft.get_new_id_matrix(PrimitiveType::Edge);
    CHECK(FE.rows() == n_faces);
    CHECK(FE.cols() == 3);

    const VectorXl VF = mmft.get_new_top_coface_vector(PrimitiveType::Vertex);
    CHECK(VF.size() == n_vertices);
    const VectorXl EF = mmft.get_new_top_coface_vector(PrimitiveType::Edge);
    CHECK(EF.size() == n_edges);

    auto substructure_mesh = mmft.compute_substructure_idf();

    auto root_pos_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto subs_pos_handle =
        substructure_mesh->register_attribute<double>("vertices", PrimitiveType::Vertex, 3);

    auto propagate_to_child_position = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        return P;
    };

    auto pos_transfer =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            subs_pos_handle,
            root_pos_handle,
            propagate_to_child_position);
    pos_transfer->run_on_all();

    ParaviewWriter writer("child_mesh", "vertices", *substructure_mesh, false, false, true, false);
    substructure_mesh->serialize(writer);
    ParaviewWriter writer2("root_mesh", "vertices", m, false, false, true, false);
    m.serialize(writer2);
}

// TODO add tests for tet_edge and tet_point
// TODO add hour glass test (non-manifold vertex) in tet_tri