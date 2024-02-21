#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <tools/DEBUG_TetMesh.hpp>
#include <tools/DEBUG_TriMesh.hpp>
#include <tools/TetMesh_examples.hpp>
#include <tools/TriMesh_examples.hpp>
#include <wmtk/components/multimesh_from_tag/internal/MultiMeshFromTag.hpp>
#include <wmtk/components/multimesh_from_tag/internal/MultiMeshFromTagOptions.hpp>
#include <wmtk/components/multimesh_from_tag/multimesh_from_tag.hpp>

using json = nlohmann::json;
using namespace wmtk;
using namespace tests;
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

    MultiMeshFromTag mmft(m, tag_handle, tag_value);
    mmft.compute_substructure_ids();

    const Eigen::MatrixX<int64_t> EV = mmft.get_new_id_matrix(PrimitiveType::Vertex);
    CHECK(EV.rows() == n_edges);
    CHECK(EV.cols() == 2);

    const VectorXl VE = mmft.get_new_top_coface_vector(PrimitiveType::Vertex);
    CHECK(VE.size() == n_vertices);
}