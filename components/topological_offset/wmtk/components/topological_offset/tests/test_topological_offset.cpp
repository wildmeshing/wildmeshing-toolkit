#include <igl/predicates/predicates.h>
#include <wmtk/TetMesh.h>
#include <wmtk/components/topological_offset/TopoOffsetMesh.h>
#include <catch2/catch_test_macros.hpp>

using namespace wmtk;
using namespace components::topological_offset;


// used for checking attribute propagatoin. values are arbitrary
const int V0_LABEL = 10;
const int V1_LABEL = 11;
const int V2_LABEL = 12;
const int V3_LABEL = 13;
const int E0_LABEL = 20;
const int E1_LABEL = 21;
const int E2_LABEL = 22;
const int E3_LABEL = 23;
const int E4_LABEL = 24;
const int E5_LABEL = 25;
const int F0_LABEL = 30;
const int F1_LABEL = 31;
const int F2_LABEL = 32;
const int F3_LABEL = 33;
const int T0_LABEL = 40;
const std::vector<double> T0_TAGS = {128.0};


TEST_CASE("edge_split", "[split_op]")
{
    Eigen::Matrix<double, Eigen::Dynamic, 3> V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::MatrixXi T(1, 4);
    T << 0, 1, 2, 3;
    Eigen::MatrixXd Tags(1, 1);
    Tags << 128.0;

    Parameters param;
    param.tag_name = "dummy";
    param.sep_tag_vals.push_back(1);
    param.sep_tag_vals.push_back(4);
    TopoOffsetMesh mesh(param, 0);
    std::vector<std::string> tag_names(1, "dummy");
    mesh.init_from_image(V, T, Tags, tag_names);

    // give every component unique tag combo
    mesh.m_vertex_attribute[0].label = V0_LABEL;
    mesh.m_vertex_attribute[1].label = V1_LABEL;
    mesh.m_vertex_attribute[2].label = V2_LABEL;
    mesh.m_vertex_attribute[3].label = V3_LABEL;
    mesh.m_edge_attribute[0].label = E0_LABEL;
    mesh.m_edge_attribute[1].label = E1_LABEL;
    mesh.m_edge_attribute[2].label = E2_LABEL;
    mesh.m_edge_attribute[3].label = E3_LABEL;
    mesh.m_edge_attribute[4].label = E4_LABEL;
    mesh.m_edge_attribute[5].label = E5_LABEL;
    mesh.m_face_attribute[0].label = F0_LABEL;
    mesh.m_face_attribute[1].label = F1_LABEL;
    mesh.m_face_attribute[2].label = F2_LABEL;
    mesh.m_face_attribute[3].label = F3_LABEL;
    mesh.m_tet_attribute[0].label = T0_LABEL;

    // split edge
    TetMesh::Tuple e = mesh.tuple_from_edge({1, 2});
    std::vector<TetMesh::Tuple> garbage;
    mesh.split_edge(e, garbage);

    // // ensure proper propagation of attributes
    // vertex
    REQUIRE(mesh.m_vertex_attribute[0].label == V0_LABEL);
    REQUIRE(mesh.m_vertex_attribute[1].label == V1_LABEL);
    REQUIRE(mesh.m_vertex_attribute[2].label == V2_LABEL);
    REQUIRE(mesh.m_vertex_attribute[3].label == V3_LABEL);
    REQUIRE(mesh.m_vertex_attribute[4].label == E1_LABEL);

    // edges
    std::array<std::array<size_t, 3>, 9> edges = {
        {// {v0id, v1id, correct label}
         {0, 1, E0_LABEL},
         {0, 2, E2_LABEL},
         {0, 3, E3_LABEL},
         {0, 4, F0_LABEL},
         {1, 3, E4_LABEL},
         {1, 4, E1_LABEL},
         {2, 3, E5_LABEL},
         {2, 4, E1_LABEL},
         {3, 4, F3_LABEL}}};
    for (int i = 0; i < 9; i++) {
        size_t v0 = edges[i][0];
        size_t v1 = edges[i][1];
        int correct_label = edges[i][2];
        int actual_label = mesh.m_edge_attribute[mesh.tuple_from_edge({v0, v1}).eid(mesh)].label;
        REQUIRE(actual_label == correct_label);
    }

    // faces
    std::array<std::array<size_t, 4>, 7> faces = {
        {{0, 1, 3, F2_LABEL},
         {0, 1, 4, F0_LABEL},
         {0, 2, 3, F1_LABEL},
         {0, 2, 4, F0_LABEL},
         {0, 3, 4, T0_LABEL},
         {1, 3, 4, F3_LABEL},
         {2, 3, 4, F3_LABEL}}};
    for (int i = 0; i < 7; i++) {
        size_t v0 = faces[i][0];
        size_t v1 = faces[i][1];
        size_t v2 = faces[i][2];
        int correct_label = faces[i][3];
        int actual_label =
            mesh.m_face_attribute[std::get<1>(mesh.tuple_from_face({v0, v1, v2}))].label;
        REQUIRE(correct_label == actual_label);
    }

    // tets
    for (int i = 0; i < 2; i++) {
        REQUIRE(mesh.m_tet_attribute[i].label == T0_LABEL);
        REQUIRE(mesh.m_tet_attribute[i].tags == T0_TAGS);
    }
}


TEST_CASE("face_split", "[split_op]")
{
    Eigen::Matrix<double, Eigen::Dynamic, 3> V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::MatrixXi T(1, 4);
    T << 0, 1, 2, 3;
    Eigen::MatrixXd Tags(1, 1);
    Tags << 128.0;

    Parameters param;
    param.tag_name = "dummy";
    param.sep_tag_vals.push_back(1);
    param.sep_tag_vals.push_back(4);
    TopoOffsetMesh mesh(param, 0);
    std::vector<std::string> tag_names(1, "dummy");
    mesh.init_from_image(V, T, Tags, tag_names);

    // give every component unique tag combo
    mesh.m_vertex_attribute[0].label = V0_LABEL;
    mesh.m_vertex_attribute[1].label = V1_LABEL;
    mesh.m_vertex_attribute[2].label = V2_LABEL;
    mesh.m_vertex_attribute[3].label = V3_LABEL;
    mesh.m_edge_attribute[0].label = E0_LABEL;
    mesh.m_edge_attribute[1].label = E1_LABEL;
    mesh.m_edge_attribute[2].label = E2_LABEL;
    mesh.m_edge_attribute[3].label = E3_LABEL;
    mesh.m_edge_attribute[4].label = E4_LABEL;
    mesh.m_edge_attribute[5].label = E5_LABEL;
    mesh.m_face_attribute[0].label = F0_LABEL;
    mesh.m_face_attribute[1].label = F1_LABEL;
    mesh.m_face_attribute[2].label = F2_LABEL;
    mesh.m_face_attribute[3].label = F3_LABEL;
    mesh.m_tet_attribute[0].label = T0_LABEL;

    // split face
    auto [ftup, _] = mesh.tuple_from_face({1, 2, 3});
    std::vector<TetMesh::Tuple> garbage;
    mesh.split_face(ftup, garbage);

    // // ensure proper propagation of attributes
    // vertex
    REQUIRE(mesh.m_vertex_attribute[0].label == V0_LABEL);
    REQUIRE(mesh.m_vertex_attribute[1].label == V1_LABEL);
    REQUIRE(mesh.m_vertex_attribute[2].label == V2_LABEL);
    REQUIRE(mesh.m_vertex_attribute[3].label == V3_LABEL);
    REQUIRE(mesh.m_vertex_attribute[4].label == F3_LABEL);

    // edges
    std::array<std::array<size_t, 3>, 10> edges = {
        {// {v0id, v1id, correct label}
         {0, 1, E0_LABEL},
         {0, 2, E2_LABEL},
         {0, 3, E3_LABEL},
         {0, 4, T0_LABEL},
         {1, 2, E1_LABEL},
         {1, 3, E4_LABEL},
         {1, 4, F3_LABEL},
         {2, 3, E5_LABEL},
         {2, 4, F3_LABEL},
         {3, 4, F3_LABEL}}};
    for (int i = 0; i < 9; i++) {
        size_t v0 = edges[i][0];
        size_t v1 = edges[i][1];
        int correct_label = edges[i][2];
        int actual_label = mesh.m_edge_attribute[mesh.tuple_from_edge({v0, v1}).eid(mesh)].label;
        REQUIRE(actual_label == correct_label);
    }

    // faces
    std::array<std::array<size_t, 4>, 9> faces = {
        {// {v0id, v1id, v2id, correct label}
         {0, 1, 2, F0_LABEL},
         {0, 1, 3, F2_LABEL},
         {0, 1, 4, T0_LABEL},
         {0, 2, 3, F1_LABEL},
         {0, 2, 4, T0_LABEL},
         {0, 3, 4, T0_LABEL},
         {1, 2, 4, F3_LABEL},
         {1, 3, 4, F3_LABEL},
         {2, 3, 4, F3_LABEL}}};
    for (int i = 0; i < 7; i++) {
        size_t v0 = faces[i][0];
        size_t v1 = faces[i][1];
        size_t v2 = faces[i][2];
        int correct_label = faces[i][3];
        int actual_label =
            mesh.m_face_attribute[std::get<1>(mesh.tuple_from_face({v0, v1, v2}))].label;
        REQUIRE(correct_label == actual_label);
    }

    // tets
    for (int i = 0; i < 3; i++) {
        REQUIRE(mesh.m_tet_attribute[i].label == T0_LABEL);
        REQUIRE(mesh.m_tet_attribute[i].tags == T0_TAGS);
    }
}


TEST_CASE("tet_split", "[split_op]")
{
    Eigen::Matrix<double, Eigen::Dynamic, 3> V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::MatrixXi T(1, 4);
    T << 0, 1, 2, 3;
    Eigen::MatrixXd Tags(1, 1);
    Tags << 128.0;

    Parameters param;
    param.tag_name = "dummy";
    param.sep_tag_vals.push_back(1);
    param.sep_tag_vals.push_back(4);
    TopoOffsetMesh mesh(param, 0);
    std::vector<std::string> tag_names(1, "dummy");
    mesh.init_from_image(V, T, Tags, tag_names);

    // give every component unique tag combo
    mesh.m_vertex_attribute[0].label = V0_LABEL;
    mesh.m_vertex_attribute[1].label = V1_LABEL;
    mesh.m_vertex_attribute[2].label = V2_LABEL;
    mesh.m_vertex_attribute[3].label = V3_LABEL;
    mesh.m_edge_attribute[0].label = E0_LABEL;
    mesh.m_edge_attribute[1].label = E1_LABEL;
    mesh.m_edge_attribute[2].label = E2_LABEL;
    mesh.m_edge_attribute[3].label = E3_LABEL;
    mesh.m_edge_attribute[4].label = E4_LABEL;
    mesh.m_edge_attribute[5].label = E5_LABEL;
    mesh.m_face_attribute[0].label = F0_LABEL;
    mesh.m_face_attribute[1].label = F1_LABEL;
    mesh.m_face_attribute[2].label = F2_LABEL;
    mesh.m_face_attribute[3].label = F3_LABEL;
    mesh.m_tet_attribute[0].label = T0_LABEL;

    // split face
    TetMesh::Tuple ttup = mesh.tuple_from_tet(0);
    std::vector<TetMesh::Tuple> garbage;
    mesh.split_tet(ttup, garbage);

    // // ensure proper propagation of attributes
    // vertex
    REQUIRE(mesh.m_vertex_attribute[0].label == V0_LABEL);
    REQUIRE(mesh.m_vertex_attribute[1].label == V1_LABEL);
    REQUIRE(mesh.m_vertex_attribute[2].label == V2_LABEL);
    REQUIRE(mesh.m_vertex_attribute[3].label == V3_LABEL);
    REQUIRE(mesh.m_vertex_attribute[4].label == T0_LABEL);

    // edges
    std::array<std::array<size_t, 3>, 10> edges = {
        {// {v0id, v1id, correct label}
         {0, 1, E0_LABEL},
         {0, 2, E2_LABEL},
         {0, 3, E3_LABEL},
         {0, 4, T0_LABEL},
         {1, 2, E1_LABEL},
         {1, 3, E4_LABEL},
         {1, 4, T0_LABEL},
         {2, 3, E5_LABEL},
         {2, 4, T0_LABEL},
         {3, 4, T0_LABEL}}};
    for (int i = 0; i < 9; i++) {
        size_t v0 = edges[i][0];
        size_t v1 = edges[i][1];
        int correct_label = edges[i][2];
        int actual_label = mesh.m_edge_attribute[mesh.tuple_from_edge({v0, v1}).eid(mesh)].label;
        REQUIRE(actual_label == correct_label);
    }

    // faces
    std::array<std::array<size_t, 4>, 10> faces = {
        {// {v0id, v1id, v2id, correct label}
         {0, 1, 2, F0_LABEL},
         {0, 1, 3, F2_LABEL},
         {0, 1, 4, T0_LABEL},
         {0, 2, 3, F1_LABEL},
         {0, 2, 4, T0_LABEL},
         {0, 3, 4, T0_LABEL},
         {1, 2, 3, F3_LABEL},
         {1, 2, 4, T0_LABEL},
         {1, 3, 4, T0_LABEL},
         {2, 3, 4, T0_LABEL}}};
    for (int i = 0; i < 7; i++) {
        size_t v0 = faces[i][0];
        size_t v1 = faces[i][1];
        size_t v2 = faces[i][2];
        int correct_label = faces[i][3];
        int actual_label =
            mesh.m_face_attribute[std::get<1>(mesh.tuple_from_face({v0, v1, v2}))].label;
        REQUIRE(correct_label == actual_label);
    }

    // tets
    for (int i = 0; i < 4; i++) {
        REQUIRE(mesh.m_tet_attribute[i].label == T0_LABEL);
        REQUIRE(mesh.m_tet_attribute[i].tags == T0_TAGS);
    }
}
