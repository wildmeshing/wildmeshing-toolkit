#include <igl/point_simplex_squared_distance.h>
#include <igl/predicates/predicates.h>
#include <math.h>
#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>
#include <wmtk/components/topological_offset/TopoOffsetMesh.h>
#include <wmtk/components/topological_offset/TopoOffsetTriMesh.h>
#include <catch2/catch_test_macros.hpp>
#include <queue>
#include <wmtk/components/topological_offset/Circle.hpp>

using namespace wmtk;
using namespace components::topological_offset;


// used for checking attribute propagation. values are arbitrary
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
const std::vector<double> F0_TAGS = {128.0}; // for trimesh
const std::vector<double> F1_TAGS = {256.0}; // for trimesh


TEST_CASE("edge_split_3d", "[split_op][3d]")
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
    TetMesh::Tuple e = mesh.tuple_from_edge({{1, 2}});
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
         {{0, 1, E0_LABEL}},
         {{0, 2, E2_LABEL}},
         {{0, 3, E3_LABEL}},
         {{0, 4, F0_LABEL}},
         {{1, 3, E4_LABEL}},
         {{1, 4, E1_LABEL}},
         {{2, 3, E5_LABEL}},
         {{2, 4, E1_LABEL}},
         {{3, 4, F3_LABEL}}}};
    for (int i = 0; i < 9; i++) {
        size_t v0 = edges[i][0];
        size_t v1 = edges[i][1];
        int correct_label = edges[i][2];
        int actual_label = mesh.m_edge_attribute[mesh.tuple_from_edge({{v0, v1}}).eid(mesh)].label;
        REQUIRE(actual_label == correct_label);
    }

    // faces
    std::array<std::array<size_t, 4>, 7> faces = {
        {{{0, 1, 3, F2_LABEL}},
         {{0, 1, 4, F0_LABEL}},
         {{0, 2, 3, F1_LABEL}},
         {{0, 2, 4, F0_LABEL}},
         {{0, 3, 4, T0_LABEL}},
         {{1, 3, 4, F3_LABEL}},
         {{2, 3, 4, F3_LABEL}}}};
    for (int i = 0; i < 7; i++) {
        size_t v0 = faces[i][0];
        size_t v1 = faces[i][1];
        size_t v2 = faces[i][2];
        int correct_label = faces[i][3];
        int actual_label =
            mesh.m_face_attribute[std::get<1>(mesh.tuple_from_face({{v0, v1, v2}}))].label;
        REQUIRE(correct_label == actual_label);
    }

    // tets
    for (int i = 0; i < 2; i++) {
        REQUIRE(mesh.m_tet_attribute[i].label == T0_LABEL);
        REQUIRE(mesh.m_tet_attribute[i].tags == T0_TAGS);
    }
}


TEST_CASE("face_split_3d", "[split_op][3d]")
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
    auto [ftup, _] = mesh.tuple_from_face({{1, 2, 3}});
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
         {{0, 1, E0_LABEL}},
         {{0, 2, E2_LABEL}},
         {{0, 3, E3_LABEL}},
         {{0, 4, T0_LABEL}},
         {{1, 2, E1_LABEL}},
         {{1, 3, E4_LABEL}},
         {{1, 4, F3_LABEL}},
         {{2, 3, E5_LABEL}},
         {{2, 4, F3_LABEL}},
         {{3, 4, F3_LABEL}}}};
    for (int i = 0; i < 9; i++) {
        size_t v0 = edges[i][0];
        size_t v1 = edges[i][1];
        int correct_label = edges[i][2];
        int actual_label = mesh.m_edge_attribute[mesh.tuple_from_edge({{v0, v1}}).eid(mesh)].label;
        REQUIRE(actual_label == correct_label);
    }

    // faces
    std::array<std::array<size_t, 4>, 9> faces = {
        {// {v0id, v1id, v2id, correct label}
         {{0, 1, 2, F0_LABEL}},
         {{0, 1, 3, F2_LABEL}},
         {{0, 1, 4, T0_LABEL}},
         {{0, 2, 3, F1_LABEL}},
         {{0, 2, 4, T0_LABEL}},
         {{0, 3, 4, T0_LABEL}},
         {{1, 2, 4, F3_LABEL}},
         {{1, 3, 4, F3_LABEL}},
         {{2, 3, 4, F3_LABEL}}}};
    for (int i = 0; i < 7; i++) {
        size_t v0 = faces[i][0];
        size_t v1 = faces[i][1];
        size_t v2 = faces[i][2];
        int correct_label = faces[i][3];
        int actual_label =
            mesh.m_face_attribute[std::get<1>(mesh.tuple_from_face({{v0, v1, v2}}))].label;
        REQUIRE(correct_label == actual_label);
    }

    // tets
    for (int i = 0; i < 3; i++) {
        REQUIRE(mesh.m_tet_attribute[i].label == T0_LABEL);
        REQUIRE(mesh.m_tet_attribute[i].tags == T0_TAGS);
    }
}


TEST_CASE("tet_split_3d", "[split_op][3d]")
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
         {{0, 1, E0_LABEL}},
         {{0, 2, E2_LABEL}},
         {{0, 3, E3_LABEL}},
         {{0, 4, T0_LABEL}},
         {{1, 2, E1_LABEL}},
         {{1, 3, E4_LABEL}},
         {{1, 4, T0_LABEL}},
         {{2, 3, E5_LABEL}},
         {{2, 4, T0_LABEL}},
         {{3, 4, T0_LABEL}}}};
    for (int i = 0; i < 9; i++) {
        size_t v0 = edges[i][0];
        size_t v1 = edges[i][1];
        int correct_label = edges[i][2];
        int actual_label = mesh.m_edge_attribute[mesh.tuple_from_edge({{v0, v1}}).eid(mesh)].label;
        REQUIRE(actual_label == correct_label);
    }

    // faces
    std::array<std::array<size_t, 4>, 10> faces = {
        {// {v0id, v1id, v2id, correct label}
         {{0, 1, 2, F0_LABEL}},
         {{0, 1, 3, F2_LABEL}},
         {{0, 1, 4, T0_LABEL}},
         {{0, 2, 3, F1_LABEL}},
         {{0, 2, 4, T0_LABEL}},
         {{0, 3, 4, T0_LABEL}},
         {{1, 2, 3, F3_LABEL}},
         {{1, 2, 4, T0_LABEL}},
         {{1, 3, 4, T0_LABEL}},
         {{2, 3, 4, T0_LABEL}}}};
    for (int i = 0; i < 7; i++) {
        size_t v0 = faces[i][0];
        size_t v1 = faces[i][1];
        size_t v2 = faces[i][2];
        int correct_label = faces[i][3];
        int actual_label =
            mesh.m_face_attribute[std::get<1>(mesh.tuple_from_face({{v0, v1, v2}}))].label;
        REQUIRE(correct_label == actual_label);
    }

    // tets
    for (int i = 0; i < 4; i++) {
        REQUIRE(mesh.m_tet_attribute[i].label == T0_LABEL);
        REQUIRE(mesh.m_tet_attribute[i].tags == T0_TAGS);
    }
}


TEST_CASE("invariant_3d", "[3d]")
{
    Eigen::Matrix<double, Eigen::Dynamic, 3> V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::MatrixXd Tags(1, 1);
    Tags << 0.0;

    { // mesh 1 (bad)
        Eigen::MatrixXi T(1, 4);
        T << 0, 1, 3, 2;

        Parameters param;
        param.tag_name = "dummy";
        param.sep_tag_vals.push_back(1);
        param.sep_tag_vals.push_back(4);
        TopoOffsetMesh mesh(param, 0);
        std::vector<std::string> tag_names(1, "dummy");
        mesh.init_from_image(V, T, Tags, tag_names);

        std::vector<TetMesh::Tuple> tets;
        tets.push_back(mesh.tuple_from_tet(0));
        REQUIRE((mesh.invariants(tets) == false));
    }
    { // mesh 2 (good)
        Eigen::MatrixXi T(1, 4);
        T << 0, 1, 2, 3;

        Parameters param;
        param.tag_name = "dummy";
        param.sep_tag_vals.push_back(1);
        param.sep_tag_vals.push_back(4);
        TopoOffsetMesh mesh(param, 0);
        std::vector<std::string> tag_names(1, "dummy");
        mesh.init_from_image(V, T, Tags, tag_names);

        std::vector<TetMesh::Tuple> tets;
        tets.push_back(mesh.tuple_from_tet(0));
        REQUIRE(mesh.invariants(tets));
    }
}


TEST_CASE("edge_split_2d_1face", "[split_op][2d]")
{
    Eigen::Matrix<double, 3, 2> V(3, 2);
    V << 0, 0, 1, 0, 0, 1;
    Eigen::MatrixXi F(1, 3);
    F << 0, 1, 2;
    Eigen::MatrixXd Tags(1, 1);
    Tags << 128.0;

    Parameters param;
    param.tag_name = "dummy";
    param.sep_tag_vals.push_back(1);
    param.sep_tag_vals.push_back(4);
    TopoOffsetTriMesh mesh(param, 0);
    std::vector<std::string> tag_names(1, "dummy");
    mesh.init_from_image(V, F, Tags, tag_names);

    // give every component unique tag combo
    mesh.m_vertex_attribute[0].label = V0_LABEL;
    mesh.m_vertex_attribute[1].label = V1_LABEL;
    mesh.m_vertex_attribute[2].label = V2_LABEL;
    mesh.m_edge_attribute[0].label = E0_LABEL;
    mesh.m_edge_attribute[1].label = E1_LABEL;
    mesh.m_edge_attribute[2].label = E2_LABEL;
    mesh.m_face_attribute[0].label = F0_LABEL;
    mesh.m_face_attribute[0].tags = F0_TAGS;

    // split edge
    TriMesh::Tuple e = mesh.tuple_from_edge(1, 2, 0);
    std::vector<TriMesh::Tuple> garbage;
    mesh.split_edge(e, garbage);

    // // ensure proper propagation of attributes
    // vertex
    REQUIRE(mesh.m_vertex_attribute[0].label == V0_LABEL);
    REQUIRE(mesh.m_vertex_attribute[1].label == V1_LABEL);
    REQUIRE(mesh.m_vertex_attribute[2].label == V2_LABEL);
    REQUIRE(mesh.m_vertex_attribute[3].label == E0_LABEL);

    // edges
    std::array<std::array<size_t, 4>, 5> edges = {
        {// {v0id, v1id, v_other, correct label}
         {{0, 1, 3, E2_LABEL}},
         {{0, 2, 3, E1_LABEL}},
         {{0, 3, 1, F0_LABEL}},
         {{1, 3, 0, E0_LABEL}},
         {{2, 3, 0, E0_LABEL}}}};
    for (int i = 0; i < 5; i++) {
        size_t v0 = edges[i][0];
        size_t v1 = edges[i][1];
        size_t v_other = edges[i][2];
        int correct_label = edges[i][3];
        TriMesh::Tuple ftup = mesh.tuple_from_simplex(simplex::Face(v0, v1, v_other));
        int actual_label =
            mesh.m_edge_attribute[mesh.tuple_from_edge(v0, v1, ftup.fid(mesh)).eid(mesh)].label;
        REQUIRE(actual_label == correct_label);
    }

    // faces
    for (int i = 0; i < 2; i++) {
        REQUIRE(mesh.m_face_attribute[i].label == F0_LABEL);
        REQUIRE(mesh.m_face_attribute[i].tags == F0_TAGS);
    }
}


TEST_CASE("edge_split_2d_2faces", "[split_op][2d]")
{
    Eigen::Matrix<double, 4, 2> V(4, 2);
    V << 0, 0, 1, 0, 0, 1, 1, 1;
    Eigen::MatrixXi F(2, 3);
    F << 0, 1, 2, 1, 3, 2;
    Eigen::MatrixXd Tags(2, 1);
    Tags << 128.0, 256.0;

    Parameters param;
    param.tag_name = "dummy";
    param.sep_tag_vals.push_back(1);
    param.sep_tag_vals.push_back(4);
    TopoOffsetTriMesh mesh(param, 0);
    std::vector<std::string> tag_names(1, "dummy");
    mesh.init_from_image(V, F, Tags, tag_names);

    // give every component unique tag combo
    mesh.m_vertex_attribute[0].label = V0_LABEL;
    mesh.m_vertex_attribute[1].label = V1_LABEL;
    mesh.m_vertex_attribute[2].label = V2_LABEL;
    mesh.m_vertex_attribute[3].label = V3_LABEL;
    mesh.m_edge_attribute[0].label = E0_LABEL;
    mesh.m_edge_attribute[1].label = E1_LABEL;
    mesh.m_edge_attribute[2].label = E2_LABEL;
    mesh.m_edge_attribute[3].label = E3_LABEL;
    mesh.m_edge_attribute[5].label = E5_LABEL;
    mesh.m_face_attribute[0].label = F0_LABEL;
    mesh.m_face_attribute[0].tags = F0_TAGS;
    mesh.m_face_attribute[1].label = F1_LABEL;
    mesh.m_face_attribute[1].tags = F1_TAGS;

    // split edge
    TriMesh::Tuple e = mesh.tuple_from_edge(1, 2, 0);
    std::vector<TriMesh::Tuple> garbage;
    mesh.split_edge(e, garbage);

    // // ensure proper propagation of attributes
    // vertex
    REQUIRE(mesh.m_vertex_attribute[0].label == V0_LABEL);
    REQUIRE(mesh.m_vertex_attribute[1].label == V1_LABEL);
    REQUIRE(mesh.m_vertex_attribute[2].label == V2_LABEL);
    REQUIRE(mesh.m_vertex_attribute[3].label == V3_LABEL);
    REQUIRE(mesh.m_vertex_attribute[4].label == E0_LABEL);

    // edges
    std::array<std::array<size_t, 4>, 8> edges = {
        {// {v0id, v1id, correct label}
         {{0, 1, E2_LABEL}},
         {{0, 2, E1_LABEL}},
         {{0, 4, F0_LABEL}},
         {{1, 3, E5_LABEL}},
         {{1, 4, E0_LABEL}},
         {{2, 3, E3_LABEL}},
         {{2, 4, E0_LABEL}},
         {{3, 4, F1_LABEL}}}};
    for (int i = 0; i < 8; i++) {
        size_t v0 = edges[i][0];
        size_t v1 = edges[i][1];
        int correct_label = edges[i][2];
        size_t e_id = mesh.edge_id_from_simplex(simplex::Edge(v0, v1));
        int actual_label = mesh.m_edge_attribute[e_id].label;
        REQUIRE(actual_label == correct_label);
    }

    // faces
    std::array<std::array<size_t, 3>, 4> faces = {
        {{{0, 1, 4}}, {{0, 4, 2}}, {{1, 3, 4}}, {{4, 3, 2}}}};
    for (int i = 0; i < 4; i++) {
        TriMesh::Tuple ftup =
            mesh.tuple_from_simplex(simplex::Face(faces[i][0], faces[i][1], faces[i][2]));
        size_t f_id = ftup.fid(mesh);
        if (i < 2) {
            REQUIRE(mesh.m_face_attribute[f_id].label == F0_LABEL);
            REQUIRE(mesh.m_face_attribute[f_id].tags == F0_TAGS);
        } else {
            REQUIRE(mesh.m_face_attribute[f_id].label == F1_LABEL);
            REQUIRE(mesh.m_face_attribute[f_id].tags == F1_TAGS);
        }
    }
}


TEST_CASE("face_split_2d", "[split_op][2d]")
{
    Eigen::Matrix<double, 3, 2> V(3, 2);
    V << 0, 0, 1, 0, 0, 1;
    Eigen::MatrixXi F(1, 3);
    F << 0, 1, 2;
    Eigen::MatrixXd Tags(1, 1);
    Tags << 128.0;

    Parameters param;
    param.tag_name = "dummy";
    param.sep_tag_vals.push_back(1);
    param.sep_tag_vals.push_back(4);
    TopoOffsetTriMesh mesh(param, 0);
    std::vector<std::string> tag_names(1, "dummy");
    mesh.init_from_image(V, F, Tags, tag_names);

    // give every component unique tag combo
    mesh.m_vertex_attribute[0].label = V0_LABEL;
    mesh.m_vertex_attribute[1].label = V1_LABEL;
    mesh.m_vertex_attribute[2].label = V2_LABEL;
    mesh.m_edge_attribute[0].label = E0_LABEL;
    mesh.m_edge_attribute[1].label = E1_LABEL;
    mesh.m_edge_attribute[2].label = E2_LABEL;
    mesh.m_face_attribute[0].label = F0_LABEL;
    mesh.m_face_attribute[0].tags = F0_TAGS;

    // split face
    TriMesh::Tuple f = mesh.tuple_from_simplex(simplex::Face(0, 1, 2));
    std::vector<TriMesh::Tuple> garbage;
    mesh.split_face(f, garbage);

    // // ensure proper propagation of attributes
    // vertex
    REQUIRE(mesh.m_vertex_attribute[0].label == V0_LABEL);
    REQUIRE(mesh.m_vertex_attribute[1].label == V1_LABEL);
    REQUIRE(mesh.m_vertex_attribute[2].label == V2_LABEL);
    REQUIRE(mesh.m_vertex_attribute[3].label == F0_LABEL);

    // edges
    std::array<std::array<size_t, 4>, 6> edges = {
        {// {v0id, v1id, correct label}
         {{0, 1, E2_LABEL}},
         {{0, 2, E1_LABEL}},
         {{0, 3, F0_LABEL}},
         {{1, 2, E0_LABEL}},
         {{1, 3, F0_LABEL}},
         {{2, 3, F0_LABEL}}}};
    for (int i = 0; i < 6; i++) {
        size_t v0 = edges[i][0];
        size_t v1 = edges[i][1];
        int correct_label = edges[i][2];
        size_t e_id = mesh.edge_id_from_simplex(simplex::Edge(v0, v1));
        int actual_label = mesh.m_edge_attribute[e_id].label;
        REQUIRE(actual_label == correct_label);
    }

    // faces
    std::array<std::array<size_t, 3>, 4> faces = {{{{0, 1, 3}}, {{0, 3, 2}}, {{1, 2, 3}}}};
    for (int i = 0; i < 3; i++) {
        TriMesh::Tuple ftup =
            mesh.tuple_from_simplex(simplex::Face(faces[i][0], faces[i][1], faces[i][2]));
        size_t f_id = ftup.fid(mesh);
        REQUIRE(mesh.m_face_attribute[f_id].label == F0_LABEL);
        REQUIRE(mesh.m_face_attribute[f_id].tags == F0_TAGS);
    }
}


TEST_CASE("invariant_2d", "[2d]")
{
    Eigen::Matrix<double, 3, 2> V(3, 2);
    V << 0, 0, 1, 0, 0, 1;
    Eigen::MatrixXd Tags(1, 1);
    Tags << 0.0;

    { // mesh 1 (bad)
        Eigen::MatrixXi F(1, 3);
        F << 0, 2, 1;

        Parameters param;
        param.tag_name = "dummy";
        param.sep_tag_vals.push_back(1);
        param.sep_tag_vals.push_back(4);
        TopoOffsetTriMesh mesh(param, 0);
        std::vector<std::string> tag_names(1, "dummy");
        mesh.init_from_image(V, F, Tags, tag_names);

        std::vector<TriMesh::Tuple> tris;
        tris.push_back(mesh.tuple_from_tri(0));
        REQUIRE((mesh.invariants(tris) == false));
    }
    { // mesh 2 (good)
        Eigen::MatrixXi F(1, 3);
        F << 0, 1, 2;

        Parameters param;
        param.tag_name = "dummy";
        param.sep_tag_vals.push_back(1);
        param.sep_tag_vals.push_back(4);
        TopoOffsetTriMesh mesh(param, 0);
        std::vector<std::string> tag_names(1, "dummy");
        mesh.init_from_image(V, F, Tags, tag_names);

        std::vector<TriMesh::Tuple> tris;
        tris.push_back(mesh.tuple_from_tri(0));
        REQUIRE(mesh.invariants(tris));
    }
}


TEST_CASE("circle_tri_overlap", "[dist_growth][2d]")
{
    Eigen::MatrixXd V(3, 2);
    V << 0, 0, 0, 1, 1, 0;
    Eigen::MatrixXi F(1, 3);
    F << 0, 1, 2;
    Eigen::MatrixXd Tags(1, 1); // dont care
    Tags << 0.0;

    Parameters param;
    param.tag_name = "dummy";
    param.sep_tag_vals.push_back(1);
    param.sep_tag_vals.push_back(4);
    TopoOffsetTriMesh mesh(param, 0);
    std::vector<std::string> tag_names(1, "dummy");
    mesh.init_from_image(V, F, Tags, tag_names);

    Circle circ1(Vector2d(1.0, 1.0), 0.5); // false
    Circle circ2(Vector2d(0.55, 0.55), 0.5); // true
    Circle circ3(Vector2d(0.2, 0.2), 0.1); // true
    Circle circ4(Vector2d(0.2, 0.2), 1000.0); // true
    Circle circ5(Vector2d(10.0, 10.0), 1.0); // false

    REQUIRE(!circ1.overlaps_tri(mesh, 0));
    REQUIRE(circ2.overlaps_tri(mesh, 0));
    REQUIRE(circ3.overlaps_tri(mesh, 0));
    REQUIRE(circ4.overlaps_tri(mesh, 0));
    REQUIRE(!circ5.overlaps_tri(mesh, 0));
}


TEST_CASE("circle_refine", "[dist_growth][2d]")
{
    Circle c(Vector2d(0.0, 0.0), 1.0);
    std::queue<Circle> q;
    c.refine(q);
    REQUIRE(q.size() == 4);

    double r_new = q.front().radius();
    REQUIRE(fabs(r_new - 0.5) < pow(10, -6));

    Vector2d c1 = q.front().center();
    REQUIRE(fabs(c1(0) + (1.0 / (2.0 * sqrt(2.0)))) < pow(10, -6));
    REQUIRE(fabs(c1(1) + (1.0 / (2.0 * sqrt(2.0)))) < pow(10, -6));
}


TEST_CASE("circle_init", "[dist_growth][2d]")
{
    MatrixXd V(3, 2);
    V << 0, 0, 1, 0, 0.5, 0.5 * sqrt(3.0);
    MatrixXi F(1, 3);
    F << 0, 1, 2;
    Eigen::MatrixXd Tags(1, 1); // dont care
    Tags << 0.0;

    Parameters param;
    param.tag_name = "dummy";
    param.sep_tag_vals.push_back(1);
    param.sep_tag_vals.push_back(4);
    TopoOffsetTriMesh mesh(param, 0);
    std::vector<std::string> tag_names(1, "dummy");
    mesh.init_from_image(V, F, Tags, tag_names);

    Circle circ(mesh, 0);
    REQUIRE(fabs(circ.radius() - (sqrt(2.0) / 2.0)) < pow(10, -6));
}


TEST_CASE("dist_to_mesh", "[dist_growth][2d]")
{
    Eigen::MatrixXd V(3, 2);
    V << 0, 0, 0, 1, 1, 0;
    Eigen::MatrixXi F(1, 3);
    F << 0, 1, 2;
    Eigen::MatrixXd Tags(1, 1); // dont care
    Tags << 0.0;

    Parameters param;
    param.tag_name = "dummy";
    param.sep_tag_vals.push_back(1);
    param.sep_tag_vals.push_back(4);
    TopoOffsetTriMesh mesh(param, 0);
    std::vector<std::string> tag_names(1, "dummy");
    mesh.init_from_image(V, F, Tags, tag_names);

    // label one vert as input
    mesh.m_vertex_attribute[0].label = 1;
    mesh.init_input_complex_bvh();
    Vector2d q(1.0, 1.0);
    double dist = mesh.dist_to_input_complex(q);
    REQUIRE(fabs(dist - sqrt(2.0)) < pow(10, -6));

    Vector2d q0(0.0, 0.0);
    dist = mesh.dist_to_input_complex(q0);
    REQUIRE(fabs(dist) < pow(10, -6));

    // label edges and vertices as input
    for (int i = 0; i < 3; i++) {
        mesh.m_edge_attribute[i].label = 1;
        mesh.m_vertex_attribute[i].label = 1;
    }
    mesh.init_input_complex_bvh();

    Vector2d q1(1.0, 1.0);
    dist = mesh.dist_to_input_complex(q1);
    REQUIRE(fabs(dist - (sqrt(2) / 2.0)) < pow(10, -6));

    Vector2d q2(0.0, 0.0);
    dist = mesh.dist_to_input_complex(q2);
    REQUIRE(fabs(dist) < pow(10, -6));

    Vector2d q3(10.0, 0.0);
    dist = mesh.dist_to_input_complex(q3);
    REQUIRE(fabs(dist - 9.0) < pow(10, -6));
}


// TEST_CASE("simplex_dist", "[debug][2d]")
// {
//     MatrixXd V(3, 2);
//     V << 0, 0, 1, 0, 0, 1;
//     MatrixXi F(1, 3);
//     F << 0, 1, 2;
//     double distance;
//     Vector2d closest_p;
//     igl::point_simplex_squared_distance<2>(Vector2d(0.2, 0.2), V, F, 0, distance, closest_p);
//     std::cout << distance << std::endl;
//     std::cout << closest_p(0) << " " << closest_p(1) << std::endl;
// }


// TEST_CASE("hex_offset_debug", "[debug][2d]")
// {
//     MatrixXd V(10, 2);
//     V.row(0) = Vector2d(0.0, 0.0);
//     for (int i = 0; i < 6; i++) {
//         double theta = (M_PI / 3.0) * i;
//         V(i + 1, 0) = 2.5 * cos(theta);
//         V(i + 1, 1) = 2.5 * sin(theta);
//     }
//     V.row(7) = Vector2d(-5.25, 0.433013);
//     V.row(8) = Vector2d(-5.5, 0.866025);
//     V.row(9) = Vector2d(-5.0, 0.866025);
//     MatrixXi F(7, 3);
//     F << 0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 5, 0, 5, 6, 0, 6, 1, 7, 8, 9;
//     MatrixXd Tags(7, 1);
//     Tags.fill(128.0);

//     Parameters param;
//     param.tag_name = "dummy";
//     param.sep_tag_vals.push_back(1);
//     param.sep_tag_vals.push_back(4);
//     param.target_distance = 3.1;
//     TopoOffsetTriMesh mesh(param, 0);
//     std::vector<std::string> tag_names(1, "dummy");
//     mesh.init_from_image(V, F, Tags, tag_names);
//     std::string path =
//         "/Users/seb9449/Desktop/wildmeshing/image_sim/adaptive_growth/hex_offset_debug";
//     mesh.write_vtu(path);

//     // label input
//     for (int i = 0; i < 6; i++) {
//         simplex::Edge e(i + 1, ((i + 1) % 6) + 1);
//         TriMesh::Tuple etup = mesh.get_tuple_from_edge(e);
//         mesh.m_edge_attribute[etup.eid(mesh)].label = 1;
//         mesh.m_vertex_attribute[etup.vid(mesh)].label = 1;
//         mesh.m_vertex_attribute[etup.switch_vertex(mesh).vid(mesh)].label = 1;
//     }
//     mesh.write_input_complex(path + "_inputcomplex");

//     bool res = mesh.tri_is_in_offset_conservative(6, 0.01 * 3.1);
//     REQUIRE(!res);
// }
