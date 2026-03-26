#include <igl/point_simplex_squared_distance.h>
#include <igl/predicates/predicates.h>
#include <math.h>
#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>
#include <wmtk/components/topological_offset/TopoOffsetTetMesh.h>
#include <wmtk/components/topological_offset/TopoOffsetTriMesh.h>
#include <catch2/catch_test_macros.hpp>
#include <queue>
#include <wmtk/components/topological_offset/Circle.hpp>
#include <wmtk/components/topological_offset/Sphere.hpp>

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
    param.offset_tags.push_back({{0, 1}});
    param.offset_tags.push_back({{0, 4}});
    TopoOffsetTetMesh mesh(param, 0);
    mesh.init_from_image(V, T, Tags);

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
    param.offset_tags.push_back({{0, 1}});
    param.offset_tags.push_back({{0, 4}});
    TopoOffsetTetMesh mesh(param, 0);
    mesh.init_from_image(V, T, Tags);

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
    param.offset_tags.push_back({{0, 1}});
    param.offset_tags.push_back({{0, 4}});
    TopoOffsetTetMesh mesh(param, 0);
    mesh.init_from_image(V, T, Tags);

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
        param.offset_tags.push_back({{0, 1}});
        param.offset_tags.push_back({{0, 4}});
        TopoOffsetTetMesh mesh(param, 0);
        mesh.init_from_image(V, T, Tags);

        std::vector<TetMesh::Tuple> tets;
        tets.push_back(mesh.tuple_from_tet(0));
        REQUIRE((mesh.invariants(tets) == false));
    }
    { // mesh 2 (good)
        Eigen::MatrixXi T(1, 4);
        T << 0, 1, 2, 3;

        Parameters param;
        param.offset_tags.push_back({{0, 1}});
        param.offset_tags.push_back({{0, 4}});
        TopoOffsetTetMesh mesh(param, 0);
        mesh.init_from_image(V, T, Tags);

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
    param.offset_tags.push_back({{0, 1}});
    param.offset_tags.push_back({{0, 4}});
    TopoOffsetTriMesh mesh(param, 0);
    mesh.init_from_image(V, F, Tags);

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
    param.offset_tags.push_back({{0, 1}});
    param.offset_tags.push_back({{0, 4}});
    TopoOffsetTriMesh mesh(param, 0);
    mesh.init_from_image(V, F, Tags);

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
    param.offset_tags.push_back({{0, 1}});
    param.offset_tags.push_back({{0, 4}});
    TopoOffsetTriMesh mesh(param, 0);
    mesh.init_from_image(V, F, Tags);

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
        param.offset_tags.push_back({{0, 1}});
        param.offset_tags.push_back({{0, 4}});
        TopoOffsetTriMesh mesh(param, 0);
        mesh.init_from_image(V, F, Tags);

        std::vector<TriMesh::Tuple> tris;
        tris.push_back(mesh.tuple_from_tri(0));
        REQUIRE((mesh.invariants(tris) == false));
    }
    { // mesh 2 (good)
        Eigen::MatrixXi F(1, 3);
        F << 0, 1, 2;

        Parameters param;
        param.offset_tags.push_back({{0, 1}});
        param.offset_tags.push_back({{0, 4}});
        TopoOffsetTriMesh mesh(param, 0);
        mesh.init_from_image(V, F, Tags);

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
    param.offset_tags.push_back({{0, 1}});
    param.offset_tags.push_back({{0, 4}});
    TopoOffsetTriMesh mesh(param, 0);
    mesh.init_from_image(V, F, Tags);

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
    param.offset_tags.push_back({{0, 1}});
    param.offset_tags.push_back({{0, 4}});
    TopoOffsetTriMesh mesh(param, 0);
    mesh.init_from_image(V, F, Tags);

    Circle circ(mesh, 0);
    REQUIRE(fabs(circ.radius() - (sqrt(2.0) / 2.0)) < pow(10, -6));
}


TEST_CASE("dist_to_trimesh", "[dist_growth][2d]")
{
    Eigen::MatrixXd V(3, 2);
    V << 0, 0, 0, 1, 1, 0;
    Eigen::MatrixXi F(1, 3);
    F << 0, 1, 2;
    Eigen::MatrixXd Tags(1, 1); // dont care
    Tags << 0.0;

    Parameters param;
    param.offset_tags.push_back({{0, 1}});
    param.offset_tags.push_back({{0, 4}});
    TopoOffsetTriMesh mesh(param, 0);
    mesh.init_from_image(V, F, Tags);

    // label one vert as input
    mesh.m_vertex_attribute[0].label = 1;
    mesh.init_input_complex_bvh();
    Vector2d q(1.0, 1.0);
    double dist = mesh.m_input_complex_bvh.dist(q);
    REQUIRE(fabs(dist - sqrt(2.0)) < pow(10, -6));

    Vector2d q0(0.0, 0.0);
    dist = mesh.m_input_complex_bvh.dist(q0);
    REQUIRE(fabs(dist) < pow(10, -6));

    // label edges and vertices as input
    for (int i = 0; i < 3; i++) {
        mesh.m_edge_attribute[i].label = 1;
        mesh.m_vertex_attribute[i].label = 1;
    }
    mesh.init_input_complex_bvh();

    Vector2d q1(1.0, 1.0);
    dist = mesh.m_input_complex_bvh.dist(q1);
    REQUIRE(fabs(dist - (sqrt(2) / 2.0)) < pow(10, -6));

    Vector2d q2(0.0, 0.0);
    dist = mesh.m_input_complex_bvh.dist(q2);
    REQUIRE(fabs(dist) < pow(10, -6));

    Vector2d q3(10.0, 0.0);
    dist = mesh.m_input_complex_bvh.dist(q3);
    REQUIRE(fabs(dist - 9.0) < pow(10, -6));
}


TEST_CASE("dist_to_tetmesh", "[dist_growth][3d]")
{
    Eigen::MatrixXd V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::MatrixXi T(1, 4);
    T << 0, 1, 2, 3;
    Eigen::MatrixXd Tags(1, 1); // dont care
    Tags << 0.0;

    Parameters param;
    param.offset_tags.push_back({{0, 1}});
    param.offset_tags.push_back({{0, 4}});
    TopoOffsetTetMesh mesh(param, 0);
    mesh.init_from_image(V, T, Tags);

    // label one vert as input
    mesh.m_vertex_attribute[0].label = 1;
    mesh.init_input_complex_bvh();
    Vector3d q(1.0, 1.0, 1.0);
    double dist = mesh.m_input_complex_bvh.dist(q);
    REQUIRE(fabs(dist - sqrt(3.0)) < pow(10, -6));

    Vector3d q0(0.0, 0.0, 0.0);
    dist = mesh.m_input_complex_bvh.dist(q0);
    REQUIRE(fabs(dist) < pow(10, -6));

    // label faces, edges, and vertices as input
    for (int i = 0; i < 4; i++) {
        mesh.m_face_attribute[i].label = 1;
    }
    for (int i = 0; i < 6; i++) {
        mesh.m_edge_attribute[i].label = 1;
    }
    for (int i = 0; i < 4; i++) {
        mesh.m_vertex_attribute[i].label = 1;
    }
    mesh.init_input_complex_bvh();

    Vector3d q1(1.0, 1.0, 1.0);
    dist = mesh.m_input_complex_bvh.dist(q1);
    REQUIRE(fabs(dist - (2.0 / sqrt(3.0))) < pow(10, -6));

    Vector3d q2(0.0, 0.0, 0.0);
    dist = mesh.m_input_complex_bvh.dist(q2);
    REQUIRE(fabs(dist) < pow(10, -6));

    Vector3d q3(10.0, 2.0, 1.0);
    dist = mesh.m_input_complex_bvh.dist(q3);
    REQUIRE(fabs(dist - sqrt(86.0)) < pow(10, -6));

    Vector3d q4(0.1, 0.2, 0.3);
    dist = mesh.m_input_complex_bvh.dist(q4);
    REQUIRE(fabs(dist - 0.1) < pow(10, -6));

    // label tet as input
    mesh.m_tet_attribute[0].label = 1;
    mesh.init_input_complex_bvh();

    dist = mesh.m_input_complex_bvh.dist(q4);
    REQUIRE(fabs(dist) < pow(10, -6));

    dist = mesh.m_input_complex_bvh.dist(q3);
    REQUIRE(fabs(dist - sqrt(86.0)) < pow(10, -6));
}


TEST_CASE("cube_tet_fit", "[dist_growth][3d]")
{
    Vector3d p0(0.0, 0.0, 0.0);
    Vector3d p1(1.0, 0.0, 0.0);
    Vector3d p2(0.0, 2.0, 0.0);
    Vector3d p3(0.0, 0.0, 3.0);
    Vector3d res_c;
    double res_l;
    Sphere::fit_cube(p0, p1, p2, p3, res_c, res_l);
    bool res = (fabs(res_c(0) - 0.5) < pow(10, -6)) && (fabs(res_c(1) - 1.0) < pow(10, -6)) &&
               (fabs(res_c(2) - 1.5) < pow(10, -6)) && (fabs(res_l - 3.0) < pow(10, -6));
    REQUIRE(res);
}


TEST_CASE("sphere_tet_overlap", "[dist_growth][3d]")
{
    Eigen::MatrixXd V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::MatrixXi T(1, 4);
    T << 0, 1, 2, 3;
    Eigen::MatrixXd Tags(1, 1); // dont care
    Tags << 0.0;

    Parameters param;
    param.offset_tags.push_back({{0, 1}});
    param.offset_tags.push_back({{0, 4}});
    TopoOffsetTetMesh mesh(param, 0);
    mesh.init_from_image(V, T, Tags);

    Sphere sphere1(Vector3d(1.0, 1.0, 1.0), 0.5); // false
    Sphere sphere2(Vector3d(0.55, 0.55, 0.55), 0.5); // true
    Sphere sphere3(Vector3d(0.2, 0.2, 0.2), 0.1); // true
    Sphere sphere4(Vector3d(0.2, 0.2, 0.2), 1000.0); // true
    Sphere sphere5(Vector3d(10.0, 10.0, 10.0), 1.0); // false

    REQUIRE(!sphere1.overlaps_tet(mesh, 0));
    REQUIRE(sphere2.overlaps_tet(mesh, 0));
    REQUIRE(sphere3.overlaps_tet(mesh, 0));
    REQUIRE(sphere4.overlaps_tet(mesh, 0));
    REQUIRE(!sphere5.overlaps_tet(mesh, 0));
}


TEST_CASE("sphere_refine", "[dist_growth][2d]")
{
    Sphere s(Vector3d(0.0, 0.0, 0.0), 1.0);
    std::queue<Sphere> q;
    s.refine(q);
    REQUIRE(q.size() == 8);

    double r_new = q.front().radius();
    REQUIRE(fabs(r_new - 0.5) < pow(10, -6));

    Vector3d c1 = q.front().center();
    REQUIRE(fabs(c1(0) + (1.0 / (2.0 * sqrt(3.0)))) < pow(10, -6));
    REQUIRE(fabs(c1(1) + (1.0 / (2.0 * sqrt(3.0)))) < pow(10, -6));
    REQUIRE(fabs(c1(2) + (1.0 / (2.0 * sqrt(3.0)))) < pow(10, -6));
}
