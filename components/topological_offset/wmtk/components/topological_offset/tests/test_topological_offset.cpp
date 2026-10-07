#include <math.h>
#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>
#include <wmtk/components/topological_offset/TopoOffsetTetMesh.h>
#include <wmtk/components/topological_offset/TopoOffsetTriMesh.h>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <functional>
#include <memory>
#include <numeric>
#include <queue>
#include <set>
#include <wmtk/Types.hpp>
#include <wmtk/components/simwild/expression_parser/Parser.hpp>
#include <wmtk/components/topological_offset/Circle.hpp>
#include <wmtk/components/topological_offset/SimplicialComplexBVH.hpp>
#include <wmtk/components/topological_offset/Sphere.hpp>
#include <wmtk/simplex/Simplex.hpp>

using namespace wmtk;
using namespace components::topological_offset;
using namespace components::simwild::expression_parser;


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
const std::set<std::string> T0_TAGS = {{"c"}};
const std::set<std::string> F0_TAGS = {{"c"}};
const std::set<std::string> F1_TAGS = {{"b"}};


TEST_CASE("edge_split_3d", "[split_op][3d]")
{
    Eigen::Matrix<double, Eigen::Dynamic, 3> V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::MatrixXi T(1, 4);
    T << 0, 1, 2, 3;

    // set tet tags
    MatrixSi Tags(1, 3);
    std::vector<std::string> tag_names = {"a", "b", "c"};
    for (int i = 0; i < tag_names.size(); i++) {
        bool exists = false;
        for (const std::string& tag : T0_TAGS) {
            if (tag == tag_names[i]) {
                exists = true;
            }
        }
        Tags.coeffRef(0, i) = exists ? 1 : 0;
    }

    Parameters param;
    // param.offset_selection = parse("tag_0 & tag_1");
    TopoOffsetTetMesh mesh(param, 0);
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;
    mesh.init_from_image(V, T, Tags, V_env_dummy, F_env_dummy, tag_names);

    // give every component unique tag combo
    mesh.m_vertex_extra[0].label = V0_LABEL;
    mesh.m_vertex_extra[1].label = V1_LABEL;
    mesh.m_vertex_extra[2].label = V2_LABEL;
    mesh.m_vertex_extra[3].label = V3_LABEL;
    mesh.m_edge_attribute[0].label = E0_LABEL;
    mesh.m_edge_attribute[1].label = E1_LABEL;
    mesh.m_edge_attribute[2].label = E2_LABEL;
    mesh.m_edge_attribute[3].label = E3_LABEL;
    mesh.m_edge_attribute[4].label = E4_LABEL;
    mesh.m_edge_attribute[5].label = E5_LABEL;
    mesh.m_face_extra[0].label = F0_LABEL;
    mesh.m_face_extra[1].label = F1_LABEL;
    mesh.m_face_extra[2].label = F2_LABEL;
    mesh.m_face_extra[3].label = F3_LABEL;
    mesh.m_tet_attribute[0].label = T0_LABEL;

    // split edge
    TetMesh::Tuple e = mesh.tuple_from_edge({{1, 2}});
    std::vector<TetMesh::Tuple> garbage;
    mesh.split_edge(e, garbage);

    // // ensure proper propagation of attributes
    // vertex
    REQUIRE(mesh.m_vertex_extra[0].label == V0_LABEL);
    REQUIRE(mesh.m_vertex_extra[1].label == V1_LABEL);
    REQUIRE(mesh.m_vertex_extra[2].label == V2_LABEL);
    REQUIRE(mesh.m_vertex_extra[3].label == V3_LABEL);
    REQUIRE(mesh.m_vertex_extra[4].label == E1_LABEL);

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
            mesh.m_face_extra[std::get<1>(mesh.tuple_from_face({{v0, v1, v2}}))].label;
        REQUIRE(correct_label == actual_label);
    }

    // tets
    for (int i = 0; i < 2; i++) {
        REQUIRE(mesh.m_tet_attribute[i].label == T0_LABEL);

        // convert tags to string for check
        std::set<std::string> tags;
        for (const size_t tag_int : mesh.m_tet_attribute[i].tag) {
            tags.insert(mesh.m_tag_id_to_name[tag_int]);
        }
        REQUIRE(tags == T0_TAGS);
    }
}


TEST_CASE("held_faces_and_one_envelope", "[3d][envelope]")
{
    // Two tets sharing a face, one tagged a and one tagged b: the shared face is the a/b
    // boundary, every outer face a domain-wall face. Every one of them is a tracked face; which
    // are HELD depends on deform_others and on the input complex, read live off the cells, and
    // one envelope holds them all.
    Eigen::Matrix<double, Eigen::Dynamic, 3> V(5, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1;
    Eigen::MatrixXi T(2, 4);
    T << 0, 1, 2, 3, 4, 1, 2, 3;

    MatrixSi Tags(2, 2);
    Tags.coeffRef(0, 0) = 1; // tet 0 -> a
    Tags.coeffRef(1, 1) = 1; // tet 1 -> b
    std::vector<std::string> tag_names = {"a", "b"};

    Parameters param;
    param.envelope_size = 1e-3; // tests skip Parameters::init(); give the build a real eps
    TopoOffsetTetMesh mesh(param, 0);
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;
    mesh.init_from_image(V, T, Tags, V_env_dummy, F_env_dummy, tag_names);

    const auto face = [&](size_t a, size_t b, size_t c) {
        return std::get<0>(mesh.tuple_from_face({{a, b, c}}));
    };
    for (const auto& f : mesh.get_faces()) {
        REQUIRE(mesh.m_face_attribute[f.fid(mesh)].m_is_surface_fs);
    }

    // deform_others false: every tracked face is held, the a/b boundary and the wall alike.
    param.deform_others = false;
    mesh.build_envelopes();
    REQUIRE(mesh.m_envelope != nullptr);
    REQUIRE(mesh.face_is_held(face(1, 2, 3)));
    REQUIRE(mesh.face_is_held(face(0, 1, 2)));
    REQUIRE(mesh.surface_envelope_for_face({{1, 2, 3}}) == mesh.m_envelope);
    for (size_t v = 0; v < 5; ++v) REQUIRE(mesh.vertex_is_held(v));
    // One envelope around all of them: a point on any held face is inside, a point off all of
    // them is outside.
    const Eigen::Vector3d on_shared = (V.row(1) + V.row(2) + V.row(3)).transpose() / 3.;
    REQUIRE(!mesh.m_envelope->is_outside(on_shared));
    REQUIRE(!mesh.m_envelope->is_outside(Eigen::Vector3d(V.row(0).transpose())));
    REQUIRE(mesh.m_envelope->is_outside(Eigen::Vector3d(0.2, 0.2, 0.2)));

    // deform_others true, no input complex: the a/b boundary stays a tracked face but nothing
    // holds it; the wall is held either way.
    param.deform_others = true;
    REQUIRE(!mesh.face_is_held(face(1, 2, 3)));
    REQUIRE(mesh.face_is_held(face(0, 1, 2)));
    mesh.build_envelopes();
    REQUIRE(mesh.surface_envelope_for_face({{1, 2, 3}}) == nullptr);
    REQUIRE(mesh.surface_envelope_for_face({{0, 1, 2}}) == mesh.m_envelope);
    REQUIRE(mesh.m_envelope->is_outside(on_shared)); // the shared face is not in the envelope

    // ... and with tet 0 the input complex, the shared face is its boundary, so held again.
    mesh.m_tet_attribute[0].label = 1;
    REQUIRE(mesh.face_is_complex_boundary(face(1, 2, 3)));
    REQUIRE(mesh.face_is_held(face(1, 2, 3)));
}


TEST_CASE("face_split_3d", "[split_op][3d]")
{
    Eigen::Matrix<double, Eigen::Dynamic, 3> V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::MatrixXi T(1, 4);
    T << 0, 1, 2, 3;

    // set tet tags
    MatrixSi Tags(1, 3);
    std::vector<std::string> tag_names = {"a", "b", "c"};
    for (int i = 0; i < tag_names.size(); i++) {
        bool exists = false;
        for (const std::string& tag : T0_TAGS) {
            if (tag == tag_names[i]) {
                exists = true;
            }
        }
        Tags.coeffRef(0, i) = exists ? 1 : 0;
    }

    Parameters param;
    // param.offset_selection = parse("tag_0 & tag_1");
    TopoOffsetTetMesh mesh(param, 0);
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;
    mesh.init_from_image(V, T, Tags, V_env_dummy, F_env_dummy, tag_names);

    // give every component unique tag combo
    mesh.m_vertex_extra[0].label = V0_LABEL;
    mesh.m_vertex_extra[1].label = V1_LABEL;
    mesh.m_vertex_extra[2].label = V2_LABEL;
    mesh.m_vertex_extra[3].label = V3_LABEL;
    mesh.m_edge_attribute[0].label = E0_LABEL;
    mesh.m_edge_attribute[1].label = E1_LABEL;
    mesh.m_edge_attribute[2].label = E2_LABEL;
    mesh.m_edge_attribute[3].label = E3_LABEL;
    mesh.m_edge_attribute[4].label = E4_LABEL;
    mesh.m_edge_attribute[5].label = E5_LABEL;
    mesh.m_face_extra[0].label = F0_LABEL;
    mesh.m_face_extra[1].label = F1_LABEL;
    mesh.m_face_extra[2].label = F2_LABEL;
    mesh.m_face_extra[3].label = F3_LABEL;
    mesh.m_tet_attribute[0].label = T0_LABEL;

    // split face
    auto [ftup, _] = mesh.tuple_from_face({{1, 2, 3}});
    std::vector<TetMesh::Tuple> garbage;
    mesh.split_face(ftup, garbage);

    // // ensure proper propagation of attributes
    // vertex
    REQUIRE(mesh.m_vertex_extra[0].label == V0_LABEL);
    REQUIRE(mesh.m_vertex_extra[1].label == V1_LABEL);
    REQUIRE(mesh.m_vertex_extra[2].label == V2_LABEL);
    REQUIRE(mesh.m_vertex_extra[3].label == V3_LABEL);
    REQUIRE(mesh.m_vertex_extra[4].label == F3_LABEL);

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
            mesh.m_face_extra[std::get<1>(mesh.tuple_from_face({{v0, v1, v2}}))].label;
        REQUIRE(correct_label == actual_label);
    }

    // tets
    for (int i = 0; i < 3; i++) {
        REQUIRE(mesh.m_tet_attribute[i].label == T0_LABEL);

        // convert tags to string for check
        std::set<std::string> tags;
        for (const size_t tag_int : mesh.m_tet_attribute[i].tag) {
            tags.insert(mesh.m_tag_id_to_name[tag_int]);
        }
        REQUIRE(tags == T0_TAGS);
    }
}


TEST_CASE("tet_split_3d", "[split_op][3d]")
{
    Eigen::Matrix<double, Eigen::Dynamic, 3> V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::MatrixXi T(1, 4);
    T << 0, 1, 2, 3;

    // set tet tags
    MatrixSi Tags(1, 3);
    std::vector<std::string> tag_names = {"a", "b", "c"};
    for (int i = 0; i < tag_names.size(); i++) {
        bool exists = false;
        for (const std::string& tag : T0_TAGS) {
            if (tag == tag_names[i]) {
                exists = true;
            }
        }
        Tags.coeffRef(0, i) = exists ? 1 : 0;
    }

    Parameters param;
    // param.offset_selection = parse("tag_0 & tag_1");
    TopoOffsetTetMesh mesh(param, 0);
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;
    mesh.init_from_image(V, T, Tags, V_env_dummy, F_env_dummy, tag_names);

    // give every component unique tag combo
    mesh.m_vertex_extra[0].label = V0_LABEL;
    mesh.m_vertex_extra[1].label = V1_LABEL;
    mesh.m_vertex_extra[2].label = V2_LABEL;
    mesh.m_vertex_extra[3].label = V3_LABEL;
    mesh.m_edge_attribute[0].label = E0_LABEL;
    mesh.m_edge_attribute[1].label = E1_LABEL;
    mesh.m_edge_attribute[2].label = E2_LABEL;
    mesh.m_edge_attribute[3].label = E3_LABEL;
    mesh.m_edge_attribute[4].label = E4_LABEL;
    mesh.m_edge_attribute[5].label = E5_LABEL;
    mesh.m_face_extra[0].label = F0_LABEL;
    mesh.m_face_extra[1].label = F1_LABEL;
    mesh.m_face_extra[2].label = F2_LABEL;
    mesh.m_face_extra[3].label = F3_LABEL;
    mesh.m_tet_attribute[0].label = T0_LABEL;

    // split tet
    TetMesh::Tuple ttup = mesh.tuple_from_tet(0);
    std::vector<TetMesh::Tuple> garbage;
    mesh.split_tet(ttup, garbage);

    // // ensure proper propagation of attributes
    // vertex
    REQUIRE(mesh.m_vertex_extra[0].label == V0_LABEL);
    REQUIRE(mesh.m_vertex_extra[1].label == V1_LABEL);
    REQUIRE(mesh.m_vertex_extra[2].label == V2_LABEL);
    REQUIRE(mesh.m_vertex_extra[3].label == V3_LABEL);
    REQUIRE(mesh.m_vertex_extra[4].label == T0_LABEL);

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
            mesh.m_face_extra[std::get<1>(mesh.tuple_from_face({{v0, v1, v2}}))].label;
        REQUIRE(correct_label == actual_label);
    }

    // tets
    for (int i = 0; i < 4; i++) {
        REQUIRE(mesh.m_tet_attribute[i].label == T0_LABEL);

        // convert tags to string for check
        std::set<std::string> tags;
        for (const size_t tag_int : mesh.m_tet_attribute[i].tag) {
            tags.insert(mesh.m_tag_id_to_name[tag_int]);
        }
        REQUIRE(tags == T0_TAGS);
    }
}


TEST_CASE("invariant_3d", "[3d]")
{
    Eigen::Matrix<double, Eigen::Dynamic, 3> V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    MatrixSi Tags(1, 1);
    std::vector<std::string> tag_names = {"a"};
    Tags.coeffRef(0, 0) = 0;
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;

    { // mesh 1 (bad)
        Eigen::MatrixXi T(1, 4);
        T << 0, 1, 3, 2;

        Parameters param;
        // param.offset_selection = parse("tag_0 & tag_1");
        TopoOffsetTetMesh mesh(param, 0);
        mesh.init_from_image(V, T, Tags, V_env_dummy, F_env_dummy, tag_names);

        std::vector<TetMesh::Tuple> tets;
        tets.push_back(mesh.tuple_from_tet(0));
        REQUIRE((mesh.invariants(tets) == false));
    }
    { // mesh 2 (good)
        Eigen::MatrixXi T(1, 4);
        T << 0, 1, 2, 3;

        Parameters param;
        // param.offset_selection = parse("tag_0 & tag_1");
        TopoOffsetTetMesh mesh(param, 0);
        mesh.init_from_image(V, T, Tags, V_env_dummy, F_env_dummy, tag_names);

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
    MatrixSi Tags(1, 3);
    std::vector<std::string> tag_names = {"a", "b", "c"};
    Tags.coeffRef(0, 0) = 0;
    Tags.coeffRef(0, 1) = 0;
    Tags.coeffRef(0, 2) = 1;

    Parameters param;
    // param.offset_selection = parse("tag_0 & tag_1");
    TopoOffsetTriMesh mesh(param, 0);
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;
    mesh.init_from_image(V, F, Tags, V_env_dummy, F_env_dummy, tag_names);

    // give every component unique tag combo
    mesh.m_vertex_extra[0].label = V0_LABEL;
    mesh.m_vertex_extra[1].label = V1_LABEL;
    mesh.m_vertex_extra[2].label = V2_LABEL;
    mesh.m_edge_extra[0].label = E0_LABEL;
    mesh.m_edge_extra[1].label = E1_LABEL;
    mesh.m_edge_extra[2].label = E2_LABEL;
    mesh.m_face_extra[0].label = F0_LABEL;

    // split edge
    TriMesh::Tuple e = mesh.tuple_from_edge(1, 2, 0);
    std::vector<TriMesh::Tuple> garbage;
    mesh.split_edge(e, garbage);

    // // ensure proper propagation of attributes
    // vertex
    REQUIRE(mesh.m_vertex_extra[0].label == V0_LABEL);
    REQUIRE(mesh.m_vertex_extra[1].label == V1_LABEL);
    REQUIRE(mesh.m_vertex_extra[2].label == V2_LABEL);
    REQUIRE(mesh.m_vertex_extra[3].label == E0_LABEL);

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
            mesh.m_edge_extra[mesh.tuple_from_edge(v0, v1, ftup.fid(mesh)).eid(mesh)].label;
        REQUIRE(actual_label == correct_label);
    }

    // faces
    for (int i = 0; i < 2; i++) {
        REQUIRE(mesh.m_face_extra[i].label == F0_LABEL);

        // convert tags to string for check
        std::set<std::string> tags;
        for (const size_t tag_int : mesh.m_face_attribute[i].tags) {
            tags.insert(mesh.m_tag_id_to_name[tag_int]);
        }
        REQUIRE(tags == F0_TAGS);
    }
}


TEST_CASE("edge_split_2d_2faces", "[split_op][2d]")
{
    Eigen::Matrix<double, 4, 2> V(4, 2);
    V << 0, 0, 1, 0, 0, 1, 1, 1;
    Eigen::MatrixXi F(2, 3);
    F << 0, 1, 2, 1, 3, 2;
    MatrixSi Tags(2, 3);
    std::vector<std::string> tag_names = {"a", "b", "c"};
    Tags.coeffRef(0, 0) = 0;
    Tags.coeffRef(0, 1) = 0;
    Tags.coeffRef(0, 2) = 1;
    Tags.coeffRef(1, 0) = 0;
    Tags.coeffRef(1, 1) = 1;
    Tags.coeffRef(1, 2) = 0;

    Parameters param;
    // param.offset_selection = parse("tag_0 & tag_1");
    TopoOffsetTriMesh mesh(param, 0);
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;
    mesh.init_from_image(V, F, Tags, V_env_dummy, F_env_dummy, tag_names);

    // give every component unique tag combo
    mesh.m_vertex_extra[0].label = V0_LABEL;
    mesh.m_vertex_extra[1].label = V1_LABEL;
    mesh.m_vertex_extra[2].label = V2_LABEL;
    mesh.m_vertex_extra[3].label = V3_LABEL;
    mesh.m_edge_extra[0].label = E0_LABEL;
    mesh.m_edge_extra[1].label = E1_LABEL;
    mesh.m_edge_extra[2].label = E2_LABEL;
    mesh.m_edge_extra[3].label = E3_LABEL;
    mesh.m_edge_extra[5].label = E5_LABEL;
    mesh.m_face_extra[0].label = F0_LABEL;
    mesh.m_face_extra[1].label = F1_LABEL;

    // split edge
    TriMesh::Tuple e = mesh.tuple_from_edge(1, 2, 0);
    std::vector<TriMesh::Tuple> garbage;
    mesh.split_edge(e, garbage);

    // // ensure proper propagation of attributes
    // vertex
    REQUIRE(mesh.m_vertex_extra[0].label == V0_LABEL);
    REQUIRE(mesh.m_vertex_extra[1].label == V1_LABEL);
    REQUIRE(mesh.m_vertex_extra[2].label == V2_LABEL);
    REQUIRE(mesh.m_vertex_extra[3].label == V3_LABEL);
    REQUIRE(mesh.m_vertex_extra[4].label == E0_LABEL);

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
        int actual_label = mesh.m_edge_extra[e_id].label;
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
            REQUIRE(mesh.m_face_extra[f_id].label == F0_LABEL);

            // convert tags to string for check
            std::set<std::string> tags;
            for (const size_t tag_int : mesh.m_face_attribute[f_id].tags) {
                tags.insert(mesh.m_tag_id_to_name[tag_int]);
            }
            REQUIRE(tags == F0_TAGS);
        } else {
            REQUIRE(mesh.m_face_extra[f_id].label == F1_LABEL);

            // convert tags to string for check
            std::set<std::string> tags;
            for (const size_t tag_int : mesh.m_face_attribute[f_id].tags) {
                tags.insert(mesh.m_tag_id_to_name[tag_int]);
            }
            REQUIRE(tags == F1_TAGS);
        }
    }
}


TEST_CASE("face_split_2d", "[split_op][2d]")
{
    Eigen::Matrix<double, 3, 2> V(3, 2);
    V << 0, 0, 1, 0, 0, 1;
    Eigen::MatrixXi F(1, 3);
    F << 0, 1, 2;
    MatrixSi Tags(1, 3);
    std::vector<std::string> tag_names = {"a", "b", "c"};
    Tags.coeffRef(0, 0) = 0;
    Tags.coeffRef(0, 1) = 0;
    Tags.coeffRef(0, 2) = 1;

    Parameters param;
    // param.offset_selection = parse("tag_0 & tag_1");
    TopoOffsetTriMesh mesh(param, 0);
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;
    mesh.init_from_image(V, F, Tags, V_env_dummy, F_env_dummy, tag_names);

    // give every component unique tag combo
    mesh.m_vertex_extra[0].label = V0_LABEL;
    mesh.m_vertex_extra[1].label = V1_LABEL;
    mesh.m_vertex_extra[2].label = V2_LABEL;
    mesh.m_edge_extra[0].label = E0_LABEL;
    mesh.m_edge_extra[1].label = E1_LABEL;
    mesh.m_edge_extra[2].label = E2_LABEL;
    mesh.m_face_extra[0].label = F0_LABEL;

    // split face
    TriMesh::Tuple f = mesh.tuple_from_simplex(simplex::Face(0, 1, 2));
    std::vector<TriMesh::Tuple> garbage;
    mesh.split_face(f, garbage);

    // // ensure proper propagation of attributes
    // vertex
    REQUIRE(mesh.m_vertex_extra[0].label == V0_LABEL);
    REQUIRE(mesh.m_vertex_extra[1].label == V1_LABEL);
    REQUIRE(mesh.m_vertex_extra[2].label == V2_LABEL);
    REQUIRE(mesh.m_vertex_extra[3].label == F0_LABEL);

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
        int actual_label = mesh.m_edge_extra[e_id].label;
        REQUIRE(actual_label == correct_label);
    }

    // faces
    std::array<std::array<size_t, 3>, 4> faces = {{{{0, 1, 3}}, {{0, 3, 2}}, {{1, 2, 3}}}};
    for (int i = 0; i < 3; i++) {
        TriMesh::Tuple ftup =
            mesh.tuple_from_simplex(simplex::Face(faces[i][0], faces[i][1], faces[i][2]));
        size_t f_id = ftup.fid(mesh);
        REQUIRE(mesh.m_face_extra[f_id].label == F0_LABEL);

        // convert tags to string for check
        std::set<std::string> tags;
        for (const size_t tag_int : mesh.m_face_attribute[i].tags) {
            tags.insert(mesh.m_tag_id_to_name[tag_int]);
        }
        REQUIRE(tags == F0_TAGS);
    }
}


TEST_CASE("invariant_2d", "[2d]")
{
    Eigen::Matrix<double, 3, 2> V(3, 2);
    V << 0, 0, 1, 0, 0, 1;
    MatrixSi Tags(1, 1);
    std::vector<std::string> tag_names = {"a"};
    Tags.coeffRef(0, 0) = 0;
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;

    { // mesh 1 (bad)
        Eigen::MatrixXi F(1, 3);
        F << 0, 2, 1;

        Parameters param;
        // param.offset_selection = parse("tag_0 & tag_1");
        TopoOffsetTriMesh mesh(param, 0);
        mesh.init_from_image(V, F, Tags, V_env_dummy, F_env_dummy, tag_names);

        std::vector<TriMesh::Tuple> tris;
        tris.push_back(mesh.tuple_from_tri(0));
        REQUIRE((mesh.invariants(tris) == false));
    }
    { // mesh 2 (good)
        Eigen::MatrixXi F(1, 3);
        F << 0, 1, 2;

        Parameters param;
        // param.offset_selection = parse("tag_0 & tag_1");
        TopoOffsetTriMesh mesh(param, 0);
        mesh.init_from_image(V, F, Tags, V_env_dummy, F_env_dummy, tag_names);

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
    MatrixSi Tags(1, 1); // dont care
    std::vector<std::string> tag_names = {"a"};
    Tags.coeffRef(0, 0) = 0;
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;

    Parameters param;
    // param.offset_selection = parse("tag_0 & tag_1");
    TopoOffsetTriMesh mesh(param, 0);
    mesh.init_from_image(V, F, Tags, V_env_dummy, F_env_dummy, tag_names);

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
    MatrixSi Tags(1, 1); // dont care
    std::vector<std::string> tag_names = {"a"};
    Tags.coeffRef(0, 0) = 0;
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;

    Parameters param;
    // param.offset_selection = parse("tag_0 & tag_1");
    TopoOffsetTriMesh mesh(param, 0);
    mesh.init_from_image(V, F, Tags, V_env_dummy, F_env_dummy, tag_names);

    Circle circ(mesh, 0);
    REQUIRE(fabs(circ.radius() - (sqrt(2.0) / 2.0)) < pow(10, -6));
}


TEST_CASE("dist_to_trimesh", "[dist_growth][2d]")
{
    Eigen::MatrixXd V(3, 2);
    V << 0, 0, 0, 1, 1, 0;
    Eigen::MatrixXi F(1, 3);
    F << 0, 1, 2;
    MatrixSi Tags(1, 1); // dont care
    std::vector<std::string> tag_names = {"a"};
    Tags.coeffRef(0, 0) = 0;
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;

    Parameters param;
    // param.offset_selection = parse("tag_0 & tag_1");
    TopoOffsetTriMesh mesh(param, 0);
    mesh.init_from_image(V, F, Tags, V_env_dummy, F_env_dummy, tag_names);

    // label one vert as input
    mesh.m_vertex_extra[0].label = 1;
    mesh.init_input_complex_bvh();
    Vector2d q(1.0, 1.0);
    double dist = mesh.m_input_complex_bvh->dist(q);
    REQUIRE(fabs(dist - sqrt(2.0)) < pow(10, -6));

    Vector2d q0(0.0, 0.0);
    dist = mesh.m_input_complex_bvh->dist(q0);
    REQUIRE(fabs(dist) < pow(10, -6));

    // label edges and vertices as input
    for (int i = 0; i < 3; i++) {
        mesh.m_edge_extra[i].label = 1;
        mesh.m_vertex_extra[i].label = 1;
    }
    mesh.init_input_complex_bvh();

    Vector2d q1(1.0, 1.0);
    dist = mesh.m_input_complex_bvh->dist(q1);
    REQUIRE(fabs(dist - (sqrt(2) / 2.0)) < pow(10, -6));

    Vector2d q2(0.0, 0.0);
    dist = mesh.m_input_complex_bvh->dist(q2);
    REQUIRE(fabs(dist) < pow(10, -6));

    Vector2d q3(10.0, 0.0);
    dist = mesh.m_input_complex_bvh->dist(q3);
    REQUIRE(fabs(dist - 9.0) < pow(10, -6));
}


TEST_CASE("dist_to_tetmesh", "[dist_growth][3d]")
{
    Eigen::MatrixXd V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::MatrixXi T(1, 4);
    T << 0, 1, 2, 3;
    MatrixSi Tags(1, 1); // dont care
    std::vector<std::string> tag_names = {"a"};
    Tags.coeffRef(0, 0) = 0;
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;

    Parameters param;
    // param.offset_selection = parse("tag_0 & tag_1");
    TopoOffsetTetMesh mesh(param, 0);
    mesh.init_from_image(V, T, Tags, V_env_dummy, F_env_dummy, tag_names);

    // label one vert as input
    mesh.m_vertex_extra[0].label = 1;
    mesh.init_input_complex_bvh();
    Vector3d q(1.0, 1.0, 1.0);
    double dist = mesh.m_input_complex_bvh->dist(q);
    REQUIRE(fabs(dist - sqrt(3.0)) < pow(10, -6));

    Vector3d q0(0.0, 0.0, 0.0);
    dist = mesh.m_input_complex_bvh->dist(q0);
    REQUIRE(fabs(dist) < pow(10, -6));

    // label faces, edges, and vertices as input
    for (int i = 0; i < 4; i++) {
        mesh.m_face_extra[i].label = 1;
    }
    for (int i = 0; i < 6; i++) {
        mesh.m_edge_attribute[i].label = 1;
    }
    for (int i = 0; i < 4; i++) {
        mesh.m_vertex_extra[i].label = 1;
    }
    mesh.init_input_complex_bvh();

    Vector3d q1(1.0, 1.0, 1.0);
    dist = mesh.m_input_complex_bvh->dist(q1);
    REQUIRE(fabs(dist - (2.0 / sqrt(3.0))) < pow(10, -6));

    Vector3d q2(0.0, 0.0, 0.0);
    dist = mesh.m_input_complex_bvh->dist(q2);
    REQUIRE(fabs(dist) < pow(10, -6));

    Vector3d q3(10.0, 2.0, 1.0);
    dist = mesh.m_input_complex_bvh->dist(q3);
    REQUIRE(fabs(dist - sqrt(86.0)) < pow(10, -6));

    Vector3d q4(0.1, 0.2, 0.3);
    dist = mesh.m_input_complex_bvh->dist(q4);
    REQUIRE(fabs(dist - 0.1) < pow(10, -6));

    // label tet as input
    mesh.m_tet_attribute[0].label = 1;
    mesh.init_input_complex_bvh();

    dist = mesh.m_input_complex_bvh->dist(q4);
    REQUIRE(fabs(dist) < pow(10, -6));

    dist = mesh.m_input_complex_bvh->dist(q3);
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
    MatrixSi Tags(1, 1); // dont care
    std::vector<std::string> tag_names = {"a"};
    Tags.coeffRef(0, 0) = 0;
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;

    Parameters param;
    // param.offset_selection = parse("tag_0 & tag_1");
    TopoOffsetTetMesh mesh(param, 0);
    mesh.init_from_image(V, T, Tags, V_env_dummy, F_env_dummy, tag_names);

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

TEST_CASE("stencil-order-point-counts", "[offset]")
{
    // The stencil the whole 3D criterion and energy sample, checked against the counts the
    // design states: 3 at order 0 (the corners alone), then the vertices of the triangle
    // subdivided k-1 times plus one centroid per sub-triangle -- 4, 10, 31, 109, 409.
    //
    // This also pins stencil_points_per_face() to for_each_face_sample(), which are two separate
    // pieces of arithmetic that have to agree: the accessor is used in the logs and in the
    // criterion's own reporting, the loop is what actually visits the points.
    Parameters param;
    TopoOffsetTetMesh mesh(param, 0);

    const Vector3d p0(0., 0., 0.), p1(1., 0., 0.), p2(0., 1., 0.);
    const std::array<int, 6> expected = {{3, 4, 10, 31, 109, 409}};

    for (int k = 0; k < int(expected.size()); ++k) {
        mesh.m_offset_params.stencil_order = k;

        std::vector<std::array<double, 3>> w;
        mesh.for_each_face_sample(
            p0,
            p1,
            p2,
            [&](const Vector3d& q, const double wa, const double wb, const double wc) {
                // The visited point must be the barycentric combination it reports.
                const Vector3d want = wa * p0 + wb * p1 + wc * p2;
                CHECK((q - want).norm() <= 1e-15);
                w.push_back({{wa, wb, wc}});
            });

        INFO("stencil_order " << k);
        CHECK(int(w.size()) == expected[size_t(k)]);
        CHECK(mesh.stencil_points_per_face() == expected[size_t(k)]);

        for (const auto& b : w) {
            CHECK(std::abs(b[0] + b[1] + b[2] - 1.) <= 1e-14);
            CHECK(b[0] >= -1e-15);
            CHECK(b[1] >= -1e-15);
            CHECK(b[2] >= -1e-15);
        }

        // No duplicated sample: a repeated point would silently weight part of the face twice.
        // Quantised into a set rather than compared pairwise, so the check stays one assertion
        // instead of ~83000 at order 5.
        std::set<std::array<long long, 3>> seen;
        for (const auto& b : w) {
            seen.insert({{llround(b[0] * 1e9), llround(b[1] * 1e9), llround(b[2] * 1e9)}});
        }
        CHECK(seen.size() == w.size());
    }

    // Order 0 is exactly the three corners, which is what lets one stencil carry the vertex
    // placement test as well as the face resolution test.
    mesh.m_offset_params.stencil_order = 0;
    std::vector<Vector3d> pts;
    mesh.for_each_face_sample(p0, p1, p2, [&](const Vector3d& q, double, double, double) {
        pts.push_back(q);
    });
    REQUIRE(pts.size() == 3);
    CHECK((pts[0] - p0).norm() <= 1e-15);
    CHECK((pts[1] - p1).norm() <= 1e-15);
    CHECK((pts[2] - p2).norm() <= 1e-15);
}

// ---------------------------------------------------------------------------------------------
// The per-tet energy (TopoOffsetTetMesh::tet_energy()) and the front smoother's offset term.
//
// Fixtures share one field: the euclidean distance to a large triangle in the plane z = 0, at
// level delta = 0.5, so relative_residual(q) = (z - 0.5) / 0.5 anywhere above the triangle's
// interior. front_conv = 0.01 makes front_conv_frac() = 0.02: a face 0.1 off its level set reads
// r / frac = 10, O = 100 -- far from the AMIPS^3 >= 27 beside it, so a missing or doubled face
// term cannot hide in the tolerance.
// ---------------------------------------------------------------------------------------------

namespace {

constexpr double kEnergyDelta = 0.5;
constexpr double kEnergyConv = 0.01;
constexpr double kEnergyFrac = kEnergyConv / kEnergyDelta;

std::shared_ptr<EuclideanOffsetPotential3D> plane_field()
{
    auto env = std::make_shared<SampleEnvelope>();
    env->use_exact = true;
    const std::vector<Eigen::Vector3d> verts = {{-10., -10., 0.}, {10., -10., 0.}, {0., 10., 0.}};
    const std::vector<Eigen::Vector3i> tris = {{0, 1, 2}};
    env->init(verts, tris, kEnergyDelta);
    return std::make_shared<EuclideanOffsetPotential3D>(env, kEnergyDelta);
}

/// The face term O(f) the energy definition states, computed here from the stencil
/// for_each_face_sample() visits and nothing else: mean over the points of
/// (relative_residual(q) / front_conv_frac())^2.
double face_term_by_hand(
    const TopoOffsetTetMesh& m,
    const OffsetPotential3D& pot,
    const Vector3d& p0,
    const Vector3d& p1,
    const Vector3d& p2)
{
    double s = 0.;
    int n = 0;
    m.for_each_face_sample(p0, p1, p2, [&](const Vector3d&, double wa, double wb, double wc) {
        const Vector3d q = wa * p0 + wb * p1 + wc * p2;
        const double r = pot.relative_residual(q) / kEnergyFrac;
        s += r * r;
        ++n;
    });
    return s / double(n);
}

/// Positive orientation, the invariant_3d test's good tet: (p1-p0).((p2-p0)x(p3-p0)) > 0.
std::array<int, 4> positive(const Eigen::MatrixXd& V, std::array<int, 4> t)
{
    const auto p = [&](int i) { return Vector3d(V.row(t[size_t(i)])); };
    if ((p(1) - p(0)).dot((p(2) - p(0)).cross(p(3) - p(0))) < 0.) std::swap(t[2], t[3]);
    return t;
}

/// A mesh of the given cells with the given construction labels on the shared field. Tet i is
/// row i of T, which the caller relies on to label and look up cells.
std::unique_ptr<TopoOffsetTetMesh> energy_mesh(
    Parameters& param,
    const Eigen::MatrixXd& V,
    const std::vector<std::array<int, 4>>& T,
    const std::vector<int>& labels,
    const std::shared_ptr<EuclideanOffsetPotential3D>& pot)
{
    param.target_distance = kEnergyDelta;
    param.front_conv = kEnergyConv;
    param.stencil_order = 1;
    auto mesh = std::make_unique<TopoOffsetTetMesh>(param, 0);
    Eigen::MatrixXi Tm(T.size(), 4);
    for (size_t i = 0; i < T.size(); ++i) {
        const auto t = positive(V, T[i]);
        for (int j = 0; j < 4; ++j) Tm(int(i), j) = t[size_t(j)];
    }
    MatrixSi Tags(int(T.size()), 1);
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;
    mesh->init_from_image(V, Tm, Tags, V_env_dummy, F_env_dummy, {"a"});
    // A piece cut out of a domain: its outer faces stand for interior faces, not the domain wall,
    // which a run holds to the envelope (face_is_held()). Untracked, so no vertex is held.
    for (const auto& f : mesh->get_faces()) {
        mesh->m_face_attribute[f.fid(*mesh)].m_is_surface_fs = false;
    }
    for (size_t v = 0; v < size_t(V.rows()); ++v) {
        mesh->m_vertex_attribute[v].m_is_on_surface = false;
    }
    for (size_t i = 0; i < T.size(); ++i) {
        std::array<size_t, 4> want{
            {size_t(Tm(int(i), 0)),
             size_t(Tm(int(i), 1)),
             size_t(Tm(int(i), 2)),
             size_t(Tm(int(i), 3))}};
        auto got = mesh->oriented_tet_vids(i);
        std::sort(want.begin(), want.end());
        std::sort(got.begin(), got.end());
        REQUIRE(got == want);
        REQUIRE(!mesh->is_inverted(mesh->tuple_from_tet(i)));
        mesh->m_tet_attribute[i].label = labels[i];
    }
    mesh->m_offset_potential = pot;
    return mesh;
}

} // namespace

TEST_CASE("stencil-energy-3d-is-the-sum-of-face-terms", "[offset][3d]")
{
    // The front smoother's offset term at offset_term_weight() is, face by face, the per-tet
    // energy's O(f): so what the smoother minimises is what the rules compare. Checked against the
    // definition computed by hand from the same stencil, and against face_offset_term(), at
    // stencil orders 0 (corners only), 1 and 2, with three faces around the moving vertex placed
    // above and below the level set.
    Parameters param;
    param.target_distance = kEnergyDelta;
    param.front_conv = kEnergyConv;
    TopoOffsetTetMesh mesh(param, 0);
    const auto pot = plane_field();
    REQUIRE(std::abs(mesh.m_offset_params.front_conv_frac() - kEnergyFrac) <= 1e-15);
    REQUIRE(mesh.offset_term_weight() == Catch::Approx(1. / (kEnergyFrac * kEnergyFrac)));

    const Vector3d x(0.1, -0.05, 0.53);
    const std::vector<std::pair<Vector3d, Vector3d>> others = {
        {{0.4, 0.1, 0.61}, {0.2, 0.35, 0.44}},
        {{0.2, 0.35, 0.44}, {-0.25, 0.2, 0.58}},
        {{-0.25, 0.2, 0.58}, {-0.1, -0.4, 0.37}}};
    for (const int k : {0, 1, 2}) {
        mesh.m_offset_params.stencil_order = k;
        std::vector<StencilEnergy3D::Face> faces;
        double by_hand = 0., by_term = 0.;
        for (const auto& [q1, q2] : others) {
            StencilEnergy3D::Face f;
            f.q1 = q1;
            f.q2 = q2;
            mesh.for_each_face_sample(
                x,
                q1,
                q2,
                [&](const Vector3d&, double a, double b, double c) {
                    f.samples.push_back({a, b, c});
                });
            faces.push_back(f);
            by_hand += face_term_by_hand(mesh, *pot, x, q1, q2);
            by_term += mesh.face_offset_term(*pot, x, q1, q2);
        }
        StencilEnergy3D energy(pot, faces, mesh.offset_term_weight());
        Eigen::VectorXd xv = x;
        INFO("stencil_order " << k);
        CHECK(by_hand > 10.); // the fixture really is off the level set
        CHECK(energy.value(xv) == Catch::Approx(by_hand).epsilon(1e-12));
        CHECK(by_term == Catch::Approx(by_hand).epsilon(1e-12));
    }
}

TEST_CASE("per-tet-energy", "[offset][3d]")
{
    // Four cells around the edge (a, b): 0 and 1 band, 2 background, 3 input complex. The band's
    // front faces are the ones with background or nothing across: cell 0's two faces on the
    // domain boundary (cell 3 across (a,b,c0) is the complex, cell 1 across (a,b,c1) is band),
    // and cell 1's two boundary faces plus (a,b,c2) against the background. Every cell off the
    // band is its weighted AMIPS^3 alone. Checked at w = 1 and at the default w.
    Eigen::MatrixXd V(6, 3);
    V << 0., 0., 0.12, // a
        0., 0., 0.88, // b
        0.4, 0., 0.5, // c0
        0., 0.4, 0.55, // c1
        -0.4, 0., 0.45, // c2
        0., -0.4, 0.5; // c3
    const int a = 0, b = 1, c0 = 2, c1 = 3, c2 = 4, c3 = 5;
    for (const double w : {1., Parameters().offset_amips_weight}) {
        Parameters param;
        param.offset_amips_weight = w;
        const auto pot = plane_field();
        auto mesh = energy_mesh(
            param,
            V,
            {{{a, b, c0, c1}}, {{a, b, c1, c2}}, {{a, b, c2, c3}}, {{a, b, c3, c0}}},
            {2, 2, 0, 1},
            pot);
        const auto P = [&](int i) { return Vector3d(V.row(i)); };
        const auto O = [&](int i, int j, int k) {
            return face_term_by_hand(*mesh, *pot, P(i), P(j), P(k));
        };
        const std::array<double, 4> extra = {
            {O(a, c0, c1) + O(b, c0, c1), O(a, c1, c2) + O(b, c1, c2) + O(a, b, c2), 0., 0.}};
        for (size_t tid = 0; tid < 4; ++tid) {
            // The engine's cell quality, AMIPS^3 from the positions.
            const double base = mesh->TetOptimizerMesh::get_quality(mesh->oriented_tet_vids(tid));
            REQUIRE(base < TetOptimizerMesh::MAX_ENERGY);
            INFO("w " << w << ", tet " << tid);
            CHECK(mesh->tet_energy(tid) == Catch::Approx(w * base + extra[tid]).epsilon(1e-12));
            if (tid < 2) CHECK(extra[tid] > 1.); // the front faces really are off the level set
        }
    }
}

TEST_CASE("smoothing-objective-is-the-ring-energy", "[offset][3d]")
{
    // One energy for the smoother and the rules: at every vertex, the objective the shared
    // smoother minimises (smoothing_extra_energy(); every caller passes w_amips 0, so the smoother
    // adds no AMIPS term of its own) is the sum of tet_energy() over the vertex's ring, up to the
    // terms that do not move with the vertex -- the ring's front faces that do not have it as a
    // corner. Checked on the per-tet-energy fixture by moving each vertex (front vertices a, b,
    // c0, c1, c2; c3 is on no front face) to three nearby positions: objective minus ring sum must
    // not change, and the shared smoother's own per-cell comparison (smoothing_cell_energy()) is
    // tet_energy(). The w AMIPS^3 part's gradient and Hessian against central differences too. At
    // w = 1 and at the default w.
    Eigen::MatrixXd V(6, 3);
    V << 0., 0., 0.12, // a
        0., 0., 0.88, // b
        0.4, 0., 0.5, // c0
        0., 0.4, 0.55, // c1
        -0.4, 0., 0.45, // c2
        0., -0.4, 0.5; // c3
    const int a = 0, b = 1, c0 = 2, c1 = 3, c2 = 4, c3 = 5;
    for (const double w : {1., Parameters().offset_amips_weight}) {
        Parameters param;
        param.offset_amips_weight = w;
        const auto pot = plane_field();
        auto mesh = energy_mesh(
            param,
            V,
            {{{a, b, c0, c1}}, {{a, b, c1, c2}}, {{a, b, c2, c3}}, {{a, b, c3, c0}}},
            {2, 2, 0, 1},
            pot);
        // What label_offset_boundary() sets in a run: a corner of a live front face.
        for (size_t v = 0; v < 6; ++v) {
            mesh->m_vertex_extra[v].m_is_on_offset = !mesh->offset_surface_faces_live_at(v).empty();
        }
        REQUIRE(!mesh->m_vertex_extra[size_t(c3)].m_is_on_offset);
        for (size_t v = 0; v < 6; ++v) {
            const std::vector<size_t> ring = mesh->get_one_ring_tids_for_vertex(v);
            const auto ring_energy = [&]() {
                double s = 0.;
                for (const size_t tid : ring) s += mesh->tet_energy(tid);
                return s;
            };
            const bool front = mesh->vertex_carries_offset_term(v);
            INFO("w " << w << ", vertex " << v << (front ? " (front)" : ""));
            REQUIRE(front == (v != size_t(c3)));
            const auto energy = mesh->smoothing_extra_energy(v);
            const Vector3d x0 = mesh->m_vertex_attribute[v].m_posf;
            Eigen::VectorXd xv = x0;
            const double d0 = energy->value(xv) - ring_energy();
            for (const Vector3d& dx :
                 {Vector3d(0.01, 0., 0.),
                  Vector3d(0., -0.008, 0.005),
                  Vector3d(-0.004, 0.006, -0.007)}) {
                mesh->set_vertex_position(v, x0 + dx);
                for (const size_t tid : ring)
                    REQUIRE(!mesh->is_inverted(mesh->tuple_from_tet(tid)));
                xv = x0 + dx;
                const double e = energy->value(xv);
                CHECK(
                    e - ring_energy() ==
                    Catch::Approx(d0).margin(1e-9 * std::max(1., std::abs(e))));
                // The shared smoother's own per-cell comparison is the same energy.
                for (const size_t tid : ring) {
                    const double q = mesh->get_quality(mesh->tuple_from_tet(tid));
                    CHECK(mesh->smoothing_cell_energy(tid, q) == mesh->tet_energy(tid));
                }
            }
            mesh->set_vertex_position(v, x0);

            // The w AMIPS^3 part's derivatives against central differences.
            const auto amips3 = mesh->amips3_energy(v);
            const double h = 1e-6;
            Eigen::VectorXd g(3);
            Eigen::MatrixXd H(3, 3);
            xv = x0;
            amips3->gradient(xv, g);
            amips3->hessian(xv, H);
            for (int i = 0; i < 3; ++i) {
                Eigen::VectorXd xp = xv, xm = xv;
                xp[i] += h;
                xm[i] -= h;
                const double fd = (amips3->value(xp) - amips3->value(xm)) / (2. * h);
                CHECK(g[i] == Catch::Approx(fd).epsilon(1e-6).margin(1e-7 * g.norm()));
                Eigen::VectorXd gp(3), gm(3);
                amips3->gradient(xp, gp);
                amips3->gradient(xm, gm);
                for (int j = 0; j < 3; ++j) {
                    const double fdh = (gp[j] - gm[j]) / (2. * h);
                    CHECK(H(j, i) == Catch::Approx(fdh).epsilon(1e-5).margin(1e-6 * H.norm()));
                }
            }
        }
    }
}

TEST_CASE("stencil-order-point-counts-2d", "[offset][2d]")
{
    // The 2D twin of stencil-order-point-counts: the chord stencil the 2D criterion and energy
    // sample, against the counts the design states -- 2 at order 0 (the ends alone), then the
    // vertices of the chord cut into 2^(k-1) pieces plus one midpoint per piece: 3, 5, 9, 17, 33.
    // Also pins stencil_points_per_edge() to for_each_edge_sample().
    Parameters param;
    TopoOffsetTriMesh mesh(param, 0);
    const Vector2d p0(0., 0.), p1(1., 0.5);
    const std::array<int, 6> expected = {{2, 3, 5, 9, 17, 33}};
    for (int k = 0; k < int(expected.size()); ++k) {
        mesh.m_offset_params.stencil_order = k;
        std::vector<std::array<double, 2>> w;
        mesh.for_each_edge_sample(p0, p1, [&](const Vector2d& q, const double wa, const double wb) {
            CHECK((q - (wa * p0 + wb * p1)).norm() <= 1e-15);
            w.push_back({{wa, wb}});
        });
        INFO("stencil_order " << k);
        CHECK(int(w.size()) == expected[size_t(k)]);
        CHECK(mesh.stencil_points_per_edge() == expected[size_t(k)]);
        std::set<long long> seen;
        for (const auto& b : w) {
            CHECK(std::abs(b[0] + b[1] - 1.) <= 1e-14);
            CHECK(b[0] >= -1e-15);
            CHECK(b[1] >= -1e-15);
            seen.insert(llround(b[0] * 1e9));
        }
        CHECK(seen.size() == w.size()); // no repeated sample
    }
    // Order 0 is exactly the two ends.
    mesh.m_offset_params.stencil_order = 0;
    std::vector<Vector2d> pts;
    mesh.for_each_edge_sample(p0, p1, [&](const Vector2d& q, double, double) { pts.push_back(q); });
    REQUIRE(pts.size() == 2);
    CHECK((pts[0] - p0).norm() <= 1e-15);
    CHECK((pts[1] - p1).norm() <= 1e-15);
}

// ---------------------------------------------------------------------------------------------
// The 2D per-cell energy (TopoOffsetTriMesh::tri_energy()) and the 2D front smoother's offset
// term, on the 2D twin of the 3D fixtures: the euclidean distance to a long segment on the x axis,
// at level delta = 0.5, so relative_residual(q) = (|y| - 0.5) / 0.5 above the segment's interior;
// front_conv = 0.01 (front_conv_frac() 0.02).
// ---------------------------------------------------------------------------------------------

namespace {

std::shared_ptr<EuclideanOffsetPotential2D> line_field()
{
    auto bvh = std::make_shared<SimplicialComplexBVH>();
    MatrixXd SV(2, 2);
    SV << -10., 0., 10., 0.;
    MatrixXi SE(1, 2);
    SE << 0, 1;
    bvh->init(SV, MatrixXi(0, 4), MatrixXi(0, 3), SE, MatrixXi(0, 1));
    return std::make_shared<EuclideanOffsetPotential2D>(bvh, kEnergyDelta);
}

/// The chord term O(e) the energy definition states, from the stencil for_each_edge_sample()
/// visits and nothing else: mean over the points of (relative_residual(q) / front_conv_frac())^2.
double edge_term_by_hand(
    const TopoOffsetTriMesh& m,
    const OffsetPotential2D& pot,
    const Vector2d& p0,
    const Vector2d& p1)
{
    double s = 0.;
    int n = 0;
    m.for_each_edge_sample(p0, p1, [&](const Vector2d&, double wa, double wb) {
        const double r = pot.relative_residual(Vector2d(wa * p0 + wb * p1)) / kEnergyFrac;
        s += r * r;
        ++n;
    });
    return s / double(n);
}

/// The 2D fan fixture: four faces around o, labelled band, band, background, input complex.
/// Face i is row i of F, which the caller relies on to look up faces.
std::unique_ptr<TopoOffsetTriMesh> energy_mesh_2d(
    Parameters& param,
    const Eigen::MatrixXd& V,
    const std::vector<std::array<int, 3>>& F,
    const std::vector<int>& labels,
    const std::shared_ptr<EuclideanOffsetPotential2D>& pot)
{
    param.target_distance = kEnergyDelta;
    param.front_conv = kEnergyConv;
    param.stencil_order = 1;
    auto mesh = std::make_unique<TopoOffsetTriMesh>(param, 0);
    Eigen::MatrixXi Fm(F.size(), 3);
    for (size_t i = 0; i < F.size(); ++i) {
        std::array<int, 3> f = F[i];
        const Vector2d a = V.row(f[0]), b = V.row(f[1]), c = V.row(f[2]);
        const double cross = (b - a).x() * (c - a).y() - (b - a).y() * (c - a).x();
        if (cross < 0.) std::swap(f[1], f[2]); // counter-clockwise
        for (int j = 0; j < 3; ++j) Fm(int(i), j) = f[size_t(j)];
    }
    MatrixSi Tags(int(F.size()), 1);
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;
    mesh->init_from_image(V, Fm, Tags, V_env_dummy, F_env_dummy, {"a"});
    for (size_t i = 0; i < F.size(); ++i) {
        std::array<size_t, 3> want{
            {size_t(Fm(int(i), 0)), size_t(Fm(int(i), 1)), size_t(Fm(int(i), 2))}};
        auto got = mesh->oriented_tri_vids(i);
        std::sort(want.begin(), want.end());
        std::sort(got.begin(), got.end());
        REQUIRE(got == want);
        REQUIRE(!mesh->is_inverted(i));
        mesh->m_face_extra[i].label = labels[i];
    }
    mesh->m_offset_potential = pot;
    return mesh;
}

/// o, r0, r1, r2, r3: a fan of four faces around o; rows of V.
Eigen::MatrixXd fan_vertices()
{
    Eigen::MatrixXd V(5, 2);
    V << 0., 0.5, // o
        0.4, 0.45, // r0
        0.05, 0.9, // r1
        -0.4, 0.55, // r2
        0.02, 0.12; // r3
    return V;
}

} // namespace

TEST_CASE("stencil-energy-2d-is-the-sum-of-edge-terms", "[offset][2d]")
{
    // The 2D front smoother's offset term at offset_term_weight() is, chord by chord, the per-cell
    // energy's O(e): checked against the definition computed by hand from the same stencil, and
    // against edge_offset_term(), at stencil orders 0 (ends only), 1 and 2, with three chords
    // around the moving vertex placed above and below the level set.
    Parameters param;
    param.target_distance = kEnergyDelta;
    param.front_conv = kEnergyConv;
    TopoOffsetTriMesh mesh(param, 0);
    const auto pot = line_field();
    REQUIRE(std::abs(mesh.m_offset_params.front_conv_frac() - kEnergyFrac) <= 1e-15);
    REQUIRE(mesh.offset_term_weight() == Catch::Approx(1. / (kEnergyFrac * kEnergyFrac)));

    const Vector2d x(0.1, 0.53);
    const std::vector<Vector2d> others = {{0.4, 0.61}, {-0.25, 0.44}, {0.05, 0.37}};
    for (const int k : {0, 1, 2}) {
        mesh.m_offset_params.stencil_order = k;
        std::vector<StencilEnergy2D::Edge> edges;
        double by_hand = 0., by_term = 0.;
        for (const Vector2d& q1 : others) {
            StencilEnergy2D::Edge e;
            e.q1 = q1;
            mesh.for_each_edge_sample(x, q1, [&](const Vector2d&, double a, double b) {
                e.samples.push_back({a, b});
            });
            edges.push_back(e);
            by_hand += edge_term_by_hand(mesh, *pot, x, q1);
            by_term += mesh.edge_offset_term(*pot, x, q1);
        }
        StencilEnergy2D energy(pot, edges, mesh.offset_term_weight());
        Eigen::VectorXd xv = x;
        INFO("stencil_order " << k);
        CHECK(by_hand > 10.); // the fixture really is off the level set
        CHECK(energy.value(xv) == Catch::Approx(by_hand).epsilon(1e-12));
        CHECK(by_term == Catch::Approx(by_hand).epsilon(1e-12));
    }
}

TEST_CASE("per-tri-energy", "[offset][2d]")
{
    // The 2D twin of per-tet-energy. Four faces around o: f0 = (o, r0, r1) and f1 = (o, r1, r2)
    // band, f2 = (o, r2, r3) background, f3 = (o, r3, r0) input complex. The band's front chords
    // are the ones with background or nothing across: f0's (r0, r1) on the domain boundary ((o, r0)
    // has the complex across, (o, r1) the band), and f1's (r1, r2) on the boundary plus (o, r2)
    // against the background. Every face off the band is its weighted AMIPS alone. At w = 1 and at
    // the default w.
    const Eigen::MatrixXd V = fan_vertices();
    const int o = 0, r0 = 1, r1 = 2, r2 = 3, r3 = 4;
    for (const double w : {1., Parameters().offset_amips_weight}) {
        Parameters param;
        param.offset_amips_weight = w;
        const auto pot = line_field();
        auto mesh = energy_mesh_2d(
            param,
            V,
            {{{o, r0, r1}}, {{o, r1, r2}}, {{o, r2, r3}}, {{o, r3, r0}}},
            {2, 2, 0, 1},
            pot);
        const auto P = [&](int i) { return Vector2d(V.row(i)); };
        const auto O = [&](int i, int j) { return edge_term_by_hand(*mesh, *pot, P(i), P(j)); };
        const std::array<double, 4> extra = {{O(r0, r1), O(r1, r2) + O(o, r2), 0., 0.}};
        for (size_t fid = 0; fid < 4; ++fid) {
            // The engine's face quality, AMIPS2D from the positions.
            const double base = mesh->TriOptimizerMesh::get_quality(fid);
            REQUIRE(base < TriOptimizerMesh::MAX_ENERGY);
            INFO("w " << w << ", face " << fid);
            CHECK(mesh->tri_energy(fid) == Catch::Approx(w * base + extra[fid]).epsilon(1e-12));
            if (fid < 2) CHECK(extra[fid] > 1.); // the front chords really are off the level set
        }
    }
}

TEST_CASE("smoothing-objective-is-the-ring-energy-2d", "[offset][2d]")
{
    // The 2D twin of smoothing-objective-is-the-ring-energy: at every vertex, the objective the
    // shared smoother minimises (smoothing_extra_energy(); every caller passes w_amips 0) is the
    // sum of tri_energy() over the vertex's ring, up to the terms that do not move with the vertex
    // -- the ring's front chords that do not have it as an end. Checked on the per-tri-energy
    // fixture by moving each vertex that carries the offset term (or is on no front chord) to three
    // nearby positions: objective minus ring sum must not change, and the shared smoother's
    // per-cell comparison (smoothing_cell_energy()) is tri_energy(). The w AMIPS part's gradient
    // and Hessian against central differences too. At w = 1 and at the default w.
    const Eigen::MatrixXd V = fan_vertices();
    const int o = 0, r0 = 1, r1 = 2, r2 = 3, r3 = 4;
    for (const double w : {1., Parameters().offset_amips_weight}) {
        Parameters param;
        param.offset_amips_weight = w;
        const auto pot = line_field();
        auto mesh = energy_mesh_2d(
            param,
            V,
            {{{o, r0, r1}}, {{o, r1, r2}}, {{o, r2, r3}}, {{o, r3, r0}}},
            {2, 2, 0, 1},
            pot);
        // What label_offset_boundary() and the operations maintain in a run: an end of a live
        // front chord.
        for (size_t v = 0; v < 5; ++v) mesh->refresh_offset_membership(v);
        REQUIRE(mesh->m_vertex_extra[size_t(o)].m_is_on_offset);
        REQUIRE(!mesh->m_vertex_extra[size_t(r3)].m_is_on_offset);
        for (size_t v = 0; v < 5; ++v) {
            const bool front = mesh->vertex_carries_offset_term(v);
            // A front vertex an envelope pins (here: one on the domain wall, held by its tag's
            // tube) carries no offset term, so its chords' terms do move with it and are not in
            // its objective; the identity is the carrying vertices' and the off-front ones'.
            if (!front && mesh->m_vertex_extra[v].m_is_on_offset) continue;
            const std::vector<size_t> ring = mesh->get_one_ring_fids_for_vertex(v);
            const auto ring_energy = [&]() {
                double s = 0.;
                for (const size_t fid : ring) s += mesh->tri_energy(fid);
                return s;
            };
            INFO("w " << w << ", vertex " << v << (front ? " (front)" : ""));
            const auto energy = mesh->smoothing_extra_energy(v);
            const Vector2d x0 = mesh->m_vertex_attribute[v].m_posf;
            Eigen::VectorXd xv = x0;
            const double d0 = energy->value(xv) - ring_energy();
            for (const Vector2d& dx :
                 {Vector2d(0.01, 0.), Vector2d(0., -0.008), Vector2d(-0.004, 0.006)}) {
                mesh->set_vertex_position(v, x0 + dx);
                for (const size_t fid : ring) REQUIRE(!mesh->is_inverted(fid));
                xv = x0 + dx;
                const double e = energy->value(xv);
                CHECK(
                    e - ring_energy() ==
                    Catch::Approx(d0).margin(1e-9 * std::max(1., std::abs(e))));
                for (const size_t fid : ring) {
                    const double q = mesh->get_quality(fid);
                    CHECK(mesh->smoothing_cell_energy(fid, q) == mesh->tri_energy(fid));
                }
            }
            mesh->set_vertex_position(v, x0);

            // The w AMIPS part's derivatives against central differences.
            const auto amips = mesh->amips_energy(v);
            const double h = 1e-6;
            Eigen::VectorXd g(2);
            Eigen::MatrixXd H(2, 2);
            xv = x0;
            amips->gradient(xv, g);
            amips->hessian(xv, H);
            for (int i = 0; i < 2; ++i) {
                Eigen::VectorXd xp = xv, xm = xv;
                xp[i] += h;
                xm[i] -= h;
                const double fd = (amips->value(xp) - amips->value(xm)) / (2. * h);
                // An absolute floor besides the relative one: at r1 the ring's AMIPS is stationary
                // (|g| ~ 1e-14, the difference ~ 1e-10 of round-off), where a margin relative to
                // |g| alone is zero.
                CHECK(g[i] == Catch::Approx(fd).epsilon(1e-6).margin(1e-7 * std::max(g.norm(), w)));
                Eigen::VectorXd gp(2), gm(2);
                amips->gradient(xp, gp);
                amips->gradient(xm, gm);
                for (int j = 0; j < 2; ++j) {
                    const double fdh = (gp[j] - gm[j]) / (2. * h);
                    CHECK(H(j, i) == Catch::Approx(fdh).epsilon(1e-5).margin(1e-6 * H.norm()));
                }
            }
        }
    }
}

TEST_CASE("swap-candidate-record", "[offset][3d]")
{
    // The swap's record (TopoOffsetTetMesh::SwapRecord) against the swapped mesh itself: mesh A
    // is the ring before the swap, mesh B the cells after it, built separately with the labels the
    // swap gives them, and every cell of B scored on A's record (candidate_energy()) must read
    // what B reads from its own labels (tet_energy()). A 4-4 surface flip (the created faces
    // (a,c,d), (b,c,d) lie inside the new cells, between the two sides), a 5-6 surface flip under
    // both of its fans, a 3-2 surface flip (the created faces are the old cell (a,b,c,d)'s, so
    // their new cell's side is its apex's), each under four side labelings; an interior 4-4 swap
    // of a band ring; a face swap of a band pair and of a background pair. For the 4-4s and 5-6s
    // the case search's score is checked too: op_case 0 is A's cells read from the mesh, a
    // candidate is B's cells scored on the record.
    const auto pot = plane_field();
    const auto ring_V = [](int k) {
        Eigen::MatrixXd V(2 + k, 3);
        V.row(0) << 0.02, -0.01, 0.12;
        V.row(1) << -0.01, 0.02, 0.88;
        for (int i = 0; i < k; ++i) {
            const double t = 2. * M_PI * i / k;
            V.row(2 + i) << 0.4 * std::cos(t), 0.4 * std::sin(t), 0.5 + 0.03 * i;
        }
        return V;
    };
    const int a = 0, b = 1;
    // search: 0 none (3-2, face swap), 4 swap_edge_44_energy(), 5 swap_edge_56_energy().
    const auto check =
        [&](const Eigen::MatrixXd& V,
            const std::vector<std::array<int, 4>>& TA,
            const std::vector<int>& labA,
            const std::vector<std::array<int, 4>>& TB,
            const std::vector<int>& labB,
            const int search,
            const std::function<void(TopoOffsetTetMesh&, const std::vector<size_t>&)>& before) {
            Parameters pa, pb;
            auto A = energy_mesh(pa, V, TA, labA, pot);
            auto B = energy_mesh(pb, V, TB, labB, pot);
            std::vector<size_t> tids(TA.size());
            std::iota(tids.begin(), tids.end(), size_t(0));
            before(*A, tids);
            std::vector<std::array<size_t, 4>> cells_b;
            double max_b = 0.;
            for (size_t tb = 0; tb < TB.size(); ++tb) {
                const auto vs = B->oriented_tet_vids(tb);
                const double truth = B->tet_energy(tb);
                INFO("new cell " << tb);
                CHECK(A->candidate_energy(vs) == Catch::Approx(truth).epsilon(1e-12));
                cells_b.push_back(vs);
                max_b = std::max(max_b, truth);
            }
            if (search == 0) return;
            std::vector<std::array<size_t, 4>> cells_a;
            double max_a = 0.;
            for (const size_t t : tids) {
                cells_a.push_back(A->oriented_tet_vids(t));
                max_a = std::max(max_a, A->tet_energy(t));
            }
            const auto score = [&](const std::vector<std::array<size_t, 4>>& cells, int op_case) {
                return search == 4 ? A->swap_edge_44_energy(cells, op_case)
                                   : A->swap_edge_56_energy(cells, op_case);
            };
            CHECK(score(cells_a, 0) == max_a);
            CHECK(score(cells_b, 1) == Catch::Approx(max_b).epsilon(1e-12));
        };
    const std::vector<std::pair<int, int>> sides = {{2, 0}, {0, 2}, {2, 1}, {1, 2}};

    // 4-4 flip of (a,b) onto (c0,c2): sides are the arcs {c1} and {c3}.
    {
        const auto V = ring_V(4);
        const int c0 = 2, c1 = 3, c2 = 4, c3 = 5;
        const std::vector<std::array<int, 4>> TA = {
            {{a, b, c0, c1}},
            {{a, b, c1, c2}},
            {{a, b, c2, c3}},
            {{a, b, c3, c0}}};
        const std::vector<std::array<int, 4>> TB = {
            {{a, c0, c1, c2}},
            {{b, c0, c1, c2}},
            {{a, c0, c2, c3}},
            {{b, c0, c2, c3}}};
        for (const auto& [x1, x2] : sides) {
            INFO("4-4 flip, sides " << x1 << " / " << x2);
            check(V, TA, {x1, x1, x2, x2}, TB, {x1, x1, x2, x2}, 4, [&](auto& m, const auto& tids) {
                REQUIRE(m.swap_before_surface(tids, size_t(a), size_t(b), size_t(c0), size_t(c2)));
            });
        }
    }
    // 5-6 flip of (a,b) onto (r0,r2): sides are the arcs {r1} and {r3, r4}. The case search
    // offers two fans that make (r0,r2), with apex r0 and with apex r2; both are checked.
    {
        const auto V = ring_V(5);
        const int r0 = 2, r1 = 3, r2 = 4, r3 = 5, r4 = 6;
        const std::vector<std::array<int, 4>> TA = {
            {{a, b, r0, r1}},
            {{a, b, r1, r2}},
            {{a, b, r2, r3}},
            {{a, b, r3, r4}},
            {{a, b, r4, r0}}};
        const std::vector<std::array<int, 4>> fan_r0 = {
            {{a, r0, r1, r2}},
            {{b, r0, r1, r2}},
            {{a, r0, r2, r3}},
            {{b, r0, r2, r3}},
            {{a, r0, r3, r4}},
            {{b, r0, r3, r4}}};
        const std::vector<std::array<int, 4>> fan_r2 = {
            {{a, r2, r0, r1}},
            {{b, r2, r0, r1}},
            {{a, r2, r3, r4}},
            {{b, r2, r3, r4}},
            {{a, r2, r4, r0}},
            {{b, r2, r4, r0}}};
        for (const auto& [x1, x2] : sides) {
            INFO("5-6 flip, sides " << x1 << " / " << x2);
            const auto flip = [&](auto& m, const auto& tids) {
                REQUIRE(m.swap_before_surface(tids, size_t(a), size_t(b), size_t(r0), size_t(r2)));
            };
            check(V, TA, {x1, x1, x2, x2, x2}, fan_r0, {x1, x1, x2, x2, x2, x2}, 5, flip);
            check(V, TA, {x1, x1, x2, x2, x2}, fan_r2, {x1, x1, x2, x2, x2, x2}, 5, flip);
        }
    }
    // 3-2 flip of (a,b) with surface faces (a,b,c0), (a,b,c1): the old cell (a,b,c0,c1) is one
    // side, the cells holding c2 the other, and both new cells take c2's side.
    {
        const auto V = ring_V(3);
        const int c0 = 2, c1 = 3, c2 = 4;
        const std::vector<std::array<int, 4>> TA = {
            {{a, b, c0, c1}},
            {{a, b, c1, c2}},
            {{a, b, c2, c0}}};
        const std::vector<std::array<int, 4>> TB = {{{a, c0, c1, c2}}, {{b, c0, c1, c2}}};
        for (const auto& [x1, x2] : sides) {
            INFO("3-2 flip, sides " << x1 << " / " << x2);
            check(V, TA, {x1, x2, x2}, TB, {x2, x2}, 0, [&](auto& m, const auto& tids) {
                REQUIRE(m.swap_before_surface(tids, size_t(a), size_t(b), size_t(c0), size_t(c1)));
            });
        }
    }
    // Interior 4-4 swap of a band ring: every boundary face with nothing across is a front face
    // before and after, carried by whichever new cell holds it.
    {
        const auto V = ring_V(4);
        const int c0 = 2, c1 = 3, c2 = 4, c3 = 5;
        const std::vector<std::array<int, 4>> TA = {
            {{a, b, c0, c1}},
            {{a, b, c1, c2}},
            {{a, b, c2, c3}},
            {{a, b, c3, c0}}};
        const std::vector<std::array<int, 4>> TB = {
            {{a, c1, c2, c3}},
            {{b, c1, c2, c3}},
            {{a, c3, c0, c1}},
            {{b, c3, c0, c1}}};
        INFO("interior 4-4");
        check(V, TA, {2, 2, 2, 2}, TB, {2, 2, 2, 2}, 4, [&](auto& m, const auto& tids) {
            REQUIRE(m.swap_before_interior(tids));
        });
    }
    // Face swap (2-3) of (a,b,c) between apexes d and e: each new cell takes one of the six
    // boundary faces on each side.
    {
        Eigen::MatrixXd V(5, 3);
        V << 0.3, 0., 0.5, //
            -0.15, 0.26, 0.52, //
            -0.15, -0.26, 0.48, //
            0.02, 0.01, 0.85, //
            -0.01, -0.02, 0.2;
        const int c = 2, d = 3, e = 4;
        const std::vector<std::array<int, 4>> TA = {{{a, b, c, d}}, {{a, b, c, e}}};
        const std::vector<std::array<int, 4>> TB = {{{a, b, d, e}}, {{b, c, d, e}}, {{c, a, d, e}}};
        for (const int x : {2, 0}) {
            INFO("face swap, label " << x);
            check(V, TA, {x, x}, TB, {x, x, x}, 0, [&](auto& m, const auto& tids) {
                REQUIRE(m.swap_before_interior(tids));
            });
        }
    }
}

TEST_CASE("swap-face-gate-on-energy", "[offset][3d]")
{
    // The face swap's gate (TopoOffsetTetMesh::swap_face_before()) scores the per-tet energy, not
    // AMIPS^3. Two regular tets glued on (a,b,c), which lies 0.1 above the level set: AMIPS^3 is
    // 27 on both, its minimum, so no new cell can beat it and the engine's AMIPS gate refuses.
    // With both cells band, every face is a front face on the domain boundary; the cell above
    // carries three faces far off the level set, and the swap spreads them one to each new cell,
    // so the energy's max falls and the gate admits it. With both cells background the energy is
    // AMIPS^3 and the swap is refused. The whole engine swap runs, with perform_sanity_checks, so
    // swap_after_cells() applies its rule and compares the gate's score with the mesh.
    const auto pot = plane_field();
    const double L = 0.4, h = L * std::sqrt(2. / 3.), z = 0.6;
    const double R = L / std::sqrt(3.);
    Eigen::MatrixXd V(5, 3);
    for (int i = 0; i < 3; ++i) {
        const double t = 2. * M_PI * i / 3.;
        V.row(i) << R * std::cos(t), R * std::sin(t), z;
    }
    V.row(3) << 0., 0., z + h;
    V.row(4) << 0., 0., z - h;
    const int a = 0, b = 1, c = 2, d = 3, e = 4;
    const std::vector<std::array<int, 4>> TA = {{{a, b, c, d}}, {{a, b, c, e}}};
    const std::vector<std::array<int, 4>> TB = {{{a, b, d, e}}, {{b, c, d, e}}, {{c, a, d, e}}};

    for (const int x : {2, 0}) {
        INFO("label " << x);
        Parameters pa, pb;
        pa.perform_sanity_checks = true;
        auto A = energy_mesh(pa, V, TA, {x, x}, pot);
        auto B = energy_mesh(pb, V, TB, {x, x, x}, pot);
        double amips_a = 0., amips_b = 0., energy_a = 0., energy_b = 0.;
        for (size_t t = 0; t < 2; ++t) {
            amips_a = std::max(amips_a, A->TetOptimizerMesh::get_quality(A->oriented_tet_vids(t)));
            energy_a = std::max(energy_a, A->tet_energy(t));
        }
        for (size_t t = 0; t < 3; ++t) {
            amips_b = std::max(amips_b, B->TetOptimizerMesh::get_quality(B->oriented_tet_vids(t)));
            energy_b = std::max(energy_b, B->tet_energy(t));
        }
        // The premises: AMIPS alone refuses this swap, and the energy admits it on the band.
        REQUIRE(amips_a == Catch::Approx(27.).epsilon(1e-9));
        REQUIRE(amips_b > amips_a);
        REQUIRE((energy_b < energy_a) == (x == 2));

        std::vector<TopoOffsetTetMesh::Tuple> new_tets;
        const auto [face, fid] =
            A->tuple_from_face(std::array<size_t, 3>{{size_t(a), size_t(b), size_t(c)}});
        (void)fid;
        CHECK(A->swap_face(face, new_tets) == (x == 2));
        CHECK(A->m_swap_scoring_mismatch.load() == 0);
        CHECK(A->m_swap_scoring_checked.load() == (x == 2 ? 1 : 0));
        if (x != 2) continue;
        REQUIRE(new_tets.size() == 3);
        double energy_new = 0.;
        for (const auto& t : new_tets) energy_new = std::max(energy_new, A->tet_energy(t.tid(*A)));
        CHECK(energy_new == Catch::Approx(energy_b).epsilon(1e-12));
    }
}

TEST_CASE("swap-44-case-search-on-energy", "[offset][3d]")
{
    // The 4-4 case search (TetMesh::swap_edge_44()) scores the per-tet energy through
    // TopoOffsetTetMesh::swap_edge_44_energy(), on the record swap_before_interior() filled
    // before it. An octahedron around the edge (a,b), made slightly shorter than the other two
    // diagonals (c0,c2) and (c1,c3), so both 4-4 cases raise max AMIPS^3 and the engine's AMIPS
    // search takes neither. The level set passes through c3 and c1 is twice as far off it as
    // the other corners: the faces at c1 carry most of the front error, each old cell holds one
    // of them on each side of the edge, and the case (c1,c3) gives each new cell only one, so
    // with the ring band the energy's max falls under that case alone. The engine then commits
    // it, the after-hook's rule agrees, and the sanity check compares the case search's score
    // with the mesh. With the ring background the energy is AMIPS^3 and nothing is taken.
    const auto pot = plane_field();
    const double s = 0.2, z = 0.7;
    Eigen::MatrixXd V(6, 3);
    V << -0.95 * s, 0., z, // a
        0.95 * s, 0., z, // b
        0., s, z, // c0
        0., 0., z + s, // c1
        0., -s, z, // c2
        0., 0., z - s; // c3, on the level set z = 0.5
    const int a = 0, b = 1, c0 = 2, c1 = 3, c2 = 4, c3 = 5;
    const std::vector<std::array<int, 4>> TA = {
        {{a, b, c0, c1}},
        {{a, b, c1, c2}},
        {{a, b, c2, c3}},
        {{a, b, c3, c0}}};
    const std::vector<std::array<int, 4>> T02 = {
        {{a, c0, c1, c2}},
        {{b, c0, c1, c2}},
        {{a, c0, c2, c3}},
        {{b, c0, c2, c3}}};
    const std::vector<std::array<int, 4>> T13 = {
        {{a, c1, c2, c3}},
        {{b, c1, c2, c3}},
        {{a, c3, c0, c1}},
        {{b, c3, c0, c1}}};
    for (const int x : {2, 0}) {
        INFO("label " << x);
        const std::vector<int> lab(4, x);
        Parameters pa, p02, p13;
        pa.perform_sanity_checks = true;
        auto A = energy_mesh(pa, V, TA, lab, pot);
        auto B02 = energy_mesh(p02, V, T02, lab, pot);
        auto B13 = energy_mesh(p13, V, T13, lab, pot);
        const auto maxima = [](TopoOffsetTetMesh& m) {
            double amips = 0., energy = 0.;
            for (size_t t = 0; t < 4; ++t) {
                amips = std::max(amips, m.TetOptimizerMesh::get_quality(m.oriented_tet_vids(t)));
                energy = std::max(energy, m.tet_energy(t));
            }
            return std::make_pair(amips, energy);
        };
        const auto [amips_a, energy_a] = maxima(*A);
        const auto [amips_02, energy_02] = maxima(*B02);
        const auto [amips_13, energy_13] = maxima(*B13);
        // The premises: AMIPS takes no case; on the band the energy takes (c1,c3) alone.
        REQUIRE(amips_02 > amips_a);
        REQUIRE(amips_13 > amips_a);
        REQUIRE((energy_13 < energy_a) == (x == 2));
        REQUIRE(!(energy_02 < energy_a));

        std::vector<TopoOffsetTetMesh::Tuple> new_tets;
        const auto edge = A->tuple_from_edge(std::array<size_t, 2>{{size_t(a), size_t(b)}});
        CHECK(A->swap_edge_44(edge, new_tets) == (x == 2));
        CHECK(A->m_swap_scoring_mismatch.load() == 0);
        CHECK(A->m_swap_scoring_checked.load() == (x == 2 ? 1 : 0));
        if (x != 2) continue;
        REQUIRE(new_tets.size() == 4);
        double energy_new = 0.;
        bool has_c1c3 = true;
        for (const auto& t : new_tets) {
            const auto vs = A->oriented_tet_vids(t);
            energy_new = std::max(energy_new, A->tet_energy(t.tid(*A)));
            has_c1c3 = has_c1c3 && std::count(vs.begin(), vs.end(), size_t(c1)) == 1 &&
                       std::count(vs.begin(), vs.end(), size_t(c3)) == 1;
        }
        CHECK(has_c1c3);
        CHECK(energy_new == Catch::Approx(energy_13).epsilon(1e-12));
    }
}
