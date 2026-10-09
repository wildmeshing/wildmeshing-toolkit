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
    // are HELD depends on the input complex and on whether the final pass is running, read live
    // off the cells, and one envelope holds them all.
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

    // In the loop, no input complex: the a/b boundary stays a tracked face but nothing holds it;
    // the wall is held.
    REQUIRE(!mesh.face_is_held(face(1, 2, 3)));
    REQUIRE(mesh.face_is_held(face(0, 1, 2)));
    mesh.build_envelopes();
    REQUIRE(mesh.m_envelope != nullptr);
    REQUIRE(mesh.surface_envelope_for_face({{1, 2, 3}}) == nullptr);
    REQUIRE(mesh.surface_envelope_for_face({{0, 1, 2}}) == mesh.m_envelope);
    const Eigen::Vector3d on_shared = (V.row(1) + V.row(2) + V.row(3)).transpose() / 3.;
    REQUIRE(mesh.m_envelope->is_outside(on_shared)); // the shared face is not in the envelope
    REQUIRE(!mesh.m_envelope->is_outside(Eigen::Vector3d(V.row(0).transpose())));

    // The final pass: every region boundary is held too, in one envelope with the wall, and the
    // loop's envelope comes back afterwards.
    const auto loop_env = mesh.m_envelope;
    mesh.build_final_envelopes();
    mesh.m_freeze_front = true;
    REQUIRE(mesh.face_is_held(face(1, 2, 3)));
    REQUIRE(mesh.surface_envelope_for_face({{1, 2, 3}}) == mesh.m_envelope);
    for (size_t v = 0; v < 5; ++v) REQUIRE(mesh.vertex_is_held(v));
    REQUIRE(!mesh.m_envelope->is_outside(on_shared));
    REQUIRE(!mesh.m_envelope->is_outside(Eigen::Vector3d(V.row(0).transpose())));
    REQUIRE(mesh.m_envelope->is_outside(Eigen::Vector3d(0.2, 0.2, 0.2)));
    mesh.m_freeze_front = false;
    mesh.release_final_envelopes();
    REQUIRE(mesh.m_envelope == loop_env);
    REQUIRE(!mesh.face_is_held(face(1, 2, 3)));

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

TEST_CASE("stencil-quadratic-weights", "[offset]")
{
    // EXPERIMENTAL_quadratic_stencil: the weights for_each_face_sample() hands a five-argument
    // visitor make the weighted mean of any quadratic over the stencil its exact mean over the
    // face, at every order >= 1; with the key off every weight is 1.
    Parameters param;
    TopoOffsetTetMesh mesh(param, 0);
    const Vector3d p0(0., 0., 0.), p1(1., 0., 0.), p2(0., 1., 0.);
    // f = 1 + 2x - 3y + 5x^2 - 7xy + 11y^2; exact means over the triangle: 1, 1/3, 1/3, 1/6,
    // 1/12, 1/6.
    const auto f = [](const Vector3d& q) {
        const double x = q[0], y = q[1];
        return 1. + 2. * x - 3. * y + 5. * x * x - 7. * x * y + 11. * y * y;
    };
    const double exact = 1. + 2. / 3. - 1. + 5. / 6. - 7. / 12. + 11. / 6.;
    for (int k = 1; k <= 4; ++k) {
        mesh.m_offset_params.stencil_order = k;
        for (const bool quad : {false, true}) {
            mesh.m_offset_params.quadratic_stencil = quad;
            double s = 0., sw = 0.;
            bool all_one = true;
            mesh.for_each_face_sample(
                p0,
                p1,
                p2,
                [&](const Vector3d& q, double, double, double, double w) {
                    s += w * f(q);
                    sw += w;
                    all_one = all_one && w == 1.;
                });
            INFO(
                "order " << k << " quadratic " << quad << " mean " << s / sw << " exact " << exact);
            if (quad) {
                CHECK(s / sw == Catch::Approx(exact).epsilon(1e-13));
            } else {
                CHECK(all_one);
            }
        }
    }
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
// The per-tet energy E_T (TopoOffsetTetMesh::tet_energy()), the smoother's E_V, and the
// criterion's face term.
//
// Fixtures share one input: a large triangle in the plane z = 0, at delta = 0.5, so the distance
// is z and relative_residual(q) = (z - 0.5) / 0.5 anywhere above the triangle's interior; D(t)'s
// one primitive is that triangle. front_conv = 0.01 makes front_conv_frac() = 0.02.
// ---------------------------------------------------------------------------------------------

namespace {

constexpr double kEnergyDelta = 0.5;
constexpr double kEnergyConv = 0.01;
constexpr double kEnergyFrac = kEnergyConv / kEnergyDelta;

/// The fixture's input triangle as D(t)'s primitives.
std::shared_ptr<InputTriangles> plane_tris()
{
    Eigen::MatrixXd V(3, 3);
    V << -10., -10., 0., 10., -10., 0., 0., 10., 0.;
    Eigen::MatrixXi F(1, 3);
    F << 0, 1, 2;
    return std::make_shared<InputTriangles>(V, F);
}

/// A tiny triangle at (0, 0, z), for D(t): above it d is (nearly) the distance to that point,
/// which is convex, so the corner mean over-estimates d's mean over a cell and a swap changes the
/// band's sum -- unlike plane_tris(), above whose interior d is affine and every swap leaves the
/// sum of V_t D(t) unchanged.
std::shared_ptr<InputTriangles> point_tris(const double z)
{
    Eigen::MatrixXd V(3, 3);
    V << 0., 0., z, 1e-3, 0., z, 0., 1e-3, z;
    Eigen::MatrixXi F(1, 3);
    F << 0, 1, 2;
    return std::make_shared<InputTriangles>(V, F);
}

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
    mesh->m_band_tris = plane_tris();
    mesh->m_n_regions = 1;
    return mesh;
}

} // namespace

TEST_CASE("face-remainder", "[offset][3d]")
{
    // face_offset_term()'s remainder, EXPERIMENTAL_unreachable_exit's refinement measure: the mean
    // over the stencil of (e - L)^2, L the least-squares function linear on the face. On a face
    // over the triangle's interior the distance is linear on the face and the remainder is 0; past
    // the triangle's vertex (0, 10, 0) the distance is the distance to that point, not linear, and
    // the remainder is (3/16) b^2 at order 1 (b = the centroid's e minus the mean of the corners')
    // and the least-squares residual computed here by hand at order 2.
    Parameters param;
    param.target_distance = kEnergyDelta;
    param.front_conv = kEnergyConv;
    TopoOffsetTetMesh mesh(param, 0);
    const auto pot = plane_field();
    const double frac = mesh.m_offset_params.front_conv_frac();
    const auto e_at = [&](const Vector3d& q) { return pot->relative_residual(q) / frac; };
    for (const int k : {1, 2}) {
        mesh.m_offset_params.stencil_order = k;
        double rem = -1.;
        mesh.face_offset_term(
            *pot,
            Vector3d(0.1, -0.05, 0.53),
            Vector3d(0.4, 0.1, 0.61),
            Vector3d(0.2, 0.35, 0.44),
            nullptr,
            &rem);
        INFO("flat, stencil_order " << k);
        CHECK(std::abs(rem) <= 1e-9);
    }
    const Vector3d a(0.0, 11.0, 0.5), b(0.6, 11.4, 0.9), c(-0.5, 11.8, 0.3);
    {
        mesh.m_offset_params.stencil_order = 1;
        double rem = -1.;
        mesh.face_offset_term(*pot, a, b, c, nullptr, &rem);
        const double bend = e_at((a + b + c) / 3.) - (e_at(a) + e_at(b) + e_at(c)) / 3.;
        CHECK(std::abs(bend) > 1.); // the fixture really bends
        CHECK(rem == Catch::Approx(3. / 16. * bend * bend).epsilon(1e-10));
    }
    {
        mesh.m_offset_params.stencil_order = 2;
        double rem = -1.;
        mesh.face_offset_term(*pot, a, b, c, nullptr, &rem);
        std::vector<Vector3d> lam;
        std::vector<double> ev;
        mesh.for_each_face_sample(a, b, c, [&](const Vector3d& q, double wa, double wb, double wc) {
            lam.push_back(Vector3d(wa, wb, wc));
            ev.push_back(e_at(q));
        });
        Eigen::MatrixXd A(lam.size(), 3);
        Eigen::VectorXd y(lam.size());
        for (size_t i = 0; i < lam.size(); ++i) {
            A.row(Eigen::Index(i)) = lam[i].transpose();
            y[Eigen::Index(i)] = ev[i];
        }
        const Eigen::VectorXd coef = A.colPivHouseholderQr().solve(y);
        const double by_hand = (y - A * coef).squaredNorm() / double(lam.size());
        CHECK(by_hand > 1e-3);
        CHECK(rem == Catch::Approx(by_hand).epsilon(1e-8));
    }
}

TEST_CASE("per-tet-energy", "[offset][3d]")
{
    // E_T(t) = V_t ( w A(t)^3 / SE^3 + [t in B] (1 - w) D(t) ), against the definition written
    // out: four cells around the edge (a, b), 0 and 1 band, 2 background, 3 input complex. The
    // AMIPS part is against the regular tet (no plastic medium here); D(t) is the corner mean of
    // (z - delta)/delta, the distance to the fixture's one triangle being z. At w = 1, at the
    // default w and at 0.
    Eigen::MatrixXd V(6, 3);
    V << 0., 0., 0.12, // a
        0., 0., 0.88, // b
        0.4, 0., 0.5, // c0
        0., 0.4, 0.55, // c1
        -0.4, 0., 0.45, // c2
        0., -0.4, 0.5; // c3
    const int a = 0, b = 1, c0 = 2, c1 = 3, c2 = 4, c3 = 5;
    for (const double w : {1., Parameters().w_amips, 0.}) {
        Parameters param;
        param.w_amips = w;
        auto mesh = energy_mesh(
            param,
            V,
            {{{a, b, c0, c1}}, {{a, b, c1, c2}}, {{a, b, c2, c3}}, {{a, b, c3, c0}}},
            {2, 2, 0, 1},
            plane_field());
        const double se = mesh->m_params.stop_energy;
        REQUIRE(mesh->amips_weight() == Catch::Approx(w / (se * se * se)));
        REQUIRE(mesh->band_weight() == Catch::Approx(1. - w));
        for (size_t tid = 0; tid < 4; ++tid) {
            const auto vs = mesh->oriented_tet_vids(tid);
            std::array<Vector3d, 4> p;
            for (int k = 0; k < 4; ++k)
                p[size_t(k)] = mesh->m_vertex_attribute[vs[size_t(k)]].m_posf;
            const double vol = (p[1] - p[0]).dot((p[2] - p[0]).cross(p[3] - p[0])) / 6.;
            REQUIRE(vol > 0.);
            // V A^3, A = ||E R^-1||_F^2 / det(E R^-1)^(2/3) against the regular tet.
            const Eigen::Matrix3d R = VolAMIPSEnergy3D::regular_rest();
            Eigen::Matrix3d E;
            for (int k = 0; k < 3; ++k) E.col(k) = p[size_t(k + 1)] - p[0];
            const Eigen::Matrix3d F = E * R.inverse();
            const double A = F.squaredNorm() / std::cbrt(F.determinant() * F.determinant());
            double want = w / (se * se * se) * vol * A * A * A;
            if (tid < 2) {
                double D = 0.;
                for (const Vector3d& q : p) D += (q.z() - kEnergyDelta) / kEnergyDelta / 4.;
                want += (1. - w) * vol * D;
            }
            INFO("w " << w << ", tet " << tid);
            CHECK(mesh->tet_energy(tid) == Catch::Approx(want).epsilon(1e-10));
        }
        std::vector<size_t> all = {0, 1, 2, 3};
        double sum = 0.;
        for (const size_t t : all) sum += mesh->tet_energy(t);
        CHECK(mesh->energy_sum(all) == Catch::Approx(sum).epsilon(1e-14));
        CHECK(mesh->total_energy() == Catch::Approx(sum).epsilon(1e-14));
    }
}

TEST_CASE("collapse-cell-sets", "[offset][3d]")
{
    // THE cell sets every collapse check compares (TopoOffsetTetMesh::CollapseSets), TetWild's:
    // before = v1's one-ring, after = v1's one-ring minus v2's (the cells without v2). On the
    // per-tet-energy fixture, four cells around (a, b): c0 is in cells 0 and 3, c1 in 0 and 1.
    Eigen::MatrixXd V(6, 3);
    V << 0., 0., 0.12, // a
        0., 0., 0.88, // b
        0.4, 0., 0.5, // c0
        0., 0.4, 0.55, // c1
        -0.4, 0., 0.45, // c2
        0., -0.4, 0.5; // c3
    const int a = 0, b = 1, c0 = 2, c1 = 3, c2 = 4, c3 = 5;
    Parameters param;
    auto mesh = energy_mesh(
        param,
        V,
        {{{a, b, c0, c1}}, {{a, b, c1, c2}}, {{a, b, c2, c3}}, {{a, b, c3, c0}}},
        {2, 2, 0, 1},
        plane_field());
    const auto sorted = [](std::vector<size_t> v) {
        std::sort(v.begin(), v.end());
        return v;
    };
    {
        // c0 into c1: c0's ring is {0, 3}; cell 0 holds c1 and vanishes, cell 3 is reshaped.
        const auto s = mesh->collapse_sets(size_t(c0), size_t(c1));
        CHECK(sorted(s.before) == std::vector<size_t>{0, 3});
        CHECK(sorted(s.after) == std::vector<size_t>{3});
    }
    {
        // a into b: every cell holds both, so every cell vanishes and none is reshaped.
        const auto s = mesh->collapse_sets(size_t(a), size_t(b));
        CHECK(sorted(s.before) == std::vector<size_t>{0, 1, 2, 3});
        CHECK(s.after.empty());
    }
    {
        // b into c2: b's ring is every cell; cells 1 and 2 hold c2.
        const auto s = mesh->collapse_sets(size_t(b), size_t(c2));
        CHECK(sorted(s.before) == std::vector<size_t>{0, 1, 2, 3});
        CHECK(sorted(s.after) == std::vector<size_t>{0, 3});
    }
}

TEST_CASE("smoothing-objective-is-the-ring-energy", "[offset][3d]")
{
    // E_V: at every vertex, what smooth_vertex() minimises (vertex_energy()) is the sum of
    // tet_energy() over the vertex's one-ring, evaluated with the vertex moved to x -- elastic,
    // then with the plastic medium on (background cell 2 against its stamped rest). Gradient and
    // Hessian against central differences, at w = 1 and at the default w.
    Eigen::MatrixXd V(6, 3);
    V << 0., 0., 0.12, // a
        0., 0., 0.88, // b
        0.4, 0., 0.5, // c0
        0., 0.4, 0.55, // c1
        -0.4, 0., 0.45, // c2
        0., -0.4, 0.5; // c3
    const int a = 0, b = 1, c0 = 2, c1 = 3, c2 = 4, c3 = 5;
    for (const double w : {1., Parameters().w_amips}) {
        Parameters param;
        param.w_amips = w;
        auto mesh = energy_mesh(
            param,
            V,
            {{{a, b, c0, c1}}, {{a, b, c1, c2}}, {{a, b, c2, c3}}, {{a, b, c3, c0}}},
            {2, 2, 0, 1},
            plane_field());
        for (const bool plastic : {false, true}) {
            mesh->m_plastic_active = plastic;
            if (plastic) {
                // Stamp, then move c2 so cell 2's rest differs from its shape.
                mesh->stamp_plastic_rests();
                REQUIRE(mesh->cell_is_plastic(2));
                REQUIRE(!mesh->cell_is_plastic(0));
                REQUIRE(!mesh->cell_is_plastic(3));
                mesh->set_vertex_position(
                    size_t(c2),
                    mesh->m_vertex_attribute[size_t(c2)].m_posf + Vector3d(-0.03, 0.02, 0.01));
            }
            for (size_t v = 0; v < 6; ++v) {
                INFO("w " << w << ", plastic " << plastic << ", vertex " << v);
                const std::vector<size_t> ring = mesh->get_one_ring_tids_for_vertex(v);
                const auto energy = mesh->vertex_energy(v);
                REQUIRE(energy);
                const Vector3d x0 = mesh->m_vertex_attribute[v].m_posf;
                for (const Vector3d& dx :
                     {Vector3d(0., 0., 0.),
                      Vector3d(0.01, 0., 0.),
                      Vector3d(0., -0.008, 0.005),
                      Vector3d(-0.004, 0.006, -0.007)}) {
                    mesh->set_vertex_position(v, x0 + dx);
                    for (const size_t tid : ring)
                        REQUIRE(!mesh->is_inverted(mesh->tuple_from_tet(tid)));
                    const Eigen::VectorXd xv = x0 + dx;
                    CHECK(
                        energy->value(xv) == Catch::Approx(mesh->energy_sum(ring)).epsilon(1e-10));
                }
                mesh->set_vertex_position(v, x0);
                const double h = 1e-6;
                const Eigen::VectorXd xv = x0 + Vector3d(0.004, -0.003, 0.002);
                Eigen::VectorXd g(3);
                Eigen::MatrixXd H(3, 3);
                energy->gradient(xv, g);
                energy->hessian(xv, H);
                REQUIRE(g.allFinite());
                for (int i = 0; i < 3; ++i) {
                    Eigen::VectorXd xp = xv, xm = xv;
                    xp[i] += h;
                    xm[i] -= h;
                    const double fd = (energy->value(xp) - energy->value(xm)) / (2. * h);
                    CHECK(g[i] == Catch::Approx(fd).epsilon(1e-5).margin(1e-7 * g.norm()));
                    Eigen::VectorXd gp(3), gm(3);
                    energy->gradient(xp, gp);
                    energy->gradient(xm, gm);
                    for (int j = 0; j < 3; ++j) {
                        const double fdh = (gp[j] - gm[j]) / (2. * h);
                        CHECK(H(j, i) == Catch::Approx(fdh).epsilon(1e-4).margin(1e-6 * H.norm()));
                    }
                }
            }
        }
        mesh->m_plastic_active = false;
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
    // D(t)'s one primitive: line_field()'s segment.
    MatrixXd SV(2, 2);
    SV << -10., 0., 10., 0.;
    MatrixXi SE(1, 2);
    SE << 0, 1;
    mesh->m_band_segs = std::make_shared<InputSegments>(SV, SE);
    mesh->m_n_regions = 1;
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

TEST_CASE("per-tri-energy", "[offset][2d]")
{
    // The 2D twin of per-tet-energy: E_T(t) = A_t ( w A(t)^2 / SE^2 + [t in B] (1 - w) D(t) )
    // against the definition written out. Four faces around o: f0, f1 band, f2 background, f3
    // input complex; AMIPS against the equilateral triangle; D(t) the corner mean of
    // (|y| - delta)/delta. At w = 1, at the default w and at 0.
    const Eigen::MatrixXd V = fan_vertices();
    const int o = 0, r0 = 1, r1 = 2, r2 = 3, r3 = 4;
    for (const double w : {1., Parameters().w_amips, 0.}) {
        Parameters param;
        param.w_amips = w;
        auto mesh = energy_mesh_2d(
            param,
            V,
            {{{o, r0, r1}}, {{o, r1, r2}}, {{o, r2, r3}}, {{o, r3, r0}}},
            {2, 2, 0, 1},
            line_field());
        const double se = mesh->m_params.stop_energy;
        REQUIRE(mesh->amips_weight() == Catch::Approx(w / (se * se)));
        REQUIRE(mesh->band_weight() == Catch::Approx(1. - w));
        for (size_t fid = 0; fid < 4; ++fid) {
            const auto vs = mesh->oriented_tri_vids(fid);
            std::array<Vector2d, 3> p;
            for (int k = 0; k < 3; ++k)
                p[size_t(k)] = mesh->m_vertex_attribute[vs[size_t(k)]].m_posf;
            Eigen::Matrix2d E;
            E.col(0) = p[1] - p[0];
            E.col(1) = p[2] - p[0];
            const double area = E.determinant() / 2.;
            REQUIRE(area > 0.);
            const Eigen::Matrix2d F = E * VolAMIPSEnergy2D::regular_rest().inverse();
            const double A = F.squaredNorm() / F.determinant();
            double want = w / (se * se) * area * A * A;
            if (fid < 2) {
                double D = 0.;
                for (const Vector2d& q : p)
                    D += (std::abs(q.y()) - kEnergyDelta) / kEnergyDelta / 3.;
                want += (1. - w) * area * D;
            }
            INFO("w " << w << ", face " << fid);
            CHECK(mesh->tri_energy(fid) == Catch::Approx(want).epsilon(1e-10));
        }
    }
}

TEST_CASE("smoothing-objective-is-the-ring-energy-2d", "[offset][2d]")
{
    // The 2D twin of smoothing-objective-is-the-ring-energy: vertex_energy() is the sum of
    // tri_energy() over the vertex's one-ring with the vertex moved to x, elastic and then with
    // the plastic medium on (background face f2 against its stamped rest); gradient and Hessian
    // against central differences. At w = 1 and at the default w.
    const Eigen::MatrixXd V = fan_vertices();
    const int o = 0, r0 = 1, r1 = 2, r2 = 3, r3 = 4;
    for (const double w : {1., Parameters().w_amips}) {
        Parameters param;
        param.w_amips = w;
        auto mesh = energy_mesh_2d(
            param,
            V,
            {{{o, r0, r1}}, {{o, r1, r2}}, {{o, r2, r3}}, {{o, r3, r0}}},
            {2, 2, 0, 1},
            line_field());
        for (const bool plastic : {false, true}) {
            mesh->m_plastic_active = plastic;
            if (plastic) {
                mesh->stamp_plastic_rests();
                REQUIRE(mesh->face_is_plastic(2));
                REQUIRE(!mesh->face_is_plastic(0));
                REQUIRE(!mesh->face_is_plastic(3));
                mesh->set_vertex_position(
                    size_t(r3),
                    mesh->m_vertex_attribute[size_t(r3)].m_posf + Vector2d(0.02, -0.03));
            }
            for (size_t v = 0; v < 5; ++v) {
                INFO("w " << w << ", plastic " << plastic << ", vertex " << v);
                const std::vector<size_t> ring = mesh->get_one_ring_fids_for_vertex(v);
                const auto energy = mesh->vertex_energy(v);
                REQUIRE(energy);
                const Vector2d x0 = mesh->m_vertex_attribute[v].m_posf;
                for (const Vector2d& dx :
                     {Vector2d(0., 0.),
                      Vector2d(0.01, 0.),
                      Vector2d(0., -0.008),
                      Vector2d(-0.004, 0.006)}) {
                    mesh->set_vertex_position(v, x0 + dx);
                    for (const size_t fid : ring) REQUIRE(!mesh->is_inverted(fid));
                    const Eigen::VectorXd xv = x0 + dx;
                    CHECK(
                        energy->value(xv) == Catch::Approx(mesh->energy_sum(ring)).epsilon(1e-10));
                }
                mesh->set_vertex_position(v, x0);
                const double h = 1e-6;
                const Eigen::VectorXd xv = x0 + Vector2d(0.004, -0.003);
                Eigen::VectorXd g(2);
                Eigen::MatrixXd H(2, 2);
                energy->gradient(xv, g);
                energy->hessian(xv, H);
                REQUIRE(g.allFinite());
                for (int i = 0; i < 2; ++i) {
                    Eigen::VectorXd xp = xv, xm = xv;
                    xp[i] += h;
                    xm[i] -= h;
                    const double fd = (energy->value(xp) - energy->value(xm)) / (2. * h);
                    CHECK(
                        g[i] ==
                        Catch::Approx(fd).epsilon(1e-5).margin(1e-7 * std::max(g.norm(), w)));
                    Eigen::VectorXd gp(2), gm(2);
                    energy->gradient(xp, gp);
                    energy->gradient(xm, gm);
                    for (int j = 0; j < 2; ++j) {
                        const double fdh = (gp[j] - gm[j]) / (2. * h);
                        CHECK(H(j, i) == Catch::Approx(fdh).epsilon(1e-4).margin(1e-6 * H.norm()));
                    }
                }
            }
        }
        mesh->m_plastic_active = false;
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
            double sum_b = 0.;
            for (size_t tb = 0; tb < TB.size(); ++tb) {
                const auto vs = B->oriented_tet_vids(tb);
                const double truth = B->tet_energy(tb);
                INFO("new cell " << tb);
                CHECK(A->candidate_energy(vs) == Catch::Approx(truth).epsilon(1e-12));
                cells_b.push_back(vs);
                sum_b += truth;
            }
            if (search == 0) return;
            std::vector<std::array<size_t, 4>> cells_a;
            double sum_a = 0.;
            for (const size_t t : tids) {
                cells_a.push_back(A->oriented_tet_vids(t));
                sum_a += A->tet_energy(t);
            }
            const auto score = [&](const std::vector<std::array<size_t, 4>>& cells, int op_case) {
                return search == 4 ? A->swap_edge_44_energy(cells, op_case)
                                   : A->swap_edge_56_energy(cells, op_case);
            };
            // Both are sums of E_T, what the swap rule compares.
            CHECK(score(cells_a, 0) == Catch::Approx(sum_a).epsilon(1e-12));
            CHECK(score(cells_b, 1) == Catch::Approx(sum_b).epsilon(1e-12));
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
    // The face swap's gate (TopoOffsetTetMesh::swap_face_before()) compares the sum of E_T, not
    // max AMIPS. Two regular tets glued on (a,b,c): AMIPS is at its minimum on both, so no new
    // cell can beat it and the engine's AMIPS gate refuses. The input is a point on the axis d-e
    // below e (point_tris()), so d is convex and the band's corner bound sum_t V_t D(t) -- the
    // integral of d's piecewise-linear interpolant -- is lower for the three cells around d-e,
    // whose interpolant at the centre of (a,b,c) is the mean of d(d) and d(e) rather than the
    // larger mean over a, b, c. With both cells band that outweighs the AMIPS part and the gate
    // admits the swap; with both cells background E_T is the AMIPS part alone and the swap is
    // refused. The whole engine swap runs, with perform_sanity_checks, so swap_after_cells()
    // applies its rule and compares the gate's score with the mesh.
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
        A->m_band_tris = point_tris(z - h - 0.05);
        B->m_band_tris = A->m_band_tris;
        double amips_a = 0., amips_b = 0., energy_a = 0., energy_b = 0.;
        for (size_t t = 0; t < 2; ++t) {
            amips_a = std::max(amips_a, A->TetOptimizerMesh::get_quality(A->oriented_tet_vids(t)));
            energy_a += A->tet_energy(t);
        }
        for (size_t t = 0; t < 3; ++t) {
            amips_b = std::max(amips_b, B->TetOptimizerMesh::get_quality(B->oriented_tet_vids(t)));
            energy_b += B->tet_energy(t);
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
        for (const auto& t : new_tets) energy_new += A->tet_energy(t.tid(*A));
        CHECK(energy_new == Catch::Approx(energy_b).epsilon(1e-12));
    }
}

TEST_CASE("swap-44-case-search-on-energy", "[offset][3d]")
{
    // The 4-4 case search (TetMesh::swap_edge_44()) scores the sum of E_T through
    // TopoOffsetTetMesh::swap_edge_44_energy(), on the record swap_before_interior() filled
    // before it. An octahedron around the edge (a,b), made slightly shorter than the other two
    // diagonals (c0,c2) and (c1,c3), so both 4-4 cases raise max AMIPS and the engine's AMIPS
    // search takes neither. The input is a point on the axis c1-c3 below c3 (point_tris()), so
    // the band's corner bound -- the integral of d's piecewise-linear interpolant -- is lowest
    // with the diagonal (c1,c3), along which d is smallest at the centre: with the ring band the
    // sum of E_T falls under that case alone. The engine then commits it, the after-hook's rule
    // agrees, and the sanity check compares the case search's score with the mesh. With the ring
    // background E_T is the AMIPS part alone and nothing is taken.
    const auto pot = plane_field();
    const double s = 0.2, z = 0.7;
    Eigen::MatrixXd V(6, 3);
    V << -0.95 * s, 0., z, // a
        0.95 * s, 0., z, // b
        0., s, z, // c0
        0., 0., z + s, // c1
        0., -s, z, // c2
        0., 0., z - s; // c3
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
        A->m_band_tris = point_tris(z - s - 0.05);
        B02->m_band_tris = A->m_band_tris;
        B13->m_band_tris = A->m_band_tris;
        // Max AMIPS (the engine's own rule) and the sum of E_T (the offsets' swap rule).
        const auto maxima = [](TopoOffsetTetMesh& m) {
            double amips = 0., energy = 0.;
            for (size_t t = 0; t < 4; ++t) {
                amips = std::max(amips, m.TetOptimizerMesh::get_quality(m.oriented_tet_vids(t)));
                energy += m.tet_energy(t);
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
            energy_new += A->tet_energy(t.tid(*A));
            has_c1c3 = has_c1c3 && std::count(vs.begin(), vs.end(), size_t(c1)) == 1 &&
                       std::count(vs.begin(), vs.end(), size_t(c3)) == 1;
        }
        CHECK(has_c1c3);
        CHECK(energy_new == Catch::Approx(energy_13).epsilon(1e-12));
    }
}
