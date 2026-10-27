#include <jse/jse.h>
#include <math.h>
#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>
#include <wmtk/components/topological_offset/TopoOffsetTetMesh.h>
#include <wmtk/components/topological_offset/TopoOffsetTriMesh.h>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cstring>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <numeric>
#include <queue>
#include <set>
#include <topological_offset_spec.hpp>
#include <tuple>
#include <wmtk/Types.hpp>
#include <wmtk/components/simwild/expression_parser/Parser.hpp>
#include <wmtk/components/topological_offset/Circle.hpp>
#include <wmtk/components/topological_offset/SimplicialComplexBVH.hpp>
#include <wmtk/components/topological_offset/Sphere.hpp>
#include <wmtk/components/topological_offset/topological_offset.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/utils/io.hpp>

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
    mesh.m_tetwild_pass = true;
    REQUIRE(mesh.face_is_held(face(1, 2, 3)));
    REQUIRE(mesh.surface_envelope_for_face({{1, 2, 3}}) == mesh.m_envelope);
    for (size_t v = 0; v < 5; ++v) REQUIRE(mesh.vertex_is_held(v));
    REQUIRE(!mesh.m_envelope->is_outside(on_shared));
    REQUIRE(!mesh.m_envelope->is_outside(Eigen::Vector3d(V.row(0).transpose())));
    REQUIRE(mesh.m_envelope->is_outside(Eigen::Vector3d(0.2, 0.2, 0.2)));
    mesh.m_tetwild_pass = false;
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


// A face or tet split reuses the split tet's slot, so a new interior face must be written whole,
// not just labelled: otherwise it keeps the surface tags its slot held. Measured on the Geneva
// base, where the simplicial embedding's face split left an input face's tag on an interior face
// 0.48 delta off the input and held the march's vertices on it.
TEST_CASE("split_new_interior_faces_untracked_3d", "[split_op][3d]")
{
    Eigen::Matrix<double, Eigen::Dynamic, 3> V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::MatrixXi T(1, 4);
    T << 0, 1, 2, 3;
    MatrixSi Tags(1, 3);
    std::vector<std::string> tag_names = {"a", "b", "c"};
    for (int i = 0; i < 3; i++) Tags.coeffRef(0, i) = T0_TAGS.count(tag_names[i]) ? 1 : 0;

    Parameters param;
    TopoOffsetTetMesh mesh(param, 0);
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;
    mesh.init_from_image(V, T, Tags, V_env_dummy, F_env_dummy, tag_names);

    // every face of the tet tracked, as an input tet's faces would be
    const std::array<std::array<size_t, 3>, 4> outer = {
        {{{0, 1, 2}}, {{0, 1, 3}}, {{0, 2, 3}}, {{1, 2, 3}}}};
    for (const auto& f : outer) {
        mesh.m_face_attribute[std::get<1>(mesh.tuple_from_face(f))].m_is_surface_fs = true;
    }
    const auto tracked = [&](size_t a, size_t b, size_t c) {
        return bool(
            mesh.m_face_attribute[std::get<1>(mesh.tuple_from_face({{a, b, c}}))].m_is_surface_fs);
    };
    std::vector<TetMesh::Tuple> garbage;

    SECTION("face split")
    {
        auto [ftup, _] = mesh.tuple_from_face({{1, 2, 3}});
        REQUIRE(mesh.split_face(ftup, garbage));
        // the three faces through the new vertex 4 and the opposite vertex 0 are interior
        for (size_t v : {1, 2, 3}) REQUIRE_FALSE(tracked(0, v, 4));
        for (int i = 0; i < 3; i++) REQUIRE(tracked(outer[i][0], outer[i][1], outer[i][2]));
    }
    SECTION("tet split")
    {
        REQUIRE(mesh.split_tet(mesh.tuple_from_tet(0), garbage));
        // the six faces through the new vertex 4 are interior
        for (size_t a = 0; a < 4; a++) {
            for (size_t b = a + 1; b < 4; b++) REQUIRE_FALSE(tracked(a, b, 4));
        }
        for (const auto& f : outer) REQUIRE(tracked(f[0], f[1], f[2]));
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
    param.envelope_size = 1e-3; // tests skip Parameters::init(); give the build a real eps
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
    param.envelope_size = 1e-3; // tests skip Parameters::init(); give the build a real eps
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
    param.envelope_size = 1e-3; // tests skip Parameters::init(); give the build a real eps
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
        param.envelope_size = 1e-3; // tests skip Parameters::init(); give the build a real eps
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
        param.envelope_size = 1e-3; // tests skip Parameters::init(); give the build a real eps
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
    param.envelope_size = 1e-3; // tests skip Parameters::init(); give the build a real eps
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
    param.envelope_size = 1e-3; // tests skip Parameters::init(); give the build a real eps
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
    param.envelope_size = 1e-3; // tests skip Parameters::init(); give the build a real eps
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
    double dist = mesh.input_complex_distance(q);
    REQUIRE(fabs(dist - sqrt(3.0)) < pow(10, -6));

    Vector3d q0(0.0, 0.0, 0.0);
    dist = mesh.input_complex_distance(q0);
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
    dist = mesh.input_complex_distance(q1);
    REQUIRE(fabs(dist - (2.0 / sqrt(3.0))) < pow(10, -6));

    Vector3d q2(0.0, 0.0, 0.0);
    dist = mesh.input_complex_distance(q2);
    REQUIRE(fabs(dist) < pow(10, -6));

    Vector3d q3(10.0, 2.0, 1.0);
    dist = mesh.input_complex_distance(q3);
    REQUIRE(fabs(dist - sqrt(86.0)) < pow(10, -6));

    Vector3d q4(0.1, 0.2, 0.3);
    dist = mesh.input_complex_distance(q4);
    REQUIRE(fabs(dist - 0.1) < pow(10, -6));

    // label tet as input
    mesh.m_tet_attribute[0].label = 1;
    mesh.init_input_complex_bvh();

    dist = mesh.input_complex_distance(q4);
    REQUIRE(fabs(dist) < pow(10, -6));

    dist = mesh.input_complex_distance(q3);
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
    // The stencil the whole 3D criterion and the "surface" energy sample, checked against the
    // counts the design states (since 2026-10-27): the interior lattice (i, j, l) / (k + 2), i, j,
    // l >= 1, k (k + 1) / 2 points -- 1, 3, 6, 10, 15, 21 for k = 1..6 -- and nothing at order 0.
    //
    // This also pins stencil_points_per_face() to for_each_face_sample(), which are two separate
    // pieces of arithmetic that have to agree: the accessor is used in the logs and in the
    // criterion's own reporting, the loop is what actually visits the points.
    Parameters param;
    TopoOffsetTetMesh mesh(param, 0);

    const Vector3d p0(0., 0., 0.), p1(1., 0., 0.), p2(0., 1., 0.);
    const std::array<int, 7> expected = {{0, 1, 3, 6, 10, 15, 21}};

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

        // Every point interior: each weight at least 1 / (k + 2), so none on an edge or corner.
        for (const auto& b : w) {
            CHECK(std::abs(b[0] + b[1] + b[2] - 1.) <= 1e-14);
            CHECK(b[0] >= 1. / double(k + 2) - 1e-15);
            CHECK(b[1] >= 1. / double(k + 2) - 1e-15);
            CHECK(b[2] >= 1. / double(k + 2) - 1e-15);
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

    // Order 1 is exactly the centroid; the lattice is symmetric, so at every order the points'
    // mean is the centroid too (the stencil integrates affine functions exactly).
    const Vector3d centroid = (p0 + p1 + p2) / 3.;
    for (int k = 1; k <= 6; ++k) {
        mesh.m_offset_params.stencil_order = k;
        Vector3d mean = Vector3d::Zero();
        int n = 0;
        mesh.for_each_face_sample(p0, p1, p2, [&](const Vector3d& q, double, double, double) {
            mean += q;
            ++n;
        });
        INFO("stencil_order " << k);
        REQUIRE(n > 0);
        CHECK((mean / double(n) - centroid).norm() <= 1e-15);
    }
    // The shared table and the visitor agree point for point.
    mesh.m_offset_params.stencil_order = 5;
    const auto& table = interior_face_stencil(5);
    size_t i = 0;
    mesh.for_each_face_sample(p0, p1, p2, [&](const Vector3d&, double wa, double wb, double wc) {
        REQUIRE(i < table.size());
        CHECK(table[i][0] == wa);
        CHECK(table[i][1] == wb);
        CHECK(table[i][2] == wc);
        ++i;
    });
    CHECK(i == 15);
}

// ---------------------------------------------------------------------------------------------
// The per-tet energy (TopoOffsetTetMesh::tet_energy()) and the face measure.
//
// Fixtures share one input: a large triangle in the plane z = 0, the euclidean field of its
// distance at level delta = 0.5, and the same triangle as the band term's input envelope. Above
// the triangle's interior d = z, affine, so relative_residual(q) = (z - 0.5) / 0.5 and the band
// term's corner mean is exact. Beyond its apex (0, 10, 0) -- y > 10, |x| < 2 (y - 10) -- d is the
// distance to the apex, curved, and the corner mean is an upper bound that depends on how a
// region is cut into cells: the swap fixtures sit there. front_conv = 0.01 makes
// front_conv_frac() = 0.02.
// ---------------------------------------------------------------------------------------------

namespace {

constexpr double kEnergyDelta = 0.5;
constexpr double kEnergyConv = 0.01;
constexpr double kEnergyFrac = kEnergyConv / kEnergyDelta;
const Vector3d kApex(0., 10., 0.);

std::shared_ptr<EuclideanOffsetPotential3D> plane_field()
{
    auto env = std::make_shared<Envelope>();
    env->use_exact = true;
    const std::vector<Eigen::Vector3d> verts = {{-10., -10., 0.}, {10., -10., 0.}, kApex};
    const std::vector<Eigen::Vector3i> tris = {{0, 1, 2}};
    env->init(verts, tris, kEnergyDelta);
    return std::make_shared<EuclideanOffsetPotential3D>(env, kEnergyDelta);
}

/// The triangle's corners.
Eigen::MatrixXd plane_vertices()
{
    Eigen::MatrixXd V(3, 3);
    V << -10., -10., 0., 10., -10., 0., kApex.x(), kApex.y(), kApex.z();
    return V;
}

/// The same triangle as the band term's convex pieces (TopoOffsetTetMesh::
/// m_input_complex_envelope, which the band term searches).
std::shared_ptr<Envelope> plane_triangles()
{
    const Eigen::MatrixXd V = plane_vertices();
    Eigen::MatrixXi F(1, 3);
    F << 0, 1, 2;
    auto env = std::make_shared<Envelope>(true);
    env->init(V, F, kEnergyDelta);
    return env;
}

/// The distance to the triangle at a point above its interior or beyond its apex, the two
/// regions the fixtures use.
double plane_distance(const Vector3d& q)
{
    const Vector3d t = q - kApex;
    if (t.y() > 0. && std::abs(t.x()) < 2. * t.y()) return t.norm();
    return std::abs(q.z());
}

/// The band term b(t) the energy definition states, computed here from the corners: unsigned
/// volume times (mean corner distance - delta) / delta (one triangle, so the min over triangles is
/// that triangle).
double band_term_by_hand(const std::array<Vector3d, 4>& p)
{
    const double vol = std::abs((p[1] - p[0]).dot((p[2] - p[0]).cross(p[3] - p[0]))) / 6.;
    double mean = 0.;
    for (const Vector3d& q : p) mean += plane_distance(q) / 4.;
    return vol * (mean - kEnergyDelta) / kEnergyDelta;
}

/// The face term O(f) the face measure's definition states, computed here from the stencil
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

/// A mesh of the given cells with the given construction labels on the shared field and
/// triangles. Tet i is row i of T, which the caller relies on to label and look up cells.
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
    mesh->m_input_complex_envelope = plane_triangles();
    mesh->m_input_complex_pad = input_pad(plane_vertices());
    return mesh;
}

/// The cells' corner positions, in the mesh's order.
std::array<Vector3d, 4> corners(const TopoOffsetTetMesh& m, const size_t tid)
{
    const auto vs = m.oriented_tet_vids(tid);
    std::array<Vector3d, 4> p;
    for (int j = 0; j < 4; ++j) p[size_t(j)] = m.m_vertex_attribute[vs[size_t(j)]].m_posf;
    return p;
}

} // namespace

TEST_CASE("face-term-is-the-stencil-mean", "[offset][3d]")
{
    // face_offset_term(), the face measure the exit, the refinement and the frames read, is its
    // definition computed by hand from the stencil for_each_face_sample() visits, at stencil
    // orders 1 (the centroid), 2 and 5, on three faces placed above and below the level set.
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
    for (const int k : {1, 2, 5}) {
        mesh.m_offset_params.stencil_order = k;
        double by_hand = 0., by_term = 0.;
        for (const auto& [q1, q2] : others) {
            by_hand += face_term_by_hand(mesh, *pot, x, q1, q2);
            by_term += mesh.face_offset_term(*pot, x, q1, q2);
        }
        INFO("stencil_order " << k);
        CHECK(by_hand > 10.); // the fixture really is off the level set
        CHECK(by_term == Catch::Approx(by_hand).epsilon(1e-12));
    }
}

TEST_CASE("per-tet-energy", "[offset][3d]")
{
    // tet_energy() against its definition computed by hand: (w / s^3) vol AMIPS^3 for every
    // cell, plus the band term b = vol (mean corner d - delta) / delta for a band cell; MAX_ENERGY
    // for a degenerate cell. Four cells around the edge (a, b): 0 and 1 band, 2 background, 3
    // input complex. At (w, s) = (1, stop_energy 3), where both terms matter, (1, 100) and the
    // default w at 100.
    // Lifted 0.25 above the level set, so the band cells' mean distance is off it.
    Eigen::MatrixXd V(6, 3);
    V << 0., 0., 0.37, // a
        0., 0., 1.13, // b
        0.4, 0., 0.75, // c0
        0., 0.4, 0.8, // c1
        -0.4, 0., 0.7, // c2
        0., -0.4, 0.75; // c3
    const int a = 0, b = 1, c0 = 2, c1 = 3, c2 = 4, c3 = 5;
    const std::vector<int> labels = {2, 2, 0, 1};
    for (const auto& [w, s] : std::vector<std::pair<double, double>>{
             {1., 3.},
             {1., 100.},
             {Parameters().offset_amips_weight, 100.}}) {
        Parameters param;
        param.offset_amips_weight = w;
        param.stop_energy = s;
        const auto pot = plane_field();
        auto mesh = energy_mesh(
            param,
            V,
            {{{a, b, c0, c1}}, {{a, b, c1, c2}}, {{a, b, c2, c3}}, {{a, b, c3, c0}}},
            labels,
            pot);
        for (size_t tid = 0; tid < 4; ++tid) {
            // The engine's cell quality, AMIPS^3 from the positions.
            const double a3 = mesh->TetOptimizerMesh::get_quality(mesh->oriented_tet_vids(tid));
            REQUIRE(a3 < TetOptimizerMesh::MAX_ENERGY);
            const auto p = corners(*mesh, tid);
            const double vol = std::abs((p[1] - p[0]).dot((p[2] - p[0]).cross(p[3] - p[0]))) / 6.;
            const double want =
                w / (s * s * s) * vol * a3 + (labels[tid] == 2 ? band_term_by_hand(p) : 0.);
            INFO("w " << w << ", s " << s << ", tet " << tid);
            CHECK(mesh->tet_energy(tid) == Catch::Approx(want).epsilon(1e-10));
            if (labels[tid] == 2) CHECK(std::abs(band_term_by_hand(p)) > 1e-3);
        }
        // A degenerate cell: c3 into the plane y = 0 of a, b, c0 and c2 flattens cells 2 and 3.
        mesh->set_vertex_position(size_t(c3), Vector3d(0.2, 0., 0.75));
        for (const size_t tid : {size_t(2), size_t(3)}) {
            REQUIRE(
                mesh->TetOptimizerMesh::get_quality(mesh->oriented_tet_vids(tid)) >=
                TetOptimizerMesh::MAX_ENERGY);
            CHECK(mesh->tet_energy(tid) == TetOptimizerMesh::MAX_ENERGY);
        }
    }
}

TEST_CASE("smoothing-objective-is-the-ring-energy", "[offset][3d]")
{
    // One energy for the smoother and the rules: at every vertex not held on the input, the
    // objective the shared smoother minimises (smoothing_extra_energy(); every caller passes
    // w_amips 0, so the smoother adds no AMIPS term of its own) is the sum of tet_energy() over
    // the vertex's ring -- every term of which moves with the vertex. Two fixtures: the
    // per-tet-energy one (front vertices a, b, c0, c1, c2; c3 on no front face and in no band
    // cell), and a star of eight band cells around a vertex inside the band, which is on no front
    // face but carries the band term of all eight. Each vertex is moved to three nearby positions;
    // the objective's gradient and Hessian against central differences. The engine's ring hook
    // (smoothing_ring_energy(), the projected step's test) is the same ring sum. use_rest_pose:
    // on the first fixture again with the plastic medium on, background cell 2 against a stamped
    // rest that its corner c2 has since left -- tet_energy() reads the rest, and the objective
    // still is the ring sum.
    Eigen::MatrixXd V1(6, 3);
    V1 << 0., 0., 0.37, // a
        0., 0., 1.13, // b
        0.4, 0., 0.75, // c0
        0., 0.4, 0.8, // c1
        -0.4, 0., 0.7, // c2
        0., -0.4, 0.75; // c3
    const std::vector<std::array<int, 4>> T1 = {
        {{0, 1, 2, 3}},
        {{0, 1, 3, 4}},
        {{0, 1, 4, 5}},
        {{0, 1, 5, 2}}};
    // Beyond the apex, where d is curved: above the triangle's interior d is affine, the band
    // term's corner mean is exact there, and the sum over a closed star would not move with its
    // inner vertex at all.
    Eigen::MatrixXd V2(7, 3);
    V2 << 0.05, 0.58, -0.02, // the inner vertex
        0.33, 0.6, 0.01, // +x
        -0.28, 0.61, -0.01, // -x
        0.02, 0.91, 0.03, // +y
        -0.01, 0.31, -0.02, // -y
        0., 0.62, 0.3, // +z
        0.03, 0.6, -0.29; // -z
    V2.rowwise() += kApex.transpose();
    std::vector<std::array<int, 4>> T2;
    for (const int x : {1, 2})
        for (const int y : {3, 4})
            for (const int z : {5, 6}) T2.push_back({{0, x, y, z}});
    struct Fixture
    {
        const Eigen::MatrixXd& V;
        const std::vector<std::array<int, 4>>& T;
        std::vector<int> labels;
        size_t not_front; ///< the one vertex on no front face
        bool has_band_ring; ///< whether that vertex's ring holds band cells
    };
    const std::vector<Fixture> fixtures = {
        {V1, T1, {2, 2, 0, 1}, 5, false},
        {V2, T2, std::vector<int>(8, 2), 0, true}};
    for (size_t fi = 0; fi < fixtures.size(); ++fi) {
        const Fixture& fx = fixtures[fi];
        for (const auto& [w, s, plastic] : std::vector<std::tuple<double, double, bool>>{
                 {1., 3., false},
                 {Parameters().offset_amips_weight, 100., false},
                 {1., 3., true},
                 {Parameters().offset_amips_weight, 100., true}}) {
            if (plastic && fi != 0) continue; // fixture 1 is all band: nothing plastic
            Parameters param;
            param.offset_amips_weight = w;
            param.stop_energy = s;
            const auto pot = plane_field();
            auto mesh = energy_mesh(param, fx.V, fx.T, fx.labels, pot);
            // What label_offset_boundary() sets in a run: a corner of a live front face.
            for (size_t v = 0; v < size_t(fx.V.rows()); ++v) {
                mesh->m_vertex_extra[v].m_is_on_offset =
                    !mesh->offset_surface_faces_live_at(v).empty();
            }
            if (plastic) {
                mesh->m_plastic_active = true;
                mesh->stamp_plastic_rests();
                REQUIRE(mesh->cell_is_plastic(2));
                REQUIRE(!mesh->cell_is_plastic(0)); // band
                REQUIRE(!mesh->cell_is_plastic(3)); // input complex
                // The exact tie's test: plastic cells only; not a band cell, not an empty set.
                REQUIRE(mesh->all_plastic({size_t(2)}));
                REQUIRE(!mesh->all_plastic({size_t(0)}));
                REQUIRE(!mesh->all_plastic({size_t(2), size_t(0)}));
                REQUIRE(!mesh->all_plastic({}));
                // The tie needs both sides plastic: an after cell in the band or the complex
                // breaks it.
                REQUIRE(mesh->plastic_tie({size_t(2)}, {0}));
                REQUIRE(!mesh->plastic_tie({size_t(2)}, {0, 2}));
                REQUIRE(!mesh->plastic_tie({size_t(2)}, {1}));
                REQUIRE(!mesh->plastic_tie({size_t(2), size_t(3)}, {0}));
                REQUIRE(!mesh->plastic_tie({size_t(2)}, {}));
                // c2 leaves its stamped rest, so cell 2 is off its minimum.
                mesh->set_vertex_position(
                    4,
                    mesh->m_vertex_attribute[4].m_posf + Vector3d(0.03, -0.02, 0.025));
                Eigen::Matrix3d R;
                REQUIRE(mesh->plastic_rest(2, R));
                const auto p = corners(*mesh, 2);
                const double rest_term = w / (s * s * s) * VolAMIPSEnergy3D::value_of(p, R);
                const double a3 = mesh->TetOptimizerMesh::get_quality(mesh->oriented_tet_vids(2));
                const double vol =
                    std::abs((p[1] - p[0]).dot((p[2] - p[0]).cross(p[3] - p[0]))) / 6.;
                INFO("w " << w << ", s " << s << ", plastic cell 2");
                CHECK(mesh->tet_energy(2) == Catch::Approx(rest_term).epsilon(1e-10));
                CHECK(
                    std::abs(rest_term - w / (s * s * s) * vol * a3) > 1e-3 * std::abs(rest_term));
            }
            for (size_t v = 0; v < size_t(fx.V.rows()); ++v) {
                const std::vector<size_t> ring = mesh->get_one_ring_tids_for_vertex(v);
                const auto ring_energy = [&]() {
                    double e = 0.;
                    for (const size_t tid : ring) e += mesh->tet_energy(tid);
                    return e;
                };
                const bool front = mesh->vertex_carries_offset_term(v);
                INFO(
                    "fixture " << fi << ", w " << w << ", s " << s << ", plastic " << plastic
                               << ", vertex " << v << (front ? " (front)" : ""));
                REQUIRE(front == (v != fx.not_front));
                REQUIRE(!mesh->vertex_is_held(v));
                if (v == fx.not_front) {
                    REQUIRE((mesh->band_volume_energy_at(v) != nullptr) == fx.has_band_ring);
                }
                const auto energy = mesh->smoothing_extra_energy(v);
                const Vector3d x0 = mesh->m_vertex_attribute[v].m_posf;
                Eigen::VectorXd xv = x0;
                const double e0 = energy->value(xv);
                const double d0 = e0 - ring_energy();
                CHECK(d0 == Catch::Approx(0.).margin(1e-12 * std::max(1., std::abs(e0))));
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
                        Catch::Approx(d0).margin(1e-12 * std::max(1., std::abs(e))));
                    // The projected step's test is the same ring sum, before and at a candidate.
                    std::vector<TetMesh::Tuple> ring_t;
                    for (const size_t tid : ring) ring_t.push_back(mesh->tuple_from_tet(tid));
                    CHECK(mesh->smoothing_ring_energy(ring_t, false) == ring_energy());
                    CHECK(mesh->smoothing_ring_energy(ring_t, true) == ring_energy());
                }
                mesh->set_vertex_position(v, x0);

                // The objective's derivatives against central differences.
                const double h = 1e-6;
                Eigen::VectorXd g(3);
                Eigen::MatrixXd H(3, 3);
                xv = x0;
                energy->gradient(xv, g);
                energy->hessian(xv, H);
                for (int i = 0; i < 3; ++i) {
                    Eigen::VectorXd xp = xv, xm = xv;
                    xp[i] += h;
                    xm[i] -= h;
                    const double fd = (energy->value(xp) - energy->value(xm)) / (2. * h);
                    CHECK(g[i] == Catch::Approx(fd).epsilon(1e-6).margin(1e-7 * g.norm()));
                    Eigen::VectorXd gp(3), gm(3);
                    energy->gradient(xp, gp);
                    energy->gradient(xm, gm);
                    for (int j = 0; j < 3; ++j) {
                        const double fdh = (gp[j] - gm[j]) / (2. * h);
                        CHECK(H(j, i) == Catch::Approx(fdh).epsilon(1e-5).margin(1e-6 * H.norm()));
                    }
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
        param.envelope_size = 1e-3; // tests skip Parameters::init(); give the build a real eps
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
        param.envelope_size = 1e-3; // tests skip Parameters::init(); give the build a real eps
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
            // envelope) carries no offset term, so its chords' terms do move with it and are not in
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
    // The swap's candidate cells (TopoOffsetTetMesh::candidate_energy()) against the swapped mesh
    // itself: mesh A is the ring before the swap, mesh B the cells after it, built separately with
    // the labels the swap gives them, and every cell of B scored on A must read what B reads from
    // its own labels (tet_energy()) -- the label a candidate takes is the whole of what the swap's
    // record supplies. A 4-4 surface flip, a 5-6 surface flip under both of its fans, a 3-2
    // surface flip (both new cells take the side of the apex c2), each under four side labelings;
    // an interior 4-4 swap of a band ring; a face swap of a band pair and of a background pair. For
    // the 4-4s and 5-6s the case search's score is checked too: op_case 0 is the sum of
    // tet_energy() over A's cells, a candidate the sum of candidate_energy() over B's cells.
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
            CHECK(score(cells_a, 0) == sum_a);
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
    // Interior 4-4 swap of a band ring: every new cell takes the ring's one label.
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
    // Face swap (2-3) of (a,b,c) between apexes d and e.
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
    // The face swap's gate (TopoOffsetTetMesh::swap_face_before()) compares the sum of the energy
    // over the three new cells with the sum over the two old ones, not AMIPS^3. Two regular tets
    // glued on (a,b,c), beyond the input triangle's apex, their axis (d,e) on a ray from the apex:
    // AMIPS^3 is 27 on both, its minimum, so the swap raises AMIPS and the engine's AMIPS gate
    // refuses it. d, the distance to the apex, is linear along the axis and convex across it, so
    // the band term's corner mean -- the integral of the linear interpolant of d -- is lower when
    // the cells share the edge (d,e) than when they share the face (a,b,c): with both cells band
    // the energy's sum falls and the gate admits the swap. With both cells background the energy
    // is the AMIPS term alone and the swap is refused. The whole engine swap runs, with
    // perform_sanity_checks, so swap_after_cells() applies its rule and compares the gate's score
    // with the mesh.
    const auto pot = plane_field();
    const double L = 0.4, h = L * std::sqrt(2. / 3.), R = L / std::sqrt(3.);
    const Vector3d m = kApex + Vector3d(0., 0.6, 0.);
    Eigen::MatrixXd V(5, 3);
    for (int i = 0; i < 3; ++i) {
        const double t = 2. * M_PI * i / 3.;
        V.row(i) = (m + Vector3d(R * std::cos(t), 0., R * std::sin(t))).transpose();
    }
    V.row(3) = (m + Vector3d(0., h, 0.)).transpose();
    V.row(4) = (m - Vector3d(0., h, 0.)).transpose();
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
    // The 4-4 case search (TetMesh::swap_edge_44()) scores the sum of the energy through
    // TopoOffsetTetMesh::swap_edge_44_energy(). An octahedron around the edge (a,b) beyond the
    // input triangle's apex, (a,b) slightly shorter than the other two diagonals (c0,c2) and
    // (c1,c3), so both 4-4 cases raise max AMIPS^3 and the engine's AMIPS search takes neither.
    // (c1,c3) lies on a ray from the apex, along which d is linear, while d is convex across it:
    // the band term's corner mean over the four cells around a diagonal is lowest around (c1,c3)
    // and highest around the longer (c0,c2). With the ring band the energy's sum falls under the
    // case (c1,c3) alone; the engine commits it, the after-hook's rule agrees, and the sanity check
    // compares the case search's score with the mesh. With the ring background the energy is the
    // AMIPS term and nothing is taken.
    const auto pot = plane_field();
    const double s = 0.2;
    const Vector3d m = kApex + Vector3d(0., 0.6, 0.);
    Eigen::MatrixXd V(6, 3);
    V.row(0) = (m + Vector3d(-0.95 * s, 0., 0.)).transpose(); // a
    V.row(1) = (m + Vector3d(0.95 * s, 0., 0.)).transpose(); // b
    V.row(2) = (m + Vector3d(0., 0., s)).transpose(); // c0
    V.row(3) = (m + Vector3d(0., s, 0.)).transpose(); // c1, on the ray from the apex
    V.row(4) = (m + Vector3d(0., 0., -s)).transpose(); // c2
    V.row(5) = (m + Vector3d(0., -s, 0.)).transpose(); // c3, on the ray from the apex
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
        const auto amips_and_sum = [](TopoOffsetTetMesh& mesh) {
            double amips = 0., energy = 0.;
            for (size_t t = 0; t < 4; ++t) {
                amips =
                    std::max(amips, mesh.TetOptimizerMesh::get_quality(mesh.oriented_tet_vids(t)));
                energy += mesh.tet_energy(t);
            }
            return std::make_pair(amips, energy);
        };
        const auto [amips_a, energy_a] = amips_and_sum(*A);
        const auto [amips_02, energy_02] = amips_and_sum(*B02);
        const auto [amips_13, energy_13] = amips_and_sum(*B13);
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

TEST_CASE("exact-band-offset-side-inward-and-outward", "[offset][3d][exact_band]")
{
    // The exact rule's band integral counts the offset side of the input only (ExactBandIntegral's
    // class doc), and the band is outside the label-1 region for both directions: that region is
    // the selection for offset_out and everything outside it for offset_in. On a 3 x 3 x 3 grid of
    // unit cubes with the center cube as tag_0, a cell just inside tag_0 and its mirror just
    // outside, both within delta of the face x = 0.5: the band side gives the unclipped integral,
    // the other side 0. MEASURED before this test (Geneva base, offset_in, 2026-10-13): the side
    // was read against the selection while the faces were oriented against the region outside
    // it, the clip kept the far side, and the band term at construction was 0 against -7.5046.
    Eigen::MatrixXd V(64, 3);
    const auto vid = [](int i, int j, int k) { return i + 4 * j + 16 * k; };
    for (int k = 0; k < 4; ++k) {
        for (int j = 0; j < 4; ++j) {
            for (int i = 0; i < 4; ++i) V.row(vid(i, j, k)) << i - 1.5, j - 1.5, k - 1.5;
        }
    }
    std::vector<std::array<int, 4>> T;
    std::vector<int> center;
    const std::array<std::array<int, 3>, 6> perms = {
        {{{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}}, {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}}}};
    for (int k = 0; k < 3; ++k) {
        for (int j = 0; j < 3; ++j) {
            for (int i = 0; i < 3; ++i) {
                for (const auto& pm : perms) { // the cube's 6 tets around its main diagonal
                    std::array<int, 3> c = {{i, j, k}};
                    std::array<int, 4> t;
                    t[0] = vid(c[0], c[1], c[2]);
                    for (int s = 0; s < 3; ++s) {
                        ++c[size_t(pm[size_t(s)])];
                        t[size_t(s + 1)] = vid(c[0], c[1], c[2]);
                    }
                    if (i == 1 && j == 1 && k == 1) center.push_back(int(T.size()));
                    T.push_back(positive(V, t));
                }
            }
        }
    }
    Eigen::MatrixXi Tm(int(T.size()), 4);
    for (size_t i = 0; i < T.size(); ++i) {
        for (int j = 0; j < 4; ++j) Tm(int(i), j) = T[i][size_t(j)];
    }
    MatrixSi Tags(int(T.size()), 1);
    for (const int t : center) Tags.coeffRef(t, 0) = 1;

    const double delta = 0.2;
    const std::array<Vector3d, 4> inside = {
        {{0.45, -0.1, -0.1}, {0.45, 0.1, -0.1}, {0.45, 0., 0.1}, {0.35, 0., 0.}}};
    std::array<Vector3d, 4> outside = inside;
    for (Vector3d& q : outside) q.x() = 1. - q.x();

    for (const bool inward : {true, false}) {
        Parameters param;
        param.offset_in = inward;
        param.offset_out = !inward;
        param.offset_field = "euclidean";
        param.band_volume_rule = "exact";
        param.target_distance = delta;
        TopoOffsetTetMesh mesh(param, 0);
        MatrixXd V_env;
        MatrixXi F_env;
        mesh.init_from_image(V, Tm, Tags, V_env, F_env, {"tag_0"});
        mesh.m_offset_params.offset_selection = parse("tag_0", mesh.m_tag_name_to_id);
        mesh.label_input_complex();
        mesh.init_input_complex_bvh();
        mesh.init_offset_potential();
        REQUIRE(mesh.m_band_exact);
        const ExactBandIntegral plain(mesh.m_phi_V, mesh.m_phi_F);
        const auto& band = inward ? inside : outside;
        const auto& far_side = inward ? outside : inside;
        INFO((inward ? "offset_in" : "offset_out"));
        const double J_band = mesh.m_band_exact->value(band, delta);
        CHECK(J_band < 0.);
        CHECK(J_band == plain.value(band, delta));
        CHECK(mesh.m_band_exact->value(far_side, delta) == 0.);
        CHECK(plain.value(far_side, delta) < 0.);
    }
}

namespace {
/// Parameters as the app builds them: the spec's defaults injected into `j`, then the bounding
/// box. A default-constructed Parameters leaves most keys unset, and construction reads many.
Parameters construction_params(nlohmann::json j, const Eigen::MatrixXd& V)
{
    j["application"] = "topological_offset";
    j["input"] = "unused.msh";
    const auto spec = jse::embed::wmtk_topological_offset_spec::topological_offset_spec::spec();
    jse::JSE spec_engine;
    spec_engine.strict = true;
    REQUIRE(spec_engine.verify_json(j, spec));
    Parameters param(spec_engine.inject_defaults(j, spec));
    param.init(V.colwise().minCoeff(), V.colwise().maxCoeff());
    return param;
}

nlohmann::json correspondence_json()
{
    return {
        {"offset_selection", "tag_0"},
        {"offset_output_tags", {"offset"}},
        {"optimize_offset", false},
        {"save_offset_correspondence", true},
        {"num_threads", 0},
        {"target_distance", 0.2}};
}

/// The marched edges as marching_tets() / marching_tris() pick them: exactly one endpoint labelled
/// 0. Each as (endpoint on the complex, the other endpoint).
template <typename Mesh>
std::vector<std::pair<size_t, size_t>> marched_edges(const Mesh& mesh)
{
    std::vector<std::pair<size_t, size_t>> P;
    for (const auto& e : mesh.get_edges()) {
        const size_t v1 = e.vid(mesh);
        const size_t v2 = e.switch_vertex(mesh).vid(mesh);
        const bool in1 = mesh.m_vertex_extra[v1].label != 0;
        const bool in2 = mesh.m_vertex_extra[v2].label != 0;
        if (in1 != in2) P.emplace_back(in1 ? v1 : v2, in1 ? v2 : v1);
    }
    return P;
}

/**
 * Checks the .msh save_offset_correspondence wrote, read the way the prismatic_mesh component
 * reads it: node k is the node with tag k + 1, the input vertices are the nodes of the tag_0
 * cells, the offset vertices those with corr_input_vid >= 0. The march makes an offset vertex v on
 * the marched edge (c, w), c on the complex, at v = c + t (w - c) with 0 < t < 1: where the
 * sphere trace reached the level set, |d(v) - D| <= sphere_trace_target_rel_tol x D (d the
 * distance to the complex, D = m_construction_distance), otherwise at t = 1/2. Construction only
 * adds vertices, so c and w keep their ids and positions. The fixtures trace every edge.
 */
template <typename Mesh, typename Dist>
void check_correspondence_msh(
    const std::string& file,
    const Mesh& mesh,
    const std::vector<std::pair<size_t, size_t>>& P,
    const size_t n_before,
    const int dim,
    const Dist& dist)
{
    MshData msh;
    msh.load(file);
    const auto& spec = msh.m_spec;

    // nodes, by row = tag - 1
    const size_t n = spec.nodes.num_nodes;
    REQUIRE(n == mesh.get_vertices().size());
    REQUIRE(n == n_before + P.size()); // no embedding split, one vertex per marched edge
    std::vector<Vector3d> pos(n);
    for (const auto& block : spec.nodes.entity_blocks) {
        for (size_t i = 0; i < block.num_nodes_in_block; ++i) {
            const size_t row = block.tags[i] - 1;
            REQUIRE(row < n);
            pos[row] = Vector3d(block.data[3 * i], block.data[3 * i + 1], block.data[3 * i + 2]);
        }
    }

    // corr_input_vid covers every node exactly once, with integer values
    std::vector<int64_t> corr(n, -2);
    size_t n_fields = 0;
    for (const auto& data : spec.node_data) {
        if (data.header.string_tags.front() != "corr_input_vid") continue;
        ++n_fields;
        REQUIRE(data.header.int_tags[1] == 1); // scalar
        REQUIRE(size_t(data.header.int_tags[2]) == n);
        REQUIRE(data.entries.size() == n);
        for (const auto& entry : data.entries) {
            const size_t row = entry.tag - 1;
            REQUIRE(row < n);
            REQUIRE(corr[row] == -2);
            REQUIRE(entry.data.size() == 1);
            REQUIRE(entry.data[0] == std::floor(entry.data[0]));
            corr[row] = int64_t(entry.data[0]);
        }
    }
    REQUIRE(n_fields == 1);

    // cells by physical group name, through the entities
    std::map<int, std::string> group_name;
    for (const auto& g : spec.physical_groups) {
        if (g.dim == dim) group_name[g.tag] = g.name;
    }
    std::map<int, std::string> entity_group;
    const auto add_entities = [&](const auto& entities) {
        for (const auto& ent : entities) {
            REQUIRE(ent.physical_group_tags.size() == 1);
            entity_group[ent.tag] = group_name.at(ent.physical_group_tags.front());
        }
    };
    if (dim == 3) {
        add_entities(spec.entities.volumes);
    } else {
        add_entities(spec.entities.surfaces);
    }
    std::set<size_t> input_nodes, band_nodes;
    for (const auto& block : spec.elements.entity_blocks) {
        REQUIRE(block.entity_dim == dim);
        const std::string& name = entity_group.at(block.entity_tag);
        REQUIRE((name == "ambient" || name == "tag_0" || name == "offset"));
        const size_t stride = size_t(dim) + 2;
        for (size_t i = 0; i < block.num_elements_in_block; ++i) {
            for (size_t j = 1; j < stride; ++j) {
                const size_t row = block.data[stride * i + j] - 1;
                if (name == "tag_0") input_nodes.insert(row);
                if (name == "offset") band_nodes.insert(row);
            }
        }
    }
    REQUIRE(!input_nodes.empty());

    // The offset vertices are exactly the band's vertices off the input, every one labelled 2;
    // every other vertex carries -1.
    std::map<std::pair<size_t, size_t>, size_t> uses;
    for (const auto& e : P) uses[e] = 0;
    const double D = mesh.m_construction_distance;
    const double tol = mesh.m_offset_params.sphere_trace_target_rel_tol * D;
    size_t n_offset = 0;
    for (size_t v = 0; v < n; ++v) {
        const bool offset = band_nodes.count(v) && !input_nodes.count(v);
        CHECK(offset == (mesh.m_vertex_extra[v].label == 2));
        if (!offset) {
            CHECK(corr[v] == -1);
            continue;
        }
        ++n_offset;
        REQUIRE(corr[v] >= 0);
        const size_t c = size_t(corr[v]);
        CHECK(input_nodes.count(c) == 1);
        // the marched edge v was split off: v on the segment from c to w, at the level set
        size_t found = 0;
        for (const auto& e : P) {
            if (e.first != c) continue;
            const Vector3d a = pos[c], b = pos[e.second];
            const double t = (pos[v] - a).dot(b - a) / (b - a).squaredNorm();
            if (!(t > 0. && t < 1.)) continue;
            if ((pos[v] - (a + t * (b - a))).norm() > 1e-12 * (b - a).norm()) continue;
            ++found;
            ++uses[e];
        }
        CHECK(found == 1);
        CHECK(std::abs(dist(pos[v]) - D) <= tol);
    }
    CHECK(n_offset == P.size());
    for (const auto& [e, k] : uses) CHECK(k == 1);
}
} // namespace

TEST_CASE("save-offset-correspondence-3d", "[offset][3d][correspondence]")
{
    // A 3 x 3 x 3 grid of unit cubes with the center cube as tag_0, construction only. Every outer
    // end is a unit away from the complex, so the target 0.2 is below the maximum marchable
    // distance and every marched edge is traced to it.
    Eigen::MatrixXd V(64, 3);
    const auto vid = [](int i, int j, int k) { return i + 4 * j + 16 * k; };
    for (int k = 0; k < 4; ++k) {
        for (int j = 0; j < 4; ++j) {
            for (int i = 0; i < 4; ++i) V.row(vid(i, j, k)) << i - 1.5, j - 1.5, k - 1.5;
        }
    }
    std::vector<std::array<int, 4>> T;
    std::vector<int> center;
    const std::array<std::array<int, 3>, 6> perms = {
        {{{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}}, {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}}}};
    for (int k = 0; k < 3; ++k) {
        for (int j = 0; j < 3; ++j) {
            for (int i = 0; i < 3; ++i) {
                for (const auto& pm : perms) { // the cube's 6 tets around its main diagonal
                    std::array<int, 3> c = {{i, j, k}};
                    std::array<int, 4> t;
                    t[0] = vid(c[0], c[1], c[2]);
                    for (int s = 0; s < 3; ++s) {
                        ++c[size_t(pm[size_t(s)])];
                        t[size_t(s + 1)] = vid(c[0], c[1], c[2]);
                    }
                    if (i == 1 && j == 1 && k == 1) center.push_back(int(T.size()));
                    T.push_back(positive(V, t));
                }
            }
        }
    }
    Eigen::MatrixXi Tm(int(T.size()), 4);
    for (size_t i = 0; i < T.size(); ++i) {
        for (int j = 0; j < 4; ++j) Tm(int(i), j) = T[i][size_t(j)];
    }
    MatrixSi Tags(int(T.size()), 1);
    for (const int t : center) Tags.coeffRef(t, 0) = 1;

    // The steps topological_offset() runs up to the construction.
    Parameters param = construction_params(correspondence_json(), V);
    TopoOffsetTetMesh mesh(param, 0);
    MatrixXd V_env;
    MatrixXi F_env;
    mesh.init_from_image(V, Tm, Tags, V_env, F_env, {"tag_0"});
    mesh.m_offset_params.offset_selection = parse("tag_0", mesh.m_tag_name_to_id);
    mesh.label_input_complex();
    mesh.init_input_complex_bvh();
    mesh.consolidate_mesh();
    const auto P = marched_edges(mesh);
    const size_t n_before = mesh.get_vertices().size();
    REQUIRE(!P.empty());

    const std::filesystem::path out =
        std::filesystem::temp_directory_path() / "wmtk_offset_correspondence_3d";
    mesh.construct_offset(out);
    REQUIRE(mesh.m_marching_midpoint_splits == 0);
    mesh.write_msh_groups(out.string());
    check_correspondence_msh(out.string() + ".msh", mesh, P, n_before, 3, [&](const Vector3d& p) {
        return mesh.input_complex_distance(p);
    });
    std::filesystem::remove(out.string() + ".msh");

    // The field is recorded by the march only, so the app refuses it with the optimization on.
    nlohmann::json j = correspondence_json();
    j["application"] = "topological_offset";
    j["input"] = "unused.msh";
    j["optimize_offset"] = true;
    REQUIRE_THROWS_WITH(
        components::topological_offset::topological_offset(j),
        Catch::Matchers::ContainsSubstring("save_offset_correspondence requires"));
}

namespace {
/// The input of save-offset-correspondence-3d: a 3 x 3 x 3 grid of unit cubes, six tets each,
/// the center cube tag_0.
struct GridInput
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi Tm;
    MatrixSi Tags;
};
GridInput grid_input()
{
    Eigen::MatrixXd V(64, 3);
    const auto vid = [](int i, int j, int k) { return i + 4 * j + 16 * k; };
    for (int k = 0; k < 4; ++k) {
        for (int j = 0; j < 4; ++j) {
            for (int i = 0; i < 4; ++i) V.row(vid(i, j, k)) << i - 1.5, j - 1.5, k - 1.5;
        }
    }
    std::vector<std::array<int, 4>> T;
    std::vector<int> center;
    const std::array<std::array<int, 3>, 6> perms = {
        {{{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}}, {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}}}};
    for (int k = 0; k < 3; ++k) {
        for (int j = 0; j < 3; ++j) {
            for (int i = 0; i < 3; ++i) {
                for (const auto& pm : perms) { // the cube's 6 tets around its main diagonal
                    std::array<int, 3> c = {{i, j, k}};
                    std::array<int, 4> t;
                    t[0] = vid(c[0], c[1], c[2]);
                    for (int s = 0; s < 3; ++s) {
                        ++c[size_t(pm[size_t(s)])];
                        t[size_t(s + 1)] = vid(c[0], c[1], c[2]);
                    }
                    if (i == 1 && j == 1 && k == 1) center.push_back(int(T.size()));
                    T.push_back(positive(V, t));
                }
            }
        }
    }
    Eigen::MatrixXi Tm(int(T.size()), 4);
    for (size_t i = 0; i < T.size(); ++i) {
        for (int j = 0; j < 4; ++j) Tm(int(i), j) = T[i][size_t(j)];
    }
    MatrixSi Tags(int(T.size()), 1);
    for (const int t : center) Tags.coeffRef(t, 0) = 1;

    return {V, Tm, Tags};
}

/// The grid constructed serially at the target in `param`, the field built and the
/// optimization's labels set: a small mesh with a band, a front and offset faces.
std::unique_ptr<TopoOffsetTetMesh>
grid_offset_mesh(const GridInput& g, Parameters& param, const std::filesystem::path& out)
{
    auto mesh = std::make_unique<TopoOffsetTetMesh>(param, 0);
    MatrixXd V_env;
    MatrixXi F_env;
    mesh->init_from_image(g.V, g.Tm, g.Tags, V_env, F_env, {"tag_0"});
    mesh->m_offset_params.offset_selection = parse("tag_0", mesh->m_tag_name_to_id);
    mesh->label_input_complex();
    mesh->init_input_complex_bvh();
    mesh->consolidate_mesh();
    mesh->construct_offset(out);
    mesh->init_offset_potential();
    mesh->label_offset_boundary();
    return mesh;
}

/// Every array of two frames bit for bit, NaN equal to NaN.
bool frames_identical(const TopoOffsetTetMesh::VtuFrame& a, const TopoOffsetTetMesh::VtuFrame& b)
{
    const auto same = [](const auto& x, const auto& y) {
        if (x.rows() != y.rows() || x.cols() != y.cols()) return false;
        for (Eigen::Index i = 0; i < x.size(); ++i) {
            const double p = double(x.data()[i]), q = double(y.data()[i]);
            if (std::isnan(p) && std::isnan(q)) continue;
            if (std::memcmp(&p, &q, sizeof(double)) != 0) return false;
        }
        return true;
    };
    bool ok = same(a.V, b.V) && same(a.T, b.T) && same(a.F_in, b.F_in) && same(a.F_off, b.F_off) &&
              same(a.E, b.E) && a.tag_names == b.tag_names && a.tags.size() == b.tags.size();
    for (size_t i = 0; ok && i < a.tags.size(); ++i) ok = same(a.tags[i], b.tags[i]);
    for (const auto m :
         {&TopoOffsetTetMesh::VtuFrame::amips,      &TopoOffsetTetMesh::VtuFrame::tet_e,
          &TopoOffsetTetMesh::VtuFrame::amips_term, &TopoOffsetTetMesh::VtuFrame::band_term,
          &TopoOffsetTetMesh::VtuFrame::band_gap,   &TopoOffsetTetMesh::VtuFrame::plastic,
          &TopoOffsetTetMesh::VtuFrame::labels,     &TopoOffsetTetMesh::VtuFrame::order,
          &TopoOffsetTetMesh::VtuFrame::vid,        &TopoOffsetTetMesh::VtuFrame::sizing,
          &TopoOffsetTetMesh::VtuFrame::in_dist,    &TopoOffsetTetMesh::VtuFrame::dist_ratio,
          &TopoOffsetTetMesh::VtuFrame::front_err,  &TopoOffsetTetMesh::VtuFrame::front_change,
          &TopoOffsetTetMesh::VtuFrame::f_err,      &TopoOffsetTetMesh::VtuFrame::f_len,
          &TopoOffsetTetMesh::VtuFrame::f_inside,   &TopoOffsetTetMesh::VtuFrame::f_outside,
          &TopoOffsetTetMesh::VtuFrame::f_fold,     &TopoOffsetTetMesh::VtuFrame::f_field_angle,
          &TopoOffsetTetMesh::VtuFrame::f_dihedral, &TopoOffsetTetMesh::VtuFrame::f_fold_dihedral,
          &TopoOffsetTetMesh::VtuFrame::ring}) {
        ok = ok && same(a.*m, b.*m);
    }
    return ok;
}
} // namespace

TEST_CASE("frame-gather-threads-identical", "[offset][3d]")
{
    // gather_vtu_frame() and energy_parts() evaluate their cells, vertices and faces on the mesh's
    // threads (diagnostic_parallel_for()); what they return must not depend on the thread count.
    // Two identical meshes -- the grid of save-offset-correspondence-3d, constructed serially at
    // target 0.2, the field built under the exact rule, the optimization's labels -- one gathered
    // at 1 thread and one at 4, each from its own empty value table, compared bit for bit (NaN
    // equal to NaN).
    const GridInput g = grid_input();

    Parameters param = construction_params(
        {{"offset_selection", "tag_0"},
         {"num_threads", 0},
         {"target_distance", 0.2},
         {"EXPERIMENTAL_band_volume_rule", "exact"}},
        g.V);
    const std::filesystem::path out =
        std::filesystem::temp_directory_path() / "wmtk_offset_frame_threads";
    const auto make_mesh = [&](const int threads) {
        auto mesh = grid_offset_mesh(g, param, out);
        mesh->NUM_THREADS = threads;
        return mesh;
    };
    const auto serial = make_mesh(1);
    const auto threaded = make_mesh(4);
    REQUIRE(serial->m_band_exact);
    const size_t chunk = TopoOffsetTetMesh::kDiagnosticChunk;
    INFO(serial->get_tets().size() << " tets, " << serial->get_vertices().size() << " vertices");
    REQUIRE(serial->get_tets().size() > 4 * chunk); // every task has cells to evaluate

    const auto [a_serial, b_serial] = serial->energy_parts();
    const auto [a_threaded, b_threaded] = threaded->energy_parts();
    CHECK(std::memcmp(&a_serial, &a_threaded, sizeof(double)) == 0);
    CHECK(std::memcmp(&b_serial, &b_threaded, sizeof(double)) == 0);
    CHECK(b_serial < 0.); // the band is inside the level set at construction

    const TopoOffsetTetMesh::VtuFrame fs = serial->gather_vtu_frame(out.string() + "_serial");
    const TopoOffsetTetMesh::VtuFrame ft = threaded->gather_vtu_frame(out.string() + "_threaded");
    // Bit for bit, NaN equal to NaN; and how many entries are finite, so no field passes empty.
    const auto same = [](const auto& x, const auto& y, size_t& n_finite) {
        if (x.rows() != y.rows() || x.cols() != y.cols()) return false;
        n_finite = 0;
        for (Eigen::Index i = 0; i < x.size(); ++i) {
            const double p = double(x.data()[i]), q = double(y.data()[i]);
            if (std::isnan(p) && std::isnan(q)) continue;
            if (std::memcmp(&p, &q, sizeof(double)) != 0) return false;
            n_finite += std::isfinite(p) ? 1 : 0;
        }
        return true;
    };
    const std::vector<std::tuple<const char*, const VectorXd*, const VectorXd*>> fields = {
        {"amips", &fs.amips, &ft.amips},
        {"tet_energy", &fs.tet_e, &ft.tet_e},
        {"amips_term", &fs.amips_term, &ft.amips_term},
        {"band_term", &fs.band_term, &ft.band_term},
        {"band_gap_bars", &fs.band_gap, &ft.band_gap},
        {"plastic", &fs.plastic, &ft.plastic},
        {"labels", &fs.labels, &ft.labels},
        {"order", &fs.order, &ft.order},
        {"vid", &fs.vid, &ft.vid},
        {"sizing", &fs.sizing, &ft.sizing},
        {"input_distance", &fs.in_dist, &ft.in_dist},
        {"distance_over_target", &fs.dist_ratio, &ft.dist_ratio},
        {"front_error_bars", &fs.front_err, &ft.front_err},
        {"front_last_change", &fs.front_change, &ft.front_change},
        {"front_err_ratio", &fs.f_err, &ft.f_err},
        {"chord_length", &fs.f_len, &ft.f_len},
        {"settle_inside_bars", &fs.f_inside, &ft.f_inside},
        {"settle_outside_bars", &fs.f_outside, &ft.f_outside},
        {"offset_foldover", &fs.f_fold, &ft.f_fold},
        {"field_angle_foldover", &fs.f_field_fold, &ft.f_field_fold},
        {"field_normal_angle", &fs.f_field_angle, &ft.f_field_angle},
        {"dihedral_angle", &fs.f_dihedral, &ft.f_dihedral},
        {"offset_foldover_dihedral", &fs.f_fold_dihedral, &ft.f_fold_dihedral},
        {"front_ring_ratio", &fs.ring, &ft.ring}};
    for (const auto& [name, x, y] : fields) {
        INFO(name);
        size_t n_finite = 0;
        CHECK(same(*x, *y, n_finite));
        CHECK(n_finite > 0);
    }
    size_t n = 0;
    CHECK(same(fs.V, ft.V, n));
    CHECK(same(fs.T, ft.T, n));
    CHECK(same(fs.F_in, ft.F_in, n));
    CHECK(same(fs.F_off, ft.F_off, n));
    CHECK(fs.F_off.rows() > 0);
    CHECK(same(fs.E, ft.E, n));
    REQUIRE(fs.tags.size() == ft.tags.size());
    for (size_t i = 0; i < fs.tags.size(); ++i) CHECK(same(fs.tags[i], ft.tags[i], n));
    CHECK(fs.tag_names == ft.tag_names);
}

TEST_CASE("mesh-snapshot-roundtrip", "[offset][3d]")
{
    // take_snapshot() / restore_snapshot(): after a restore the mesh is the one the snapshot left,
    // bit for bit in every frame array, and the same operations from it give the same result
    // again -- the storage is the same, not only the geometry (the operations depend on the order
    // of each vertex's tet fan). The grid mesh, serial; the change is a split pass at a lowered
    // sizing field, which adds vertices and cells.
    const GridInput g = grid_input();
    Parameters param = construction_params(
        {{"offset_selection", "tag_0"},
         {"num_threads", 0},
         {"target_distance", 0.2},
         {"EXPERIMENTAL_band_volume_rule", "exact"}},
        g.V);
    const std::filesystem::path out =
        std::filesystem::temp_directory_path() / "wmtk_offset_snapshot";
    const auto mesh = grid_offset_mesh(g, param, out);
    // As optimize_offset() sets them before the loop.
    mesh->m_edge_split_mode = TopoOffsetTetMesh::EdgeSplitMode::Optimization;
    mesh->init_vertex_order();
    const TopoOffsetTetMesh::MeshSnapshot snap = mesh->take_snapshot();
    REQUIRE(mesh->check_mesh_connectivity_validity());
    const size_t n_v = mesh->get_vertices().size(), n_t = mesh->get_tets().size();
    const auto frame = [&](const char* name) {
        return mesh->gather_vtu_frame(out.string() + name);
    };
    const auto change = [&]() {
        for (const auto& v : mesh->get_vertices()) {
            mesh->m_vertex_attribute[v.vid(*mesh)].m_sizing_scalar = 0.3;
        }
        mesh->local_operations({{1, 0, 0, 0}});
        mesh->consolidate_mesh();
    };
    const TopoOffsetTetMesh::VtuFrame before = frame("_before");
    change();
    INFO(n_t << " tets before the split pass, " << mesh->get_tets().size() << " after");
    REQUIRE(mesh->get_tets().size() > n_t);
    const TopoOffsetTetMesh::VtuFrame changed = frame("_changed");
    CHECK_FALSE(frames_identical(before, changed)); // the comparison can fail

    mesh->restore_snapshot(snap);
    CHECK(mesh->check_mesh_connectivity_validity());
    CHECK(mesh->get_vertices().size() == n_v);
    CHECK(mesh->get_tets().size() == n_t);
    CHECK(frames_identical(before, frame("_restored")));
    change();
    CHECK(frames_identical(changed, frame("_changed_again")));
}

TEST_CASE("folded-faces-at-vertex", "[offset][3d]")
{
    // growth_mode "individual_op_reject" counts folds per operation with folded_front_faces_at(),
    // which reads one vertex's own tet list (lock-safe in the operation passes). Over every vertex
    // it must find exactly the folded faces fold_census() finds over the whole offset surface
    // (offset_surface_faces()). The grid mesh with one front vertex dragged to the input's centre,
    // which turns its offset faces back toward the input: folds to find.
    const GridInput g = grid_input();
    Parameters param = construction_params(
        {{"offset_selection", "tag_0"},
         {"num_threads", 0},
         {"target_distance", 0.2},
         {"EXPERIMENTAL_band_volume_rule", "exact"}},
        g.V);
    const std::filesystem::path out =
        std::filesystem::temp_directory_path() / "wmtk_offset_folded_faces";
    const auto mesh = grid_offset_mesh(g, param, out);
    CHECK_FALSE(mesh->fold_reject_active()); // growth_mode "none"
    const auto census_set = [&]() {
        std::set<std::array<size_t, 3>> s;
        for (const auto& f : mesh->offset_surface_faces()) {
            if (mesh->front_face_is_foldover(f[0], f[1], f[2])) {
                auto k = f;
                std::sort(k.begin(), k.end());
                s.insert(k);
            }
        }
        return s;
    };
    const auto per_vertex_set = [&]() {
        std::set<std::array<size_t, 3>> s;
        for (const auto& v : mesh->get_vertices()) mesh->folded_front_faces_at(v.vid(*mesh), s);
        return s;
    };
    CHECK(census_set().empty()); // as constructed
    CHECK(per_vertex_set().empty());
    size_t front = static_cast<size_t>(-1);
    for (const auto& v : mesh->get_vertices()) {
        if (mesh->m_vertex_extra[v.vid(*mesh)].m_is_on_offset) {
            front = v.vid(*mesh);
            break;
        }
    }
    REQUIRE(front != static_cast<size_t>(-1));
    mesh->set_vertex_position(front, Vector3d::Zero());
    const auto census = census_set();
    INFO(census.size() << " folded faces");
    REQUIRE(!census.empty());
    CHECK(per_vertex_set() == census);
    CHECK(mesh->fold_census().n_fold == census.size());
    // Every folded face found through the moved vertex goes through it.
    std::set<std::array<size_t, 3>> at;
    mesh->folded_front_faces_at(front, at);
    CHECK(!at.empty());
    for (const auto& f : at) CHECK(std::find(f.begin(), f.end(), front) != f.end());

    // The rule is on in the loops of growth_mode "individual_op_reject" only.
    mesh->m_offset_params.growth_mode = "individual_op_reject";
    CHECK(mesh->fold_reject_active());
    mesh->m_tetwild_pass = true;
    CHECK_FALSE(mesh->fold_reject_active());
    mesh->m_tetwild_pass = false;
}

TEST_CASE("save-offset-correspondence-2d", "[offset][2d][correspondence]")
{
    // The 2D twin: a 3 x 3 grid of unit squares, two triangles each along the same diagonal, the
    // center square as tag_0, construction only. Every outer end is a unit away, so every marched
    // edge is traced to the target 0.2.
    Eigen::MatrixXd V(16, 2);
    const auto vid = [](int i, int j) { return i + 4 * j; };
    for (int j = 0; j < 4; ++j) {
        for (int i = 0; i < 4; ++i) V.row(vid(i, j)) << i - 1.5, j - 1.5;
    }
    std::vector<std::array<int, 3>> F;
    std::vector<int> center;
    for (int j = 0; j < 3; ++j) {
        for (int i = 0; i < 3; ++i) {
            if (i == 1 && j == 1) center.push_back(int(F.size()));
            F.push_back({{vid(i, j), vid(i + 1, j), vid(i + 1, j + 1)}}); // counter-clockwise
            if (i == 1 && j == 1) center.push_back(int(F.size()));
            F.push_back({{vid(i, j), vid(i + 1, j + 1), vid(i, j + 1)}});
        }
    }
    Eigen::MatrixXi Fm(int(F.size()), 3);
    for (size_t i = 0; i < F.size(); ++i) {
        for (int j = 0; j < 3; ++j) Fm(int(i), j) = F[i][size_t(j)];
    }
    MatrixSi Tags(int(F.size()), 1);
    for (const int f : center) Tags.coeffRef(f, 0) = 1;

    // The steps the 2D branch of topological_offset() runs up to the construction.
    Parameters param = construction_params(correspondence_json(), V);
    TopoOffsetTriMesh mesh(param, 0);
    MatrixXd V_env;
    MatrixXi F_env;
    mesh.init_from_image(V, Fm, Tags, V_env, F_env, {"tag_0"});
    mesh.m_offset_params.offset_selection = parse("tag_0", mesh.m_tag_name_to_id);
    mesh.label_input_complex();
    mesh.init_input_complex_bvh();
    mesh.consolidate_mesh();
    const auto P = marched_edges(mesh);
    const size_t n_before = mesh.get_vertices().size();
    REQUIRE(!P.empty());

    const std::filesystem::path out =
        std::filesystem::temp_directory_path() / "wmtk_offset_correspondence_2d";
    mesh.construct_offset(out);
    REQUIRE(mesh.m_marching_midpoint_splits == 0);
    mesh.write_msh_groups(out.string());
    check_correspondence_msh(out.string() + ".msh", mesh, P, n_before, 2, [&](const Vector3d& p) {
        REQUIRE(p.z() == 0.);
        return mesh.m_input_complex_bvh->dist(Vector2d(p.x(), p.y()));
    });
    std::filesystem::remove(out.string() + ".msh");
}

TEST_CASE("damping-factor-key", "[offset][3d]")
{
    // EXPERIMENTAL_damping_factor through the spec, as the app reads it: the default 0.5 (half
    // the shortest edge, EXPERIMENTAL_damping's bound before the key existed), a value read
    // through, and the two refusals -- a factor that is not > 0, and a factor other than the
    // default with EXPERIMENTAL_damping false.
    const auto spec = jse::embed::wmtk_topological_offset_spec::topological_offset_spec::spec();
    const auto injected = [&](nlohmann::json j) {
        j["application"] = "topological_offset";
        j["input"] = "unused.msh";
        jse::JSE spec_engine;
        spec_engine.strict = true;
        REQUIRE(spec_engine.verify_json(j, spec));
        return spec_engine.inject_defaults(j, spec);
    };
    const Parameters by_default(injected(nlohmann::json::object()));
    CHECK_FALSE(by_default.damping);
    CHECK(by_default.damping_factor == 0.5);
    const Parameters set(
        injected({{"EXPERIMENTAL_damping", true}, {"EXPERIMENTAL_damping_factor", 0.1}}));
    CHECK(set.damping);
    CHECK(set.damping_factor == 0.1);
    const nlohmann::json without_damping = injected({{"EXPERIMENTAL_damping_factor", 0.1}});
    CHECK_THROWS(Parameters{without_damping});
    const nlohmann::json zero =
        injected({{"EXPERIMENTAL_damping", true}, {"EXPERIMENTAL_damping_factor", 0.}});
    CHECK_THROWS(Parameters{zero});
}

TEST_CASE("growth-mode-keys", "[offset][3d]")
{
    // growth_mode, linear_growth_steps, smart_growth_attempts and smart_growth_max_fails through
    // the spec, as the app reads them: the defaults (none, 4, 2, 10), values read through, and the
    // refusals -- an unknown mode, a mode's key at a non-default value under another mode, growth
    // with construction_mode "midpoint_fallback" or with init_optimize, counts below 1 (the
    // spec's min), and the two keys growth_mode replaced. Then linear's stage targets.
    const auto spec = jse::embed::wmtk_topological_offset_spec::topological_offset_spec::spec();
    const auto with_app = [](nlohmann::json j) {
        j["application"] = "topological_offset";
        j["input"] = "unused.msh";
        return j;
    };
    const auto injected = [&](const nlohmann::json& j) {
        jse::JSE spec_engine;
        spec_engine.strict = true;
        REQUIRE(spec_engine.verify_json(with_app(j), spec));
        return spec_engine.inject_defaults(with_app(j), spec);
    };
    const auto refused_by_spec = [&](const nlohmann::json& j) {
        jse::JSE spec_engine;
        spec_engine.strict = true;
        return !spec_engine.verify_json(with_app(j), spec);
    };
    const Parameters by_default(injected(nlohmann::json::object()));
    CHECK(by_default.growth_mode == "none");
    CHECK(by_default.linear_growth_steps == 4);
    CHECK(by_default.smart_growth_attempts == 2);
    CHECK(by_default.smart_growth_max_fails == 10);
    const Parameters linear(injected({{"growth_mode", "linear"}, {"linear_growth_steps", 1}}));
    CHECK(linear.growth_mode == "linear");
    CHECK(linear.linear_growth_steps == 1);
    const Parameters smart(injected(
        {{"growth_mode", "smart"}, {"smart_growth_attempts", 3}, {"smart_growth_max_fails", 5}}));
    CHECK(smart.growth_mode == "smart");
    CHECK(smart.smart_growth_attempts == 3);
    CHECK(smart.smart_growth_max_fails == 5);
    CHECK(
        Parameters(injected({{"growth_mode", "individual_op_reject"}})).growth_mode ==
        "individual_op_reject");
    CHECK_THROWS(
        Parameters{
            injected({{"growth_mode", "individual_op_reject"}, {"linear_growth_steps", 2}})});
    CHECK(refused_by_spec({{"growth_mode", "exponential"}}));
    CHECK_THROWS(Parameters{injected({{"linear_growth_steps", 2}})});
    CHECK_THROWS(Parameters{injected({{"growth_mode", "smart"}, {"linear_growth_steps", 2}})});
    CHECK_THROWS(Parameters{injected({{"smart_growth_attempts", 3}})});
    CHECK_THROWS(Parameters{injected({{"growth_mode", "linear"}, {"smart_growth_max_fails", 3}})});
    for (const char* mode : {"linear", "smart", "individual_op_reject"}) {
        INFO(mode);
        CHECK_THROWS(
            Parameters{
                injected({{"growth_mode", mode}, {"construction_mode", "midpoint_fallback"}})});
        CHECK_THROWS(Parameters{injected({{"growth_mode", mode}, {"init_optimize", true}})});
    }
    CHECK(refused_by_spec({{"growth_mode", "linear"}, {"linear_growth_steps", 0}}));
    CHECK(refused_by_spec({{"growth_mode", "smart"}, {"smart_growth_attempts", 0}}));
    CHECK(refused_by_spec({{"growth_mode", "smart"}, {"smart_growth_max_fails", 0}}));
    CHECK(refused_by_spec({{"EXPERIMENTAL_incremental_growth", true}}));
    CHECK(refused_by_spec({{"EXPERIMENTAL_incremental_growth_steps", 4}}));

    // Stage k of n at d0 + k (delta - d0) / n: d0 at k = 0, delta at k = n. With n = 1, two
    // loops: at d0, then at delta.
    CHECK(TopoOffsetTetMesh::growth_stage_target(0.3, 0.7, 0, 1) == 0.3);
    CHECK(TopoOffsetTetMesh::growth_stage_target(0.3, 0.7, 1, 1) == 0.7);
    const std::vector<double> quarters = {0.5, 1., 1.5, 2., 2.5};
    for (int k = 0; k <= 4; ++k) {
        CHECK(TopoOffsetTetMesh::growth_stage_target(0.5, 2.5, k, 4) == quarters[size_t(k)]);
    }
    // The last stage is delta itself, which the formula can miss by rounding (in IEEE double,
    // 0.1 + 3 x (0.9 - 0.1) / 3 = 0.9000000000000001); the stages increase.
    CHECK(TopoOffsetTetMesh::growth_stage_target(0.1, 0.9, 3, 3) == 0.9);
    for (int k = 0; k < 7; ++k) {
        CHECK(
            TopoOffsetTetMesh::growth_stage_target(0.1, 0.9, k, 7) <
            TopoOffsetTetMesh::growth_stage_target(0.1, 0.9, k + 1, 7));
    }
}

TEST_CASE("optimize-construction-key", "[offset][3d]")
{
    // optimize_construction through the spec, as the app reads it: on by default, false read
    // through, and accepted with optimize_offset false (the pass is then never reached). The keys
    // it replaced, and the features removed with them (EXPERIMENTAL_amips_pow,
    // EXPERIMENTAL_laplacian, EXPERIMENTAL_initial_smoothing_passes), are refused by the strict
    // spec.
    const auto spec = jse::embed::wmtk_topological_offset_spec::topological_offset_spec::spec();
    const auto base = [](nlohmann::json j) {
        j["application"] = "topological_offset";
        j["input"] = "unused.msh";
        return j;
    };
    const auto injected = [&](nlohmann::json j) {
        jse::JSE spec_engine;
        spec_engine.strict = true;
        REQUIRE(spec_engine.verify_json(base(j), spec));
        return spec_engine.inject_defaults(base(j), spec);
    };
    CHECK(Parameters(injected(nlohmann::json::object())).optimize_construction);
    CHECK_FALSE(Parameters(injected({{"optimize_construction", false}})).optimize_construction);
    CHECK_NOTHROW(Parameters(injected({{"optimize_offset", false}})));
    for (const char* removed :
         {"EXPERIMENTAL_construction_tetwild_run",
          "EXPERIMENTAL_refine_construction",
          "EXPERIMENTAL_amips_pow"}) {
        INFO(removed);
        jse::JSE spec_engine;
        spec_engine.strict = true;
        CHECK_FALSE(spec_engine.verify_json(base({{removed, true}}), spec));
    }
    for (const char* removed :
         {"EXPERIMENTAL_refine_construction_factor",
          "EXPERIMENTAL_laplacian",
          "EXPERIMENTAL_initial_smoothing_passes"}) {
        INFO(removed);
        jse::JSE spec_engine;
        spec_engine.strict = true;
        CHECK_FALSE(spec_engine.verify_json(base({{removed, 1.}}), spec));
    }
}

TEST_CASE("front-conv-rel-of-target", "[offset][3d]")
{
    // front_conv_rel is a fraction of target_distance since 2026-10-21 (Parameters::init()): the
    // default 0.025, a value read through, an absolute front_conv with its fraction derived, and
    // the refusal of a bar coarser than the offset. The box's diagonal is 13, so a bar read
    // against it would come out 26 times the target-relative one here.
    Eigen::MatrixXd V(2, 3);
    V << 0, 0, 0, 3, 4, 12;
    const Parameters by_default = construction_params({{"target_distance", 0.5}}, V);
    CHECK(by_default.front_conv_rel == 0.025);
    CHECK(by_default.front_conv == 0.025 * 0.5);
    const Parameters rel =
        construction_params({{"target_distance_rel", 0.02}, {"front_conv_rel", 0.1}}, V);
    CHECK(rel.target_distance == Catch::Approx(0.26));
    CHECK(rel.front_conv == 0.1 * rel.target_distance);
    const Parameters abs = construction_params({{"target_distance", 0.5}, {"front_conv", 0.05}}, V);
    CHECK(abs.front_conv == 0.05);
    CHECK(abs.front_conv_rel == Catch::Approx(0.1));
    CHECK_THROWS(construction_params({{"target_distance", 0.5}, {"front_conv_rel", 1.5}}, V));
}

TEST_CASE("band-side-dihedral", "[offset][3d]")
{
    // The debug frames' dihedral angle (band_side_dihedral()) and its foldover test
    // (dihedral_is_foldover()). Face 1 lies in z = 0 with its normal +z out of the band and runs
    // the shared edge along +y; face 2 turns about that edge by phi, its normal (sin phi, 0, cos
    // phi): phi > 0 bends the front away from the band (convex seen from the offset side), the
    // band-side angle 180 - phi; phi -> 180 folds face 2 flat onto face 1 from the offset side
    // (angle -> 0), phi -> -180 from the band side (-> 360).
    const Vector3d n1(0., 0., 1.), t(0., 1., 0.);
    const auto n2 = [](const double deg) {
        const double r = deg * M_PI / 180.;
        return Vector3d(std::sin(r), 0., std::cos(r));
    };
    const auto angle = [&](const double deg) {
        return TopoOffsetTetMesh::band_side_dihedral(n1, n2(deg), t);
    };
    CHECK(angle(0.) == Catch::Approx(180.));
    CHECK(angle(45.) == Catch::Approx(135.));
    CHECK(angle(-45.) == Catch::Approx(225.));
    CHECK(angle(170.) == Catch::Approx(10.));
    CHECK(angle(-170.) == Catch::Approx(350.));
    // Swapping the faces runs the edge the other way: the same angle.
    CHECK(TopoOffsetTetMesh::band_side_dihedral(n2(45.), n1, -t) == Catch::Approx(135.));
    // Within foldover_angle of a full fold, either way, is a foldover; a crease is not.
    const double th = 30.;
    CHECK(TopoOffsetTetMesh::dihedral_is_foldover(angle(170.), th));
    CHECK(TopoOffsetTetMesh::dihedral_is_foldover(angle(-170.), th));
    CHECK_FALSE(TopoOffsetTetMesh::dihedral_is_foldover(angle(90.), th)); // a 90-degree crease
    CHECK_FALSE(TopoOffsetTetMesh::dihedral_is_foldover(angle(-90.), th));
    CHECK_FALSE(TopoOffsetTetMesh::dihedral_is_foldover(angle(0.), th));
    CHECK_FALSE(TopoOffsetTetMesh::dihedral_is_foldover(angle(149.), th)); // 31 degrees left
    CHECK(TopoOffsetTetMesh::dihedral_is_foldover(angle(151.), th)); // 29 degrees left
}

TEST_CASE("smart-growth-schedule", "[offset][3d]")
{
    // growth_mode "smart"'s targets and decisions (TopoOffsetTetMesh::SmartGrowthSchedule), apart
    // from the mesh: from base b the j-th try after j failures is at b + (delta - b) / 2^j, delta
    // itself first; a success below delta becomes the new base and the next try is at delta
    // again; max_fails failures from one base give up; a success at delta is the end.
    using Schedule = TopoOffsetTetMesh::SmartGrowthSchedule;
    using Next = Schedule::Next;
    Schedule s{1., 9., 3};
    CHECK(s.target() == 9.);
    CHECK(s.report(false) == Next::Retry); // 9 failed
    CHECK(s.target() == 5.); // 1 + 8 / 2
    CHECK(s.report(false) == Next::Retry); // 5 failed
    CHECK(s.target() == 3.); // 1 + 8 / 4
    CHECK(s.report(true) == Next::Advance); // 3 converged: the new base
    CHECK(s.base == 3.);
    CHECK(s.level == 2);
    CHECK(s.target() == 9.); // delta again
    CHECK(s.report(false) == Next::Retry);
    CHECK(s.target() == 6.); // 3 + 6 / 2
    CHECK(s.report(true) == Next::Advance);
    CHECK(s.target() == 9.);
    CHECK(s.report(true) == Next::Done);

    // The count is per base, and the max_fails-th failure gives up.
    Schedule g{0., 1., 3};
    CHECK(g.report(false) == Next::Retry);
    CHECK(g.report(false) == Next::Retry);
    CHECK(g.report(false) == Next::GiveUp);
    Schedule h{0., 1., 2};
    CHECK(h.report(false) == Next::Retry);
    CHECK(h.report(true) == Next::Advance); // at 0.5
    CHECK(h.report(false) == Next::Retry); // the count started again
    CHECK(h.report(false) == Next::GiveUp);
    CHECK(Schedule{0., 1., 1}.report(false) == Next::GiveUp);
}

TEST_CASE("foldover-face-angle", "[offset][3d]")
{
    // foldover_angle through the spec (default 30, range 0..180), and front_face_field_angle():
    // the angle between a front face's normal, out of its band cell, and grad d at its centroid.
    // One band tet, the unit tet, over an input plane x + y + z = -1, where grad d =
    // (1, 1, 1) / sqrt(3): the face opposite vertex 0 faces along the field (0 degrees), each face
    // through vertex 0 faces away from it, acos(-1 / sqrt(3)) = 125.26 degrees.
    const auto spec = jse::embed::wmtk_topological_offset_spec::topological_offset_spec::spec();
    const auto with_app = [](nlohmann::json j) {
        j["application"] = "topological_offset";
        j["input"] = "unused.msh";
        return j;
    };
    const auto injected = [&](const nlohmann::json& j) {
        jse::JSE spec_engine;
        spec_engine.strict = true;
        REQUIRE(spec_engine.verify_json(with_app(j), spec));
        return spec_engine.inject_defaults(with_app(j), spec);
    };
    CHECK(Parameters(injected(nlohmann::json::object())).foldover_angle == 30.);
    // offset_envelope / offset_envelope_rel are gone (2026-10-20): the offset surface's envelope
    // is envelope_size's; the strict spec refuses a config that still sets either.
    for (const char* gone : {"offset_envelope", "offset_envelope_rel"}) {
        jse::JSE spec_engine;
        spec_engine.strict = true;
        CHECK_FALSE(spec_engine.verify_json(with_app({{gone, 1e-3}}), spec));
    }
    CHECK(Parameters(injected({{"foldover_angle", 45.}})).foldover_angle == 45.);
    for (const double bad : {-1., 181.}) {
        jse::JSE spec_engine;
        spec_engine.strict = true;
        CHECK_FALSE(spec_engine.verify_json(with_app({{"foldover_angle", bad}}), spec));
    }

    Eigen::Matrix<double, Eigen::Dynamic, 3> V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::MatrixXi T(1, 4);
    T << 0, 1, 2, 3;
    MatrixSi Tags(1, 3);
    std::vector<std::string> tag_names = {"a", "b", "c"};
    for (int i = 0; i < 3; i++) Tags.coeffRef(0, i) = T0_TAGS.count(tag_names[i]) ? 1 : 0;
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;
    Parameters param;
    TopoOffsetTetMesh mesh(param, 0);
    mesh.init_from_image(V, T, Tags, V_env_dummy, F_env_dummy, tag_names);
    // Before an input envelope exists nothing is measurable.
    CHECK(std::isnan(mesh.front_face_field_angle(1, 2, 3)));
    auto env = std::make_shared<Envelope>(true);
    env->init(
        std::vector<Eigen::Vector3d>{{-21., 10., 10.}, {10., -21., 10.}, {10., 10., -21.}},
        std::vector<Eigen::Vector3i>{{0, 1, 2}},
        0.1);
    mesh.m_input_complex_envelope = env;
    mesh.m_input_complex_pad = 1e-9;
    // Not a band cell: no side to orient by.
    mesh.m_tet_attribute[0].label = 0;
    CHECK(std::isnan(mesh.front_face_field_angle(1, 2, 3)));
    mesh.m_tet_attribute[0].label = 2;
    const double away = std::acos(-1. / std::sqrt(3.)) * 180. / M_PI;
    CHECK(mesh.front_face_field_angle(1, 2, 3) == Catch::Approx(0.).margin(1e-6));
    CHECK(mesh.front_face_field_angle(3, 2, 1) == Catch::Approx(0.).margin(1e-6));
    CHECK(mesh.front_face_field_angle(0, 1, 2) == Catch::Approx(away).epsilon(1e-9));
    CHECK(mesh.front_face_field_angle(0, 1, 3) == Catch::Approx(away).epsilon(1e-9));
    CHECK_FALSE(mesh.front_face_is_foldover(1, 2, 3));
    CHECK(mesh.front_face_is_foldover(0, 1, 2));
    mesh.m_offset_params.foldover_angle = 130.;
    CHECK_FALSE(mesh.front_face_is_foldover(0, 1, 2));
}

TEST_CASE("debug-remote-output-key", "[offset][3d]")
{
    // DEBUG_remote_output: off by default; true overrides DEBUG_output (sets it), so every debug
    // computation runs while the frames follow the remote schedule.
    const auto spec = jse::embed::wmtk_topological_offset_spec::topological_offset_spec::spec();
    const auto injected = [&](nlohmann::json j) {
        j["application"] = "topological_offset";
        j["input"] = "unused.msh";
        jse::JSE spec_engine;
        spec_engine.strict = true;
        REQUIRE(spec_engine.verify_json(j, spec));
        return spec_engine.inject_defaults(j, spec);
    };
    const Parameters by_default(injected(nlohmann::json::object()));
    CHECK_FALSE(by_default.debug_remote_output);
    CHECK_FALSE(by_default.debug_output);
    const Parameters remote(injected({{"DEBUG_remote_output", true}, {"DEBUG_output", false}}));
    CHECK(remote.debug_remote_output);
    CHECK(remote.debug_output);
}

TEST_CASE("op-rules-switch", "[offset][3d]")
{
    // m_op_rules: in the loop the engine's AMIPS^3 rules admit everything, the smoother gets the
    // component's energy and the projected step the ring's sum of tet_energy(); under TetWild's
    // rules (every TetWild pass) the base's own: q <= ring_max for a collapse, after < before for
    // a swap, no extra energy (the raw AMIPS sum), the base's ring measure. One unit tet.
    Eigen::Matrix<double, Eigen::Dynamic, 3> V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::MatrixXi T(1, 4);
    T << 0, 1, 2, 3;
    MatrixSi Tags(1, 3);
    std::vector<std::string> tag_names = {"a", "b", "c"};
    for (int i = 0; i < 3; i++) Tags.coeffRef(0, i) = T0_TAGS.count(tag_names[i]) ? 1 : 0;
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;
    Parameters param;
    TopoOffsetTetMesh mesh(param, 0);
    mesh.init_from_image(V, T, Tags, V_env_dummy, F_env_dummy, tag_names);
    REQUIRE(mesh.m_vertex_attribute[0].m_is_rounded);
    const std::vector<TetMesh::Tuple> ring =
        mesh.get_one_ring_tets_for_vertex(mesh.tuple_from_vertex(0));
    for (const auto& t : ring) mesh.set_cell_quality(t.tid(mesh), mesh.get_quality(t));

    REQUIRE(mesh.m_op_rules == TopoOffsetTetMesh::OpRules::Loop);
    CHECK(mesh.collapse_quality_allowed(0, 2., 1.));
    CHECK(mesh.swap_quality_allowed(2., 1., false));
    CHECK(mesh.smoothing_extra_energy(0) != nullptr);
    CHECK(mesh.smoothing_ring_energy(ring, true) == mesh.tet_energy(ring[0].tid(mesh)));
    CHECK_FALSE(mesh.optimization_bare_coarsen_passes());

    mesh.m_op_rules = TopoOffsetTetMesh::OpRules::TetWild;
    CHECK_FALSE(mesh.collapse_quality_allowed(0, 2., 1.));
    CHECK(mesh.collapse_quality_allowed(0, 1., 2.));
    CHECK_FALSE(mesh.swap_quality_allowed(2., 1., false));
    CHECK_FALSE(mesh.swap_quality_allowed(1., 1., false));
    CHECK(mesh.swap_quality_allowed(1., 2., false));
    CHECK(mesh.smoothing_extra_energy(0) == nullptr);
    CHECK(mesh.optimization_bare_coarsen_passes());
    CHECK(
        mesh.smoothing_ring_energy(ring, true) ==
        mesh.TetOptimizerMesh::smoothing_ring_energy(ring, true));
}

TEST_CASE("damping-loop-only", "[offset][3d]")
{
    // EXPERIMENTAL_damping bounds the loop's smoothing solves only: damping_step() is
    // damping_factor x the shortest edge at the vertex there, and infinity in a TetWild pass
    // (m_tetwild_pass: the optimize_construction pass, the final pass) and
    // with damping off. The unit tet: every edge at vertex 0 has length 1.
    Eigen::Matrix<double, Eigen::Dynamic, 3> V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::MatrixXi T(1, 4);
    T << 0, 1, 2, 3;
    MatrixSi Tags(1, 3);
    std::vector<std::string> tag_names = {"a", "b", "c"};
    for (int i = 0; i < 3; i++) Tags.coeffRef(0, i) = T0_TAGS.count(tag_names[i]) ? 1 : 0;
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;
    const double inf = std::numeric_limits<double>::infinity();
    {
        Parameters param;
        param.damping = true;
        param.damping_factor = 0.25;
        TopoOffsetTetMesh mesh(param, 0);
        mesh.init_from_image(V, T, Tags, V_env_dummy, F_env_dummy, tag_names);
        REQUIRE(mesh.damping_active());
        CHECK(mesh.damping_step(0) == 0.25);
        mesh.m_tetwild_pass = true;
        CHECK_FALSE(mesh.damping_active());
        CHECK(mesh.damping_step(0) == inf);
        mesh.m_tetwild_pass = false;
        CHECK(mesh.damping_step(0) == 0.25);
    }
    {
        Parameters param;
        TopoOffsetTetMesh mesh(param, 0);
        mesh.init_from_image(V, T, Tags, V_env_dummy, F_env_dummy, tag_names);
        CHECK_FALSE(mesh.damping_active());
        CHECK(mesh.damping_step(0) == inf);
    }
}

TEST_CASE("construction-splits-carry-sizing", "[split_op][3d]")
{
    // The construction's own splits (the simplicial embedding's, and the march's under
    // EdgeSplitMode::Midpoint) give the new vertex the sizing field's value where it is: the mean
    // of its edge's, face's or tet's scalars, as the engine's split does for an edge. So the field
    // set before the march reaches the loop. Powers of two keep every sum exact in any
    // order; with every scalar 1 the new one is exactly 1.
    Eigen::Matrix<double, Eigen::Dynamic, 3> V(4, 3);
    V << 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
    Eigen::MatrixXi T(1, 4);
    T << 0, 1, 2, 3;
    MatrixSi Tags(1, 3);
    std::vector<std::string> tag_names = {"a", "b", "c"};
    for (int i = 0; i < 3; i++) Tags.coeffRef(0, i) = T0_TAGS.count(tag_names[i]) ? 1 : 0;
    Parameters param;
    TopoOffsetTetMesh mesh(param, 0);
    MatrixXd V_env_dummy;
    MatrixXi F_env_dummy;
    mesh.init_from_image(V, T, Tags, V_env_dummy, F_env_dummy, tag_names);
    REQUIRE(mesh.m_edge_split_mode == TopoOffsetTetMesh::EdgeSplitMode::Midpoint);
    const std::array<double, 4> s = {{0.125, 0.25, 0.5, 1.}};
    std::vector<TetMesh::Tuple> garbage;
    // The one vertex the split added (the mesh had 0..3).
    const auto new_scalar = [&]() {
        std::vector<size_t> added;
        for (const TetMesh::Tuple& v : mesh.get_vertices()) {
            if (v.vid(mesh) > 3) added.push_back(v.vid(mesh));
        }
        REQUIRE(added.size() == 1);
        return mesh.m_vertex_attribute[added[0]].m_sizing_scalar;
    };
    SECTION("every scalar 1")
    {
        REQUIRE(mesh.split_edge(mesh.tuple_from_edge({{0, 1}}), garbage));
        CHECK(new_scalar() == 1.);
    }
    for (size_t v = 0; v < 4; ++v) mesh.m_vertex_attribute[v].m_sizing_scalar = s[v];
    SECTION("edge")
    {
        REQUIRE(mesh.split_edge(mesh.tuple_from_edge({{0, 1}}), garbage));
        CHECK(new_scalar() == (s[0] + s[1]) / 2);
    }
    SECTION("face")
    {
        REQUIRE(mesh.split_face(std::get<0>(mesh.tuple_from_face({{0, 1, 2}})), garbage));
        CHECK(new_scalar() == (s[0] + s[1] + s[2]) / 3);
    }
    SECTION("tet")
    {
        REQUIRE(mesh.split_tet(mesh.tuple_from_tet(0), garbage));
        CHECK(new_scalar() == (s[0] + s[1] + s[2] + s[3]) / 4);
    }
}

namespace {

/// The pocket-split fixtures (TopoOffsetTetMesh::front_pocket_split()). The corner cell
/// P = (o, x, y, z) -- o the origin, x, y, z the unit points on the axes -- is background
/// (label 0) and tet 0. On the face of P in the plane where coordinate a is 0, for each axis a in
/// `axes`, sits a band blob: the tet on that face with its apex u 0.5 beyond the face's centroid,
/// coned from its own centroid e into four band cells (label 2). A blob lies where coordinate a
/// is <= 0 and the other two are >= 0, so no two cells overlap, and every blob cell has e, which
/// is on no offset face, as a corner. The offset surface (face_is_offset_surface_live()) is each
/// blob's face against P and its three faces on the domain boundary. Vertex ids: o 0, x 1, y 2,
/// z 3, then u and e of each blob in turn. The front flags are derived from the labels
/// (refresh_offset_membership()), and every split is the loop's (EdgeSplitMode::Optimization).
/// `cone`: P is instead coned from its centroid c (vertex 4, before the blobs) into four cells, so
/// no cell of P is pinned while P's edges stay where they were.
std::unique_ptr<TopoOffsetTetMesh>
pocket_mesh(Parameters& param, const std::vector<int>& axes, const bool cone = false)
{
    std::vector<Vector3d> pos = {
        Vector3d(0., 0., 0.),
        Vector3d(1., 0., 0.),
        Vector3d(0., 1., 0.),
        Vector3d(0., 0., 1.)};
    std::vector<std::array<int, 4>> T = {{{0, 1, 2, 3}}};
    std::vector<int> labels = {0};
    if (cone) {
        pos.push_back(Vector3d(0.25, 0.25, 0.25));
        T = {{{0, 1, 2, 4}}, {{0, 1, 4, 3}}, {{0, 4, 2, 3}}, {{4, 1, 2, 3}}};
        labels = {0, 0, 0, 0};
    }
    for (const int a : axes) {
        // The face of P in the plane coordinate a = 0: every corner but the one on axis a.
        std::array<int, 3> f{};
        int n = 0;
        for (int i = 0; i < 4; ++i) {
            if (i != a + 1) f[size_t(n++)] = i;
        }
        const Vector3d& p0 = pos[size_t(f[0])];
        const Vector3d& p1 = pos[size_t(f[1])];
        const Vector3d& p2 = pos[size_t(f[2])];
        Vector3d u = (p0 + p1 + p2) / 3.;
        u[a] = -0.5;
        const Vector3d e = (p0 + p1 + p2 + u) / 4.;
        const int iu = int(pos.size()), ie = iu + 1;
        pos.push_back(u);
        pos.push_back(e);
        T.push_back({{f[0], f[1], f[2], ie}});
        T.push_back({{f[0], f[1], iu, ie}});
        T.push_back({{f[1], f[2], iu, ie}});
        T.push_back({{f[2], f[0], iu, ie}});
        labels.insert(labels.end(), 4, 2);
    }
    Eigen::MatrixXd V(int(pos.size()), 3);
    for (size_t i = 0; i < pos.size(); ++i) V.row(int(i)) = pos[i].transpose();
    auto mesh = energy_mesh(param, V, T, labels, plane_field());
    for (const TetMesh::Tuple& v : mesh->get_vertices()) {
        mesh->refresh_offset_membership(v.vid(*mesh));
    }
    mesh->m_edge_split_mode = TopoOffsetTetMesh::EdgeSplitMode::Optimization;
    return mesh;
}

/// The pinned cells, counted from the definition: outside the input complex, all four corners
/// front vertices (m_is_on_offset).
size_t count_pinned(const TopoOffsetTetMesh& m)
{
    size_t n = 0;
    for (const TetMesh::Tuple& t : m.get_tets()) {
        const size_t tid = t.tid(m);
        if (m.cell_is_input_complex(tid)) continue;
        const auto vs = m.oriented_tet_vids(tid);
        if (std::all_of(vs.begin(), vs.end(), [&](const size_t v) {
                return m.m_vertex_extra[v].m_is_on_offset;
            })) {
            ++n;
        }
    }
    return n;
}

/// The spanned edges (front_pocket_split()'s EXPERIMENTAL_front_spanned_split): both ends front
/// vertices, on no offset-surface face, no cell around them in the input complex.
size_t count_spanned_edges(const TopoOffsetTetMesh& m)
{
    size_t n = 0;
    for (const TetMesh::Tuple& e : m.get_edges()) {
        const size_t a = e.vid(m), b = e.switch_vertex(m).vid(m);
        if (!m.m_vertex_extra[a].m_is_on_offset || !m.m_vertex_extra[b].m_is_on_offset) continue;
        if (m.edge_is_offset_surface_live(a, b)) continue;
        const auto tids = m.get_incident_tids_for_edge(a, b);
        if (std::any_of(tids.begin(), tids.end(), [&](const size_t t) {
                return m.cell_is_input_complex(t);
            })) {
            continue;
        }
        ++n;
    }
    return n;
}

/// Whether every vertex's front flag is what the cell labels say (offset_membership_mismatches()).
bool front_flags_live(const TopoOffsetTetMesh& m)
{
    const auto mismatches = m.offset_membership_mismatches();
    return mismatches.first == 0 && mismatches.second == 0;
}

/// The live cells as sorted corner arrays, sorted: the connectivity, independent of tet ids.
std::vector<std::array<size_t, 4>> sorted_cells(const TopoOffsetTetMesh& m)
{
    std::vector<std::array<size_t, 4>> cells;
    for (const TetMesh::Tuple& t : m.get_tets()) {
        auto vs = m.oriented_tet_vids(t.tid(m));
        std::sort(vs.begin(), vs.end());
        cells.push_back(vs);
    }
    std::sort(cells.begin(), cells.end());
    return cells;
}

bool has_face(const std::vector<std::array<size_t, 3>>& faces, std::array<size_t, 3> f)
{
    std::sort(f.begin(), f.end());
    return std::find(faces.begin(), faces.end(), f) != faces.end();
}

} // namespace

TEST_CASE("pocket-split-pinned-cell", "[offset][3d][pocket_split]")
{
    // EXPERIMENTAL_front_pocket_split on one pinned cell with one edge off the offset surface: P
    // with blobs on its faces z = 0 and x = 0, so o, x, y are on the one offset face of P and z
    // on the other, and the edge (x, z) is on neither. The slot splits (x, z), whose only cell is
    // P; both children have the midpoint, off the offset surface, as a corner, so no pinned cell
    // is left, and the offset surface, none of whose faces has the split edge, is the same set of
    // faces.
    Parameters param;
    auto mesh = pocket_mesh(param, {2, 0});
    mesh->m_offset_params.front_pocket_split = true;
    // The premises: P is the one pinned cell, and (x, z) is its one edge off the offset surface.
    REQUIRE(count_pinned(*mesh) == 1);
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = i + 1; j < 4; ++j) {
            INFO("edge " << i << " " << j);
            REQUIRE(mesh->edge_is_offset_surface_live(i, j) == !(i == 1 && j == 3));
        }
    }
    REQUIRE(front_flags_live(*mesh));
    const auto faces0 = mesh->offset_surface_faces();
    const size_t n_cells0 = mesh->get_tets().size(), n_verts0 = mesh->get_vertices().size();

    mesh->front_pocket_split("test");

    CHECK(count_pinned(*mesh) == 0);
    CHECK(mesh->check_mesh_connectivity_validity());
    CHECK(front_flags_live(*mesh));
    CHECK(mesh->offset_surface_faces() == faces0);
    // One split, of (x, z): one vertex and one cell more, and the edge is gone.
    CHECK(mesh->get_vertices().size() == n_verts0 + 1);
    CHECK(mesh->get_tets().size() == n_cells0 + 1);
    CHECK(mesh->get_incident_tids_for_edge(1, 3).empty());
}

TEST_CASE("pocket-split-six-edge-cell", "[offset][3d][pocket_split]")
{
    // A pinned cell with all six edges on the offset surface: P with blobs on its three faces
    // through o, so (x, y, z) is its one face that is not an offset face. The slot splits an edge
    // (p, q) of (x, y, z) at m, on the offset surface: P's two children are pinned, each with the
    // edge (m, c) off it, c the face's third corner, and the next round splits (m, c) at
    // (p + q + 2c) / 4, a corner of all four of P's descendants. Two splits, no pinned cell left;
    // the offset surface is the same point set, its faces through (p, q) halved at m.
    Parameters param;
    auto mesh = pocket_mesh(param, {0, 1, 2});
    mesh->m_offset_params.front_pocket_split = true;
    // The premises: P is the one pinned cell, its six edges are on the offset surface, and so are
    // its faces but (x, y, z).
    REQUIRE(count_pinned(*mesh) == 1);
    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = i + 1; j < 4; ++j) REQUIRE(mesh->edge_is_offset_surface_live(i, j));
    }
    REQUIRE(front_flags_live(*mesh));
    const auto faces0 = mesh->offset_surface_faces();
    REQUIRE_FALSE(has_face(faces0, {{1, 2, 3}}));
    REQUIRE(has_face(faces0, {{0, 1, 2}}));
    REQUIRE(has_face(faces0, {{0, 1, 3}}));
    REQUIRE(has_face(faces0, {{0, 2, 3}}));
    const size_t n_cells0 = mesh->get_tets().size(), n_verts0 = mesh->get_vertices().size();
    std::vector<Vector3d> pos0;
    for (size_t v = 0; v < n_verts0; ++v) pos0.push_back(mesh->m_vertex_attribute[v].m_posf);

    mesh->front_pocket_split("test");

    CHECK(count_pinned(*mesh) == 0);
    CHECK(mesh->check_mesh_connectivity_validity());
    CHECK(front_flags_live(*mesh));
    // Two splits: m on the offset surface, n off it. (p, q) is an edge of P and of two cells of
    // the blob on (o, p, q); (m, c) of P's two children alone: five cells more.
    std::vector<size_t> added;
    for (const TetMesh::Tuple& v : mesh->get_vertices()) {
        if (v.vid(*mesh) >= n_verts0) added.push_back(v.vid(*mesh));
    }
    REQUIRE(added.size() == 2);
    CHECK(mesh->get_tets().size() == n_cells0 + 5);
    size_t m = added[0], n = added[1];
    if (!mesh->m_vertex_extra[m].m_is_on_offset) std::swap(m, n);
    REQUIRE(mesh->m_vertex_extra[m].m_is_on_offset);
    REQUIRE_FALSE(mesh->m_vertex_extra[n].m_is_on_offset);
    // m is the midpoint of an edge (p, q) of (x, y, z), and n is (p + q + 2c) / 4: the corner
    // coordinates are 0 and 1, so both are exact.
    const Vector3d xm = mesh->m_vertex_attribute[m].m_posf;
    const Vector3d xn = mesh->m_vertex_attribute[n].m_posf;
    size_t p = 0, q = 0, c = 0;
    for (size_t i = 1; i < 4; ++i) {
        for (size_t j = i + 1; j < 4; ++j) {
            if (xm == (pos0[i] + pos0[j]) / 2.) {
                p = i;
                q = j;
                c = 6 - i - j;
            }
        }
    }
    REQUIRE(p != 0);
    CHECK(xn == (pos0[p] + pos0[q] + 2. * pos0[c]) / 4.);
    // The offset surface as a point set: a face before without both p and q is still a face, one
    // with both is gone; a face now without m is a face before, and one with m is a half of a face
    // before through (p, q).
    const auto faces1 = mesh->offset_surface_faces();
    const auto holds = [](const std::array<size_t, 3>& f, const size_t v) {
        return std::find(f.begin(), f.end(), v) != f.end();
    };
    size_t through_pq = 0;
    for (const auto& f : faces0) {
        const bool pq = holds(f, p) && holds(f, q);
        if (pq) ++through_pq;
        CHECK(has_face(faces1, f) == !pq);
    }
    CHECK(through_pq == 2); // (o, p, q), and the blob's face on the domain boundary
    size_t with_m = 0;
    for (const auto& f : faces1) {
        if (!holds(f, m)) {
            CHECK(has_face(faces0, f));
            continue;
        }
        ++with_m;
        std::array<size_t, 3> to_p = f, to_q = f;
        std::replace(to_p.begin(), to_p.end(), m, p);
        std::replace(to_q.begin(), to_q.end(), m, q);
        CHECK(
            ((holds(to_p, q) && has_face(faces0, to_p)) ||
             (holds(to_q, p) && has_face(faces0, to_q))));
    }
    CHECK(with_m == 2 * through_pq);
}

TEST_CASE("pocket-split-key-off", "[offset][3d][pocket_split]")
{
    // With EXPERIMENTAL_front_pocket_split false (the default) the slot is a no-op, and so it is
    // in a TetWild pass with the key on: the first fixture's pinned cell stays and the mesh is
    // unchanged.
    Parameters param;
    auto mesh = pocket_mesh(param, {2, 0});
    REQUIRE_FALSE(mesh->m_offset_params.front_pocket_split);
    REQUIRE(count_pinned(*mesh) == 1);
    const auto cells0 = sorted_cells(*mesh);
    const auto faces0 = mesh->offset_surface_faces();
    const size_t n_verts0 = mesh->get_vertices().size();
    const auto unchanged = [&]() {
        CHECK(count_pinned(*mesh) == 1);
        CHECK(mesh->get_vertices().size() == n_verts0);
        CHECK(sorted_cells(*mesh) == cells0);
        CHECK(mesh->offset_surface_faces() == faces0);
    };

    mesh->front_pocket_split("test");
    unchanged();

    mesh->m_offset_params.front_pocket_split = true;
    mesh->m_tetwild_pass = true;
    mesh->front_pocket_split("test");
    unchanged();
    mesh->m_tetwild_pass = false;
}

TEST_CASE("spanned-split-edge", "[offset][3d][pocket_split]")
{
    // EXPERIMENTAL_front_spanned_split on a spanned edge with no pinned cell: the first fixture
    // with P coned from its centroid c, so (x, z) is still on no offset face while both its cells,
    // (o, x, z, c) and (x, y, z, c), have c off the front. The pinned-cell split alone leaves it;
    // with the key, the slot splits (x, z), and the offset surface is the same set of faces.
    Parameters param;
    auto mesh = pocket_mesh(param, {2, 0}, true);
    mesh->m_offset_params.front_pocket_split = true;
    REQUIRE(count_pinned(*mesh) == 0);
    REQUIRE_FALSE(mesh->edge_is_offset_surface_live(1, 3));
    REQUIRE(mesh->get_incident_tids_for_edge(1, 3).size() == 2);
    REQUIRE(count_spanned_edges(*mesh) == 1);
    REQUIRE(front_flags_live(*mesh));
    const auto faces0 = mesh->offset_surface_faces();
    const auto cells0 = sorted_cells(*mesh);
    const size_t n_cells0 = mesh->get_tets().size(), n_verts0 = mesh->get_vertices().size();

    mesh->front_pocket_split("test"); // key off: nothing pinned, nothing split
    CHECK(sorted_cells(*mesh) == cells0);
    CHECK(count_spanned_edges(*mesh) == 1);

    mesh->m_offset_params.front_spanned_split = true;
    mesh->front_pocket_split("test");

    CHECK(count_spanned_edges(*mesh) == 0);
    CHECK(count_pinned(*mesh) == 0);
    CHECK(mesh->check_mesh_connectivity_validity());
    CHECK(front_flags_live(*mesh));
    CHECK(mesh->offset_surface_faces() == faces0);
    // One split, of (x, z): one vertex, off the front, and two cells more; the edge is gone.
    CHECK(mesh->get_vertices().size() == n_verts0 + 1);
    CHECK(mesh->get_tets().size() == n_cells0 + 2);
    CHECK(mesh->get_incident_tids_for_edge(1, 3).empty());
    CHECK_FALSE(mesh->m_vertex_extra[n_verts0].m_is_on_offset);
}

TEST_CASE("surface-offset-error-3d", "[offset][3d]")
{
    // EXPERIMENTAL_band_volume_rule "surface": SurfaceOffsetError3D, a front vertex's part of the
    // face term, against its definition computed by hand -- W sum_f A_f mean over the interior
    // stencil of r^2, r = (d - delta) / delta -- and its exact gradient and Hessian against
    // central differences, at stencil orders 1, 2 and 5, on three faces around x placed above and
    // below the plane field's level set.
    const auto pot = plane_field();
    const Vector3d x(0.1, -0.05, 0.53);
    const std::vector<std::pair<Vector3d, Vector3d>> others = {
        {{0.4, 0.1, 0.61}, {0.2, 0.35, 0.44}},
        {{0.2, 0.35, 0.44}, {-0.25, 0.2, 0.58}},
        {{-0.25, 0.2, 0.58}, {-0.1, -0.4, 0.37}}};
    const double W = 0.7;
    for (const int k : {1, 2, 5}) {
        INFO("stencil_order " << k);
        std::vector<SurfaceOffsetError3D::Face> faces;
        double by_hand = 0.;
        const auto& st = interior_face_stencil(k);
        REQUIRE(int(st.size()) == k * (k + 1) / 2);
        for (const auto& [q1, q2] : others) {
            faces.push_back({q1, q2});
            double m = 0.;
            for (const auto& w : st) {
                const double r = pot->relative_residual(w[0] * x + w[1] * q1 + w[2] * q2);
                m += r * r;
            }
            by_hand += W * 0.5 * (q1 - x).cross(q2 - x).norm() * m / double(st.size());
        }
        SurfaceOffsetError3D e(pot, faces, k, W);
        Eigen::VectorXd xv = x;
        CHECK(by_hand > 0.);
        CHECK(e.value(xv) == Catch::Approx(by_hand).epsilon(1e-13));

        Eigen::VectorXd g;
        e.gradient(xv, g);
        Eigen::MatrixXd H;
        e.hessian(xv, H);
        const double h = 1e-6;
        for (int i = 0; i < 3; ++i) {
            Eigen::VectorXd xp = xv, xm = xv;
            xp[i] += h;
            xm[i] -= h;
            const double fd = (e.value(xp) - e.value(xm)) / (2. * h);
            CHECK(g[i] == Catch::Approx(fd).epsilon(1e-6).margin(1e-9));
            Eigen::VectorXd gp, gm;
            e.gradient(xp, gp);
            e.gradient(xm, gm);
            for (int j = 0; j < 3; ++j) {
                const double fdh = (gp[j] - gm[j]) / (2. * h);
                CHECK(H(j, i) == Catch::Approx(fdh).epsilon(1e-5).margin(1e-7));
            }
        }
    }
}

TEST_CASE("surface-energy-keys", "[offset][3d]")
{
    // EXPERIMENTAL_band_volume_rule "surface" through the spec, as the app reads it: the rule's
    // own offset_amips_weight default (0.5 under "surface", 1 otherwise), a weight read through,
    // the refusals (a weight > 1 under "surface", a negative weight other than -1,
    // settle_stall_eps away from its default under another rule), stencil_order's new default 5
    // and its min 1.
    const auto spec = jse::embed::wmtk_topological_offset_spec::topological_offset_spec::spec();
    const auto with_app = [](nlohmann::json j) {
        j["application"] = "topological_offset";
        j["input"] = "unused.msh";
        return j;
    };
    const auto injected = [&](const nlohmann::json& j) {
        jse::JSE spec_engine;
        spec_engine.strict = true;
        REQUIRE(spec_engine.verify_json(with_app(j), spec));
        return spec_engine.inject_defaults(with_app(j), spec);
    };
    const Parameters by_default(injected(nlohmann::json::object()));
    CHECK(by_default.band_volume_rule == "corner_bound");
    CHECK(by_default.offset_amips_weight == 1.);
    CHECK(by_default.stencil_order == 5);
    CHECK(by_default.settle_stall_eps == 0.1);
    const Parameters surface(injected({{"EXPERIMENTAL_band_volume_rule", "surface"}}));
    CHECK(surface.band_volume_rule == "surface");
    CHECK(surface.offset_amips_weight == 0.5);
    const Parameters weighted(injected(
        {{"EXPERIMENTAL_band_volume_rule", "surface"},
         {"offset_amips_weight", 0.25},
         {"settle_stall_eps", 0.05}}));
    CHECK(weighted.offset_amips_weight == 0.25);
    CHECK(weighted.settle_stall_eps == 0.05);
    const Parameters exact_w(injected({{"EXPERIMENTAL_band_volume_rule", "exact"}}));
    CHECK(exact_w.offset_amips_weight == 1.);
    CHECK_THROWS(
        Parameters{injected(
            {{"EXPERIMENTAL_band_volume_rule", "surface"}, {"offset_amips_weight", 1.5}})});
    CHECK_THROWS(Parameters{injected({{"offset_amips_weight", -0.5}})});
    CHECK_THROWS(Parameters{injected({{"settle_stall_eps", 0.2}})});
    jse::JSE strict;
    strict.strict = true;
    CHECK_FALSE(strict.verify_json(with_app({{"stencil_order", 0}}), spec));
    // EXPERIMENTAL_settle_test: "stall" by default, "bar" read through under "surface", refused
    // under another rule, and settle_stall_eps refused beside "bar".
    CHECK(by_default.settle_test == "stall");
    const Parameters bar(injected(
        {{"EXPERIMENTAL_band_volume_rule", "surface"}, {"EXPERIMENTAL_settle_test", "bar"}}));
    CHECK(bar.settle_test == "bar");
    CHECK_THROWS(Parameters{injected({{"EXPERIMENTAL_settle_test", "bar"}})});
    CHECK_THROWS(
        Parameters{injected(
            {{"EXPERIMENTAL_band_volume_rule", "surface"},
             {"EXPERIMENTAL_settle_test", "bar"},
             {"settle_stall_eps", 0.2}})});
}

TEST_CASE("surface-energy-sums", "[offset][3d]")
{
    // EXPERIMENTAL_band_volume_rule "surface" on the grid constructed at 0.2: no cell carries a
    // band term (tet_energy() is the AMIPS term of every cell), the face term is read the same in
    // any corner order, and the sums every rule and log read are the same faces' terms --
    // surface_term_total() over offset_surface_faces(), surface_term_at() over the faces through
    // a vertex, global_energy() and energy_parts() with the face term as the offset part -- and
    // the front smoother's objective (front_energy()) is surface_term_at() up to rounding.
    const GridInput g = grid_input();
    Parameters param = construction_params(
        {{"offset_selection", "tag_0"},
         {"num_threads", 0},
         {"target_distance", 0.2},
         {"EXPERIMENTAL_band_volume_rule", "surface"}},
        g.V);
    REQUIRE(param.offset_amips_weight == 0.5);
    const std::filesystem::path out =
        std::filesystem::temp_directory_path() / "wmtk_offset_surface_sums";
    const auto mesh = grid_offset_mesh(g, param, out);
    REQUIRE(mesh->surface_energy());
    const double f = mesh->m_offset_params.front_conv_frac();
    // (1 - w) L / front_conv_frac()^2, L the bounding box diagonal
    REQUIRE(mesh->m_offset_params.diag_l > 0.);
    CHECK(
        mesh->surface_term_weight() == Catch::Approx(0.5 * mesh->m_offset_params.diag_l / (f * f)));

    const auto faces = mesh->offset_surface_faces();
    REQUIRE(faces.size() > 10);
    double total = 0.;
    for (const auto& fc : faces) {
        const double t = mesh->surface_face_term(fc[0], fc[1], fc[2]);
        CHECK(t >= 0.);
        const double perm = mesh->surface_face_term(fc[2], fc[0], fc[1]);
        CHECK(std::memcmp(&t, &perm, sizeof(double)) == 0);
        total += t;
    }
    CHECK(total > 0.);
    CHECK(mesh->surface_term_total() == Catch::Approx(total).epsilon(1e-12));
    CHECK(mesh->offset_energy() == Catch::Approx(total).epsilon(1e-12));

    // O*, the settle test's metric: the max over every offset face's stencil points of
    // (d - delta)^2, by hand from the same stencil and the field's distance.
    double o_star = 0.;
    const auto& st = interior_face_stencil(mesh->m_offset_params.stencil_order);
    REQUIRE(st.size() == 15);
    for (const auto& fc : faces) {
        for (const auto& w : st) {
            const Vector3d q = w[0] * mesh->m_vertex_attribute[fc[0]].m_posf +
                               w[1] * mesh->m_vertex_attribute[fc[1]].m_posf +
                               w[2] * mesh->m_vertex_attribute[fc[2]].m_posf;
            // the euclidean field's value is d / delta
            const double e = mesh->m_offset_potential->value(q) * 0.2 - 0.2;
            o_star = std::max(o_star, e * e);
        }
    }
    CHECK(o_star > 0.);
    CHECK(mesh->stencil_max_sq_error() == Catch::Approx(o_star).epsilon(1e-10));
    // O_g, EXPERIMENTAL_settle_test "bar"'s metric: the face term without its weight, sum_f
    // (A_f / |S|) sum_k (d - delta)^2, by hand; and the settle metric of each test.
    double o_g = 0.;
    for (const auto& fc : faces) {
        const Vector3d& p0 = mesh->m_vertex_attribute[fc[0]].m_posf;
        const Vector3d& p1 = mesh->m_vertex_attribute[fc[1]].m_posf;
        const Vector3d& p2 = mesh->m_vertex_attribute[fc[2]].m_posf;
        double m = 0.;
        for (const auto& w : st) {
            const double e =
                mesh->m_offset_potential->value(w[0] * p0 + w[1] * p1 + w[2] * p2) * 0.2 - 0.2;
            m += e * e;
        }
        o_g += 0.5 * (p1 - p0).cross(p2 - p0).norm() * m / double(st.size());
    }
    CHECK(mesh->surface_integral_raw() == Catch::Approx(o_g).epsilon(1e-10));
    CHECK(mesh->settle_metric() == mesh->stencil_max_sq_error());
    mesh->m_offset_params.settle_test = "bar";
    CHECK(mesh->settle_metric() == mesh->surface_integral_raw());
    mesh->m_offset_params.settle_test = "stall";

    double amips = 0.;
    for (const auto& t : mesh->get_tets()) {
        const size_t tid = t.tid(*mesh);
        const double e = mesh->tet_energy(tid);
        if (mesh->cell_is_offset_band(tid)) {
            const auto vids = mesh->oriented_tet_vids(tid);
            CHECK(e == mesh->cell_energy(vids, mesh->get_quality(t), false));
        }
        amips += e;
    }
    CHECK(mesh->global_energy() == Catch::Approx(amips + total).epsilon(1e-12));
    const auto [a_part, o_part] = mesh->energy_parts();
    CHECK(a_part == Catch::Approx(amips).epsilon(1e-12));
    CHECK(o_part == Catch::Approx(total).epsilon(1e-12));

    size_t checked = 0;
    for (const auto& v : mesh->get_vertices()) {
        const size_t vid = v.vid(*mesh);
        if (!mesh->m_vertex_extra[vid].m_is_on_offset) continue;
        double by_faces = 0.;
        for (const auto& fc : faces) {
            if (fc[0] == vid || fc[1] == vid || fc[2] == vid) {
                by_faces += mesh->surface_face_term(fc[0], fc[1], fc[2]);
            }
        }
        CHECK(mesh->surface_term_at(vid) == Catch::Approx(by_faces).epsilon(1e-12));
        const auto obj = mesh->front_energy(vid);
        Eigen::VectorXd xv = mesh->m_vertex_attribute[vid].m_posf;
        CHECK(obj->value(xv) == Catch::Approx(by_faces).epsilon(1e-12));
        if (++checked >= 20) break;
    }
    CHECK(checked > 0);
}
