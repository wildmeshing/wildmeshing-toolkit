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
#include <wmtk/utils/SizingField.hpp>
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

/// A closed fan of front faces over the input triangle's interior (d = z there, g = +z): front
/// vertex 0 near the level set z = delta = 0.5 with ring 1-4; band cells below the front on
/// input-complex cells (so their lower faces are no offset surface), background cells above. The
/// only offset faces are the four (0, i, i+1), nearly horizontal, normals out of the band
/// within ~10 degrees of +z. Vertices 5 (below), 6 (above), 7 (bottom) are on no front face.
struct FanFixture
{
    Eigen::MatrixXd V;
    std::vector<std::array<int, 4>> T;
    std::vector<int> labels; ///< band 2, background 0, input complex 1
};
FanFixture fan_fixture()
{
    FanFixture f;
    f.V.resize(8, 3);
    f.V << 0.02, -0.01, 0.56, // 0: the front vertex
        0.4, 0., 0.5, // 1
        0., 0.4, 0.47, // 2
        -0.4, 0., 0.53, // 3
        0., -0.4, 0.5, // 4
        0., 0., 0.15, // 5: below the front, in the band
        0., 0., 1.0, // 6: above the front
        0., 0., -0.2; // 7: below the input-complex cells
    for (int i = 1; i <= 4; ++i) {
        const int j = i % 4 + 1;
        f.T.push_back({{0, i, j, 5}});
        f.labels.push_back(2);
        f.T.push_back({{0, i, j, 6}});
        f.labels.push_back(0);
        f.T.push_back({{i, j, 5, 7}});
        f.labels.push_back(1);
    }
    return f;
}

} // namespace

TEST_CASE("face-term-is-the-stencil-mean", "[offset][3d]")
{
    // face_offset_term(), the face measure the exit, the refinement and the frames read, is its
    // definition computed by hand from the stencil for_each_face_sample() visits, at stencil
    // orders 0 (corners only), 1 and 2, on three faces placed above and below the level set.
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
        double by_hand = 0., by_term = 0.;
        for (const auto& [q1, q2] : others) {
            by_hand += face_term_by_hand(mesh, *pot, x, q1, q2);
            by_term += mesh.face_offset_term(*pot, x, q1, q2);
        }
        INFO("stencil_order " << k);
        CHECK(by_hand > 10.); // the fixture really is off the level set
        CHECK(by_term == Catch::Approx(by_hand).epsilon(1e-12));
        // face_offset_mean_sq() is the same mean without the 1 / front_conv_frac()^2 scale:
        // mean of ((d - delta) / delta)^2 over the stencil.
        double by_err = 0.;
        for (const auto& [q1, q2] : others) {
            by_err += mesh.face_offset_mean_sq(*pot, x, q1, q2);
        }
        CHECK(by_err == Catch::Approx(by_hand * kEnergyFrac * kEnergyFrac).epsilon(1e-12));
    }
}

TEST_CASE("face-offset-error-3d", "[offset][3d]")
{
    // The offset error since 2026-10-22: per face its area times the mean of
    // ((d - delta) / delta)^2 over the 3 interior points (2/3, 1/6, 1/6) and permutations
    // (face_offset_error_of()). Above the input triangle's interior d = z is affine, so the
    // integrand is quadratic and the rule is EXACT: E_f = (1/delta^2) int_f (z - delta)^2 dA
    // = A/6 (L1^2 + L2^2 + L3^2 + L1 L2 + L2 L3 + L3 L1), L_i = (z_i - delta) / delta. And
    // FaceOffsetError3D, the smoother's term, sums face_offset_error_of() over the faces at the
    // moving corner x, with gradient and Hessian matching central differences, at several x above
    // and astride the level set.
    const auto pot = plane_field();
    const std::vector<FaceOffsetError3D::Face> faces = {
        {{0.4, 0.1, 0.61}, {0.2, 0.35, 0.44}},
        {{0.2, 0.35, 0.44}, {-0.25, 0.2, 0.58}},
        {{-0.25, 0.2, 0.58}, {-0.1, -0.4, 0.37}}};
    FaceOffsetError3D energy(pot, faces);
    const auto exact = [](const Vector3d& a, const Vector3d& b, const Vector3d& c) {
        const double A = 0.5 * (b - a).cross(c - a).norm();
        const double l1 = (a.z() - kEnergyDelta) / kEnergyDelta,
                     l2 = (b.z() - kEnergyDelta) / kEnergyDelta,
                     l3 = (c.z() - kEnergyDelta) / kEnergyDelta;
        return A / 6. * (l1 * l1 + l2 * l2 + l3 * l3 + l1 * l2 + l2 * l3 + l3 * l1);
    };
    for (const Vector3d& x :
         {Vector3d(0.1, -0.05, 0.53), Vector3d(-0.02, 0.12, 0.74), Vector3d(0.3, 0.2, 0.05)}) {
        INFO("x " << x.transpose());
        double want = 0., want_exact = 0.;
        for (const auto& f : faces) {
            want += face_offset_error_of(*pot, x, f.q1, f.q2);
            want_exact += exact(x, f.q1, f.q2);
        }
        CHECK(want == Catch::Approx(want_exact).epsilon(1e-12)); // the rule is exact here
        CHECK(want > 0.);
        Eigen::VectorXd xv = x;
        CHECK(energy.value(xv) == Catch::Approx(want).epsilon(1e-12));
        const double h = 1e-6;
        Eigen::VectorXd g(3);
        Eigen::MatrixXd H(3, 3);
        energy.gradient(xv, g);
        energy.hessian(xv, H);
        for (int i = 0; i < 3; ++i) {
            Eigen::VectorXd xp = xv, xm = xv;
            xp[i] += h;
            xm[i] -= h;
            const double fd = (energy.value(xp) - energy.value(xm)) / (2. * h);
            CHECK(g[i] == Catch::Approx(fd).epsilon(1e-6).margin(1e-7 * g.norm()));
            Eigen::VectorXd gp(3), gm(3);
            energy.gradient(xp, gp);
            energy.gradient(xm, gm);
            for (int j = 0; j < 3; ++j) {
                const double fdh = (gp[j] - gm[j]) / (2. * h);
                CHECK(H(j, i) == Catch::Approx(fdh).epsilon(1e-5).margin(1e-6 * H.norm()));
            }
        }
    }
}

TEST_CASE("face-offset-point-max", "[offset][3d]")
{
    // The stencil-point offset error the front max rule bounds: the largest
    // ((d - delta) / delta)^2 over the same 3 interior points (2/3, 1/6, 1/6) and permutations
    // (face_offset_point_max_of()), no area. Above the input triangle's interior d = z, so each
    // point's value is ((z(q) - delta) / delta)^2 with z(q) the barycentric mean of the corners'.
    const auto pot = plane_field();
    const std::vector<std::array<Vector3d, 3>> faces = {
        {{{0.1, -0.05, 0.53}, {0.4, 0.1, 0.61}, {0.2, 0.35, 0.44}}},
        {{{-0.02, 0.12, 0.74}, {0.2, 0.35, 0.44}, {-0.25, 0.2, 0.58}}},
        {{{0.3, 0.2, 0.05}, {-0.25, 0.2, 0.58}, {-0.1, -0.4, 0.37}}}};
    for (const auto& f : faces) {
        double want = 0.;
        for (const auto& w : kFaceErrorPoints) {
            const double z = w[0] * f[0].z() + w[1] * f[1].z() + w[2] * f[2].z();
            const double r = (z - kEnergyDelta) / kEnergyDelta;
            want = std::max(want, r * r);
        }
        CHECK(want > 0.);
        CHECK(
            face_offset_point_max_of(*pot, f[0], f[1], f[2]) == Catch::Approx(want).epsilon(1e-12));
        // The corner order does not matter: the 3 points are a symmetric set.
        CHECK(
            face_offset_point_max_of(*pot, f[2], f[0], f[1]) == Catch::Approx(want).epsilon(1e-12));
    }
}

TEST_CASE("front-op-rule", "[offset][3d]")
{
    // front_op_allowed(), the front max rule (Spencer's table, 2026-10-26), with bounds
    // o_bar = 1 and a_bar = 10: per starting cell, a collapse (strict false, after <= before)
    // and a swap (strict true, after < before).
    const double ob = 1., ab = 10.;
    const auto collapse = [&](double o0, double o1, double a0, double a1) {
        return TopoOffsetTetMesh::front_op_allowed(o0, o1, a0, a1, ob, ab, false);
    };
    const auto swap = [&](double o0, double o1, double a0, double a1) {
        return TopoOffsetTetMesh::front_op_allowed(o0, o1, a0, a1, ob, ab, true);
    };
    // Both in bound: both lower.
    CHECK(collapse(0.5, 0.4, 5., 4.));
    CHECK(collapse(0.5, 0.5, 5., 5.)); // a tie passes a collapse
    CHECK_FALSE(swap(0.5, 0.5, 5., 4.)); // and refuses a swap
    CHECK(swap(0.5, 0.4, 5., 4.));
    CHECK_FALSE(collapse(0.5, 0.6, 5., 4.));
    CHECK_FALSE(collapse(0.5, 0.4, 5., 6.)); // a rise below the bound still refuses
    // Offset over, AMIPS in: the offset max lower, the AMIPS max below its bound (may rise).
    CHECK(collapse(2., 1.5, 5., 9.));
    CHECK(swap(2., 1.5, 5., 9.));
    CHECK_FALSE(collapse(2., 1.5, 5., 10.)); // the bound itself is over
    CHECK_FALSE(collapse(2., 2.5, 5., 4.));
    CHECK(collapse(2., 2., 5., 9.));
    CHECK_FALSE(swap(2., 2., 5., 9.));
    // Offset in, AMIPS over: the AMIPS max lower, the offset max below its bound (may rise).
    CHECK(collapse(0.5, 0.9, 20., 15.));
    CHECK(swap(0.5, 0.9, 20., 15.));
    CHECK_FALSE(collapse(0.5, 1., 20., 15.));
    CHECK_FALSE(collapse(0.5, 0.9, 20., 25.));
    CHECK_FALSE(swap(0.5, 0.9, 20., 20.));
    // Both over: both lower.
    CHECK(collapse(2., 1.5, 20., 15.));
    CHECK(collapse(2., 2., 20., 20.));
    CHECK_FALSE(swap(2., 2., 20., 15.));
    CHECK_FALSE(collapse(2., 2.5, 20., 15.));
    CHECK_FALSE(collapse(2., 1.5, 20., 25.));
    // A value AT its bound counts as over: o = 1 starts in the offset-over cell.
    CHECK(collapse(1., 0.9, 5., 9.));
    CHECK_FALSE(collapse(1., 0.9, 5., 10.));
    // A NaN anywhere refuses.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(collapse(nan, 0.4, 5., 4.));
    CHECK_FALSE(collapse(0.5, nan, 5., 4.));
    CHECK_FALSE(collapse(0.5, 0.4, nan, 4.));
    CHECK_FALSE(collapse(0.5, 0.4, 5., nan));
    CHECK_FALSE(collapse(2., 1.5, 5., nan));
    CHECK_FALSE(collapse(0.5, nan, 20., 15.));
}

TEST_CASE("fold-barrier-3d", "[offset][3d]")
{
    // FoldBarrier3D, the front smoother's per-face fold barrier: 0 while the angle between the
    // face normal ((a - x) x (b - x)) and g is at most foldover_angle, positive and climbing past
    // it, finite past 90 degrees, +inf at 180 (the normal straight against g); gradient and
    // Hessian against central differences in the active range, past 90 degrees included.
    // One face: a = (1, 0, 0), b = (0, 1, 0), x moving; g = +z. With x = (0, 0, h) the normal
    // (a - x) x (b - x) = (h, h, 1), so cos = 1 / sqrt(1 + 2 h^2); x = (s, s, 0) with s > 1/2 puts
    // the normal at (0, 0, 1 - 2s), straight down: 180 degrees.
    const double theta = 60.;
    const double c0 = std::cos(theta * M_PI / 180.);
    const FoldBarrier3D::Face f{{1., 0., 0.}, {0., 1., 0.}, {0., 0., 1.}};
    FoldBarrier3D b({f}, c0);
    const auto at = [&](const Vector3d& x) {
        Eigen::VectorXd xv = x;
        return b.value(xv);
    };
    const auto cos_at_h = [](double h) { return 1. / std::sqrt(1. + 2. * h * h); };
    CHECK(FoldBarrier3D::cosine(f, Vector3d(0., 0., 0.3)) == Catch::Approx(cos_at_h(0.3)));
    // cos 60 = 0.5 at h = sqrt(3/2): below it the barrier is 0, beyond it positive.
    const double h60 = std::sqrt(1.5);
    CHECK(at(Vector3d(0., 0., 0.)) == 0.);
    CHECK(at(Vector3d(0., 0., 0.99 * h60)) == 0.);
    CHECK(at(Vector3d(0., 0., 1.5 * h60)) > 0.);
    CHECK(at(Vector3d(0., 0., 4. * h60)) > at(Vector3d(0., 0., 1.5 * h60)));
    // Past 90 degrees (here about 161): finite, and larger still.
    const Vector3d past90(0.7, 0.7, -0.1);
    REQUIRE(FoldBarrier3D::cosine(f, past90) < -0.9);
    CHECK(std::isfinite(at(past90)));
    CHECK(at(past90) > at(Vector3d(0., 0., 4. * h60)));
    // At 180 degrees: +inf.
    CHECK(FoldBarrier3D::cosine(f, Vector3d(0.8, 0.8, 0.)) == Catch::Approx(-1.));
    CHECK(std::isinf(at(Vector3d(0.8, 0.8, 0.))));
    // Derivatives in the active range.
    for (const Vector3d& x :
         {Vector3d(0.1, -0.05, 1.6), Vector3d(-0.2, 0.15, 2.5), Vector3d(0.05, 0.3, 4.), past90}) {
        INFO("x " << x.transpose());
        REQUIRE(at(x) > 0.);
        Eigen::VectorXd xv = x;
        const double h = 1e-6;
        Eigen::VectorXd g(3);
        Eigen::MatrixXd H(3, 3);
        b.gradient(xv, g);
        b.hessian(xv, H);
        for (int i = 0; i < 3; ++i) {
            Eigen::VectorXd xp = xv, xm = xv;
            xp[i] += h;
            xm[i] -= h;
            const double fd = (b.value(xp) - b.value(xm)) / (2. * h);
            CHECK(g[i] == Catch::Approx(fd).epsilon(1e-5).margin(1e-8 * (1. + g.norm())));
            Eigen::VectorXd gp(3), gm(3);
            b.gradient(xp, gp);
            b.gradient(xm, gm);
            for (int j = 0; j < 3; ++j) {
                const double fdh = (gp[j] - gm[j]) / (2. * h);
                CHECK(H(j, i) == Catch::Approx(fdh).epsilon(1e-4).margin(1e-6 * (1. + H.norm())));
            }
        }
    }
    // is_step_valid() refuses a step that ends at 180 degrees, admits one that ends past 90.
    Eigen::VectorXd x0(3), x1(3);
    x0 << 0., 0., 0.;
    x1 << 0.8, 0.8, 0.;
    CHECK_FALSE(b.is_step_valid(x0, x1));
    x1 = past90;
    CHECK(b.is_step_valid(x0, x1));
}

TEST_CASE("front-fold-barrier-on-the-mesh", "[offset][3d]")
{
    // front_fold_barrier_energy() on the fan fixture reads the debug frames' own angle
    // (front_face_field_angle(), the field_normal_angle field): at each front vertex whose faces
    // are below 180 degrees, its value is the sum over the faces of b((cos(angle) + 1) / (c0 + 1))
    // (0 for angles within foldover_angle); at a vertex with a face at 180 degrees it would throw.
    // Run on the fan and on it with the band and background labels swapped, which turns the fan
    // faces' normals out of the band around (angle -> 180 - angle, here past 90 but short of
    // 180): both value, neither throws. foldover_angle 180 is refused.
    const FanFixture fan = fan_fixture();
    const double theta = 3.; // below the fan's ~5-10 degree tilts: the barrier is active
    const double c0 = std::cos(theta * M_PI / 180.);
    const auto b_of = [](const double u) {
        return u >= 1. ? 0. : -(u - 1.) * (u - 1.) * std::log(u);
    };
    size_t n_checked = 0, n_threw = 0, n_valued = 0, n_active = 0;
    std::vector<int> swapped = fan.labels; // band <-> background, the complex kept
    for (int& l : swapped) l = l == 2 ? 0 : l == 0 ? 2 : l;
    for (const std::vector<int>& labels : {fan.labels, swapped}) {
        Parameters param;
        param.foldover_angle = theta;
        const auto pot = plane_field();
        auto mesh = energy_mesh(param, fan.V, fan.T, labels, pot);
        for (size_t v = 0; v < size_t(fan.V.rows()); ++v) {
            mesh->m_vertex_extra[v].m_is_on_offset = !mesh->offset_surface_faces_live_at(v).empty();
        }
        for (size_t v = 0; v < size_t(fan.V.rows()); ++v) {
            if (!mesh->m_vertex_extra[v].m_is_on_offset) continue;
            double worst = -1., want = 0.;
            for (const TetMesh::Tuple& ft : mesh->offset_surface_faces_live_at(v)) {
                const auto f = mesh->get_face_vids(ft);
                const double ang = mesh->front_face_field_angle(f[0], f[1], f[2]);
                if (!std::isfinite(ang)) continue;
                worst = std::max(worst, ang);
                want += b_of((std::cos(ang * M_PI / 180.) + 1.) / (c0 + 1.));
            }
            INFO(
                "labels " << (labels == fan.labels ? "fan" : "swapped") << ", vertex " << v
                          << ", worst front-face angle " << worst);
            ++n_checked;
            if (worst >= 180.) {
                CHECK_THROWS(mesh->front_fold_barrier_energy(v));
                ++n_threw;
                continue;
            }
            const auto fold = mesh->front_fold_barrier_energy(v);
            Eigen::VectorXd xv = mesh->m_vertex_attribute[v].m_posf;
            const double got = fold ? fold->value(xv) : 0.;
            CHECK(got == Catch::Approx(want).epsilon(1e-9).margin(1e-12));
            ++n_valued;
            if (got > 0.) ++n_active;
        }
        // foldover_angle 180 or more is refused.
        mesh->m_offset_params.foldover_angle = 180.;
        for (size_t v = 0; v < size_t(fan.V.rows()); ++v) {
            if (mesh->m_vertex_extra[v].m_is_on_offset) {
                CHECK_THROWS(mesh->front_fold_barrier_energy(v));
                break;
            }
        }
    }
    INFO(n_checked << " vertices checked, " << n_threw << " threw, " << n_valued << " valued");
    CHECK(n_threw == 0); // no face of the fan or its swap is at 180 degrees
    CHECK(n_valued > 0);
    CHECK(n_active > 0); // the barrier is really on somewhere (angles past foldover_angle)
}

TEST_CASE("ring-barrier-3d", "[offset][3d]")
{
    // RingBarrier3D, the front smoother's IPC barrier on the one-ring's boundary: 0 at and beyond
    // the activation distance, finite and positive inside it, +inf at and past a wall; gradient
    // and Hessian against central differences inside the active band; max_step_size() is 0.9x the
    // exact first crossing; is_step_valid() refuses a crossing.
    // One wall: the plane z = 0, the start side z > 0, start distance 1 (dhat 0.2).
    std::vector<RingBarrier3D::Wall> walls = {{Vector3d(0., 0., 1.), 0., 0.2}};
    RingBarrier3D b(walls);
    const auto value_at = [&](const double z) {
        Eigen::VectorXd x(3);
        x << 0.3, -0.2, z;
        return b.value(x);
    };
    CHECK(value_at(1.) == 0.);
    CHECK(value_at(0.2) == 0.); // exactly at the activation distance: u = 1, b = 0
    CHECK(value_at(0.1) > 0.);
    CHECK(value_at(0.01) > value_at(0.1)); // climbs toward the wall
    CHECK(std::isinf(value_at(0.)));
    CHECK(std::isinf(value_at(-0.5)));
    // Derivatives inside the active band.
    for (const double z : {0.15, 0.08, 0.02}) {
        Eigen::VectorXd xv(3);
        xv << 0.1, 0.4, z;
        const double h = 1e-7;
        Eigen::VectorXd g(3), xp = xv, xm = xv;
        Eigen::MatrixXd H(3, 3);
        b.gradient(xv, g);
        b.hessian(xv, H);
        INFO("z " << z);
        for (int i = 0; i < 3; ++i) {
            xp = xv;
            xm = xv;
            xp[i] += h;
            xm[i] -= h;
            const double fd = (b.value(xp) - b.value(xm)) / (2. * h);
            CHECK(g[i] == Catch::Approx(fd).epsilon(1e-5).margin(1e-8 * (1. + g.norm())));
            Eigen::VectorXd gp(3), gm(3);
            b.gradient(xp, gp);
            b.gradient(xm, gm);
            for (int j = 0; j < 3; ++j) {
                const double fdh = (gp[j] - gm[j]) / (2. * h);
                CHECK(H(j, i) == Catch::Approx(fdh).epsilon(1e-4).margin(1e-6 * (1. + H.norm())));
            }
        }
    }
    // The exact first crossing: from z = 1 toward z = -1 the wall is crossed at t = 0.5;
    // max_step_size returns 0.9x that, and the full step is invalid.
    Eigen::VectorXd x0(3), x1(3);
    x0 << 0., 0., 1.;
    x1 << 0., 0., -1.;
    CHECK(b.max_step_size(x0, x1) == Catch::Approx(0.45).epsilon(1e-12));
    CHECK_FALSE(b.is_step_valid(x0, x1));
    x1 << 0.7, -0.3, 0.4; // stays on the start side: full step, valid
    CHECK(b.max_step_size(x0, x1) == 1.);
    CHECK(b.is_step_valid(x0, x1));
}

TEST_CASE("per-tet-energy", "[offset][3d]")
{
    // tet_energy() against its definition computed by hand: (1 / s^3) vol AMIPS^3 for every
    // cell; MAX_ENERGY
    // for a degenerate cell. Four cells around the edge (a, b): 0 and 1 band, 2 background, 3
    // input complex (the label changes nothing since the band term's removal, 2026-10-22). At
    // (w, s) = (1, stop_energy 3), (1, 100) and the default w at 100.
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
    for (const double s : {3., 100.}) {
        Parameters param;
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
            const double want = vol * a3 / (s * s * s);
            INFO("s " << s << ", tet " << tid);
            CHECK(mesh->tet_energy(tid) == Catch::Approx(want).epsilon(1e-10));
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

TEST_CASE("front-smoothing-objective", "[offset][3d]")
{
    // The smoothing objective since 2026-10-22: a front vertex the loop places gets the ring
    // barrier plus the offset error over its live front faces (front_objective(); the barrier is
    // exactly 0 at the position the objective is built at), NO AMIPS; every other vertex gets no
    // extra energy (nullptr: the engine's raw AMIPS sum). Three fixtures: the per-tet-energy one
    // (front vertices a, b, c0, c1, c2; c3 on no front face), a star of eight band cells around
    // an off-front vertex -- both with front faces past 90 degrees from the field gradient, where
    // the fold barrier is large -- and the fan (fan_fixture()), whose front is unfolded. The front
    // objective's value at the build position against the sum of face_offset_error() over the
    // vertex's live front faces plus the fold barrier, and its gradient and Hessian against central
    // differences (small enough steps that the ring barrier stays 0 and flat). use_rest_pose: the
    // plastic machinery still prices tet_energy() (checked below), although no operation rule reads
    // it any more.
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
    const FanFixture fan = fan_fixture();
    struct Fixture
    {
        const Eigen::MatrixXd& V;
        const std::vector<std::array<int, 4>>& T;
        std::vector<int> labels;
        std::vector<size_t> not_front; ///< the vertices on no front face
    };
    const std::vector<Fixture> fixtures = {
        {V1, T1, {2, 2, 0, 1}, {5}},
        {V2, T2, std::vector<int>(8, 2), {0}},
        {fan.V, fan.T, fan.labels, {5, 6, 7}}};
    size_t n_front_valued = 0;
    for (size_t fi = 0; fi < fixtures.size(); ++fi) {
        const Fixture& fx = fixtures[fi];
        for (const auto& [s, plastic] : std::vector<std::pair<double, bool>>{
                 {3., false},
                 {100., false},
                 {3., true},
                 {100., true}}) {
            if (plastic && fi != 0) continue; // fixture 1 is all band: nothing plastic
            Parameters param;
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
                const double rest_term = VolAMIPSEnergy3D::value_of(p, R) / (s * s * s);
                const double a3 = mesh->TetOptimizerMesh::get_quality(mesh->oriented_tet_vids(2));
                const double vol =
                    std::abs((p[1] - p[0]).dot((p[2] - p[0]).cross(p[3] - p[0]))) / 6.;
                INFO("s " << s << ", plastic cell 2");
                CHECK(mesh->tet_energy(2) == Catch::Approx(rest_term).epsilon(1e-10));
                CHECK(std::abs(rest_term - vol * a3 / (s * s * s)) > 1e-3 * std::abs(rest_term));
            }
            for (size_t v = 0; v < size_t(fx.V.rows()); ++v) {
                const bool front = mesh->m_vertex_extra[v].m_is_on_offset;
                INFO(
                    "fixture " << fi << ", s " << s << ", plastic " << plastic << ", vertex " << v
                               << (front ? " (front)" : ""));
                REQUIRE(
                    front ==
                    (std::find(fx.not_front.begin(), fx.not_front.end(), v) == fx.not_front.end()));
                REQUIRE(!mesh->vertex_is_held(v));
                REQUIRE(mesh->vertex_front_placed(v) == front);
                // The loop's two blocks (m_smooth_set): the front block smooths exactly the
                // front-placed vertices, the background block every other one; outside the
                // blocks every vertex. smooth_before() refuses (and counts) the others.
                {
                    using S = TopoOffsetTetMesh::SmoothSet;
                    CHECK(mesh->smooth_set_admits(v));
                    mesh->m_smooth_set = S::Front;
                    CHECK(mesh->smooth_set_admits(v) == front);
                    mesh->m_smooth_set = S::Background;
                    CHECK(mesh->smooth_set_admits(v) == !front);
                    mesh->m_smooth_set = front ? S::Background : S::Front;
                    const size_t skipped = mesh->m_smooth_set_skipped.load();
                    CHECK_FALSE(mesh->smooth_before(mesh->tuple_from_vertex(v)));
                    CHECK(mesh->m_smooth_set_skipped.load() == skipped + 1);
                    mesh->m_smooth_set = S::All;
                }
                if (!front) {
                    const auto energy = mesh->smoothing_extra_energy(v);
                    if (mesh->ring_has_plastic_rest(v)) {
                        // use_rest_pose with a plastic ring cell: the raw AMIPS sum with that
                        // cell against its stamped rest -- per cell the cube root of
                        // plastic_quality(), the regular-rest cells reading the engine's value.
                        REQUIRE(energy != nullptr);
                        double want = 0.;
                        for (const size_t tid : mesh->get_one_ring_tids_for_vertex(v)) {
                            want += std::cbrt(mesh->plastic_quality(tid));
                        }
                        want *= mesh->m_params.w_amips;
                        Eigen::VectorXd xv = mesh->m_vertex_attribute[v].m_posf;
                        CHECK(energy->value(xv) == Catch::Approx(want).epsilon(1e-10));
                    } else {
                        // Off the front: no extra energy, the engine's raw AMIPS sum smooths it.
                        CHECK(energy == nullptr);
                    }
                    continue;
                }
                // A front face at 180 degrees from the field gradient: the fold barrier cannot
                // start there and the objective throws (no fixture has one; fixtures 0 and 1 have
                // faces past 90, where the barrier is large but finite).
                double worst = -1.;
                for (const TetMesh::Tuple& ft : mesh->offset_surface_faces_live_at(v)) {
                    const auto f = mesh->get_face_vids(ft);
                    const double ang = mesh->front_face_field_angle(f[0], f[1], f[2]);
                    if (std::isfinite(ang)) worst = std::max(worst, ang);
                }
                if (worst >= 180.) {
                    CHECK_THROWS(mesh->smoothing_extra_energy(v));
                    continue;
                }
                const auto energy = mesh->smoothing_extra_energy(v);
                REQUIRE(energy != nullptr);
                ++n_front_valued;
                // At the build position the ring barrier is 0, so the objective is the offset
                // error over the vertex's live front faces plus the fold barrier.
                double want = 0.;
                size_t n_faces = 0;
                for (const TetMesh::Tuple& ft : mesh->offset_surface_faces_live_at(v)) {
                    const auto f = mesh->get_face_vids(ft);
                    const double fe = mesh->face_offset_error(f[0], f[1], f[2]);
                    REQUIRE(fe >= 0.);
                    want += fe;
                    ++n_faces;
                }
                REQUIRE(n_faces > 0);
                const Vector3d x0 = mesh->m_vertex_attribute[v].m_posf;
                Eigen::VectorXd xv = x0;
                CHECK(want > 0.); // the fixture is off the level set
                // Plus the fold barrier at the build position (0 for faces within
                // foldover_angle of the field gradient).
                if (const auto fold = mesh->front_fold_barrier_energy(v)) want += fold->value(xv);
                CHECK(energy->value(xv) == Catch::Approx(want).epsilon(1e-12));

                // The objective's derivatives against central differences (h small enough that
                // every sample stays inside the ring barrier's zero region). Not next to the fold
                // barrier's wall, where it is near-infinite and a difference step can cross it.
                INFO("worst front-face angle " << worst);
                if (worst > 175.) continue;
                const double h = 1e-6;
                Eigen::VectorXd g(3);
                Eigen::MatrixXd H(3, 3);
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
    CHECK(n_front_valued > 0); // the fan fixture exercises the whole front objective
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
         {&TopoOffsetTetMesh::VtuFrame::amips,           &TopoOffsetTetMesh::VtuFrame::tet_e,
          &TopoOffsetTetMesh::VtuFrame::plastic,         &TopoOffsetTetMesh::VtuFrame::labels,
          &TopoOffsetTetMesh::VtuFrame::order,           &TopoOffsetTetMesh::VtuFrame::vid,
          &TopoOffsetTetMesh::VtuFrame::sizing,          &TopoOffsetTetMesh::VtuFrame::in_dist,
          &TopoOffsetTetMesh::VtuFrame::dist_ratio,      &TopoOffsetTetMesh::VtuFrame::front_err,
          &TopoOffsetTetMesh::VtuFrame::front_change,    &TopoOffsetTetMesh::VtuFrame::f_err,
          &TopoOffsetTetMesh::VtuFrame::f_len,           &TopoOffsetTetMesh::VtuFrame::f_inside,
          &TopoOffsetTetMesh::VtuFrame::f_outside,       &TopoOffsetTetMesh::VtuFrame::f_fold,
          &TopoOffsetTetMesh::VtuFrame::f_field_angle,   &TopoOffsetTetMesh::VtuFrame::f_dihedral,
          &TopoOffsetTetMesh::VtuFrame::f_fold_dihedral, &TopoOffsetTetMesh::VtuFrame::ring}) {
        ok = ok && same(a.*m, b.*m);
    }
    return ok;
}
} // namespace

TEST_CASE("frame-gather-threads-identical", "[offset][3d]")
{
    // gather_vtu_frame() and global_energy_parallel() evaluate their cells, vertices and faces on
    // the mesh's threads (diagnostic_parallel_for()); what they return must not depend on the
    // thread count. Two identical meshes -- the grid of save-offset-correspondence-3d, constructed
    // serially at target 0.2, the optimization's labels -- one gathered
    // at 1 thread and one at 4, compared bit for bit (NaN
    // equal to NaN).
    const GridInput g = grid_input();

    Parameters param = construction_params(
        {{"offset_selection", "tag_0"}, {"num_threads", 0}, {"target_distance", 0.2}},
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
    const size_t chunk = TopoOffsetTetMesh::kDiagnosticChunk;
    INFO(serial->get_tets().size() << " tets, " << serial->get_vertices().size() << " vertices");
    REQUIRE(serial->get_tets().size() > 4 * chunk); // every task has cells to evaluate

    const double e_serial = serial->global_energy_parallel();
    const double e_threaded = threaded->global_energy_parallel();
    const double e_plain = serial->global_energy();
    CHECK(std::memcmp(&e_serial, &e_threaded, sizeof(double)) == 0);
    CHECK(std::memcmp(&e_serial, &e_plain, sizeof(double)) == 0);

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
        {{"offset_selection", "tag_0"}, {"num_threads", 0}, {"target_distance", 0.2}},
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
        {{"offset_selection", "tag_0"}, {"num_threads", 0}, {"target_distance", 0.2}},
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

TEST_CASE("plastic-amips-quality", "[offset][3d]")
{
    // use_rest_pose since 2026-10-22: the loop's rules read PLASTIC AMIPS. plastic_quality() is
    // 27 exactly for a plastic cell at its stamped rest, the strained value against the rest
    // after a corner moves, and the engine's AMIPS^3 for an ordinary cell;
    // candidate_plastic_quality() is 27 for a plastic label and the engine's value otherwise;
    // smoothing_cell_energy() routes a plastic cell to plastic_quality() and passes everything
    // else through. plastic_amips_energy() is the raw AMIPS sum over a ring with the plastic
    // cell against its rest: its value is the sum of the cube roots of plastic_quality(), and
    // its gradient and Hessian match central differences.
    Eigen::MatrixXd V(6, 3);
    V << 0., 0., 0.37, 0., 0., 1.13, 0.4, 0., 0.75, 0., 0.4, 0.8, -0.4, 0., 0.7, 0., -0.4, 0.75;
    const std::vector<std::array<int, 4>> T = {
        {{0, 1, 2, 3}},
        {{0, 1, 3, 4}},
        {{0, 1, 4, 5}},
        {{0, 1, 5, 2}}};
    Parameters param;
    const auto pot = plane_field();
    auto mesh = energy_mesh(param, V, T, {2, 2, 0, 1}, pot);
    mesh->m_plastic_active = true;
    mesh->stamp_plastic_rests();
    REQUIRE(mesh->cell_is_plastic(2));
    REQUIRE(!mesh->cell_is_plastic(0)); // band
    REQUIRE(!mesh->cell_is_plastic(3)); // input complex
    // At the stamped rest a plastic cell reads the regular tet's value exactly.
    CHECK(mesh->plastic_quality(2) == Catch::Approx(27.).epsilon(1e-12));
    // An ordinary cell reads the engine's AMIPS^3.
    CHECK(
        mesh->plastic_quality(0) ==
        mesh->TetOptimizerMesh::get_quality(mesh->oriented_tet_vids(0)));
    // Strain cell 2 off its rest: its corner c2 (vertex 4) moves.
    const Vector3d x4 = mesh->m_vertex_attribute[4].m_posf;
    mesh->set_vertex_position(4, x4 + Vector3d(0.03, -0.02, 0.025));
    const double pq = mesh->plastic_quality(2);
    const double eq = mesh->TetOptimizerMesh::get_quality(mesh->oriented_tet_vids(2));
    CHECK(pq > 27.);
    CHECK(std::abs(pq - eq) > 1e-6 * eq); // against the rest, not the regular tet
    // The smoothing veto's per-cell hook.
    CHECK(mesh->smoothing_cell_energy(2, 123.) == pq);
    CHECK(mesh->smoothing_cell_energy(0, 123.) == 123.);
    // Candidates: a plastic label is stamped at creation and reads 27; others the engine's value.
    const auto vids0 = mesh->oriented_tet_vids(0);
    CHECK(mesh->candidate_plastic_quality(vids0, 0) == 27.);
    CHECK(mesh->candidate_plastic_quality(vids0, 2) == mesh->TetOptimizerMesh::get_quality(vids0));
    // The plastic smoothing sum at vertex 5 (ring: plastic cell 2, complex cell 3).
    REQUIRE(mesh->ring_has_plastic_rest(5));
    const auto energy = mesh->plastic_amips_energy(5);
    REQUIRE(energy != nullptr);
    double want = 0.;
    for (const size_t tid : mesh->get_one_ring_tids_for_vertex(5)) {
        want += std::cbrt(mesh->plastic_quality(tid));
    }
    want *= mesh->m_params.w_amips;
    Eigen::VectorXd xv = mesh->m_vertex_attribute[5].m_posf;
    CHECK(energy->value(xv) == Catch::Approx(want).epsilon(1e-10));
    // Derivatives against central differences.
    const double h = 1e-6;
    Eigen::VectorXd g(3);
    Eigen::MatrixXd H(3, 3);
    energy->gradient(xv, g);
    energy->hessian(xv, H);
    CHECK(g.norm() > 0.);
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
    // A ring without a plastic cell gives no plastic sum (vertex 3's cells are both band).
    CHECK(!mesh->ring_has_plastic_rest(3));
    CHECK(mesh->plastic_amips_energy(3) == nullptr);
    // With the medium off, plastic_quality() is the engine's everywhere.
    mesh->m_plastic_active = false;
    CHECK(mesh->plastic_quality(2) == eq);
}

TEST_CASE("convergence-keys", "[offset][3d]")
{
    // EXPERIMENTAL_settle_refinement through the spec, as the app reads it: on by default, false
    // read through. Since 2026-10-26 its exit is the max-based one (convergence_turn()), and the
    // old settle test's keys, settle_rel_thresh and EXPERIMENTAL_settle_mark, are gone: the strict
    // spec refuses a config that sets either, as it does the removed band keys and AMIPS weight.
    const auto spec = jse::embed::wmtk_topological_offset_spec::topological_offset_spec::spec();
    const auto verify = [&](nlohmann::json j) {
        j["application"] = "topological_offset";
        j["input"] = "unused.msh";
        jse::JSE spec_engine;
        spec_engine.strict = true;
        return spec_engine.verify_json(j, spec);
    };
    const auto injected = [&](nlohmann::json j) {
        j["application"] = "topological_offset";
        j["input"] = "unused.msh";
        jse::JSE spec_engine;
        spec_engine.strict = true;
        REQUIRE(spec_engine.verify_json(j, spec));
        return spec_engine.inject_defaults(j, spec);
    };
    const Parameters by_default(injected(nlohmann::json::object()));
    CHECK(by_default.settle_refinement);
    const Parameters off(injected({{"EXPERIMENTAL_settle_refinement", false}}));
    CHECK_FALSE(off.settle_refinement);
    CHECK_FALSE(verify({{"settle_rel_thresh", 0.1}}));
    CHECK_FALSE(verify({{"EXPERIMENTAL_settle_mark", "vertex"}}));
    CHECK_FALSE(verify({{"EXPERIMENTAL_band_volume_rule", "exact"}}));
    CHECK_FALSE(verify({{"EXPERIMENTAL_gap_refinement", true}}));
    CHECK_FALSE(verify({{"offset_amips_weight", 1.}}));
}

TEST_CASE("stuck-refine-stall", "[offset][3d]")
{
    // metric_stalled(), the stall test both of the loop's stuck-refines share with TetWild's
    // mesh_improvement(): (prev - cur) <= eps (cur - target).
    using M = TopoOffsetTetMesh;
    CHECK(M::metric_stalled(20., 19.5, 10., 0.1)); // fell 0.5 <= 0.1 x 9.5
    CHECK_FALSE(M::metric_stalled(20., 18., 10., 0.1)); // fell 2 > 0.95
    CHECK(M::metric_stalled(20., 21., 10., 0.1)); // a rise is a stall
    CHECK(M::metric_stalled(20., 20., 10., 0.)); // eps 0: only no fall at all
    CHECK_FALSE(M::metric_stalled(20., 19.99, 10., 0.));
    // The loop's state starts with both cooldowns at stuck_refine_cooldown.
    Parameters param;
    TopoOffsetTetMesh mesh(param, 0);
    mesh.m_params.stuck_refine_cooldown = 3;
    const M::StuckState s = mesh.stuck_state_start();
    CHECK(s.cooldown_amips == 3);
    CHECK(s.cooldown_offset == 3);
    CHECK(s.prev_amips < 0.);
    CHECK(s.prev_offset < 0.);
}

TEST_CASE("offset-stuck-refine", "[offset][3d]")
{
    // convergence_turn() on the grid of save-offset-correspondence-3d, constructed serially at
    // target 0.2, its offset faces nowhere near a tiny front_conv: not converged, and with the
    // offset error stalled (turn 2, no previous max) every face with a stencil point at or over
    // front_conv_frac()^2 has its longest edge queued for a forced split and its corners' sizing
    // halved (or at the floor). The AMIPS stuck-refine is kept out (its previous max far above,
    // so not stalled). With both bounds wide open the same mesh converges and nothing is queued.
    const GridInput g = grid_input();
    Parameters param = construction_params(
        {{"offset_selection", "tag_0"}, {"num_threads", 0}, {"target_distance", 0.2}},
        g.V);
    const std::filesystem::path out =
        std::filesystem::temp_directory_path() / "wmtk_offset_stuck_refine";
    auto mesh = grid_offset_mesh(g, param, out);
    mesh->m_params.stuck_refine_force_split = true;

    {
        mesh->m_offset_params.front_conv = 1e-9;
        mesh->m_params.stop_energy = 1e30;
        const double o_bar = mesh->front_offset_bar();
        std::vector<std::array<size_t, 3>> want;
        for (const auto& f : mesh->offset_surface_faces()) {
            if (!(mesh->face_offset_point_max(f[0], f[1], f[2]) < o_bar)) want.push_back(f);
        }
        REQUIRE(!want.empty());
        std::vector<double> before(mesh->vert_capacity());
        for (size_t v = 0; v < before.size(); ++v) {
            before[v] = mesh->m_vertex_attribute[v].m_sizing_scalar;
        }
        TopoOffsetTetMesh::StuckState s;
        s.prev_amips = 1e300;
        mesh->m_force_split_edges.clear();
        CHECK_FALSE(mesh->convergence_turn(2, s, /*refine_now=*/true));
        const double l = std::max(mesh->m_params.l, 1e-300);
        const double s_floor = std::max(
            mesh->m_offset_params.min_sizing_scalar,
            mesh->m_offset_params.min_edge_length / l);
        for (const auto& f : want) {
            const auto e = wmtk::utils::longest_edge(f, [&](size_t v) -> const Vector3d& {
                return mesh->m_vertex_attribute[v].m_posf;
            });
            const auto ev = e.vertices();
            CHECK(mesh->is_force_split_edge(ev[0], ev[1]));
            for (const size_t v : f) {
                CHECK(
                    mesh->m_vertex_attribute[v].m_sizing_scalar <=
                    std::max(0.5 * before[v], s_floor));
            }
        }
        CHECK(mesh->m_force_split_edges.size() <= want.size());
        CHECK(s.prev_offset > o_bar);
        CHECK(s.cooldown_offset == std::max(0, mesh->m_params.stuck_refine_cooldown));
    }
    {
        // Wide open: front_conv at the target distance itself (bound 1) and stop_energy far above
        // every cell.
        mesh->m_offset_params.front_conv = 10. * mesh->m_offset_params.target_distance;
        mesh->m_params.stop_energy = 1e30;
        mesh->m_force_split_edges.clear();
        TopoOffsetTetMesh::StuckState s;
        CHECK(mesh->convergence_turn(2, s, /*refine_now=*/true));
        CHECK(mesh->m_force_split_edges.empty());
    }
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
    // m_op_rules: in the loop (since 2026-10-22) the base's max rules decide every collapse and
    // swap off the offset surface, and an op on it is admitted by the quality hooks (the face
    // error judges it in the after-hooks); off the front the smoother gets no extra energy.
    // Under TetWild's rules (every TetWild pass) the base's own throughout. The projected step's
    // ring measure is the base's everywhere. One unit tet.
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
    // Off the offset surface the base's max rules decide.
    CHECK_FALSE(mesh.collapse_quality_allowed(0, 2., 1.));
    CHECK(mesh.collapse_quality_allowed(0, 1., 2.));
    CHECK_FALSE(mesh.swap_quality_allowed(2., 1., false));
    CHECK(mesh.swap_quality_allowed(1., 2., false));
    // On it the quality hooks admit everything: the front max rule decides in the after-hooks.
    mesh.m_collapse_front.local().o_before = 0.5;
    CHECK(mesh.collapse_quality_allowed(0, 2., 1.));
    mesh.m_collapse_front.local().o_before = -1.;
    // Off the front: no extra energy (the engine's raw AMIPS sum smooths it); the base's ring
    // measure.
    CHECK(mesh.smoothing_extra_energy(0) == nullptr);
    CHECK(
        mesh.smoothing_ring_energy(ring, true) ==
        mesh.TetOptimizerMesh::smoothing_ring_energy(ring, true));
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
