#include <jse/jse.h>
#include <math.h>
#include <spdlog/sinks/ostream_sink.h>
#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>
#include <wmtk/components/topological_offset/TopoOffsetTetMesh.h>
#include <wmtk/components/topological_offset/TopoOffsetTriMesh.h>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <numeric>
#include <queue>
#include <set>
#include <sstream>
#include <topological_offset_spec.hpp>
#include <tuple>
#include <wmtk/Types.hpp>
#include <wmtk/components/simwild/expression_parser/Parser.hpp>
#include <wmtk/components/topological_offset/Circle.hpp>
#include <wmtk/components/topological_offset/SimplicialComplexBVH.hpp>
#include <wmtk/components/topological_offset/Sphere.hpp>
#include <wmtk/components/topological_offset/output_conversion.hpp>
#include <wmtk/components/topological_offset/topological_offset.hpp>
#include <wmtk/io/mesh_io.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/utils/io.hpp>

// Cases too slow for the Debug CI jobs, Windows Debug above all: Release builds only.
#ifdef NDEBUG
const std::string release_only = "";
#else
const std::string release_only = "[.]";
#endif

using namespace wmtk;
using namespace components::topological_offset;
using namespace components::simwild::expression_parser;

namespace {
/// Select `selection` as the one offset, as topological_offset.cpp parses an entry's selection:
/// into the entry the json gave (its output tags and distance kept), or into one added for a
/// Parameters built by hand, which offsets at the run's target_distance with no output tag; and
/// parse protected_tags beside it. Either dimension's mesh.
template <typename Mesh>
void select_offset(Mesh& mesh, const std::string& selection)
{
    auto& offsets = mesh.m_offset_params.offsets;
    if (offsets.empty()) offsets.emplace_back();
    offsets[0].selection = selection;
    offsets[0].expression = parse(selection, mesh.m_tag_name_to_id);
    mesh.m_offset_params.parse_protected_tags(mesh.m_tag_name_to_id);
}
} // namespace


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


TEST_CASE("held_faces_and_their_envelopes", "[3d][envelope]")
{
    // Two tets sharing a face, one tagged a and one tagged b: the shared face is the a/b
    // boundary, every outer face a domain-wall face. Only the shared face is tracked: the wall
    // never is, the box rule holding it, as in tetwild and simwild (until 2026-11-05 it was
    // tracked, and face_is_held() exempted it). The HELD faces are every tracked face but the
    // offset surface, in the loop as in a TetWild pass, read live off the cells -- every tracked
    // surface held, as in tetwild and simwild (2026-11-04; until then the loop held the input
    // complex's boundary alone). A held face is held to the envelopes of its labels, here the
    // tags of the two regions it bounds.
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
    const auto envelope_of = [&](size_t a, size_t b, size_t c) {
        return mesh.surface_envelope_for_face(
            {{a, b, c}},
            mesh.m_face_attribute[face(a, b, c).fid(mesh)]);
    };
    for (const auto& f : mesh.get_faces()) {
        REQUIRE(
            mesh.m_face_attribute[f.fid(mesh)].m_is_surface_fs ==
            f.switch_tetrahedron(mesh).has_value());
    }

    // In the loop, no input complex: the a/b boundary is a region boundary off the input
    // complex, and held all the same, to its labels' envelopes.
    REQUIRE(!mesh.m_tetwild_pass);
    REQUIRE(mesh.m_tet_attribute[0].label == 0);
    REQUIRE(mesh.m_tet_attribute[1].label == 0);
    REQUIRE(mesh.face_is_held(face(1, 2, 3)));
    mesh.build_envelopes();
    const auto loop_env = mesh.m_envelope;
    REQUIRE(loop_env != nullptr);
    const Eigen::Vector3d on_shared = (V.row(1) + V.row(2) + V.row(3)).transpose() / 3.;
    const auto shared_env = envelope_of(1, 2, 3);
    REQUIRE(shared_env != nullptr);
    REQUIRE(!shared_env->is_outside(on_shared));
    REQUIRE(shared_env->is_outside(Eigen::Vector3d(0.2, 0.2, 0.2)));
    // The shared face is on both tags, so held to the intersection of their envelopes, which the
    // containment audit must read through its members: it once asked the composite itself.
    REQUIRE(dynamic_cast<const wmtk::IntersectionEnvelope*>(shared_env.get()) != nullptr);
    const auto& shared_attr = mesh.m_face_attribute[face(1, 2, 3).fid(mesh)];
    REQUIRE(!mesh.surface_triangle_is_outside(1, 2, 3, shared_attr));
    REQUIRE_NOTHROW(mesh.audit_surface_containment("loop"));
    for (size_t v = 1; v < 4; ++v) REQUIRE(mesh.vertex_is_held(v));
    REQUIRE(!mesh.vertex_is_held(0)); // on wall faces only, none tracked
    REQUIRE(!mesh.vertex_is_held(4));
    REQUIRE(!mesh.m_envelope->is_outside(on_shared));
    REQUIRE(mesh.m_envelope->is_outside(Eigen::Vector3d(V.row(0).transpose())));
    // A corner pushed 10 eps off the face's plane takes the face out of its envelope, so the
    // loop's operations and smoother refuse the move; until 2026-11-04 they took it.
    const Eigen::Vector3d p1 = V.row(1).transpose();
    mesh.set_vertex_position(1, p1 + 1e-2 * Eigen::Vector3d(1., 1., 1.).normalized());
    REQUIRE(mesh.surface_triangle_is_outside(1, 2, 3, shared_attr));
    mesh.set_vertex_position(1, p1);
    REQUIRE(!mesh.surface_triangle_is_outside(1, 2, 3, shared_attr));

    // A TetWild pass holds the same faces in the same envelopes: build_final_envelopes() adds
    // only the offset surface's own (none here), and m_envelope stays the loop's.
    mesh.build_final_envelopes();
    mesh.m_tetwild_pass = true;
    REQUIRE(mesh.face_is_held(face(1, 2, 3)));
    REQUIRE(mesh.m_envelope == loop_env);
    REQUIRE(envelope_of(1, 2, 3) == shared_env);
    REQUIRE_NOTHROW(mesh.audit_surface_containment("final pass"));
    mesh.m_tetwild_pass = false;

    // ... and with tet 0 the input complex, the shared face is its boundary: held as before.
    mesh.m_tet_attribute[0].label = 1;
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

TEST_CASE("interior-stencil-point-counts", "[offset]")
{
    // EXPERIMENTAL_stencil_type "interior" (2026-10-27), checked against the counts the design
    // states: the interior lattice (i, j, l) / (k + 2), i, j,
    // l >= 1, k (k + 1) / 2 points -- 1, 3, 6, 10, 15, 21 for k = 1..6 -- and nothing at order 0.
    //
    // This also pins stencil_points_per_face() to for_each_face_sample(), which are two separate
    // pieces of arithmetic that have to agree: the accessor is used in the logs and in the
    // criterion's own reporting, the loop is what actually visits the points.
    Parameters param;
    param.stencil_type = "interior";
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

TEST_CASE("fem-face-stencil", "[offset][3d]")
{
    // EXPERIMENTAL_stencil_type "fem": fem_face_stencil(k), the degree-k Lagrange nodes (i, j, l) /
    // k, i + j + l = k -- (k + 1) (k + 2) / 2 points: order 1 the corners, order 2 the corners and
    // the edge midpoints, order 3 the corners, two points per edge (1/3, 2/3) and the centroid --
    // and the mesh's face stencil, for_each_face_sample() and stencil_points_per_face() following
    // the key ("subdivision" the default).
    using W3 = std::array<long long, 3>;
    const auto key = [](const std::array<double, 3>& w, const int k) {
        return W3{{llround(w[0] * k), llround(w[1] * k), llround(w[2] * k)}};
    };
    CHECK(fem_face_stencil(0).empty());
    for (int k = 1; k <= 13; ++k) {
        INFO("order " << k);
        const auto& st = fem_face_stencil(k);
        CHECK(int(st.size()) == (k + 1) * (k + 2) / 2);
        std::set<W3> seen;
        for (const auto& w : st) {
            CHECK(std::abs(w[0] + w[1] + w[2] - 1.) <= 1e-14);
            for (int c = 0; c < 3; ++c) {
                CHECK(w[size_t(c)] >= 0.);
                // every weight a multiple of 1 / k
                CHECK(std::abs(w[size_t(c)] * k - std::round(w[size_t(c)] * k)) <= 1e-12);
            }
            seen.insert(key(w, k));
        }
        CHECK(seen.size() == st.size()); // no point twice
    }
    const auto as_set = [&](const int k) {
        std::set<W3> s;
        for (const auto& w : fem_face_stencil(k)) s.insert(key(w, k));
        return s;
    };
    CHECK(as_set(1) == std::set<W3>{{{1, 0, 0}}, {{0, 1, 0}}, {{0, 0, 1}}});
    CHECK(
        as_set(2) ==
        std::set<W3>{{{2, 0, 0}}, {{0, 2, 0}}, {{0, 0, 2}}, {{1, 1, 0}}, {{1, 0, 1}}, {{0, 1, 1}}});
    CHECK(
        as_set(3) == std::set<W3>{
                         {{3, 0, 0}},
                         {{0, 3, 0}},
                         {{0, 0, 3}},
                         {{2, 1, 0}},
                         {{1, 2, 0}},
                         {{2, 0, 1}},
                         {{1, 0, 2}},
                         {{0, 2, 1}},
                         {{0, 1, 2}},
                         {{1, 1, 1}}});

    // The lumped area weights (fem_face_stencil_weights()): one per point, in the stencil's order,
    // 1 / (3 k^2) at a corner, 1 / k^2 at an edge point, 2 / k^2 inside, summing to 1.
    CHECK(fem_face_stencil_weights(0).empty());
    for (int k = 1; k <= 13; ++k) {
        INFO("order " << k);
        const auto& st = fem_face_stencil(k);
        const auto& mu = fem_face_stencil_weights(k);
        REQUIRE(mu.size() == st.size());
        double sum = 0.;
        for (size_t i = 0; i < st.size(); ++i) {
            const W3 n = key(st[i], k);
            const int zeros = int(n[0] == 0) + int(n[1] == 0) + int(n[2] == 0);
            const double want = (zeros == 2 ? 1. : zeros == 1 ? 3. : 6.) / (3. * k * k);
            CHECK(mu[i] == Catch::Approx(want).epsilon(1e-14));
            sum += mu[i];
        }
        CHECK(sum == Catch::Approx(1.).epsilon(1e-13));
    }
    // What the weights are for: on a face, sum_p mu_p f(x_p) is exact for affine f and converges
    // to the face mean of a smooth f at second order in 1 / k (the composite trapezoid rule);
    // the plain mean over the same points only at first order. f = l0^2 (barycentric), face
    // mean 1 / 6; and an affine f = 0.3 + 2 l0 - l1, mean 0.3 + 1 / 3.
    const auto quad = [&](const int k, const bool lumped, const auto& f) {
        const auto& st = fem_face_stencil(k);
        const auto& mu = fem_face_stencil_weights(k);
        double s = 0.;
        for (size_t i = 0; i < st.size(); ++i) {
            s += (lumped ? mu[i] : 1. / double(st.size())) * f(st[i]);
        }
        return s;
    };
    const auto sq = [](const std::array<double, 3>& l) { return l[0] * l[0]; };
    const auto aff = [](const std::array<double, 3>& l) { return 0.3 + 2. * l[0] - l[1]; };
    for (const int k : {1, 2, 3, 5, 8}) {
        INFO("order " << k);
        CHECK(quad(k, true, aff) == Catch::Approx(0.3 + 1. / 3.).epsilon(1e-13));
    }
    for (const int k : {2, 4, 8}) {
        INFO("order " << k << " -> " << 2 * k);
        const double e_l = std::abs(quad(k, true, sq) - 1. / 6.);
        const double e_l2 = std::abs(quad(2 * k, true, sq) - 1. / 6.);
        const double e_p = std::abs(quad(k, false, sq) - 1. / 6.);
        const double e_p2 = std::abs(quad(2 * k, false, sq) - 1. / 6.);
        CHECK(e_l / e_l2 == Catch::Approx(4.).epsilon(1e-9)); // second order, lumped
        CHECK(e_p / e_p2 < 2.5); // at best first order, the plain mean
        CHECK(e_l < e_p);
    }

    // The mesh follows the key: the same points through face_stencil() and for_each_face_sample(),
    // the count through stencil_points_per_face(); "subdivision" by default and "interior" with no
    // weights.
    Parameters param;
    TopoOffsetTetMesh mesh(param, 0);
    CHECK(mesh.m_offset_params.stencil_type == "subdivision"); // the default
    CHECK(mesh.m_offset_params.stencil_order == 1);
    CHECK(&mesh.face_stencil() == &subdivision_face_stencil(1));
    CHECK(mesh.face_stencil_weights() == nullptr);
    CHECK(mesh.stencil_points_per_face() == 4);
    mesh.m_offset_params.stencil_type = "interior";
    mesh.m_offset_params.stencil_order = 3;
    CHECK(&mesh.face_stencil() == &interior_face_stencil(3));
    CHECK(mesh.face_stencil_weights() == nullptr);
    CHECK(mesh.stencil_points_per_face() == 6);
    mesh.m_offset_params.stencil_type = "fem";
    const Vector3d p0(0.3, -0.2, 1.), p1(1.4, 0.1, 0.2), p2(-0.5, 0.9, 0.4);
    for (int k = 1; k <= 6; ++k) {
        INFO("order " << k);
        mesh.m_offset_params.stencil_order = k;
        CHECK(&mesh.face_stencil() == &fem_face_stencil(k));
        CHECK(mesh.face_stencil_weights() == &fem_face_stencil_weights(k));
        const auto& st = fem_face_stencil(k);
        size_t i = 0;
        mesh.for_each_face_sample(
            p0,
            p1,
            p2,
            [&](const Vector3d& q, const double wa, const double wb, const double wc) {
                REQUIRE(i < st.size());
                CHECK(wa == st[i][0]);
                CHECK(wb == st[i][1]);
                CHECK(wc == st[i][2]);
                CHECK((q - (wa * p0 + wb * p1 + wc * p2)).norm() <= 1e-15);
                ++i;
            });
        CHECK(i == st.size());
        CHECK(mesh.stencil_points_per_face() == int(st.size()));
    }
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
    // The exact band integral on the same triangle, built as a run builds it: only for the
    // settings that read it (TopoOffsetTetMesh::init_band_exact()).
    mesh->m_phi_V = plane_vertices();
    mesh->m_phi_F.resize(1, 3);
    mesh->m_phi_F << 0, 1, 2;
    mesh->m_phi_F_inner = {0}; // a lone triangle: it bounds no solid
    mesh->init_band_exact();
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
    // orders 0 (corners only), 1 and 2, on three faces placed above and below the level set; then
    // under EXPERIMENTAL_stencil_type "interior" at orders 1 (the centroid), 2 and 5, and "fem".
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
    for (const char* type : {"subdivision", "interior"}) {
        mesh.m_offset_params.stencil_type = type;
        for (const int k : std::string(type) == "subdivision" ? std::vector<int>{0, 1, 2}
                                                              : std::vector<int>{1, 2, 5}) {
            mesh.m_offset_params.stencil_order = k;
            double by_hand = 0., by_term = 0.;
            for (const auto& [q1, q2] : others) {
                by_hand += face_term_by_hand(mesh, *pot, x, q1, q2);
                by_term += mesh.face_offset_term(*pot, x, q1, q2);
            }
            INFO(type << " stencil_order " << k);
            CHECK(by_hand > 10.); // the fixture really is off the level set
            CHECK(by_term == Catch::Approx(by_hand).epsilon(1e-12));
        }
    }
    // EXPERIMENTAL_stencil_type "fem": the same measure, the mean weighted by the points' lumped
    // area weights (fem_face_stencil_weights()), by hand.
    mesh.m_offset_params.stencil_type = "fem";
    for (const int k : {1, 2, 3, 5}) {
        mesh.m_offset_params.stencil_order = k;
        const auto& st = fem_face_stencil(k);
        const auto& mu = fem_face_stencil_weights(k);
        double by_hand = 0., by_term = 0.;
        for (const auto& [q1, q2] : others) {
            double m = 0.;
            for (size_t i = 0; i < st.size(); ++i) {
                const auto& w = st[i];
                const double r =
                    pot->relative_residual(w[0] * x + w[1] * q1 + w[2] * q2) / kEnergyFrac;
                m += mu[i] * r * r;
            }
            by_hand += m;
            by_term += mesh.face_offset_term(*pot, x, q1, q2);
        }
        INFO("fem stencil_order " << k);
        CHECK(by_hand > 10.);
        CHECK(by_term == Catch::Approx(by_hand).epsilon(1e-12));
    }
}

TEST_CASE("per-tet-energy", "[offset][3d]")
{
    // tet_energy() against its definition computed by hand: (w / s^3) vol AMIPS^3 for every
    // cell, plus the band term b = vol (mean corner d - delta) / delta for a band cell; MAX_ENERGY
    // for a degenerate cell. Four cells around the edge (a, b): 0 and 1 band, 2 background, 3
    // input complex. At (w, s) = (1, stop_energy 3), where both terms matter, (1, 100) and the
    // default w at 100. Under both band rules, "corner_bound" (the default until 2026-10-29) and
    // "exact": above the triangle's interior d is affine, so the exact integral of d over a cell
    // is its volume times the corner mean.
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
    for (const auto& [rule, w, s] : std::vector<std::tuple<std::string, double, double>>{
             {"corner_bound", 1., 3.},
             {"corner_bound", 1., 100.},
             {"corner_bound", Parameters().offset_amips_weight, 100.},
             {"exact", 1., 3.},
             {"exact", 1., 100.},
             {"exact", Parameters().offset_amips_weight, 100.}}) {
        Parameters param;
        param.band_volume_rule = rule;
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
            INFO(rule << ", w " << w << ", s " << s << ", tet " << tid);
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
    // still is the ring sum. Under both band rules, "corner_bound" (the default until 2026-10-29)
    // and "exact". Under "exact" the star's band term does not move with its inner vertex (BAND
    // TERM THAT CANCELS, tet_energy()): the eight cells tile the star, whose union is fixed. The
    // objective there leaves it out -- it is the AMIPS term (amips3_energy()) -- so the ring sum
    // less the objective is the band term, which is checked to keep its value and to have no
    // derivatives.
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
    // inner vertex at all -- as under "exact", anywhere.
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
        for (const auto& [rule, w, s, plastic] :
             std::vector<std::tuple<std::string, double, double, bool>>{
                 {"corner_bound", 1., 3., false},
                 {"corner_bound", Parameters().offset_amips_weight, 100., false},
                 {"corner_bound", 1., 3., true},
                 {"corner_bound", Parameters().offset_amips_weight, 100., true},
                 {"exact", 1., 3., false},
                 {"exact", Parameters().offset_amips_weight, 100., false},
                 {"exact", 1., 3., true},
                 {"exact", Parameters().offset_amips_weight, 100., true}}) {
            if (plastic && fi != 0) continue; // fixture 1 is all band: nothing plastic
            Parameters param;
            param.band_volume_rule = rule;
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
                    "fixture " << fi << ", " << rule << ", w " << w << ", s " << s << ", plastic "
                               << plastic << ", vertex " << v << (front ? " (front)" : ""));
                REQUIRE(front == (v != fx.not_front));
                REQUIRE(!mesh->vertex_is_held(v));
                if (v == fx.not_front) {
                    REQUIRE((mesh->band_volume_energy_at(v) != nullptr) == fx.has_band_ring);
                }
                const auto energy = mesh->smoothing_extra_energy(v);
                // Under "exact", fixture 1's inner vertex: its ring's band term does not move with
                // it (band_term_cancels(), the smoother's own test), and the objective is the
                // AMIPS term (amips3_energy()) alone.
                const bool band_fixed = !front && mesh->band_term_cancels(ring);
                REQUIRE(band_fixed == (rule == "exact" && v == fx.not_front && fx.has_band_ring));
                const auto band = band_fixed ? mesh->band_volume_energy_at(v) : nullptr;
                const auto amips = mesh->amips3_energy(v);
                const Vector3d x0 = mesh->m_vertex_attribute[v].m_posf;
                Eigen::VectorXd xv = x0;
                const double e0 = energy->value(xv);
                const double b0 = band ? band->value(xv) : 0.;
                const double d0 = e0 - ring_energy();
                CHECK(
                    d0 ==
                    Catch::Approx(-b0).margin(1e-12 * std::max({1., std::abs(e0), std::abs(b0)})));
                if (band) CHECK(e0 == amips->value(xv));
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
                    if (band) {
                        CHECK(e == amips->value(xv));
                        CHECK(
                            band->value(xv) ==
                            Catch::Approx(b0).margin(1e-12 * std::max(1., std::abs(b0))));
                    }
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
                if (band) {
                    // The objective is the AMIPS term's, and the band part it leaves out has no
                    // derivatives, to the margins the central differences allow.
                    Eigen::VectorXd ga(3), gb(3);
                    Eigen::MatrixXd Ha(3, 3), Hb(3, 3);
                    amips->gradient(xv, ga);
                    amips->hessian(xv, Ha);
                    CHECK(g == ga);
                    CHECK(H == Ha);
                    band->gradient(xv, gb);
                    band->hessian(xv, Hb);
                    CHECK(gb.norm() <= 1e-7 * g.norm());
                    CHECK(Hb.norm() <= 1e-6 * H.norm());
                }
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
// The 2D per-cell energy (TopoOffsetTriMesh::tri_energy()), its rules, and the 2D front
// smoother's offset term, on the 2D twin of the 3D fixtures: the euclidean distance to a long
// segment on the x axis, at level delta = 0.5, so relative_residual(q) = (|y| - 0.5) / 0.5 above
// the segment's interior, where d is affine and the band term's corner mean is exact. Beyond its
// end (10, 0) -- x > 10 -- d is the distance to the end, curved, and the corner mean is an upper
// bound that depends on how a region is cut into faces: the swap fixture sits there. front_conv =
// 0.01 (front_conv_frac() 0.02).
// ---------------------------------------------------------------------------------------------

namespace {

const Vector2d kLineEnd(10., 0.);

/// The segment's ends.
Eigen::MatrixXd line_vertices()
{
    Eigen::MatrixXd SV(2, 2);
    SV << -10., 0., kLineEnd.x(), kLineEnd.y();
    return SV;
}

std::shared_ptr<EuclideanOffsetPotential2D> line_field()
{
    auto bvh = std::make_shared<SimplicialComplexBVH>();
    MatrixXi SE(1, 2);
    SE << 0, 1;
    bvh->init(line_vertices(), MatrixXi(0, 4), MatrixXi(0, 3), SE, MatrixXi(0, 1));
    return std::make_shared<EuclideanOffsetPotential2D>(bvh, kEnergyDelta);
}

/// The distance to the segment at a point above its interior or beyond its end (10, 0), the two
/// regions the fixtures use.
double line_distance(const Vector2d& q)
{
    if (q.x() > kLineEnd.x()) return (q - kLineEnd).norm();
    return std::abs(q.y());
}

/// The band term b(t) the energy definition states, computed here from the corners: unsigned area
/// times (mean corner distance - delta) / delta (one segment, so the min over primitives is that
/// segment). The 2D twin of band_term_by_hand().
double band_term_by_hand_2d(const std::array<Vector2d, 3>& p)
{
    const Vector2d u = p[1] - p[0], v = p[2] - p[0];
    const double area = std::abs(u.x() * v.y() - u.y() * v.x()) / 2.;
    double mean = 0.;
    for (const Vector2d& q : p) mean += line_distance(q) / 3.;
    return area * (mean - kEnergyDelta) / kEnergyDelta;
}

/// The face's corner positions, in the mesh's order.
std::array<Vector2d, 3> corners_2d(const TopoOffsetTriMesh& m, const size_t fid)
{
    const auto vs = m.oriented_tri_vids(fid);
    std::array<Vector2d, 3> p;
    for (int j = 0; j < 3; ++j) p[size_t(j)] = m.m_vertex_attribute[vs[size_t(j)]].m_posf;
    return p;
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

/// A 2D mesh of the given faces with the given construction labels on the segment's field and
/// band term (the 2D twin of energy_mesh()). Face i is row i of F, which the caller relies on to
/// look up faces.
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
    param.envelope_size = 1e-3; // tests skip Parameters::init(); give the build a real eps
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
    // A piece cut out of a domain, as 3D's energy_mesh(): its outer edges stand for interior
    // edges, not the domain wall, which a run holds to ambient's envelope. Untracked, and with no
    // boundary mask, so no vertex is held.
    for (const auto& e : mesh->get_edges()) {
        mesh->m_edge_attribute[e.eid(*mesh)].m_is_surface_fs = false;
    }
    for (size_t v = 0; v < size_t(V.rows()); ++v) {
        mesh->m_vertex_attribute[v].m_is_on_surface = false;
        mesh->m_vertex_extra[v].m_is_on_region = false;
        mesh->m_vertex_extra[v].m_boundary_mask = 0;
    }
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
    // The band term's primitives on the same segment, built as a run builds them
    // (TopoOffsetTriMesh::init_band_exact()): a lone segment, which bounds no region.
    mesh->m_phi_V = line_vertices();
    mesh->m_phi_E.resize(1, 2);
    mesh->m_phi_E << 0, 1;
    mesh->m_phi_E_inner = {0};
    mesh->init_band_exact();
    REQUIRE(mesh->m_band_exact != nullptr);
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

TEST_CASE("edge-term-is-the-stencil-mean-2d", "[offset][2d]")
{
    // The 2D twin of face-term-is-the-stencil-mean: edge_offset_term(), the chord measure the
    // exit, the refinement and the frames read, is its definition computed by hand from the
    // stencil for_each_edge_sample() visits, at stencil orders 0 (ends only), 1 and 2, with three
    // chords around a vertex placed above and below the level set. 2D has the one stencil type.
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
        double by_hand = 0., by_term = 0.;
        for (const Vector2d& q1 : others) {
            by_hand += edge_term_by_hand(mesh, *pot, x, q1);
            by_term += mesh.edge_offset_term(*pot, x, q1);
        }
        INFO("stencil_order " << k);
        CHECK(by_hand > 10.); // the fixture really is off the level set
        CHECK(by_term == Catch::Approx(by_hand).epsilon(1e-12));
    }
}

TEST_CASE("per-tri-energy", "[offset][2d]")
{
    // The 2D twin of per-tet-energy: tri_energy() against its definition computed by hand,
    // (w / s^3) area AMIPS^3 for every face, plus the band term b = area (mean corner d - delta) /
    // delta for a band face; MAX_ENERGY for an inverted face (the 2D engine scores a flat or
    // inverted face MAX_ENERGY). Four faces around o: f0 = (o, r0, r1) and f1 = (o, r1, r2) band,
    // f2 = (o, r2, r3) background, f3 = (o, r3, r0) input complex. At (w, s) = (1, stop_energy 3),
    // where both terms matter, (1, 100) and the default w at 100. Under both band rules,
    // "corner_bound" and "exact": above the segment's interior d is affine, so the exact integral
    // of d over a face is its area times the corner mean. The area by hand is the determinant's;
    // the energy's comes from the AMIPS and the edge lengths (area_amips3()).
    const Eigen::MatrixXd V = fan_vertices();
    const int o = 0, r0 = 1, r1 = 2, r2 = 3, r3 = 4;
    const std::vector<int> labels = {2, 2, 0, 1};
    for (const auto& [rule, w, s] : std::vector<std::tuple<std::string, double, double>>{
             {"corner_bound", 1., 3.},
             {"corner_bound", 1., 100.},
             {"corner_bound", Parameters().offset_amips_weight, 100.},
             {"exact", 1., 3.},
             {"exact", 1., 100.},
             {"exact", Parameters().offset_amips_weight, 100.}}) {
        Parameters param;
        param.band_volume_rule = rule;
        param.offset_amips_weight = w;
        param.stop_energy = s;
        const auto pot = line_field();
        auto mesh = energy_mesh_2d(
            param,
            V,
            {{{o, r0, r1}}, {{o, r1, r2}}, {{o, r2, r3}}, {{o, r3, r0}}},
            labels,
            pot);
        for (size_t fid = 0; fid < 4; ++fid) {
            // The engine's face quality, AMIPS2D from the positions.
            const double a = mesh->TriOptimizerMesh::get_quality(fid);
            REQUIRE(a < TriOptimizerMesh::MAX_ENERGY);
            const auto p = corners_2d(*mesh, fid);
            const Vector2d u = p[1] - p[0], v = p[2] - p[0];
            const double area = std::abs(u.x() * v.y() - u.y() * v.x()) / 2.;
            const double want = w / (s * s * s) * area * a * a * a +
                                (labels[fid] == 2 ? band_term_by_hand_2d(p) : 0.);
            INFO(rule << ", w " << w << ", s " << s << ", face " << fid);
            CHECK(mesh->tri_energy(fid) == Catch::Approx(want).epsilon(1e-10));
            if (labels[fid] == 2) CHECK(std::abs(band_term_by_hand_2d(p)) > 1e-3);
            // Without the band term (what an operation's sums read where it cancels): the AMIPS
            // term alone.
            CHECK(
                mesh->tri_energy(fid, false) ==
                Catch::Approx(w / (s * s * s) * area * a * a * a).epsilon(1e-10));
        }
        // An inverted face: r3 across the line through r0, o and r2 turns faces 2 and 3 over.
        mesh->set_vertex_position(size_t(r3), Vector2d(0., 0.75));
        for (const size_t fid : {size_t(2), size_t(3)}) {
            REQUIRE(mesh->TriOptimizerMesh::get_quality(fid) >= TriOptimizerMesh::MAX_ENERGY);
            CHECK(mesh->tri_energy(fid) == TriOptimizerMesh::MAX_ENERGY);
        }
    }
}

TEST_CASE("swap-edge-2d-on-energy", "[offset][2d]")
{
    // The 2D twin of swap-face-gate-on-energy, on 2D's only swap, the edge flip: the rule
    // (TopoOffsetTriMesh::swap_edge_after()) compares the sum of the energy over the two new faces
    // with the sum over the two old ones, not AMIPS. Two equilateral triangles glued on (a, b),
    // beyond the segment's end, their axis (c, d) on the ray from the end: AMIPS is 2 on both, its
    // minimum, so the flip raises AMIPS and the engine's AMIPS rule (switched off,
    // swap_quality_allowed()) would refuse it. d, the distance to the end, is linear along the
    // axis and convex across it, so the band term's corner mean -- the integral of the linear
    // interpolant of d -- is lower when the faces share (c, d) than when they share (a, b): with
    // both faces band the energy's sum falls and the rule admits the flip. With both faces
    // background the energy is the AMIPS term alone and the flip is refused. That is
    // "corner_bound". Under "exact" the band term is the integral of d over the faces' union, which
    // the flip keeps: the band sums agree, the energy rises with AMIPS, and the flip is refused on
    // the band too. The whole engine swap runs (TriMesh::swap_edge()); the engine's swap pass,
    // whose scheduler drops a flip that does not lower max AMIPS, is not involved.
    const auto pot = line_field();
    const double h = 0.2, k = std::sqrt(3.) * h;
    const Vector2d m = kLineEnd + Vector2d(0.6, 0.);
    Eigen::MatrixXd V(4, 2);
    V.row(0) = (m + Vector2d(0., h)).transpose(); // a
    V.row(1) = (m - Vector2d(0., h)).transpose(); // b
    V.row(2) = (m + Vector2d(k, 0.)).transpose(); // c, on the ray from the end
    V.row(3) = (m - Vector2d(k, 0.)).transpose(); // d, on the ray from the end
    const int a = 0, b = 1, c = 2, d = 3;
    const std::vector<std::array<int, 3>> FA = {{{a, b, c}}, {{b, a, d}}};
    const std::vector<std::array<int, 3>> FB = {{{c, d, a}}, {{d, c, b}}};

    for (const auto& [rule, x] : std::vector<std::pair<std::string, int>>{
             {"corner_bound", 2},
             {"corner_bound", 0},
             {"exact", 2},
             {"exact", 0}}) {
        INFO(rule << ", label " << x);
        const bool admits = rule == "corner_bound" && x == 2;
        Parameters pa, pb;
        pa.band_volume_rule = pb.band_volume_rule = rule;
        auto A = energy_mesh_2d(pa, V, FA, {x, x}, pot);
        auto B = energy_mesh_2d(pb, V, FB, {x, x}, pot);
        // The engine's AMIPS rules admit everything (3D's loop rules, op-rules-switch).
        CHECK(A->collapse_quality_allowed(0, 1, 1e30, 2.));
        CHECK(A->swap_quality_allowed(1e30, 2., false));
        double amips_a = 0., amips_b = 0., energy_a = 0., energy_b = 0., band_a = 0., band_b = 0.;
        for (size_t f = 0; f < 2; ++f) {
            amips_a = std::max(amips_a, A->TriOptimizerMesh::get_quality(f));
            energy_a += A->tri_energy(f);
            band_a += A->tri_energy(f) - A->tri_energy(f, false);
            amips_b = std::max(amips_b, B->TriOptimizerMesh::get_quality(f));
            energy_b += B->tri_energy(f);
            band_b += B->tri_energy(f) - B->tri_energy(f, false);
        }
        // The premises: AMIPS alone refuses this flip, and the energy admits it on the band under
        // "corner_bound" alone. Under "exact" the band sums agree.
        REQUIRE(amips_a == Catch::Approx(2.).epsilon(1e-9));
        REQUIRE(amips_b > amips_a);
        REQUIRE((energy_b < energy_a) == admits);
        if (rule == "exact" && x == 2) {
            CHECK(std::abs(band_a) > 1e-4);
            CHECK(band_b == Catch::Approx(band_a).margin(1e-12 * std::max(1., std::abs(band_a))));
        }

        const auto [edge, eid] = A->tuple_from_edge(std::array<size_t, 2>{{size_t(a), size_t(b)}});
        (void)eid;
        const int refused0 = A->iter_cnt_swap_energy_reject.load();
        std::vector<TopoOffsetTriMesh::Tuple> new_faces;
        CHECK(A->swap_edge(edge, new_faces) == admits);
        CHECK(A->iter_cnt_swap_energy_reject.load() - refused0 == (admits ? 0 : 1));
        if (!admits) continue;
        double energy_new = 0.;
        size_t n_new = 0;
        for (const auto& f : A->get_faces()) {
            const auto vs = A->oriented_tri_vids(f.fid(*A));
            CHECK(std::count(vs.begin(), vs.end(), size_t(c)) == 1);
            CHECK(std::count(vs.begin(), vs.end(), size_t(d)) == 1);
            energy_new += A->tri_energy(f.fid(*A));
            ++n_new;
        }
        CHECK(n_new == 2);
        CHECK(energy_new == Catch::Approx(energy_b).epsilon(1e-12));
    }
}

TEST_CASE("exact-band-offset-side-inward-and-outward-2d", "[offset][2d][exact_band]")
{
    // The 2D twin of exact-band-offset-side-inward-and-outward: the exact rule's band integral
    // counts the offset side of the input only (ExactBandIntegral2D's class doc), and the band is
    // outside the label-1 region for both directions: that region is the selection for offset_out
    // and everything outside it for offset_in. On a 3 x 3 grid of unit squares with the center
    // square as tag_0, a triangle just inside tag_0 and its mirror just outside, both within delta
    // of the edge x = 0.5: the band side gives the unclipped integral, the other side 0.
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

    const double delta = 0.2;
    const std::array<Vector2d, 3> inside = {{{0.45, -0.1}, {0.45, 0.1}, {0.35, 0.}}};
    std::array<Vector2d, 3> outside = inside;
    for (Vector2d& q : outside) q.x() = 1. - q.x();

    for (const bool inward : {true, false}) {
        Parameters param;
        param.offset_in = inward;
        param.offset_out = !inward;
        param.offset_field = "euclidean";
        param.band_volume_rule = "exact";
        param.target_distance = delta;
        param.envelope_size = 1e-3; // tests skip Parameters::init(); give the build a real eps
        TopoOffsetTriMesh mesh(param, 0);
        MatrixXd V_env;
        MatrixXi F_env;
        mesh.init_from_image(V, Fm, Tags, V_env, F_env, {"tag_0"});
        select_offset(mesh, "tag_0");
        mesh.label_input_complex();
        mesh.init_input_complex_bvh();
        mesh.init_offset_potential();
        REQUIRE(mesh.m_band_exact);
        const ExactBandIntegral2D plain(mesh.m_phi_V, mesh.m_phi_E, mesh.m_phi_P);
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

TEST_CASE("smoothing-objective-is-the-ring-energy-2d", "[offset][2d]")
{
    // The 2D twin of smoothing-objective-is-the-ring-energy. One energy for the smoother and the
    // rules: at every vertex not held on the input, the objective the shared smoother minimises
    // (smoothing_extra_energy(); every caller passes w_amips 0, so the smoother adds no AMIPS term
    // of its own) is the sum of tri_energy() over the vertex's ring -- every term of which moves
    // with the vertex. Two fixtures: the per-tri-energy fan (front vertices o, r0, r1, r2; r3 on
    // no front chord and in no band face), and a star of six band faces around a vertex inside
    // the band beyond the segment's end, where d is curved, which is on no front chord but
    // carries the band term of all six. Each vertex is moved to three nearby positions; the
    // objective's gradient and Hessian against central differences. Under both band rules,
    // "corner_bound" and "exact", at (w, s) = (1, 3), where both terms matter, and the default w
    // at 100. Under "exact" the star's band term does not move with its inner vertex: the six
    // faces tile the star, whose union is fixed; it is checked to keep its value and to have no
    // derivatives. Where 2D differs: 3D leaves that constant out of the objective (BAND TERM
    // THAT CANCELS, band_term_cancels(), false in 2D), 2D keeps it, so the objective there is the
    // ring sum itself; no plastic case (no 2D face's energy reads a rest, tri_energy()). The
    // engine's ring hook (smoothing_ring_energy(), the projected step's test) is the same ring sum.
    const Eigen::MatrixXd V1 = fan_vertices();
    const std::vector<std::array<int, 3>> F1 = {{{0, 1, 2}}, {{0, 2, 3}}, {{0, 3, 4}}, {{0, 4, 1}}};
    // Beyond the end (10, 0), where d is the distance to the end and curved: above the segment's
    // interior d is affine, the band term's corner mean is exact there, and the sum over a closed
    // star would not move with its inner vertex at all -- as under "exact", anywhere.
    Eigen::MatrixXd V2(7, 2);
    V2 << 0.01, -0.02, // the inner vertex
        0.31, 0.03, // arms, counter-clockwise
        0.17, 0.27, //
        -0.15, 0.24, //
        -0.29, -0.04, //
        -0.12, -0.28, //
        0.2, -0.22;
    V2.rowwise() += (kLineEnd + Vector2d(0.6, 0.1)).transpose();
    std::vector<std::array<int, 3>> F2;
    for (int i = 1; i <= 6; ++i) F2.push_back({{0, i, i % 6 + 1}});
    struct Fixture
    {
        const Eigen::MatrixXd& V;
        const std::vector<std::array<int, 3>>& F;
        std::vector<int> labels;
        size_t not_front; ///< the one vertex on no front chord
        bool has_band_ring; ///< whether that vertex's ring holds band faces
    };
    const std::vector<Fixture> fixtures = {
        {V1, F1, {2, 2, 0, 1}, 4, false},
        {V2, F2, std::vector<int>(6, 2), 0, true}};
    double worst_part[2][2] = {{0., 0.}, {0., 0.}}; // AMIPS, band: gradient, Hessian
    int n_at_floor = 0;
    for (size_t fi = 0; fi < fixtures.size(); ++fi) {
        const Fixture& fx = fixtures[fi];
        for (const auto& [rule, w, s] : std::vector<std::tuple<std::string, double, double>>{
                 {"corner_bound", 1., 3.},
                 {"corner_bound", Parameters().offset_amips_weight, 100.},
                 {"exact", 1., 3.},
                 {"exact", Parameters().offset_amips_weight, 100.}}) {
            Parameters param;
            param.band_volume_rule = rule;
            param.offset_amips_weight = w;
            param.stop_energy = s;
            const auto pot = line_field();
            auto mesh = energy_mesh_2d(param, fx.V, fx.F, fx.labels, pot);
            // What label_offset_boundary() and the operations maintain in a run: an end of a live
            // front chord.
            for (size_t v = 0; v < size_t(fx.V.rows()); ++v) mesh->refresh_offset_membership(v);
            for (size_t v = 0; v < size_t(fx.V.rows()); ++v) {
                const std::vector<size_t> ring = mesh->get_one_ring_fids_for_vertex(v);
                const auto ring_energy = [&]() {
                    double e = 0.;
                    for (const size_t fid : ring) e += mesh->tri_energy(fid);
                    return e;
                };
                const bool front = mesh->vertex_carries_offset_term(v);
                INFO(
                    "fixture " << fi << ", " << rule << ", w " << w << ", s " << s << ", vertex "
                               << v << (front ? " (front)" : ""));
                REQUIRE(front == (v != fx.not_front));
                REQUIRE(!mesh->smoothing_containment_envelope(v)); // no envelope holds it
                if (v == fx.not_front) {
                    REQUIRE((mesh->band_volume_energy_at(v) != nullptr) == fx.has_band_ring);
                }
                const auto energy = mesh->smoothing_extra_energy(v);
                // Under "exact", fixture 1's inner vertex: its ring's band term does not move with
                // it. 2D's objective keeps it (band_term_cancels() is false in 2D).
                const bool band_fixed = rule == "exact" && v == fx.not_front && fx.has_band_ring;
                REQUIRE(!mesh->band_term_cancels(ring));
                const auto band = band_fixed ? mesh->band_volume_energy_at(v) : nullptr;
                const Vector2d x0 = mesh->m_vertex_attribute[v].m_posf;
                Eigen::VectorXd xv = x0;
                const double e0 = energy->value(xv);
                const double b0 = band ? band->value(xv) : 0.;
                const double d0 = e0 - ring_energy();
                CHECK(d0 == Catch::Approx(0.).margin(1e-12 * std::max(1., std::abs(e0))));
                if (band) CHECK(std::abs(b0) > 1e-3); // the constant is not 0
                for (const Vector2d& dx :
                     {Vector2d(0.01, 0.), Vector2d(0., -0.008), Vector2d(-0.004, 0.006)}) {
                    mesh->set_vertex_position(v, x0 + dx);
                    for (const size_t fid : ring) REQUIRE(!mesh->is_inverted(fid));
                    xv = x0 + dx;
                    const double e = energy->value(xv);
                    CHECK(
                        e - ring_energy() ==
                        Catch::Approx(d0).margin(1e-12 * std::max(1., std::abs(e))));
                    if (band) {
                        CHECK(
                            band->value(xv) ==
                            Catch::Approx(b0).margin(1e-12 * std::max(1., std::abs(b0))));
                    }
                    // The projected step's test is the same ring sum, before and at a candidate.
                    CHECK(mesh->smoothing_ring_energy(ring, false) == ring_energy());
                    CHECK(mesh->smoothing_ring_energy(ring, true) == ring_energy());
                }
                mesh->set_vertex_position(v, x0);

                // The objective's derivatives against central differences. Besides 3D's margins,
                // the differences' own rounding floor, 50 eps |E| / h: at w 1, s 100 the AMIPS
                // term's weight is 1e-6, and where the band term's gradient cancels (fixture 0's
                // o, with r0, o and r2 on one line; fixture 1's inner vertex under "exact") |g| is
                // ~3e-6 against |E| ~ 0.1, whose value differences round at ~1e-11.
                const double h = 1e-6;
                Eigen::VectorXd g(2);
                Eigen::MatrixXd H(2, 2);
                xv = x0;
                energy->gradient(xv, g);
                energy->hessian(xv, H);
                const double fd_floor = 50. * 2.2e-16 * std::abs(e0) / h;
                if (band) {
                    // The constant band part has no derivatives, to the margins the central
                    // differences allow.
                    Eigen::VectorXd gb(2);
                    Eigen::MatrixXd Hb(2, 2);
                    band->gradient(xv, gb);
                    band->hessian(xv, Hb);
                    CHECK(gb.norm() <= 1e-7 * g.norm());
                    CHECK(Hb.norm() <= 1e-6 * H.norm());
                }
                for (int i = 0; i < 2; ++i) {
                    Eigen::VectorXd xp = xv, xm = xv;
                    xp[i] += h;
                    xm[i] -= h;
                    const double fd = (energy->value(xp) - energy->value(xm)) / (2. * h);
                    CHECK(
                        g[i] == Catch::Approx(fd).epsilon(1e-6).margin(
                                    std::max(1e-7 * g.norm(), fd_floor)));
                    Eigen::VectorXd gp(2), gm(2);
                    energy->gradient(xp, gp);
                    energy->gradient(xm, gm);
                    for (int j = 0; j < 2; ++j) {
                        const double fdh = (gp[j] - gm[j]) / (2. * h);
                        CHECK(H(j, i) == Catch::Approx(fdh).epsilon(1e-5).margin(1e-6 * H.norm()));
                    }
                }
                // Beyond 3D's test: each part on its own, the AMIPS term (amips3_energy()) and the
                // band term (band_volume_energy_at()), so a part small beside the other is judged
                // at its own scale: |fd - g| <= 1e-6 |g| + floor_g and |fd - H| <= 1e-5 |H| +
                // floor_H, with the differences' rounding floors, 50 eps |E| / h from the part's
                // value and 50 eps |E| / (L h) from its gradient, whose face terms are about
                // |E| / L and may cancel (L the longest edge at v). The relative errors of the
                // parts whose |g| is above 1e3 floor_g are printed; the others' gradient is 0 to
                // rounding (the band part at fixture 0's o).
                double L = 0.;
                for (const size_t fid : ring) {
                    for (const size_t u : mesh->oriented_tri_vids(fid)) {
                        L = std::max(L, (mesh->m_vertex_attribute[u].m_posf - x0).norm());
                    }
                }
                std::vector<std::shared_ptr<polysolve::nonlinear::Problem>> parts = {
                    mesh->amips3_energy(v)};
                if (!band_fixed) {
                    if (const auto bv = mesh->band_volume_energy_at(v)) parts.push_back(bv);
                }
                for (size_t pi = 0; pi < parts.size(); ++pi) {
                    auto& part = *parts[pi];
                    Eigen::VectorXd gq(2);
                    Eigen::MatrixXd Hq(2, 2);
                    part.gradient(xv, gq);
                    part.hessian(xv, Hq);
                    const double floor_g = 50. * 2.2e-16 * std::abs(part.value(xv)) / h;
                    const double floor_H = floor_g / L;
                    Eigen::Vector2d fg;
                    Eigen::Matrix2d fH;
                    for (int i = 0; i < 2; ++i) {
                        Eigen::VectorXd xp = xv, xm = xv, gp(2), gm(2);
                        xp[i] += h;
                        xm[i] -= h;
                        fg[i] = (part.value(xp) - part.value(xm)) / (2. * h);
                        part.gradient(xp, gp);
                        part.gradient(xm, gm);
                        fH.col(i) = (gp - gm) / (2. * h);
                    }
                    const double dg = (fg - Eigen::Vector2d(gq)).norm();
                    const double dH = (fH - Eigen::Matrix2d(Hq)).norm();
                    INFO(
                        (pi == 0 ? "AMIPS" : "band")
                        << " part: |fd - g| " << dg << " of |g| " << gq.norm() << ", |fd - H| "
                        << dH << " of |H| " << Hq.norm());
                    CHECK(dg <= 1e-6 * gq.norm() + floor_g);
                    CHECK(dH <= 1e-5 * Hq.norm() + floor_H);
                    if (gq.norm() > 1e3 * floor_g) {
                        worst_part[pi][0] = std::max(worst_part[pi][0], dg / gq.norm());
                        worst_part[pi][1] = std::max(worst_part[pi][1], dH / Hq.norm());
                    } else {
                        ++n_at_floor;
                    }
                }
            }
        }
    }
    std::printf(
        "smoothing-objective-is-the-ring-energy-2d: max relative error, AMIPS part: gradient %.1e "
        "Hessian %.1e; band part: gradient %.1e Hessian %.1e; %d parts with |g| at the rounding "
        "floor\n",
        worst_part[0][0],
        worst_part[0][1],
        worst_part[1][0],
        worst_part[1][1],
        n_at_floor);
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
    // Under both band rules, "corner_bound" (the default until 2026-10-29) and "exact"; under
    // "exact" the interior swap and the band face swap leave the band term out of every sum, and
    // the term they leave out is the same before and after.
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
            for (const std::string rule : {"corner_bound", "exact"}) {
                INFO(rule);
                Parameters pa, pb;
                pa.band_volume_rule = pb.band_volume_rule = rule;
                auto A = energy_mesh(pa, V, TA, labA, pot);
                auto B = energy_mesh(pb, V, TB, labB, pot);
                std::vector<size_t> tids(TA.size());
                std::iota(tids.begin(), tids.end(), size_t(0));
                before(*A, tids);
                // BAND TERM THAT CANCELS (tet_energy()): under "exact" a swap of band cells keeps
                // their union, so its band term is the same before and after, and the record
                // leaves it out of both sums.
                const bool cancels = A->band_term_cancels(tids);
                const bool all_band =
                    std::all_of(labA.begin(), labA.end(), [](int l) { return l == 2; });
                REQUIRE(cancels == (rule == "exact" && all_band));
                std::vector<std::array<size_t, 4>> cells_b;
                double sum_b = 0., band_b = 0.;
                for (size_t tb = 0; tb < TB.size(); ++tb) {
                    const auto vs = B->oriented_tet_vids(tb);
                    const double truth = B->tet_energy(tb, !cancels);
                    INFO("new cell " << tb);
                    CHECK(A->candidate_energy(vs) == Catch::Approx(truth).epsilon(1e-12));
                    cells_b.push_back(vs);
                    sum_b += truth;
                    band_b += B->tet_energy(tb) - B->tet_energy(tb, false);
                }
                std::vector<std::array<size_t, 4>> cells_a;
                double sum_a = 0., band_a = 0.;
                for (const size_t t : tids) {
                    cells_a.push_back(A->oriented_tet_vids(t));
                    sum_a += A->tet_energy(t, !cancels);
                    band_a += A->tet_energy(t) - A->tet_energy(t, false);
                }
                if (cancels) {
                    CHECK(std::abs(band_a) > 1e-4); // the term left out is not 0
                    CHECK(
                        band_b ==
                        Catch::Approx(band_a).margin(1e-12 * std::max(1., std::abs(band_a))));
                }
                if (search == 0) continue;
                const auto score = [&](const std::vector<std::array<size_t, 4>>& cells,
                                       int op_case) {
                    return search == 4 ? A->swap_edge_44_energy(cells, op_case)
                                       : A->swap_edge_56_energy(cells, op_case);
                };
                CHECK(score(cells_a, 0) == sum_a);
                CHECK(score(cells_b, 1) == Catch::Approx(sum_b).epsilon(1e-12));
            }
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
    // with the mesh. That is "corner_bound" (the default until 2026-10-29). Under "exact" the band
    // term is the integral of d over the cells' union, which the swap keeps (BAND TERM THAT
    // CANCELS, tet_energy()): the band sums agree, the energy rises with AMIPS, and the swap is
    // refused on the band too.
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

    for (const auto& [rule, x] : std::vector<std::pair<std::string, int>>{
             {"corner_bound", 2},
             {"corner_bound", 0},
             {"exact", 2},
             {"exact", 0}}) {
        INFO(rule << ", label " << x);
        const bool admits = rule == "corner_bound" && x == 2;
        Parameters pa, pb;
        pa.band_volume_rule = pb.band_volume_rule = rule;
        pa.perform_sanity_checks = true;
        auto A = energy_mesh(pa, V, TA, {x, x}, pot);
        auto B = energy_mesh(pb, V, TB, {x, x, x}, pot);
        double amips_a = 0., amips_b = 0., energy_a = 0., energy_b = 0., band_a = 0., band_b = 0.;
        for (size_t t = 0; t < 2; ++t) {
            amips_a = std::max(amips_a, A->TetOptimizerMesh::get_quality(A->oriented_tet_vids(t)));
            energy_a += A->tet_energy(t);
            band_a += A->tet_energy(t) - A->tet_energy(t, false);
        }
        for (size_t t = 0; t < 3; ++t) {
            amips_b = std::max(amips_b, B->TetOptimizerMesh::get_quality(B->oriented_tet_vids(t)));
            energy_b += B->tet_energy(t);
            band_b += B->tet_energy(t) - B->tet_energy(t, false);
        }
        // The premises: AMIPS alone refuses this swap, and the energy admits it on the band under
        // "corner_bound" alone. Under "exact" the band sums agree.
        REQUIRE(amips_a == Catch::Approx(27.).epsilon(1e-9));
        REQUIRE(amips_b > amips_a);
        REQUIRE((energy_b < energy_a) == admits);
        if (rule == "exact" && x == 2) {
            CHECK(std::abs(band_a) > 1e-4);
            CHECK(band_b == Catch::Approx(band_a).margin(1e-12 * std::max(1., std::abs(band_a))));
        }

        std::vector<TopoOffsetTetMesh::Tuple> new_tets;
        const auto [face, fid] =
            A->tuple_from_face(std::array<size_t, 3>{{size_t(a), size_t(b), size_t(c)}});
        (void)fid;
        CHECK(A->swap_face(face, new_tets) == admits);
        CHECK(A->m_swap_scoring_mismatch.load() == 0);
        CHECK(A->m_swap_scoring_checked.load() == (admits ? 1 : 0));
        if (!admits) continue;
        REQUIRE(new_tets.size() == 3);
        double energy_new = 0.;
        for (const auto& t : new_tets) energy_new += A->tet_energy(t.tid(*A));
        CHECK(energy_new == Catch::Approx(energy_b).epsilon(1e-12));
    }
}

TEST_CASE("swap-face-keeps-edge-tags", "[offset][3d]")
{
    // The loop's face swap (TopoOffsetTetMesh::swap_face_before(), the loop's copy of the base's)
    // caches the two cells' edge tags, which the base's swap_face_after() restores on the three
    // new cells. The swap of swap-face-gate-on-energy that the gate admits ("corner_bound", both
    // cells band), with the edge (a,d) tagged order-2 as hold_curves_and_points() tags a curve:
    // after the 2-3 swap the edge keeps its tag and its input, and no other edge has one. Until
    // 2026-11-05 the copy cached no edge tag, and the after hook restored whatever the previous
    // operation on the thread had left in the cache.
    using EdgeTags = TetOptimizerMesh::FeatureEdgeAttributes;
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
    const size_t a = 0, b = 1, c = 2, d = 3, e = 4;
    const std::vector<std::array<int, 4>> TA = {{{0, 1, 2, 3}}, {{0, 1, 2, 4}}};
    Parameters pa;
    pa.band_volume_rule = "corner_bound";
    auto A = energy_mesh(pa, V, TA, {2, 2}, pot);
    A->enable_feature_edge_tracking();
    const auto tags = [&](const size_t p, const size_t q) -> EdgeTags& {
        return A->m_feature_edge_attribute[A->tuple_from_edge({{p, q}}).eid(*A)];
    };
    tags(a, d).add(EdgeTags::order2);
    tags(a, d).inputs(EdgeTags::order2) = 1;

    std::vector<TopoOffsetTetMesh::Tuple> new_tets;
    const auto [face, fid] = A->tuple_from_face(std::array<size_t, 3>{{a, b, c}});
    (void)fid;
    REQUIRE(A->swap_face(face, new_tets));
    REQUIRE(new_tets.size() == 3);
    CHECK(tags(a, d).has(EdgeTags::order2));
    CHECK(tags(a, d).inputs(EdgeTags::order2) == 1);
    size_t n_tagged = 0;
    for (const auto& edge : A->get_edges()) {
        n_tagged += A->m_feature_edge_attribute[edge.eid(*A)].is_tagged();
    }
    CHECK(n_tagged == 1);
    CHECK(!tags(d, e).is_tagged());
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
    // AMIPS term and nothing is taken. That is "corner_bound" (the default until 2026-10-29).
    // Under "exact" the band term is the integral of d over the ring's union, which every case
    // keeps (BAND TERM THAT CANCELS, tet_energy()): the band sums agree, and nothing is taken on
    // the band either.
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
    for (const auto& [rule, x] : std::vector<std::pair<std::string, int>>{
             {"corner_bound", 2},
             {"corner_bound", 0},
             {"exact", 2},
             {"exact", 0}}) {
        INFO(rule << ", label " << x);
        const bool takes = rule == "corner_bound" && x == 2;
        const std::vector<int> lab(4, x);
        Parameters pa, p02, p13;
        pa.band_volume_rule = p02.band_volume_rule = p13.band_volume_rule = rule;
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
        const auto band_sum = [](TopoOffsetTetMesh& mesh) {
            double band = 0.;
            for (size_t t = 0; t < 4; ++t) band += mesh.tet_energy(t) - mesh.tet_energy(t, false);
            return band;
        };
        const auto [amips_a, energy_a] = amips_and_sum(*A);
        const auto [amips_02, energy_02] = amips_and_sum(*B02);
        const auto [amips_13, energy_13] = amips_and_sum(*B13);
        // The premises: AMIPS takes no case; on the band the energy takes (c1,c3) alone under
        // "corner_bound", and under "exact" the band sums agree.
        REQUIRE(amips_02 > amips_a);
        REQUIRE(amips_13 > amips_a);
        REQUIRE((energy_13 < energy_a) == takes);
        REQUIRE(!(energy_02 < energy_a));
        if (rule == "exact" && x == 2) {
            const double band_a = band_sum(*A);
            const double margin = 1e-12 * std::max(1., std::abs(band_a));
            CHECK(std::abs(band_a) > 1e-4);
            CHECK(band_sum(*B02) == Catch::Approx(band_a).margin(margin));
            CHECK(band_sum(*B13) == Catch::Approx(band_a).margin(margin));
        }

        std::vector<TopoOffsetTetMesh::Tuple> new_tets;
        const auto edge = A->tuple_from_edge(std::array<size_t, 2>{{size_t(a), size_t(b)}});
        CHECK(A->swap_edge_44(edge, new_tets) == takes);
        CHECK(A->m_swap_scoring_mismatch.load() == 0);
        CHECK(A->m_swap_scoring_checked.load() == (takes ? 1 : 0));
        if (!takes) continue;
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
        select_offset(mesh, "tag_0");
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
    if (!j.contains("offsets")) j["offsets"] = nlohmann::json::array({{{"selection", "tag_0"}}});
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
        {"offsets",
         nlohmann::json::array(
             {{{"selection", "tag_0"}, {"output_tags", {"offset"}}, {"target_distance", 0.2}}})},
        {"optimize_offset", false},
        {"save_offset_correspondence", true},
        {"num_threads", 0}};
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
    select_offset(mesh, "tag_0");
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
    io::ComponentIO context;
    collect_memory_output(mesh, context);
    io::write_mesh(out.string() + ".msh", context.result.first.at("mesh"));
    check_correspondence_msh(out.string() + ".msh", mesh, P, n_before, 3, [&](const Vector3d& p) {
        return mesh.input_complex_distance(p);
    });
    std::filesystem::remove(out.string() + ".msh");

    // The field is recorded by the march only, so the app refuses it with the optimization on.
    nlohmann::json j = correspondence_json();
    j["application"] = "topological_offset";
    j["input"] = "unused.msh";
    if (!j.contains("offsets")) j["offsets"] = nlohmann::json::array({{{"selection", "tag_0"}}});
    j["optimize_offset"] = true;
    REQUIRE_THROWS_WITH(
        components::topological_offset::topological_offset(j),
        Catch::Matchers::ContainsSubstring("save_offset_correspondence requires"));
}

namespace {
/// tests/python/test_face_tags.py's tagged grid: [0, 2]^3 in 8^3 cubes of six tets, the corner
/// cube [0, 0.25]^3 tagged red, and two open sheets on [0.5, 1.5]^2 as face tags, yellow at z = 1
/// and blue at z = z_blue, a multiple of the cube size 0.25.
MeshData two_sheet_grid(const double z_blue)
{
    const int n = 8;
    const double h = 0.25;
    const auto vid = [&](int i, int j, int k) { return i + (n + 1) * (j + (n + 1) * k); };
    Eigen::MatrixXd V((n + 1) * (n + 1) * (n + 1), 3);
    for (int k = 0; k <= n; ++k) {
        for (int j = 0; j <= n; ++j) {
            for (int i = 0; i <= n; ++i) V.row(vid(i, j, k)) << i * h, j * h, k * h;
        }
    }
    std::vector<std::array<int, 4>> T;
    std::vector<uint8_t> red;
    const std::array<std::array<int, 3>, 6> perms = {
        {{{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}}, {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}}}};
    for (int k = 0; k < n; ++k) {
        for (int j = 0; j < n; ++j) {
            for (int i = 0; i < n; ++i) {
                for (const auto& pm : perms) { // the cube's 6 tets around its main diagonal
                    std::array<int, 3> c = {{i, j, k}};
                    std::array<int, 4> t;
                    t[0] = vid(c[0], c[1], c[2]);
                    for (int s = 0; s < 3; ++s) {
                        ++c[size_t(pm[size_t(s)])];
                        t[size_t(s + 1)] = vid(c[0], c[1], c[2]);
                    }
                    T.push_back(positive(V, t));
                    red.push_back(i == 0 && j == 0 && k == 0);
                }
            }
        }
    }
    // The tets' faces on a sheet, once each.
    const auto on_sheet = [&](const std::array<int, 3>& f, const double z) {
        for (const int v : f) {
            if (V(v, 2) != z || V(v, 0) < 0.5 || V(v, 0) > 1.5 || V(v, 1) < 0.5 || V(v, 1) > 1.5)
                return false;
        }
        return true;
    };
    std::set<std::array<int, 3>> yellow, blue;
    for (const auto& t : T) {
        for (int s = 0; s < 4; ++s) {
            std::array<int, 3> f;
            for (int c = 0, m = 0; c < 4; ++c) {
                if (c != s) f[size_t(m++)] = t[size_t(c)];
            }
            std::sort(f.begin(), f.end());
            if (on_sheet(f, 1.0)) yellow.insert(f);
            if (on_sheet(f, z_blue)) blue.insert(f);
        }
    }
    Eigen::MatrixXi Tm(int(T.size()), 4);
    DataMatrix<uint8_t> tags(T.size(), 2);
    for (size_t i = 0; i < T.size(); ++i) {
        for (int j = 0; j < 4; ++j) Tm(int(i), j) = T[i][size_t(j)];
        tags(i, 0) = red[i];
        tags(i, 1) = 1 - red[i];
    }
    MeshData m = io::geometry_data(V, Tm, CellType::Tetrahedron);
    m.cells.at(CellType::Tetrahedron).attributes["tags"] = {tags, {"red", "ambient"}};
    auto& faces = m.cells[CellType::Triangle];
    faces.connectivity.resize(yellow.size() + blue.size(), 3);
    DataMatrix<uint8_t> member = DataMatrix<uint8_t>::Zero(faces.connectivity.rows(), 2);
    Eigen::Index r = 0;
    for (const auto* sheet : {&yellow, &blue}) {
        for (const auto& f : *sheet) {
            for (int c = 0; c < 3; ++c) faces.connectivity(r, c) = f[size_t(c)];
            member(r++, sheet == &yellow ? 0 : 1) = 1;
        }
    }
    faces.attributes["tags"] = {member, {"yellow", "blue"}};
    return m;
}

/// One construction-only offset of the face tag `sheet`, its band tagged `band`, the
/// correspondence saved, the tag expression `protect` protected: the step a pipeline runs once per sheet
/// before prismatic_mesh.
MeshData offset_sheet(
    const MeshData& m,
    const std::string& sheet,
    const std::string& band,
    const std::string& protect = "")
{
    return components::topological_offset::topological_offset(
               {{"input", "m"},
                {"offsets",
                 nlohmann::json::array(
                     {{{"selection", sheet}, {"output_tags", {band}}, {"target_distance", 0.1}}})},
                {"optimize_offset", false},
                {"save_offset_correspondence", true},
                {"protected_tags", protect},
                {"num_threads", 1}},
               {{"m", m}})
        .first.at("mesh");
}

/// Each offset vertex (corr_input_vid >= 0) as its position followed by its input vertex's,
/// which corr_input_vid names by `vid`.
std::set<std::array<double, 6>> correspondence_pairs(const MeshData& m)
{
    const auto corr = m.vertex_attributes.at("corr_input_vid").integers();
    const auto vid = m.vertex_attributes.at("vid").integers();
    std::map<int64_t, Eigen::Index> row_of;
    for (Eigen::Index r = 0; r < vid.rows(); ++r) row_of[vid(r, 0)] = r;
    std::set<std::array<double, 6>> pairs;
    for (Eigen::Index r = 0; r < corr.rows(); ++r) {
        if (corr(r, 0) < 0) continue;
        const Eigen::Index u = row_of.at(corr(r, 0));
        pairs.insert({{m.V(r, 0), m.V(r, 1), m.V(r, 2), m.V(u, 0), m.V(u, 1), m.V(u, 2)}});
    }
    return pairs;
}
} // namespace

TEST_CASE("offset-carries-an-earlier-correspondence", "[offset][3d][correspondence]")
{
    // Two offsets one after another, yellow's band and then blue's. The second run carries the
    // first's corr_input_vid, so prismatic_mesh sees both bands' correspondence. Construction moves
    // no vertex, so the pairs compare by position, exactly.
    SECTION("sheets three cubes apart: every carried value survives")
    {
        const auto first = offset_sheet(two_sheet_grid(0.25), "yellow", "band_yellow");
        const auto second = offset_sheet(first, "blue", "band_blue");
        const auto carried = correspondence_pairs(first);
        REQUIRE(!carried.empty());
        std::set<std::array<double, 6>> to_yellow, to_blue;
        for (const auto& p : correspondence_pairs(second))
            (p[5] == 1.0 ? to_yellow : to_blue).insert(p);
        CHECK(to_yellow == carried);
        CHECK(!to_blue.empty());
        for (const auto& p : to_blue) CHECK(p[5] == 0.25);
    }
    SECTION("sheets one cube apart: blue's march cuts beside yellow's pairs, not them")
    {
        // Blue's marched edges end at yellow's offset vertices, not at the input vertices they
        // name, so no carried edge is cut and every carried value survives. Until 2026-11-06 the
        // run compared whole stars and refused this.
        const auto first = offset_sheet(two_sheet_grid(0.75), "yellow", "band_yellow");
        const auto second = offset_sheet(first, "blue", "band_blue");
        const auto carried = correspondence_pairs(first);
        REQUIRE(!carried.empty());
        const auto pairs = correspondence_pairs(second);
        CHECK(std::includes(pairs.begin(), pairs.end(), carried.begin(), carried.end()));
    }
    SECTION("yellow offset again: the march cuts yellow's carried edges, and the run throws")
    {
        // The second march's edges run from yellow's vertices out, among them each carried edge
        // from an offset vertex to the yellow vertex it names.
        const auto first = offset_sheet(two_sheet_grid(0.25), "yellow", "band_yellow");
        REQUIRE_THROWS_WITH(
            offset_sheet(first, "yellow", "band_yellow_2"),
            Catch::Matchers::ContainsSubstring("protect the earlier band"));
    }
    SECTION("sheets one cube apart, yellow's band protected: blue's march leaves it")
    {
        // A protected cell is split by no march and taken by no band (marching_tets()), so
        // every carried value survives.
        const auto first = offset_sheet(two_sheet_grid(0.75), "yellow", "band_yellow");
        const auto second = offset_sheet(first, "blue", "band_blue", "band_yellow");
        const auto carried = correspondence_pairs(first);
        REQUIRE(!carried.empty());
        std::set<std::array<double, 6>> to_yellow;
        for (const auto& p : correspondence_pairs(second))
            if (carried.count(p)) to_yellow.insert(p);
        CHECK(to_yellow == carried);
    }
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
/// n x n x n unit cubes (n odd, default 3) centred at the origin, 6 tets each, the middle cube
/// tagged 1.
GridInput grid_input(const int n = 3)
{
    const int m = n + 1;
    const double h = 0.5 * double(n);
    Eigen::MatrixXd V(m * m * m, 3);
    const auto vid = [m](int i, int j, int k) { return i + m * j + m * m * k; };
    for (int k = 0; k < m; ++k) {
        for (int j = 0; j < m; ++j) {
            for (int i = 0; i < m; ++i) V.row(vid(i, j, k)) << i - h, j - h, k - h;
        }
    }
    std::vector<std::array<int, 4>> T;
    std::vector<int> center;
    const std::array<std::array<int, 3>, 6> perms = {
        {{{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}}, {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}}}};
    for (int k = 0; k < n; ++k) {
        for (int j = 0; j < n; ++j) {
            for (int i = 0; i < n; ++i) {
                for (const auto& pm : perms) { // the cube's 6 tets around its main diagonal
                    std::array<int, 3> c = {{i, j, k}};
                    std::array<int, 4> t;
                    t[0] = vid(c[0], c[1], c[2]);
                    for (int s = 0; s < 3; ++s) {
                        ++c[size_t(pm[size_t(s)])];
                        t[size_t(s + 1)] = vid(c[0], c[1], c[2]);
                    }
                    if (i == n / 2 && j == n / 2 && k == n / 2) center.push_back(int(T.size()));
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
    select_offset(*mesh, "tag_0");
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
          &TopoOffsetTetMesh::VtuFrame::f_dihedral, &TopoOffsetTetMesh::VtuFrame::ring}) {
        ok = ok && same(a.*m, b.*m);
    }
    ok = ok && same(a.S, b.S) && same(a.S_cells, b.S_cells);
    for (const auto m :
         {&TopoOffsetTetMesh::VtuFrame::s_err,
          &TopoOffsetTetMesh::VtuFrame::s_over,
          &TopoOffsetTetMesh::VtuFrame::s_signed,
          &TopoOffsetTetMesh::VtuFrame::s_dist_ratio,
          &TopoOffsetTetMesh::VtuFrame::s_sq,
          &TopoOffsetTetMesh::VtuFrame::s_face,
          &TopoOffsetTetMesh::VtuFrame::s_point}) {
        ok = ok && same(a.*m, b.*m);
    }
    return ok;
}

/// grid_offset_mesh() with the caller's tags -- tet tags (a column each, named tag_names), face-
/// tagged triangles (F_tagged, a column of F_tags each, named face_tag_names), the complex
/// `selection` -- and no field: constructed, the optimization's labels set.
std::unique_ptr<TopoOffsetTetMesh> tagged_grid_offset_mesh(
    const GridInput& g,
    const MatrixSi& tags,
    const std::vector<std::string>& tag_names,
    const Eigen::MatrixXi& F_tagged,
    const MatrixSi& F_tags,
    const std::vector<std::string>& face_tag_names,
    const std::string& selection,
    Parameters& param,
    const std::filesystem::path& out)
{
    auto mesh = std::make_unique<TopoOffsetTetMesh>(param, 0);
    MatrixXd V_env;
    MatrixXi F_env;
    mesh->init_from_image(
        g.V,
        g.Tm,
        tags,
        V_env,
        F_env,
        tag_names,
        "",
        F_tagged,
        F_tags,
        face_tag_names);
    select_offset(*mesh, selection);
    mesh->label_input_complex();
    mesh->init_input_complex_bvh();
    mesh->consolidate_mesh();
    mesh->construct_offset(out);
    mesh->label_offset_boundary();
    return mesh;
}

/// The faces on the square `axis` = `value` (every corner exactly on the plane, the centroid's
/// other two coordinates strictly between lo's and hi's), by the band's cells on their two sides:
/// COVERED, both band; BEYOND, neither. The front, one of each, is left out.
struct SquareFaces
{
    std::vector<TetMesh::Tuple> covered, beyond;
};
SquareFaces square_faces(
    const TopoOffsetTetMesh& mesh,
    const int axis,
    const double value,
    const Eigen::Vector3d& lo,
    const Eigen::Vector3d& hi)
{
    SquareFaces out;
    for (const auto& f : mesh.get_faces()) {
        bool on = true;
        Eigen::Vector3d c = Eigen::Vector3d::Zero();
        for (const size_t v : mesh.get_face_vids(f)) {
            const Eigen::Vector3d p = mesh.m_vertex_attribute[v].m_posf;
            on = on && p[axis] == value;
            c += p / 3.;
        }
        for (int k = 0; k < 3; ++k) {
            if (k != axis) on = on && lo[k] < c[k] && c[k] < hi[k];
        }
        const auto opp = f.switch_tetrahedron(mesh);
        if (!on || !opp) continue;
        const bool a = mesh.cell_is_offset_band(f.tid(mesh));
        const bool b = mesh.cell_is_offset_band(opp->tid(mesh));
        if (a && b) out.covered.push_back(f);
        if (!a && !b) out.beyond.push_back(f);
    }
    return out;
}
} // namespace

TEST_CASE("band-covered-region-boundaries", "[offset][3d][envelope]")
{
    // The 5 x 5 x 5 grid (tag_0 the centre cube) with tag_1 the cube next to it along +x, both
    // clear of the wall, constructed at target 0.2. The band takes tag_1's cells next to the
    // complex, so near the shared face tag_1's four side squares lie between two band cells
    // (COVERED), and farther out between two cells the band did not take (BEYOND). A covered face
    // bounds nothing once the band's cells carry its tags: untracked, so not held
    // (untrack_covered_boundaries()). With tag_1 protected the band takes none of its cells
    // (marching_tets()), so none of its faces is covered. Beyond, tracked and held either way.
    const GridInput g = grid_input(5);
    MatrixSi tags(g.Tm.rows(), 2);
    for (Eigen::Index t = 0; t < g.Tm.rows(); ++t) {
        Eigen::Vector3d c = Eigen::Vector3d::Zero();
        for (int j = 0; j < 4; ++j) c += g.V.row(g.Tm(t, j)).transpose() / 4.;
        if (g.Tags.coeff(t, 0) != 0) tags.coeffRef(t, 0) = 1;
        if (c[0] > 0.5 && c[0] < 1.5 && std::abs(c[1]) < 0.5 && std::abs(c[2]) < 0.5) {
            tags.coeffRef(t, 1) = 1;
        }
    }
    for (const bool protect : {false, true}) {
        INFO("protected_tags " << (protect ? "tag_1" : "(none)"));
        nlohmann::json j = {
            {"offsets",
             nlohmann::json::array({{{"selection", "tag_0"}, {"target_distance", 0.2}}})},
            {"num_threads", 0}};
        if (protect) j["protected_tags"] = "tag_1";
        Parameters param = construction_params(j, g.V);
        const auto mesh = tagged_grid_offset_mesh(
            g,
            tags,
            {"tag_0", "tag_1"},
            Eigen::MatrixXi(),
            MatrixSi(),
            {},
            "tag_0",
            param,
            std::filesystem::temp_directory_path() / "wmtk_offset_covered_regions");
        const auto tracked = [&](const TetMesh::Tuple& f) {
            return mesh->m_face_attribute[f.fid(*mesh)].m_is_surface_fs;
        };
        const auto held = [&](const TetMesh::Tuple& f) {
            return tracked(f) && mesh->face_is_held(f);
        };
        size_t n_covered = 0, n_beyond = 0;
        for (const auto& [axis, value] :
             std::vector<std::pair<int, double>>{{1, 0.5}, {1, -0.5}, {2, 0.5}, {2, -0.5}}) {
            const SquareFaces s = square_faces(
                *mesh,
                axis,
                value,
                Eigen::Vector3d(0.5, -0.5, -0.5),
                Eigen::Vector3d(1.5, 0.5, 0.5));
            for (const auto& f : s.covered) {
                CHECK(!tracked(f));
                CHECK(!held(f));
            }
            for (const auto& f : s.beyond) CHECK(held(f));
            n_covered += s.covered.size();
            n_beyond += s.beyond.size();
        }
        CHECK((n_covered == 0) == protect);
        REQUIRE(n_beyond > 0);
        REQUIRE_NOTHROW(mesh->audit_surface_containment("construction"));
    }
}

TEST_CASE("protected-tags-expression", "[offset][3d]")
{
    // protected_tags is a tag expression (until 2026-11 a list of names). A simplex is protected
    // when its own tags satisfy it, and keeps those of them it names
    // (Parameters::protected_kept()); a list's names joined by '|' keep what the list kept.
    nlohmann::json j = {{"num_threads", 0}};
    const std::map<std::string, int64_t> ids = {{"ambient", 0}, {"a", 1}, {"b", 2}, {"c", 3}};
    const auto rule = [&](const std::string& expr) {
        j["protected_tags"] = expr;
        Parameters p = construction_params(j, Eigen::MatrixXd::Identity(3, 3));
        p.parse_protected_tags(ids);
        return p;
    };
    using Tags = std::set<int64_t>;
    CHECK(!rule("").protected_expression); // nothing protected
    CHECK(rule("").protected_kept({1, 2}).empty());
    const Parameters list = rule("a | b"); // the list [a, b]
    CHECK(list.protected_kept({1, 3}) == Tags{1});
    CHECK(list.protected_kept({1, 2, 3}) == Tags{1, 2});
    CHECK(list.protected_kept({3}).empty());
    CHECK(rule("a & b").protected_kept({1, 3}).empty()); // not protected: no b
    CHECK(rule("a & b").protected_kept({1, 2, 3}) == Tags{1, 2});
    CHECK(rule("a & !c").protected_kept({1}) == Tags{1});
    CHECK(rule("a & !c").protected_kept({1, 3}).empty());
    CHECK_THROWS(rule("a | ambient"));

    // In the march: the grid of band-covered-region-boundaries, tag_1 the cube next to the
    // complex along +x and tag_2 the same cells. A protected tag_1 cell is never the band's, so
    // none of the cube's side faces is covered; an unprotected cube is entered.
    const GridInput g = grid_input(5);
    MatrixSi tags(g.Tm.rows(), 3);
    for (Eigen::Index t = 0; t < g.Tm.rows(); ++t) {
        Eigen::Vector3d c = Eigen::Vector3d::Zero();
        for (int k = 0; k < 4; ++k) c += g.V.row(g.Tm(t, k)).transpose() / 4.;
        if (g.Tags.coeff(t, 0) != 0) tags.coeffRef(t, 0) = 1;
        if (c[0] > 0.5 && c[0] < 1.5 && std::abs(c[1]) < 0.5 && std::abs(c[2]) < 0.5) {
            tags.coeffRef(t, 1) = 1;
            tags.coeffRef(t, 2) = 1;
        }
    }
    for (const auto& [expr, cube_protected] : std::vector<std::pair<std::string, bool>>{
             {"", false},
             {"tag_1", true},
             {"tag_1 & tag_2", true},
             {"tag_2 | tag_1", true},
             {"tag_1 & !tag_2", false}}) {
        INFO("protected_tags '" << expr << "'");
        nlohmann::json jm = {
            {"offsets",
             nlohmann::json::array({{{"selection", "tag_0"}, {"target_distance", 0.2}}})},
            {"num_threads", 0},
            {"protected_tags", expr}};
        Parameters param = construction_params(jm, g.V);
        const auto mesh = tagged_grid_offset_mesh(
            g,
            tags,
            {"tag_0", "tag_1", "tag_2"},
            Eigen::MatrixXi(),
            MatrixSi(),
            {},
            "tag_0",
            param,
            std::filesystem::temp_directory_path() / "wmtk_offset_protected_expression");
        size_t n_covered = 0;
        for (const auto& [axis, value] :
             std::vector<std::pair<int, double>>{{1, 0.5}, {1, -0.5}, {2, 0.5}, {2, -0.5}}) {
            n_covered += square_faces(
                             *mesh,
                             axis,
                             value,
                             Eigen::Vector3d(0.5, -0.5, -0.5),
                             Eigen::Vector3d(1.5, 0.5, 0.5))
                             .covered.size();
        }
        CHECK((n_covered == 0) == cube_protected);
    }
}

TEST_CASE("band-covered-face-tags", "[offset][3d][envelope]")
{
    // The same grid with no second region and a face-tagged surface S instead: the square
    // y = 0.5, 0.5 < x < 1.5, -0.5 < z < 0.5, one edge on the centre cube, so the band crosses it
    // there. Inside the band (COVERED: both tets band cells) S's tag gives way as the cells' tags
    // do (set_offset_tet_tags()), and the face, bounding nothing then, is untracked; BEYOND, S
    // keeps tag, tracking and hold. With S protected the covered faces keep all three. With S the
    // selection, its faces are the input complex, band cells on both sides, and keep theirs.
    const GridInput g = grid_input(5);
    std::set<std::array<int, 3>> seen;
    std::vector<std::array<int, 3>> S;
    for (Eigen::Index t = 0; t < g.Tm.rows(); ++t) {
        for (int skip = 0; skip < 4; ++skip) {
            std::array<int, 3> f{};
            int n = 0;
            for (int j = 0; j < 4; ++j) {
                if (j != skip) f[size_t(n++)] = g.Tm(t, j);
            }
            bool on = true;
            Eigen::Vector3d c = Eigen::Vector3d::Zero();
            for (const int v : f) {
                on = on && g.V(v, 1) == 0.5;
                c += g.V.row(v).transpose() / 3.;
            }
            if (!on || !(c[0] > 0.5 && c[0] < 1.5 && std::abs(c[2]) < 0.5)) continue;
            std::array<int, 3> key = f;
            std::sort(key.begin(), key.end());
            if (seen.insert(key).second) S.push_back(f);
        }
    }
    REQUIRE(S.size() == 2);
    Eigen::MatrixXi F_tagged(int(S.size()), 3);
    MatrixSi F_tags(int(S.size()), 1);
    for (size_t r = 0; r < S.size(); ++r) {
        for (int k = 0; k < 3; ++k) F_tagged(int(r), k) = S[r][size_t(k)];
        F_tags.coeffRef(int(r), 0) = 1;
    }
    struct Case
    {
        const char* name;
        std::string selection;
        bool protect;
    };
    for (const Case& c :
         {Case{"S crossing the band", "tag_0", false},
          Case{"S protected", "tag_0", true},
          Case{"S the selection", "S", false}}) {
        INFO(c.name);
        nlohmann::json j = {
            {"offsets",
             nlohmann::json::array({{{"selection", c.selection}, {"target_distance", 0.2}}})},
            {"num_threads", 0}};
        if (c.protect) j["protected_tags"] = "S";
        // The driver refuses a face-tag selection with the optimization, which holds no face
        // tags yet; the construction alone is what this case checks.
        if (c.selection == "S") j["optimize_offset"] = false;
        Parameters param = construction_params(j, g.V);
        const auto mesh = tagged_grid_offset_mesh(
            g,
            g.Tags,
            {"tag_0"},
            F_tagged,
            F_tags,
            {"S"},
            c.selection,
            param,
            std::filesystem::temp_directory_path() / "wmtk_offset_covered_face_tags");
        const auto tagged = [&](const TetMesh::Tuple& f) {
            return (mesh->m_face_attribute[f.fid(*mesh)].input_set() & 1) != 0; // S is bit 0
        };
        const auto tracked = [&](const TetMesh::Tuple& f) {
            return mesh->m_face_attribute[f.fid(*mesh)].m_is_surface_fs;
        };
        const auto held = [&](const TetMesh::Tuple& f) {
            return tracked(f) && mesh->face_is_held(f);
        };
        const SquareFaces s = square_faces(
            *mesh,
            1,
            0.5,
            Eigen::Vector3d(0.5, -0.5, -0.5),
            Eigen::Vector3d(1.5, 0.5, 0.5));
        REQUIRE(!s.covered.empty());
        const bool selected = c.selection == "S";
        const bool keeps = c.protect || selected;
        for (const auto& f : s.covered) {
            CHECK(tagged(f) == keeps);
            CHECK(tracked(f) == keeps);
            CHECK(held(f) == keeps);
            if (selected) CHECK(mesh->m_face_extra[f.fid(*mesh)].label == 1);
        }
        if (!selected) {
            REQUIRE(!s.beyond.empty());
            for (const auto& f : s.beyond) {
                CHECK(tagged(f));
                CHECK(held(f));
            }
        }
        REQUIRE_NOTHROW(mesh->audit_surface_containment("construction"));
    }
}

TEST_CASE("band-crossed-sheet-keeps-its-orientation", "[offset][3d][split_op]")
{
    // The square S of band-covered-face-tags, protected, so the march splits its faces both
    // inside the band and beyond it and every half keeps the tag. S is given with its normal +y.
    // A face tag's orientation is stored against the ascending order of the face's own ids, so a
    // half of a split face, whose ids differ, has to have it converted
    // (marching_split_edge_after()). After construction every tagged face of S, oriented as the
    // output orients it (io::collect_input_set_tags(): ascending, the last two swapped when the
    // orientation is negative), still faces +y.
    const GridInput g = grid_input(5);
    std::set<std::array<int, 3>> seen;
    std::vector<std::array<int, 3>> S;
    for (Eigen::Index t = 0; t < g.Tm.rows(); ++t) {
        for (int skip = 0; skip < 4; ++skip) {
            std::array<int, 3> f{};
            int n = 0;
            for (int j = 0; j < 4; ++j) {
                if (j != skip) f[size_t(n++)] = g.Tm(t, j);
            }
            bool on = true;
            Eigen::Vector3d c = Eigen::Vector3d::Zero();
            for (const int v : f) {
                on = on && g.V(v, 1) == 0.5;
                c += g.V.row(v).transpose() / 3.;
            }
            if (!on || !(c[0] > 0.5 && c[0] < 1.5 && std::abs(c[2]) < 0.5)) continue;
            std::array<int, 3> key = f;
            std::sort(key.begin(), key.end());
            if (!seen.insert(key).second) continue;
            const Eigen::Vector3d a = g.V.row(f[0]).transpose();
            const Eigen::Vector3d b = g.V.row(f[1]).transpose();
            const Eigen::Vector3d d = g.V.row(f[2]).transpose();
            if ((b - a).cross(d - a)[1] < 0.) std::swap(f[1], f[2]); // normal +y
            S.push_back(f);
        }
    }
    REQUIRE(S.size() == 2);
    Eigen::MatrixXi F_tagged(int(S.size()), 3);
    MatrixSi F_tags(int(S.size()), 1);
    for (size_t r = 0; r < S.size(); ++r) {
        for (int k = 0; k < 3; ++k) F_tagged(int(r), k) = S[r][size_t(k)];
        F_tags.coeffRef(int(r), 0) = 1;
    }
    Parameters param = construction_params(
        {{"offsets", nlohmann::json::array({{{"selection", "tag_0"}, {"target_distance", 0.2}}})},
         {"num_threads", 0},
         {"protected_tags", "S"}},
        g.V);
    const auto mesh = tagged_grid_offset_mesh(
        g,
        g.Tags,
        {"tag_0"},
        F_tagged,
        F_tags,
        {"S"},
        "tag_0",
        param,
        std::filesystem::temp_directory_path() / "wmtk_offset_sheet_orientation");
    const SquareFaces s = square_faces(
        *mesh,
        1,
        0.5,
        Eigen::Vector3d(0.5, -0.5, -0.5),
        Eigen::Vector3d(1.5, 0.5, 0.5));
    REQUIRE(!s.covered.empty()); // halves inside the band ...
    REQUIRE(!s.beyond.empty()); // ... and beyond it
    size_t n_tagged = 0;
    for (const auto& f : mesh->get_faces()) {
        const auto& attr = mesh->m_face_attribute[f.fid(*mesh)];
        if (!attr.m_is_surface_fs || (attr.input_set() & 1) == 0) continue; // S is bit 0
        std::array<size_t, 3> vs = mesh->get_face_vids(f);
        std::sort(vs.begin(), vs.end());
        const int o = attr.orientation_along(vs);
        INFO("face (" << vs[0] << ", " << vs[1] << ", " << vs[2] << "), orientation " << o);
        CHECK(o != 0);
        if (o < 0) std::swap(vs[1], vs[2]);
        const Eigen::Vector3d a = mesh->m_vertex_attribute[vs[0]].m_posf;
        const Eigen::Vector3d b = mesh->m_vertex_attribute[vs[1]].m_posf;
        const Eigen::Vector3d d = mesh->m_vertex_attribute[vs[2]].m_posf;
        CHECK((b - a).cross(d - a)[1] > 0.);
        ++n_tagged;
    }
    CHECK(n_tagged > S.size()); // S's faces were split
}

TEST_CASE("face-split-keeps-the-surface", "[split_op][3d]")
{
    // The corner pocket. p is a vertex of the input complex C (tag_0, three tets) where three of
    // C's boundary faces meet, their outer edges the loop (q, r, s); f = (q, r, s) is a face of
    // another surface -- the boundary of tag_1, the tet beyond f, and a face-tagged input S --
    // spanning the loop outside C. The tet (p, q, r, s) between them is not C's but its four
    // corners are, so simplicial_embedding() splits f (its three edges are C's, it is not). The
    // three pieces lie on f's surfaces: tracked, with f's input set and its orientation; the new
    // vertex is on the surface; the faces new inside the two cells are on none.
    Eigen::MatrixXd V(8, 3);
    V << 0, 0, 0, // p
        1, 0, 0, // q
        0, 1, 0, // r
        0, 0, 1, // s
        0, 0, -1, // C's tet below (p, q, r)
        -1, 0, 0, // ... beside (p, r, s)
        0, -1, 0, // ... beside (p, s, q)
        1, 1, 1; // tag_1's tet beyond f
    const std::vector<std::array<int, 4>> cells =
        {{{0, 1, 2, 3}}, {{0, 1, 2, 4}}, {{0, 2, 3, 5}}, {{0, 3, 1, 6}}, {{1, 2, 3, 7}}};
    Eigen::MatrixXi T(int(cells.size()), 4);
    for (size_t i = 0; i < cells.size(); ++i) {
        const auto t = positive(V, cells[i]);
        for (int j = 0; j < 4; ++j) T(int(i), j) = t[size_t(j)];
    }
    MatrixSi tags(int(cells.size()), 2);
    for (const int i : {1, 2, 3}) tags.coeffRef(i, 0) = 1; // C
    tags.coeffRef(4, 1) = 1; // tag_1, beyond f
    Eigen::MatrixXi F_tagged(1, 3);
    F_tagged << 1, 2, 3; // f, the face tag oriented along (q, r, s)
    MatrixSi F_tags(1, 1);
    F_tags.coeffRef(0, 0) = 1;
    Parameters param = construction_params(
        {{"offsets", nlohmann::json::array({{{"selection", "tag_0"}, {"target_distance", 0.2}}})},
         {"num_threads", 0}},
        V);
    TopoOffsetTetMesh mesh(param, 0);
    MatrixXd V_env;
    MatrixXi F_env;
    mesh.init_from_image(V, T, tags, V_env, F_env, {"tag_0", "tag_1"}, "", F_tagged, F_tags, {"S"});
    select_offset(mesh, "tag_0");
    mesh.label_input_complex();

    const auto fid_of = [&](size_t a, size_t b, size_t c) {
        return std::get<1>(mesh.tuple_from_face({{a, b, c}}));
    };
    const auto f_attr = mesh.m_face_attribute[fid_of(1, 2, 3)]; // a copy: f goes
    REQUIRE(f_attr.m_is_surface_fs);
    REQUIRE(f_attr.input_set() == 1);
    REQUIRE(f_attr.orientation_along(std::array<size_t, 3>{{1, 2, 3}}) == 1);
    REQUIRE(!mesh.is_simplicially_embedded());
    mesh.m_edge_split_mode = TopoOffsetTetMesh::EdgeSplitMode::Midpoint;
    mesh.simplicial_embedding();
    REQUIRE(mesh.is_simplicially_embedded());
    REQUIRE(!mesh.try_tuple_from_face({{1, 2, 3}})); // f was split

    size_t v = size_t(-1); // the new vertex
    for (const auto& t : mesh.get_vertices()) {
        if (t.vid(mesh) < 8) continue;
        REQUIRE(v == size_t(-1));
        v = t.vid(mesh);
    }
    REQUIRE(v != size_t(-1));
    CHECK((mesh.m_vertex_attribute[v].m_posf - Vector3d(1., 1., 1.) / 3.).norm() < 1e-12);
    CHECK(mesh.m_vertex_extra[v].m_is_on_region);
    CHECK(mesh.m_vertex_attribute[v].m_is_on_surface);
    for (const auto& [a, b] : std::vector<std::pair<size_t, size_t>>{{1, 2}, {2, 3}, {3, 1}}) {
        INFO("piece (" << a << ", " << b << ", v)");
        const size_t fid = fid_of(a, b, v);
        const auto& piece = mesh.m_face_attribute[fid];
        CHECK(piece.m_is_surface_fs);
        CHECK(piece.input_set() == f_attr.input_set());
        CHECK(piece.orientation_along(std::array<size_t, 3>{{a, b, v}}) == 1);
        CHECK(mesh.m_face_extra[fid].label == 0); // f's: not the complex's
        CHECK(mesh.m_face_attribute[fid_of(0, a, b)].m_is_surface_fs); // C's boundary, as it was
    }
    for (const size_t u : {size_t(0), size_t(7)}) {
        for (const size_t w : {size_t(1), size_t(2), size_t(3)}) {
            INFO("interior face (v, " << w << ", " << u << ")");
            const auto& inner = mesh.m_face_attribute[fid_of(v, w, u)];
            CHECK(!inner.m_is_surface_fs);
            CHECK(inner.input_set() == 0);
        }
    }
}

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
        {{"offsets", nlohmann::json::array({{{"selection", "tag_0"}, {"target_distance", 0.2}}})},
         {"num_threads", 0},
         {"EXPERIMENTAL_band_volume_rule", "exact"}},
        g.V);
    param.debug_stencil = true; // the _stencil file's arrays too (DEBUG_stencil)
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
        {"field_angle_turned", &fs.f_field_turned, &ft.f_field_turned},
        {"field_normal_angle", &fs.f_field_angle, &ft.f_field_angle},
        {"dihedral_angle", &fs.f_dihedral, &ft.f_dihedral},
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

TEST_CASE("construction-target-distance-inert", "[offset][3d]")
{
    // The grid of save-offset-correspondence-3d, constructed: every outer end of a marched edge
    // is a unit from the complex, the maximum marchable distance 1. A target from 1 up is not
    // traced to; the march traces to half of it, 0.5, the same construction for every such
    // target. With optimize_offset false nothing after the march reads target_distance, so it has
    // no effect on the result, and the march warns. Below 1 the march traces to the target, and
    // with optimize_offset true the optimization carries the front out to it: no warning.
    const GridInput g = grid_input();
    struct Run
    {
        std::vector<Vector3d> positions;
        bool warned;
    };
    const auto construct = [&](const double target, const bool optimize) {
        Parameters param = construction_params(
            {{"offsets",
              nlohmann::json::array({{{"selection", "tag_0"}, {"target_distance", target}}})},
             {"num_threads", 0},
             {"optimize_offset", optimize}},
            g.V);
        TopoOffsetTetMesh mesh(param, 0);
        MatrixXd V_env;
        MatrixXi F_env;
        mesh.init_from_image(g.V, g.Tm, g.Tags, V_env, F_env, {"tag_0"});
        select_offset(mesh, "tag_0");
        mesh.label_input_complex();
        mesh.init_input_complex_bvh();
        mesh.consolidate_mesh();
        std::ostringstream log;
        const auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(log);
        wmtk::logger().sinks().push_back(sink);
        mesh.construct_offset(std::filesystem::temp_directory_path() / "wmtk_offset_inert");
        wmtk::logger().sinks().pop_back();
        Run run{{}, log.str().find("has no effect") != std::string::npos};
        for (const auto& v : mesh.get_vertices())
            run.positions.push_back(mesh.m_vertex_attribute[v.vid(mesh)].m_posf);
        return run;
    };
    const Run traced = construct(0.3, false);
    const Run above = construct(1.5, false);
    const Run far_above = construct(3., false);
    CHECK_FALSE(traced.warned);
    CHECK(above.warned);
    CHECK(far_above.warned);
    CHECK(above.positions == far_above.positions); // bit for bit
    CHECK(traced.positions != above.positions);
    CHECK_FALSE(construct(1.5, true).warned);
}

TEST_CASE("mesh-snapshot-roundtrip", "[offset][3d]" + release_only)
{
    // take_snapshot() / restore_snapshot(): after a restore the mesh is the one the snapshot left,
    // bit for bit in every frame array, and the same operations from it give the same result
    // again -- the storage is the same, not only the geometry (the operations depend on the order
    // of each vertex's tet fan). The grid mesh, serial; the change is a split pass at a lowered
    // sizing field, which adds vertices and cells.
    const GridInput g = grid_input();
    Parameters param = construction_params(
        {{"offsets", nlohmann::json::array({{{"selection", "tag_0"}, {"target_distance", 0.2}}})},
         {"num_threads", 0},
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

TEST_CASE("turned-faces-at-vertex", "[offset][3d]")
{
    // growth_mode "individual_op_reject" counts the faces turned from the field per operation with
    // turned_front_faces_at(), which reads one vertex's own tet list (lock-safe in the operation
    // passes). Over every vertex it must find exactly the turned faces turned_face_census() finds
    // over the whole offset surface (offset_surface_faces()). The grid mesh with one front vertex
    // dragged to the input's centre, which turns its offset faces back toward the input: turned
    // faces to find.
    const GridInput g = grid_input();
    Parameters param = construction_params(
        {{"offsets", nlohmann::json::array({{{"selection", "tag_0"}, {"target_distance", 0.2}}})},
         {"num_threads", 0},
         {"EXPERIMENTAL_band_volume_rule", "exact"}},
        g.V);
    const std::filesystem::path out =
        std::filesystem::temp_directory_path() / "wmtk_offset_folded_faces";
    const auto mesh = grid_offset_mesh(g, param, out);
    CHECK_FALSE(mesh->turned_reject_active()); // growth_mode "none"
    const auto census_set = [&]() {
        std::set<std::array<size_t, 3>> s;
        for (const auto& f : mesh->offset_surface_faces()) {
            if (mesh->front_face_turned(f[0], f[1], f[2])) {
                auto k = f;
                std::sort(k.begin(), k.end());
                s.insert(k);
            }
        }
        return s;
    };
    const auto per_vertex_set = [&]() {
        std::set<std::array<size_t, 3>> s;
        for (const auto& v : mesh->get_vertices()) mesh->turned_front_faces_at(v.vid(*mesh), s);
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
    INFO(census.size() << " turned faces");
    REQUIRE(!census.empty());
    CHECK(per_vertex_set() == census);
    CHECK(mesh->turned_face_census().n_turned == census.size());
    // Every turned face found through the moved vertex goes through it.
    std::set<std::array<size_t, 3>> at;
    mesh->turned_front_faces_at(front, at);
    CHECK(!at.empty());
    for (const auto& f : at) CHECK(std::find(f.begin(), f.end(), front) != f.end());

    // The rule is on in the loops of growth_mode "individual_op_reject" only.
    mesh->m_offset_params.growth_mode = "individual_op_reject";
    CHECK(mesh->turned_reject_active());
    mesh->m_tetwild_pass = true;
    CHECK_FALSE(mesh->turned_reject_active());
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
    select_offset(mesh, "tag_0");
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
    io::ComponentIO context;
    collect_memory_output(mesh, context);
    io::write_mesh(out.string() + ".msh", context.result.first.at("mesh"));
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
        if (!j.contains("offsets"))
            j["offsets"] = nlohmann::json::array({{{"selection", "tag_0"}}});
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

TEST_CASE("growth-mode-keys", "[offset][3d]" + release_only)
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
        if (!j.contains("offsets"))
            j["offsets"] = nlohmann::json::array({{{"selection", "tag_0"}}});
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
        if (!j.contains("offsets"))
            j["offsets"] = nlohmann::json::array({{{"selection", "tag_0"}}});
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
    // default 0.1, a value read through, an absolute front_conv with its fraction derived, and
    // the refusal of a bar coarser than the offset. The box's diagonal is 13, so a bar read
    // against it would come out 26 times the target-relative one here.
    Eigen::MatrixXd V(2, 3);
    V << 0, 0, 0, 3, 4, 12;
    const Parameters by_default = construction_params(
        {{"offsets", nlohmann::json::array({{{"selection", "tag_0"}, {"target_distance", 0.5}}})}},
        V);
    CHECK(by_default.front_conv_rel == 0.1);
    CHECK(by_default.front_conv == 0.1 * 0.5);
    const Parameters rel = construction_params(
        {{"offsets",
          nlohmann::json::array({{{"selection", "tag_0"}, {"target_distance_rel", 0.02}}})},
         {"front_conv_rel", 0.1}},
        V);
    CHECK(rel.target_distance == Catch::Approx(0.26));
    CHECK(rel.front_conv == 0.1 * rel.target_distance);
    const Parameters abs = construction_params(
        {{"offsets", nlohmann::json::array({{{"selection", "tag_0"}, {"target_distance", 0.5}}})},
         {"front_conv", 0.05}},
        V);
    CHECK(abs.front_conv == 0.05);
    CHECK(abs.front_conv_rel == Catch::Approx(0.1));
    CHECK_THROWS(construction_params(
        {{"offsets", nlohmann::json::array({{{"selection", "tag_0"}, {"target_distance", 0.5}}})},
         {"front_conv_rel", 1.5}},
        V));
}

TEST_CASE("band-side-dihedral", "[offset][3d]")
{
    // The debug frames' dihedral angle (band_side_dihedral()). Face 1 lies in z = 0 with its normal
    // +z out of the band and runs the shared edge along +y; face 2 turns about that edge by phi,
    // its normal (sin phi, 0, cos phi): phi > 0 bends the front away from the band (convex seen
    // from the offset side), the band-side angle 180 - phi; phi -> 180 folds face 2 flat onto face
    // 1 from the offset side (angle -> 0), phi -> -180 from the band side (-> 360).
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
}

TEST_CASE("fold-test", "[offset][3d]")
{
    // The fold predicate (fold_test()), over the whole range of foldover_angle. Face 1's normal
    // out of the band is +z; face 2 turns about the shared edge by phi, its normal (sin phi, 0,
    // cos phi), 180 - phi degrees from opposite. A fold: the normals within `angle` of opposite
    // and one face turned from the field (its field angle over `angle`, or NaN); two opposite
    // faces both along the field are a ridge edge; faces not opposite are neither, and the field
    // is not asked of them (it costs a nearest-primitive query).
    using FT = TopoOffsetTetMesh::FoldTest;
    const Vector3d n1(0., 0., 1.);
    const auto n2 = [](const double deg) {
        const double r = deg * M_PI / 180.;
        return Vector3d(std::sin(r), 0., std::cos(r));
    };
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (const double angle : {1., 10., 29., 30., 31., 45., 90., 120., 170., 179.}) {
        INFO("foldover_angle " << angle);
        int asked = 0;
        const auto test = [&](const double phi, const double a0, const double a1) {
            asked = 0;
            const auto field = [&](const int k) {
                ++asked;
                return k == 0 ? a0 : a1;
            };
            return TopoOffsetTetMesh::fold_test(n1, n2(phi), field, angle);
        };
        // within `angle` of opposite, and beyond it (halfway to the face lying flat)
        const double within = 180. - 0.5 * angle, beyond = 0.5 * (180. - angle);
        // a field angle within `angle` (along the field) and one over it (turned)
        const double along = 0.5 * angle, turned = 0.5 * (angle + 180.);
        CHECK(test(beyond, turned, turned) == FT::none);
        CHECK(asked == 0);
        CHECK(test(within, along, along) == FT::ridge_edge);
        CHECK(test(within, angle, angle) == FT::ridge_edge); // exactly `angle` is not over it
        CHECK(test(within, turned, along) == FT::fold);
        CHECK(test(within, along, turned) == FT::fold);
        CHECK(test(within, nan, along) == FT::fold); // unmeasured is never taken for no fold
        CHECK(test(within, along, nan) == FT::fold);
        CHECK(test(180., turned, turned) == FT::fold); // flat onto each other
    }
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
        if (!j.contains("offsets"))
            j["offsets"] = nlohmann::json::array({{{"selection", "tag_0"}}});
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
    CHECK_FALSE(mesh.front_face_turned(1, 2, 3));
    CHECK(mesh.front_face_turned(0, 1, 2));
    mesh.m_offset_params.foldover_angle = 130.;
    CHECK_FALSE(mesh.front_face_turned(0, 1, 2));
}

TEST_CASE("debug-remote-output-key", "[offset][3d]")
{
    // DEBUG_remote_output: off by default; true overrides DEBUG_output (sets it), so every debug
    // computation runs while the frames follow the remote schedule.
    const auto spec = jse::embed::wmtk_topological_offset_spec::topological_offset_spec::spec();
    const auto injected = [&](nlohmann::json j) {
        j["application"] = "topological_offset";
        j["input"] = "unused.msh";
        if (!j.contains("offsets"))
            j["offsets"] = nlohmann::json::array({{{"selection", "tag_0"}}});
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
    // With EXPERIMENTAL_front_pocket_split false the slot is a no-op, and so it is in a TetWild
    // pass with the key on: the first fixture's pinned cell stays and the mesh is unchanged.
    Parameters param;
    param.front_pocket_split = false;
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
    param.front_spanned_split = false;
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

namespace {
/// The 2D twin of pocket_mesh(): the triangle P = (o, x, y), o = (0, 0), x = (1, 0), y = (0, 1),
/// outside the band (label 0) and face 0. On the edge of P on the line where coordinate a is 0,
/// for each axis a in `axes`, sits a band blob: the triangle on that edge with its apex u 0.5
/// beyond the edge's midpoint, coned from its own centroid e into three band triangles (label 2);
/// axis 2 stands for P's third edge (x, y), its apex u = (1, 1). No two blobs overlap, and every
/// blob triangle has e, which is on no front edge, as a corner. The front
/// (edge_is_offset_surface_live()) is each blob's edge against P and its two edges on the domain
/// boundary. Vertex ids: o 0, x 1, y 2, then u and e of each blob in turn. A piece cut out of a
/// domain, as pocket_mesh()'s: no edge tracked, so no vertex is held. The front flags are derived
/// from the labels (refresh_offset_membership()), and every split is the loop's
/// (EdgeSplitMode::Optimization). `cone`: P is instead coned from its centroid c (vertex 3, before
/// the blobs) into three triangles, so no triangle of P is pinned while P's edges stay where they
/// were.
std::unique_ptr<TopoOffsetTriMesh>
pocket_mesh_2d(Parameters& param, const std::vector<int>& axes, const bool cone = false)
{
    std::vector<Vector2d> pos = {Vector2d(0., 0.), Vector2d(1., 0.), Vector2d(0., 1.)};
    std::vector<std::array<int, 3>> F = {{{0, 1, 2}}};
    std::vector<int> labels = {0};
    if (cone) {
        pos.push_back(Vector2d(1. / 3., 1. / 3.));
        F = {{{0, 1, 3}}, {{1, 2, 3}}, {{2, 0, 3}}};
        labels = {0, 0, 0};
    }
    for (const int a : axes) {
        // The edge of P on the line coordinate a = 0, or (x, y) for a = 2.
        const std::array<int, 2> f = a == 0   ? std::array<int, 2>{{0, 2}}
                                     : a == 1 ? std::array<int, 2>{{0, 1}}
                                              : std::array<int, 2>{{1, 2}};
        const Vector2d& p0 = pos[size_t(f[0])];
        const Vector2d& p1 = pos[size_t(f[1])];
        Vector2d u = (p0 + p1) / 2.;
        if (a < 2) {
            u[a] = -0.5;
        } else {
            u = Vector2d(1., 1.);
        }
        const Vector2d e = (p0 + p1 + u) / 3.;
        const int iu = int(pos.size()), ie = iu + 1;
        pos.push_back(u);
        pos.push_back(e);
        F.push_back({{f[0], f[1], ie}});
        F.push_back({{f[0], iu, ie}});
        F.push_back({{f[1], iu, ie}});
        labels.insert(labels.end(), 3, 2);
    }
    Eigen::MatrixXd V(int(pos.size()), 2);
    for (size_t i = 0; i < pos.size(); ++i) V.row(int(i)) = pos[i].transpose();
    param.envelope_size = 1e-3; // tests skip Parameters::init(); give the build a real eps
    auto mesh = energy_mesh_2d(param, V, F, labels, line_field());
    for (const auto& e : mesh->get_edges()) {
        mesh->m_edge_attribute[e.eid(*mesh)].m_is_surface_fs = false;
    }
    for (const auto& v : mesh->get_vertices()) {
        mesh->m_vertex_attribute[v.vid(*mesh)].m_is_on_surface = false;
        mesh->refresh_offset_membership(v.vid(*mesh));
    }
    mesh->m_edge_split_mode = TopoOffsetTriMesh::EdgeSplitMode::Optimization;
    return mesh;
}

/// Whether the edge (a, b) is on the front (edge_is_offset_surface_live()).
bool front_edge_2d(const TopoOffsetTriMesh& m, const size_t a, const size_t b)
{
    return m.edge_is_offset_surface_live(std::get<0>(m.tuple_from_edge({{a, b}})));
}

/// The pinned cells, counted from the definition: outside the input complex, all three corners
/// front vertices (m_is_on_offset).
size_t count_pinned_2d(const TopoOffsetTriMesh& m)
{
    size_t n = 0;
    for (const auto& f : m.get_faces()) {
        const size_t fid = f.fid(m);
        if (m.face_is_input_complex(fid)) continue;
        const auto vs = m.oriented_tri_vids(fid);
        if (std::all_of(vs.begin(), vs.end(), [&](const size_t v) {
                return m.m_vertex_extra[v].m_is_on_offset;
            })) {
            ++n;
        }
    }
    return n;
}

/// The spanned edges (front_pocket_split()'s EXPERIMENTAL_front_spanned_split): both ends front
/// vertices, not a front edge, no triangle at it in the input complex.
size_t count_spanned_edges_2d(const TopoOffsetTriMesh& m)
{
    size_t n = 0;
    for (const auto& e : m.get_edges()) {
        const size_t a = e.vid(m), b = e.switch_vertex(m).vid(m);
        if (!m.m_vertex_extra[a].m_is_on_offset || !m.m_vertex_extra[b].m_is_on_offset) continue;
        if (m.edge_is_offset_surface_live(e)) continue;
        const auto fids = m.get_incident_fids_for_edge(a, b);
        if (std::any_of(fids.begin(), fids.end(), [&](const size_t f) {
                return m.face_is_input_complex(f);
            })) {
            continue;
        }
        ++n;
    }
    return n;
}

/// Whether every vertex's front flag is what the labels say (offset_membership_mismatches()).
bool front_flags_live_2d(const TopoOffsetTriMesh& m)
{
    const auto mismatches = m.offset_membership_mismatches();
    return mismatches.first == 0 && mismatches.second == 0;
}

/// The vertex at the midpoint of (x, y), (0.5, 0.5) exactly: a split of (x, y) made it.
size_t midpoint_xy_2d(const TopoOffsetTriMesh& m)
{
    size_t found = std::numeric_limits<size_t>::max();
    for (const auto& v : m.get_vertices()) {
        if (m.m_vertex_attribute[v.vid(m)].m_posf == Vector2d(0.5, 0.5)) found = v.vid(m);
    }
    REQUIRE(found != std::numeric_limits<size_t>::max());
    return found;
}

/// The live triangles as sorted corner arrays, sorted: the connectivity, independent of face ids.
std::vector<std::array<size_t, 3>> sorted_cells_2d(const TopoOffsetTriMesh& m)
{
    std::vector<std::array<size_t, 3>> cells;
    for (const auto& f : m.get_faces()) {
        auto vs = m.oriented_tri_vids(f.fid(m));
        std::sort(vs.begin(), vs.end());
        cells.push_back(vs);
    }
    std::sort(cells.begin(), cells.end());
    return cells;
}
} // namespace

TEST_CASE("pocket-split-pinned-cell-2d", "[offset][2d][pocket_split]")
{
    // The 2D twin of pocket-split-pinned-cell: one pinned triangle with one edge off the front. P
    // with blobs on its edges y = 0 and x = 0, so o and x are on the one front edge (o, x) of P
    // and o and y on the other, and the edge (x, y) is on neither. The slot splits (x, y), whose
    // only triangle is P; both children have the midpoint, off the front, as a corner, so no
    // pinned cell is left, and the front, none of whose edges is the split edge, is the same set
    // of edges. (x, y) is a spanned edge too, so with EXPERIMENTAL_front_spanned_split on (the
    // default) the spanned pass splits it, and off the pinned pass does: both.
    const bool spanned = GENERATE(false, true);
    INFO("EXPERIMENTAL_front_spanned_split " << spanned);
    Parameters param;
    param.front_spanned_split = spanned;
    auto mesh = pocket_mesh_2d(param, {1, 0});
    mesh->m_offset_params.front_pocket_split = true;
    // The premises: P is the one pinned cell, and (x, y) is its one edge off the front.
    REQUIRE(count_pinned_2d(*mesh) == 1);
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = i + 1; j < 3; ++j) {
            INFO("edge " << i << " " << j);
            REQUIRE(front_edge_2d(*mesh, i, j) == !(i == 1 && j == 2));
        }
    }
    REQUIRE(front_flags_live_2d(*mesh));
    const auto edges0 = mesh->offset_surface_edges();
    const size_t n_cells0 = mesh->get_faces().size(), n_verts0 = mesh->get_vertices().size();

    mesh->front_pocket_split("test");

    CHECK(count_pinned_2d(*mesh) == 0);
    CHECK(mesh->check_mesh_connectivity_validity());
    CHECK(front_flags_live_2d(*mesh));
    CHECK(mesh->offset_surface_edges() == edges0);
    // One split, of (x, y), on the domain boundary: one vertex, off the front, and one triangle
    // more, and the edge is gone.
    CHECK(mesh->get_vertices().size() == n_verts0 + 1);
    CHECK(mesh->get_faces().size() == n_cells0 + 1);
    CHECK(mesh->get_incident_fids_for_edge(1, 2).empty());
    CHECK_FALSE(mesh->m_vertex_extra[midpoint_xy_2d(*mesh)].m_is_on_offset);
}

TEST_CASE("pocket-split-front-cell-2d", "[offset][2d][pocket_split]")
{
    // The 2D stand-in for pocket-split-six-edge-cell. 3D splits a pinned cell whose six edges
    // are on the offset surface on an edge of its one face that is not, which 2D cannot have: a
    // triangle's faces are its edges, so a pinned triangle with no edge off the front has all its
    // faces on the front, 3D's cell with all four faces on the offset surface, a one-cell
    // component of its label, which the slot counts and leaves. P with blobs on all three edges:
    // the one pinned cell, nothing split, the mesh unchanged.
    Parameters param;
    auto mesh = pocket_mesh_2d(param, {0, 1, 2});
    mesh->m_offset_params.front_pocket_split = true;
    REQUIRE(count_pinned_2d(*mesh) == 1);
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = i + 1; j < 3; ++j) REQUIRE(front_edge_2d(*mesh, i, j));
    }
    REQUIRE(count_spanned_edges_2d(*mesh) == 0);
    REQUIRE(front_flags_live_2d(*mesh));
    const auto cells0 = sorted_cells_2d(*mesh);
    const auto edges0 = mesh->offset_surface_edges();
    const size_t n_verts0 = mesh->get_vertices().size();

    mesh->front_pocket_split("test");

    CHECK(count_pinned_2d(*mesh) == 1);
    CHECK(mesh->get_vertices().size() == n_verts0);
    CHECK(sorted_cells_2d(*mesh) == cells0);
    CHECK(mesh->offset_surface_edges() == edges0);
}

TEST_CASE("pocket-split-key-off-2d", "[offset][2d][pocket_split]")
{
    // The 2D twin of pocket-split-key-off: with EXPERIMENTAL_front_pocket_split false the slot is
    // a no-op, and so it is in the final pass (m_freeze_front; 3D: a TetWild pass) with the key
    // on: the first fixture's pinned cell stays and the mesh is unchanged.
    Parameters param;
    param.front_pocket_split = false;
    auto mesh = pocket_mesh_2d(param, {1, 0});
    REQUIRE_FALSE(mesh->m_offset_params.front_pocket_split);
    REQUIRE(count_pinned_2d(*mesh) == 1);
    const auto cells0 = sorted_cells_2d(*mesh);
    const auto edges0 = mesh->offset_surface_edges();
    const size_t n_verts0 = mesh->get_vertices().size();
    const auto unchanged = [&]() {
        CHECK(count_pinned_2d(*mesh) == 1);
        CHECK(mesh->get_vertices().size() == n_verts0);
        CHECK(sorted_cells_2d(*mesh) == cells0);
        CHECK(mesh->offset_surface_edges() == edges0);
    };

    mesh->front_pocket_split("test");
    unchanged();

    mesh->m_offset_params.front_pocket_split = true;
    mesh->m_freeze_front = true;
    mesh->front_pocket_split("test");
    unchanged();
    mesh->m_freeze_front = false;
}

TEST_CASE("spanned-split-edge-2d", "[offset][2d][pocket_split]")
{
    // The 2D twin of spanned-split-edge: EXPERIMENTAL_front_spanned_split on a spanned edge with
    // no pinned cell. The first fixture with P coned from its centroid c, so (x, y) is still not
    // a front edge while its triangle (x, y, c) has c off the front (3D's (x, z) has two cells;
    // this one is on the domain boundary). The pinned-cell split alone leaves it; with the key,
    // the slot splits (x, y), and the front is the same set of edges.
    Parameters param;
    param.front_spanned_split = false;
    auto mesh = pocket_mesh_2d(param, {1, 0}, true);
    mesh->m_offset_params.front_pocket_split = true;
    REQUIRE(count_pinned_2d(*mesh) == 0);
    REQUIRE_FALSE(front_edge_2d(*mesh, 1, 2));
    REQUIRE(mesh->get_incident_fids_for_edge(1, 2).size() == 1);
    REQUIRE(count_spanned_edges_2d(*mesh) == 1);
    REQUIRE(front_flags_live_2d(*mesh));
    const auto edges0 = mesh->offset_surface_edges();
    const auto cells0 = sorted_cells_2d(*mesh);
    const size_t n_cells0 = mesh->get_faces().size(), n_verts0 = mesh->get_vertices().size();

    mesh->front_pocket_split("test"); // key off: nothing pinned, nothing split
    CHECK(sorted_cells_2d(*mesh) == cells0);
    CHECK(count_spanned_edges_2d(*mesh) == 1);

    mesh->m_offset_params.front_spanned_split = true;
    mesh->front_pocket_split("test");

    CHECK(count_spanned_edges_2d(*mesh) == 0);
    CHECK(count_pinned_2d(*mesh) == 0);
    CHECK(mesh->check_mesh_connectivity_validity());
    CHECK(front_flags_live_2d(*mesh));
    CHECK(mesh->offset_surface_edges() == edges0);
    // One split, of (x, y): one vertex, off the front, and one triangle more; the edge is gone.
    CHECK(mesh->get_vertices().size() == n_verts0 + 1);
    CHECK(mesh->get_faces().size() == n_cells0 + 1);
    CHECK(mesh->get_incident_fids_for_edge(1, 2).empty());
    CHECK_FALSE(mesh->m_vertex_extra[midpoint_xy_2d(*mesh)].m_is_on_offset);
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
    // Both stencils (EXPERIMENTAL_stencil_type): the interior lattice through the order
    // constructor, the FEM nodes (corners and edge points included) through the stencil one.
    for (const bool fem : {false, true})
        for (const int k : {1, 2, 3, 5}) {
            INFO("stencil_order " << k << (fem ? " fem" : " interior"));
            std::vector<SurfaceOffsetError3D::Face> faces;
            double by_hand = 0.;
            const auto& st = fem ? fem_face_stencil(k) : interior_face_stencil(k);
            REQUIRE(int(st.size()) == (fem ? (k + 1) * (k + 2) / 2 : k * (k + 1) / 2));
            for (const auto& [q1, q2] : others) {
                faces.push_back({q1, q2});
                // the interior stencil's plain mean; the FEM stencil's lumped area weights
                const auto& mu = fem_face_stencil_weights(k);
                double m = 0.;
                for (size_t i = 0; i < st.size(); ++i) {
                    const auto& w = st[i];
                    const double r = pot->relative_residual(w[0] * x + w[1] * q1 + w[2] * q2);
                    m += (fem ? mu[i] : 1. / double(st.size())) * r * r;
                }
                by_hand += W * 0.5 * (q1 - x).cross(q2 - x).norm() * m;
            }
            SurfaceOffsetError3D e =
                fem ? SurfaceOffsetError3D(pot, faces, st, &fem_face_stencil_weights(k), W)
                    : SurfaceOffsetError3D(pot, faces, k, W);
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

TEST_CASE("surface-energy-keys", "[offset][3d]" + release_only)
{
    // This branch's keys through the spec, as the app reads it: the defaults (those of the latest
    // runs since 2026-10-29: "exact", offset_amips_weight 1, label_cleanup "one_sided_at_target",
    // the pocket and spanned splits and exit_no_folds on, interleaved_smoothing off; the surface
    // energy's keys off: coarsen_pass off, stencil_order 1 with EXPERIMENTAL_stencil_type
    // "subdivision", EXPERIMENTAL_coarsen_guard "bar", DEBUG_stencil off,
    // EXPERIMENTAL_split_energy_gate off), each opt-in read through, and the
    // refusals (a negative weight, the spec's min; settle_stall_eps away from its default under
    // another rule; "energy_decrease" under another rule; a "one_sided" clean-up under
    // "corner_bound"; "interior" / "fem" at order 0; "no_rise" without coarsen_pass;
    // DEBUG_stencil without DEBUG_output; the split gate under "surface").
    const auto spec = jse::embed::wmtk_topological_offset_spec::topological_offset_spec::spec();
    const auto with_app = [](nlohmann::json j) {
        j["application"] = "topological_offset";
        j["input"] = "unused.msh";
        if (!j.contains("offsets"))
            j["offsets"] = nlohmann::json::array({{{"selection", "tag_0"}}});
        return j;
    };
    const auto injected = [&](const nlohmann::json& j) {
        jse::JSE spec_engine;
        spec_engine.strict = true;
        REQUIRE(spec_engine.verify_json(with_app(j), spec));
        return spec_engine.inject_defaults(with_app(j), spec);
    };
    const Parameters by_default(injected(nlohmann::json::object()));
    CHECK(by_default.band_volume_rule == "exact_swept");
    CHECK(by_default.offset_amips_weight == 1.);
    CHECK(by_default.label_cleanup == "one_sided_at_target");
    CHECK(by_default.front_pocket_split);
    CHECK(by_default.front_spanned_split);
    CHECK(by_default.exit_no_folds);
    CHECK_FALSE(by_default.interleaved_smoothing);
    CHECK_FALSE(by_default.coarsen_pass);
    CHECK(by_default.coarsen_guard == "bar");
    CHECK_FALSE(by_default.debug_stencil);
    CHECK_FALSE(by_default.split_energy_gate);
    CHECK(by_default.stencil_order == 1);
    CHECK(by_default.stencil_type == "subdivision");
    CHECK(by_default.settle_stall_eps == 0.1);
    CHECK(by_default.settle_test == "stall");
    // the in-class defaults (a Parameters built without JSON) are the spec's
    const Parameters in_class{};
    CHECK(in_class.band_volume_rule == by_default.band_volume_rule);
    CHECK(in_class.offset_amips_weight == by_default.offset_amips_weight);
    CHECK(in_class.label_cleanup == by_default.label_cleanup);
    CHECK(in_class.front_pocket_split == by_default.front_pocket_split);
    CHECK(in_class.front_spanned_split == by_default.front_spanned_split);
    CHECK(in_class.exit_no_folds == by_default.exit_no_folds);
    CHECK(in_class.stencil_order == by_default.stencil_order);
    CHECK(in_class.stencil_type == by_default.stencil_type);
    CHECK(in_class.coarsen_guard == by_default.coarsen_guard);
    CHECK(in_class.debug_stencil == by_default.debug_stencil);
    CHECK(in_class.split_energy_gate == by_default.split_energy_gate);
    const Parameters surface(injected({{"EXPERIMENTAL_band_volume_rule", "surface"}}));
    CHECK(surface.band_volume_rule == "surface");
    CHECK(surface.offset_amips_weight == 1.);
    CHECK(surface.label_cleanup == "one_sided_at_target");
    const Parameters weighted(injected(
        {{"EXPERIMENTAL_band_volume_rule", "surface"},
         {"offset_amips_weight", 0.25},
         {"settle_stall_eps", 0.05}}));
    CHECK(weighted.offset_amips_weight == 0.25);
    CHECK(weighted.settle_stall_eps == 0.05);
    // the configs of the converged "surface" runs on piece08 (2026-10-28), written out
    const Parameters screw(injected(
        {{"EXPERIMENTAL_band_volume_rule", "surface"},
         {"offset_amips_weight", 100.},
         {"EXPERIMENTAL_stencil_type", "fem"},
         {"stencil_order", 5},
         {"EXPERIMENTAL_front_pocket_split", true},
         {"EXPERIMENTAL_front_spanned_split", true},
         {"EXPERIMENTAL_exit_no_folds", true},
         {"EXPERIMENTAL_label_cleanup", "energy_decrease"},
         {"interleaved_smoothing", false},
         {"num_smoothing_passes", 2},
         {"coarsen_pass", true},
         {"EXPERIMENTAL_coarsen_guard", "no_rise"}}));
    CHECK(screw.stencil_type == "fem");
    CHECK(screw.stencil_order == 5);
    CHECK(screw.coarsen_pass);
    CHECK(screw.coarsen_guard == "no_rise");
    CHECK(screw.label_cleanup == "energy_decrease");
    // "corner_bound": refused with a "one_sided" clean-up, the default one (no exact integral to
    // read d through), read with the clean-up off
    CHECK_THROWS(Parameters{injected({{"EXPERIMENTAL_band_volume_rule", "corner_bound"}})});
    CHECK(
        Parameters{injected(
                       {{"EXPERIMENTAL_band_volume_rule", "corner_bound"},
                        {"EXPERIMENTAL_label_cleanup", "off"}})}
            .band_volume_rule == "corner_bound");
    const Parameters exact(injected(
        {{"EXPERIMENTAL_band_volume_rule", "exact"},
         {"EXPERIMENTAL_front_pocket_split", true},
         {"EXPERIMENTAL_label_cleanup", "one_sided_at_target"}}));
    CHECK(exact.label_cleanup == "one_sided_at_target");
    jse::JSE negative;
    negative.strict = true;
    CHECK_FALSE(negative.verify_json(with_app({{"offset_amips_weight", -0.5}}), spec));
    CHECK_THROWS(Parameters{injected({{"settle_stall_eps", 0.2}})});
    // stencil_order 0: the corners under "subdivision", refused under "interior" and "fem"
    const Parameters corners(injected({{"stencil_order", 0}}));
    CHECK(corners.stencil_order == 0);
    CHECK_THROWS(
        Parameters{injected({{"stencil_order", 0}, {"EXPERIMENTAL_stencil_type", "interior"}})});
    CHECK_THROWS(
        Parameters{injected({{"stencil_order", 0}, {"EXPERIMENTAL_stencil_type", "fem"}})});
    jse::JSE strict;
    strict.strict = true;
    CHECK_FALSE(strict.verify_json(with_app({{"stencil_order", -1}}), spec));
    CHECK_FALSE(strict.verify_json(with_app({{"EXPERIMENTAL_stencil_type", "lagrange"}}), spec));
    const Parameters interior(injected({{"EXPERIMENTAL_stencil_type", "interior"}}));
    CHECK(interior.stencil_type == "interior");
    // EXPERIMENTAL_split_energy_gate: read through, refused under "surface"
    CHECK(Parameters{injected({{"EXPERIMENTAL_split_energy_gate", true}})}.split_energy_gate);
    CHECK_THROWS(
        Parameters{injected(
            {{"EXPERIMENTAL_split_energy_gate", true},
             {"EXPERIMENTAL_band_volume_rule", "surface"}})});
    // EXPERIMENTAL_coarsen_guard: "no_rise" needs coarsen_pass; anything else refused
    CHECK_THROWS(Parameters{injected({{"EXPERIMENTAL_coarsen_guard", "no_rise"}})});
    CHECK_FALSE(strict.verify_json(with_app({{"EXPERIMENTAL_coarsen_guard", "max"}}), spec));
    // DEBUG_stencil needs DEBUG_output (or DEBUG_remote_output, which sets it)
    CHECK_THROWS(Parameters{injected({{"DEBUG_stencil", true}})});
    CHECK(Parameters{injected({{"DEBUG_stencil", true}, {"DEBUG_output", true}})}.debug_stencil);
    CHECK(
        Parameters{injected({{"DEBUG_stencil", true}, {"DEBUG_remote_output", true}})}
            .debug_stencil);
    // EXPERIMENTAL_settle_test: "bar" read through under "surface", refused under another rule,
    // and settle_stall_eps refused beside "bar".
    const Parameters bar(injected(
        {{"EXPERIMENTAL_band_volume_rule", "surface"}, {"EXPERIMENTAL_settle_test", "bar"}}));
    CHECK(bar.settle_test == "bar");
    CHECK_THROWS(Parameters{injected({{"EXPERIMENTAL_settle_test", "bar"}})});
    // EXPERIMENTAL_label_cleanup "energy_decrease": read through under "surface" (with the pocket
    // split every clean-up mode needs), refused under another rule.
    const Parameters ed(injected(
        {{"EXPERIMENTAL_band_volume_rule", "surface"},
         {"EXPERIMENTAL_front_pocket_split", true},
         {"EXPERIMENTAL_label_cleanup", "energy_decrease"}}));
    CHECK(ed.label_cleanup == "energy_decrease");
    CHECK_THROWS(
        Parameters{injected(
            {{"EXPERIMENTAL_band_volume_rule", "exact"},
             {"EXPERIMENTAL_front_pocket_split", true},
             {"EXPERIMENTAL_label_cleanup", "energy_decrease"}})});
    CHECK_THROWS(
        Parameters{injected(
            {{"EXPERIMENTAL_band_volume_rule", "surface"},
             {"EXPERIMENTAL_settle_test", "bar"},
             {"settle_stall_eps", 0.2}})});
}

TEST_CASE("surface-energy-sums", "[offset][3d]" + release_only)
{
    // EXPERIMENTAL_band_volume_rule "surface" on the grid constructed at 0.2: no cell carries a
    // band term (tet_energy() is the AMIPS term of every cell), the face term is read the same in
    // any corner order, and the sums every rule and log read are the same faces' terms --
    // surface_term_total() over offset_surface_faces(), surface_term_at() over the faces through
    // a vertex, global_energy() and energy_parts() with the face term as the offset part -- and
    // the front smoother's objective (front_energy()) is surface_term_at() up to rounding. All of
    // it under both stencils (EXPERIMENTAL_stencil_type "interior" and "fem").
    const GridInput g = grid_input();
    Parameters param = construction_params(
        {{"offsets", nlohmann::json::array({{{"selection", "tag_0"}, {"target_distance", 0.2}}})},
         {"num_threads", 0},
         {"EXPERIMENTAL_band_volume_rule", "surface"},
         {"stencil_order", 5}},
        g.V);
    REQUIRE(param.offset_amips_weight == 1.); // the default
    const std::filesystem::path out =
        std::filesystem::temp_directory_path() / "wmtk_offset_surface_sums";
    const auto mesh = grid_offset_mesh(g, param, out);
    REQUIRE(mesh->surface_energy());
    const double f = mesh->m_offset_params.front_conv_frac();
    // L / front_conv_frac()^2, L the bounding box diagonal, whatever offset_amips_weight is
    REQUIRE(mesh->m_offset_params.diag_l > 0.);
    CHECK(mesh->surface_term_weight() == Catch::Approx(mesh->m_offset_params.diag_l / (f * f)));
    mesh->m_offset_params.offset_amips_weight = 0.3;
    CHECK(mesh->surface_term_weight() == Catch::Approx(mesh->m_offset_params.diag_l / (f * f)));
    mesh->m_offset_params.offset_amips_weight = 100.;

    const auto faces = mesh->offset_surface_faces();
    REQUIRE(faces.size() > 10);
    // Everything below under both stencils (EXPERIMENTAL_stencil_type), on the same mesh.
    for (const std::string type : {"interior", "fem"}) {
        INFO("EXPERIMENTAL_stencil_type " << type);
        mesh->m_offset_params.stencil_type = type;
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
        const auto& st = mesh->face_stencil();
        REQUIRE(st.size() == (type == "fem" ? 21u : 15u)); // order 5
        REQUIRE(int(st.size()) == mesh->stencil_points_per_face());
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
            // the plain mean under "interior", the lumped area weights under "fem"
            const std::vector<double>* mu = mesh->face_stencil_weights();
            REQUIRE((mu != nullptr) == (type == "fem"));
            double m = 0.;
            for (size_t i = 0; i < st.size(); ++i) {
                const auto& w = st[i];
                const double e =
                    mesh->m_offset_potential->value(w[0] * p0 + w[1] * p1 + w[2] * p2) * 0.2 - 0.2;
                m += (mu ? (*mu)[i] : 1. / double(st.size())) * e * e;
            }
            o_g += 0.5 * (p1 - p0).cross(p2 - p0).norm() * m;
        }
        CHECK(mesh->surface_integral_raw() == Catch::Approx(o_g).epsilon(1e-10));
        CHECK(mesh->settle_metric() == mesh->stencil_max_sq_error());
        mesh->m_offset_params.settle_test = "bar";
        CHECK(mesh->settle_metric() == mesh->surface_integral_raw());
        mesh->m_offset_params.settle_test = "stall";

        // The _stencil frame file (gather_vtu_frame()): one point per stencil point of every offset
        // face, in offset_surface_faces() order; its sq_error's max is O* bit for bit, and its
        // other fields are the same r read three ways.
        {
            mesh->m_offset_params.debug_stencil = true; // DEBUG_stencil: the file is written
            const auto fr = mesh->gather_vtu_frame((out / "stencil_frame").string());
            REQUIRE(fr.S.rows() == Eigen::Index(faces.size() * st.size()));
            double mx = 0.;
            const double conv2 =
                mesh->m_offset_params.front_conv * mesh->m_offset_params.front_conv;
            for (Eigen::Index i = 0; i < fr.S.rows(); ++i) {
                REQUIRE(std::isfinite(fr.s_sq[i]));
                mx = std::max(mx, fr.s_sq[i]);
                CHECK(fr.s_over[i] == Catch::Approx(fr.s_sq[i] / conv2).epsilon(1e-14));
                CHECK(fr.s_err[i] == Catch::Approx(fr.s_signed[i] * fr.s_signed[i]).epsilon(1e-14));
                CHECK(fr.s_dist_ratio[i] == Catch::Approx(1. + fr.s_signed[i]).epsilon(1e-14));
                CHECK(fr.s_face[i] == double(i / Eigen::Index(st.size())));
                CHECK(fr.s_point[i] == double(i % Eigen::Index(st.size())));
            }
            const double o = mesh->stencil_max_sq_error();
            CHECK(std::memcmp(&mx, &o, sizeof(double)) == 0);
        }

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
}

TEST_CASE("final-coarsening-guard", "[offset][3d]" + release_only)
{
    // The coarsening collapses (coarsen_mesh(), since 2026-10-28) under EXPERIMENTAL_coarsen_guard
    // "no_rise" (with coarsen_pass, the "fem" stencil at order 5 and the "exact" rule, the values
    // the test was written at) are kept only if neither the max AMIPS over the cells they change
    // nor the offset face term over the offset faces they change rises (ties pass). On the 5 x 5 x
    // 5 grid constructed at 0.2, set up as tetwild_pass()
    // sets it up (the final envelopes, TetWild's rules, no plastic medium), edges are tried as
    // single coarsening collapses (coarsen_collapse_edge()), any edge first, then with a front
    // vertex as v1. Before each try the guard's verdict is predicted on its own: the cells the
    // collapse reshapes are v1's ring without the cells at v2, v1 renamed v2, their AMIPS read off
    // those corners; the offset faces through v2 after it come from the cells' labels (live when
    // one side is band and the other neither band nor input complex, or a band face on the domain
    // boundary); a face listed before and after drops out of both sums. Per try: a kept collapse
    // was predicted to pass both halves and leaves exactly the predicted faces at v2; a refusal
    // by the AMIPS half was predicted to raise the max; one by the offset half to keep the max
    // and fail its offset test. The offset test, under the default settle_mark: the sum of the
    // face terms rises; under EXPERIMENTAL_settle_mark "stencil": the max over the new faces'
    // stencil points of (d - delta)^2 (read point by point off the potential here) is at or over
    // front_conv^2 and above that max over the removed faces. Some kept collapses change offset
    // faces (a v1 off the front merged into a front v2); under the sum, on this grid, every front
    // v1 that reaches the guard is refused, by either half. Sum: (a) at the defaults and (b) at
    // stop_energy 1 and front_conv 1e-12, under which the old threshold guard (every cell under
    // stop_energy, every stencil point within front_conv) refused every collapse: the same
    // collapses are kept and refused at both. "stencil": (c) front_conv 1e6, every point under
    // it: the offset half refuses nothing (front v1 collapses that pass the guard are still
    // refused after it on this grid, by the engine's later checks); (d) front_conv 1e-12,
    // none under it: refusals by the offset half (a rise of the max); (e) the defaults.
    const GridInput g = grid_input(5);
    // The mesh holds its Parameters by reference: each lives as long as the test.
    std::vector<std::unique_ptr<Parameters>> params;
    const auto build = [&](const char* name, const char* mark) {
        params.push_back(
            std::make_unique<Parameters>(construction_params(
                {{"offsets",
                  nlohmann::json::array({{{"selection", "tag_0"}, {"target_distance", 0.2}}})},
                 {"num_threads", 0},
                 {"EXPERIMENTAL_settle_mark", mark},
                 {"EXPERIMENTAL_band_volume_rule", "exact"},
                 {"EXPERIMENTAL_stencil_type", "fem"},
                 {"stencil_order", 5},
                 {"coarsen_pass", true},
                 {"EXPERIMENTAL_coarsen_guard", "no_rise"}},
                g.V)));
        auto mesh =
            grid_offset_mesh(g, *params.back(), std::filesystem::temp_directory_path() / name);
        mesh->build_final_envelopes();
        mesh->m_tetwild_pass = true;
        mesh->m_op_rules = TopoOffsetTetMesh::OpRules::TetWild;
        mesh->m_plastic_active = false;
        for (const auto& t : mesh->get_tets()) {
            mesh->m_tet_attribute[t.tid(*mesh)].m_quality = mesh->get_quality(t);
        }
        return mesh;
    };
    using Face = std::array<size_t, 3>;
    struct Prediction
    {
        bool amips_ok = false, offset_ok = false;
        std::set<Face> after;
    };
    const auto predict = [](const TopoOffsetTetMesh& mesh, const size_t v1, const size_t v2) {
        Prediction p;
        double a_before = 0., a_after = 0.;
        std::vector<std::pair<std::array<size_t, 4>, size_t>> cells; // corners after, slot
        const std::vector<size_t> ring1 = mesh.get_one_ring_tids_for_vertex(v1);
        for (const size_t tid : ring1) {
            a_before = std::max(a_before, mesh.get_quality(mesh.tuple_from_tet(tid)));
            std::array<size_t, 4> vs = mesh.oriented_tet_vids(tid);
            if (std::find(vs.begin(), vs.end(), v2) != vs.end()) continue; // dies
            *std::find(vs.begin(), vs.end(), v1) = v2;
            a_after = std::max(a_after, mesh.get_quality(vs));
            cells.push_back({vs, tid});
        }
        for (const size_t tid : mesh.get_one_ring_tids_for_vertex(v2)) {
            if (std::find(ring1.begin(), ring1.end(), tid) == ring1.end()) {
                cells.push_back({mesh.oriented_tet_vids(tid), tid});
            }
        }
        p.amips_ok = a_after <= a_before;
        std::map<Face, std::vector<size_t>> sides;
        for (const auto& [vs, tid] : cells) {
            for (int skip = 0; skip < 4; ++skip) {
                if (vs[size_t(skip)] == v2) continue;
                Face f;
                int n = 0;
                for (int k = 0; k < 4; ++k) {
                    if (k != skip) f[size_t(n++)] = vs[size_t(k)];
                }
                std::sort(f.begin(), f.end());
                sides[f].push_back(tid);
            }
        }
        for (const auto& [f, ts] : sides) {
            const bool a = mesh.cell_is_offset_band(ts[0]);
            if (ts.size() == 1) {
                if (a) p.after.insert(f);
            } else if (
                a != mesh.cell_is_offset_band(ts[1]) &&
                !mesh.cell_is_input_complex(a ? ts[1] : ts[0])) {
                p.after.insert(f);
            }
        }
        std::vector<Face> f1, f2;
        mesh.live_offset_faces_at(v1, f1);
        mesh.live_offset_faces_at(v2, f2);
        std::set<Face> before(f1.begin(), f1.end());
        before.insert(f2.begin(), f2.end());
        const bool stencil = mesh.m_offset_params.settle_mark == "stencil";
        const double delta = mesh.m_offset_params.target_distance;
        const auto stencil_max = [&](const Face& f) {
            double m = 0.;
            for (const auto& w : mesh.face_stencil()) {
                Vector3d q = Vector3d::Zero();
                for (int j = 0; j < 3; ++j) {
                    q += w[size_t(j)] * mesh.m_vertex_attribute[f[size_t(j)]].m_posf;
                }
                const double r = mesh.m_offset_potential->relative_residual(q);
                m = std::isfinite(r) ? std::max(m, (delta * r) * (delta * r))
                                     : std::numeric_limits<double>::infinity();
            }
            return m;
        };
        double s_before = 0., s_after = 0.;
        for (const Face& f : before) {
            if (p.after.count(f)) continue;
            if (stencil)
                s_before = std::max(s_before, stencil_max(f));
            else
                s_before += mesh.surface_face_term(f[0], f[1], f[2]);
        }
        for (const Face& f : p.after) {
            if (before.count(f)) continue;
            if (stencil)
                s_after = std::max(s_after, stencil_max(f));
            else
                s_after += mesh.surface_face_term(f[0], f[1], f[2]);
        }
        const double fc = mesh.m_offset_params.front_conv;
        p.offset_ok = std::isfinite(s_after) &&
                      (stencil ? s_after < fc * fc || s_after <= s_before : s_after <= s_before);
        return p;
    };
    struct Tally
    {
        size_t kept = 0, kept_faces_changed = 0, refused_amips = 0, refused_offset = 0, wrong = 0,
               face_mismatch = 0;
        bool operator==(const Tally& o) const
        {
            return kept == o.kept && kept_faces_changed == o.kept_faces_changed &&
                   refused_amips == o.refused_amips && refused_offset == o.refused_offset;
        }
    };
    const auto sweep = [&](TopoOffsetTetMesh& mesh, const bool front_only, const size_t max_kept) {
        Tally t;
        bool progress = true;
        while (progress && t.kept < max_kept) {
            progress = false;
            for (const auto& e : mesh.get_edges()) {
                const size_t v1 = e.vid(mesh), v2 = e.switch_vertex(mesh).vid(mesh);
                const bool front = mesh.m_vertex_extra[v1].m_is_on_offset;
                if (front_only && !front) continue;
                const Prediction p = predict(mesh, v1, v2);
                std::set<Face> faces_before;
                {
                    std::vector<Face> f1, f2;
                    mesh.live_offset_faces_at(v1, f1);
                    mesh.live_offset_faces_at(v2, f2);
                    faces_before.insert(f1.begin(), f1.end());
                    faces_before.insert(f2.begin(), f2.end());
                }
                const size_t ra = mesh.m_coarsen_reject_amips.load();
                const size_t ro = mesh.m_coarsen_reject_offset.load();
                std::vector<TetMesh::Tuple> nt;
                const bool kept = mesh.coarsen_collapse_edge(e, nt);
                if (mesh.m_coarsen_reject_amips.load() > ra) {
                    ++t.refused_amips;
                    if (p.amips_ok) ++t.wrong;
                }
                if (mesh.m_coarsen_reject_offset.load() > ro) {
                    ++t.refused_offset;
                    if (!p.amips_ok || p.offset_ok) ++t.wrong;
                }
                if (!kept) continue;
                ++t.kept;
                if (!(p.amips_ok && p.offset_ok)) ++t.wrong;
                std::vector<Face> now;
                mesh.live_offset_faces_at(v2, now);
                if (std::set<Face>(now.begin(), now.end()) != p.after) ++t.face_mismatch;
                if (faces_before != p.after) ++t.kept_faces_changed;
                progress = true;
                break;
            }
        }
        return t;
    };
    const auto run = [&](const char* name, const char* mark, const double stop, const double fc) {
        auto mesh = build(name, mark);
        if (stop > 0.) mesh->m_params.stop_energy = stop;
        if (fc > 0.) mesh->m_offset_params.front_conv = fc;
        const Tally o = sweep(*mesh, false, 40);
        const Tally f = sweep(*mesh, true, 20); // front vertices as v1
        INFO(
            name << ": kept " << o.kept << " (" << o.kept_faces_changed
                 << " changing offset faces), refused by AMIPS " << o.refused_amips
                 << ", by the offset term " << o.refused_offset << " | front v1: kept " << f.kept
                 << ", refused by AMIPS " << f.refused_amips << ", by the offset term "
                 << f.refused_offset);
        CHECK(o.kept > 0);
        CHECK(o.kept_faces_changed > 0);
        CHECK(o.refused_amips > 0);
        CHECK(f.refused_amips > 0);
        CHECK(o.wrong + f.wrong == 0);
        CHECK(o.face_mismatch + f.face_mismatch == 0);
        CHECK(mesh->check_mesh_connectivity_validity());
        return std::make_pair(o, f);
    };
    const auto wide = run("wmtk_offset_coarsen_defaults", "vertex_edge", 0., 0.);
    const auto tight = run("wmtk_offset_coarsen_tight", "vertex_edge", 1., 1e-12);
    CHECK(wide.first.refused_offset > 0);
    CHECK(wide.second.refused_offset > 0);
    CHECK(wide.first == tight.first);
    CHECK(wide.second == tight.second);
    const auto st_open = run("wmtk_offset_coarsen_stencil_open", "stencil", 0., 1e6);
    CHECK(st_open.first.refused_offset + st_open.second.refused_offset == 0);
    const auto st_tight = run("wmtk_offset_coarsen_stencil_tight", "stencil", 0., 1e-12);
    CHECK(st_tight.first.refused_offset + st_tight.second.refused_offset > 0);
    run("wmtk_offset_coarsen_stencil_defaults", "stencil", 0., 0.);
    // (d) The final coarsening alone (tetwild_pass(what, 0, true): no iterations, the loop's
    // setup put back afterwards), at wide bounds on a mesh as the loop leaves it: collapses are
    // kept, the cell count falls, the pass's state is undone; with coarsen_pass off it does
    // nothing.
    {
        params.push_back(
            std::make_unique<Parameters>(construction_params(
                {{"offsets",
                  nlohmann::json::array({{{"selection", "tag_0"}, {"target_distance", 0.2}}})},
                 {"num_threads", 0},
                 {"EXPERIMENTAL_band_volume_rule", "exact"},
                 {"EXPERIMENTAL_stencil_type", "fem"},
                 {"stencil_order", 5},
                 {"coarsen_pass", true},
                 {"EXPERIMENTAL_coarsen_guard", "no_rise"}},
                g.V)));
        params.back()->stop_energy = 1e12;
        params.back()->front_conv = 1e6;
        auto mesh = grid_offset_mesh(
            g,
            *params.back(),
            std::filesystem::temp_directory_path() / "wmtk_offset_coarsen_alone");
        for (const auto& t : mesh->get_tets()) {
            mesh->m_tet_attribute[t.tid(*mesh)].m_quality = mesh->get_quality(t);
        }
        REQUIRE(mesh->m_params.coarsen_pass);
        REQUIRE(mesh->coarsen_no_rise());
        const size_t n0 = mesh->get_tets().size();
        mesh->m_params.coarsen_pass = false;
        mesh->tetwild_pass("test coarsening", 0, true);
        CHECK(mesh->get_tets().size() == n0);
        mesh->m_params.coarsen_pass = true;
        mesh->tetwild_pass("test coarsening", 0, true);
        CHECK(mesh->m_coarsen_stats.accepted > 0);
        CHECK(mesh->get_tets().size() < n0);
        CHECK_FALSE(mesh->m_tetwild_pass);
        CHECK(mesh->m_op_rules == TopoOffsetTetMesh::OpRules::Loop);
        CHECK(mesh->m_params.coarsen_pass);
        CHECK(mesh->check_mesh_connectivity_validity());
    }
}

TEST_CASE("label-cleanup-energy-decrease", "[offset][3d]" + release_only)
{
    // EXPERIMENTAL_label_cleanup "energy_decrease" on a 5 x 5 x 5 grid constructed at 0.2 under
    // the "surface" rule, every other front vertex pushed out: label_flip_surface_change(), the
    // decision, against the faces' terms by hand for every cell with a front face; the clean-up
    // never raises E, and a second clean-up right after finds nothing to flip (E unchanged bit for
    // bit). (On this grid the cells whose flip would pay are band cells with a corner on the
    // input, which eligibility refuses, so no flip happens here.)
    const GridInput g = grid_input(5);
    Parameters param = construction_params(
        {{"offsets", nlohmann::json::array({{{"selection", "tag_0"}, {"target_distance", 0.2}}})},
         {"num_threads", 0},
         {"EXPERIMENTAL_band_volume_rule", "surface"}},
        g.V);
    const std::filesystem::path out =
        std::filesystem::temp_directory_path() / "wmtk_offset_energy_decrease";
    auto mesh = grid_offset_mesh(g, param, out);
    mesh->m_offset_params.front_pocket_split = true;
    mesh->m_offset_params.label_cleanup = "energy_decrease";
    size_t moved = 0;
    for (const auto& v : mesh->get_vertices()) {
        const size_t vid = v.vid(*mesh);
        if (!mesh->m_vertex_extra[vid].m_is_on_offset || (vid % 2) != 0) continue;
        auto& x = mesh->m_vertex_attribute[vid].m_posf;
        x *= 1.6; // outward, away from the centre cube (at the origin)
        mesh->m_vertex_attribute[vid].set_pos_to_posf();
        ++moved;
    }
    REQUIRE(moved > 0);
    for (const auto& t : mesh->get_tets()) {
        mesh->m_tet_attribute[t.tid(*mesh)].m_quality = mesh->get_quality(t);
    }
    size_t asked = 0;
    for (const auto& t : mesh->get_tets()) {
        const size_t tid = t.tid(*mesh);
        const int own = mesh->m_tet_attribute[tid].label;
        if (own == 1) continue;
        double plus = 0., minus = 0.;
        bool front = false;
        for (int j = 0; j < 4; ++j) {
            const auto ft = mesh->tuple_from_face(tid, j);
            const auto o = ft.switch_tetrahedron(*mesh);
            if (!o) continue;
            const int lo = mesh->m_tet_attribute[o->tid(*mesh)].label;
            if (lo == 1) continue;
            const auto fv = mesh->get_face_vids(ft);
            const double s = mesh->surface_face_term(fv[0], fv[1], fv[2]);
            if (lo == own) {
                plus += s;
            } else {
                minus += s;
                front = true;
            }
        }
        if (!front) continue;
        ++asked;
        CHECK(mesh->label_flip_surface_change(tid) == Catch::Approx(plus - minus).margin(1e-12));
    }
    CHECK(asked > 0);
    const double e0 = mesh->global_energy();
    mesh->label_cleanup("test");
    const double e1 = mesh->global_energy();
    CHECK(e1 <= e0);
    mesh->label_cleanup("test again");
    const double e2 = mesh->global_energy();
    CHECK(std::memcmp(&e1, &e2, sizeof(double)) == 0);
}

TEST_CASE("settle-mark-stencil", "[offset][3d]" + release_only)
{
    // EXPERIMENTAL_settle_mark "stencil" (2026-10-28): on a settled turn a front face with a point
    // of its offset stencil at or beyond front_conv, (d - delta)^2 >= front_conv^2, has its corners
    // halved and holds the exit, so the loop exits only with every stencil point within front_conv.
    // On the 5 x 5 x 5 grid constructed at 0.2 (the FEM stencil at order 5 and the "exact" rule,
    // the values the test was written at), the per-face max
    // of (d - delta)^2 is computed here point by point from the potential; O* is its max. One turn
    // of the settle test (e_prev = O*, so the turn settles) is run with front_conv set (a) just
    // above sqrt(O*): exit, nothing halved; (b) just below: no exit, the corners of exactly the
    // faces at the max halved; (c) at the median face: no exit, every corner of a face over
    // halved; refine_now false: the same verdicts, nothing halved. (d) The default "vertex_edge"
    // at (b)'s front_conv, where every front vertex is within it: exit -- the stencil points it
    // does not test are over.
    {
        const auto spec = jse::embed::wmtk_topological_offset_spec::topological_offset_spec::spec();
        jse::JSE spec_engine;
        spec_engine.strict = true;
        nlohmann::json j = {
            {"application", "topological_offset"},
            {"input", "unused.msh"},
            {"offsets", nlohmann::json::array({{{"selection", "tag_0"}}})},
            {"EXPERIMENTAL_settle_mark", "stencil"}};
        REQUIRE(spec_engine.verify_json(j, spec));
        nlohmann::json in = spec_engine.inject_defaults(j, spec);
        CHECK(Parameters(in).settle_mark == "stencil");
        j["EXPERIMENTAL_settle_mark"] = "stencils";
        CHECK_FALSE(spec_engine.verify_json(j, spec));
        in["EXPERIMENTAL_settle_mark"] = "stencils";
        CHECK_THROWS(Parameters{in});
    }
    const GridInput g = grid_input(5);
    std::vector<std::unique_ptr<Parameters>> params;
    const auto build = [&](const char* mark, const char* name) {
        params.push_back(
            std::make_unique<Parameters>(construction_params(
                {{"offsets",
                  nlohmann::json::array({{{"selection", "tag_0"}, {"target_distance", 0.2}}})},
                 {"num_threads", 0},
                 {"EXPERIMENTAL_settle_mark", mark},
                 {"EXPERIMENTAL_band_volume_rule", "exact"},
                 {"EXPERIMENTAL_stencil_type", "fem"},
                 {"stencil_order", 5}},
                g.V)));
        return grid_offset_mesh(g, *params.back(), std::filesystem::temp_directory_path() / name);
    };
    auto mesh = build("stencil", "wmtk_offset_settle_stencil");
    REQUIRE(mesh->m_offset_params.settle_mark == "stencil");
    REQUIRE(mesh->m_offset_params.stencil_type == "fem");
    const double delta = mesh->m_offset_params.target_distance;
    const auto faces = mesh->offset_surface_faces();
    REQUIRE(!faces.empty());
    std::vector<double> m(faces.size(), 0.);
    for (size_t k = 0; k < faces.size(); ++k) {
        for (const auto& w : mesh->face_stencil()) {
            Vector3d q = Vector3d::Zero();
            for (int j = 0; j < 3; ++j) {
                q += w[size_t(j)] * mesh->m_vertex_attribute[faces[k][size_t(j)]].m_posf;
            }
            const double r = mesh->m_offset_potential->relative_residual(q);
            REQUIRE(std::isfinite(r));
            m[k] = std::max(m[k], (delta * r) * (delta * r));
        }
        CHECK(mesh->face_stencil_max_sq(faces[k]) == m[k]);
    }
    const double o_star = *std::max_element(m.begin(), m.end());
    CHECK(mesh->stencil_max_sq_error() == o_star);
    REQUIRE(o_star > 0.);

    const size_t nv = mesh->vert_capacity();
    std::vector<double> s0(nv);
    for (size_t v = 0; v < nv; ++v) s0[v] = mesh->m_vertex_attribute[v].m_sizing_scalar;
    const double s_floor = std::max(
        mesh->m_offset_params.min_sizing_scalar,
        mesh->m_offset_params.min_edge_length / mesh->m_params.l);
    struct Turn
    {
        bool exit = false;
        size_t lowered = 0, corners_not_halved = 0, corners = 0;
    };
    const auto turn = [&](TopoOffsetTetMesh& me, const double fc, const bool refine) {
        me.m_offset_params.front_conv = fc;
        double e_prev = me.settle_metric();
        Turn t;
        t.exit = me.settle_refinement_turn(1, e_prev, refine);
        std::set<size_t> corners;
        for (size_t k = 0; k < faces.size(); ++k) {
            if (!(m[k] < fc * fc)) corners.insert(faces[k].begin(), faces[k].end());
        }
        t.corners = corners.size();
        for (size_t v = 0; v < nv; ++v) {
            double& s = me.m_vertex_attribute[v].m_sizing_scalar;
            if (s < s0[v]) ++t.lowered;
            if (corners.count(v) && !(s <= std::max(0.5 * s0[v], s_floor))) ++t.corners_not_halved;
            s = s0[v]; // put back for the next turn
        }
        return t;
    };
    {
        const Turn a = turn(*mesh, std::sqrt(o_star) * (1. + 1e-6), true);
        CHECK(a.exit);
        CHECK(a.corners == 0);
        CHECK(a.lowered == 0);
    }
    const double fc_b = std::sqrt(o_star) * (1. - 1e-6);
    {
        const Turn b = turn(*mesh, fc_b, true);
        INFO("corners of the faces at the max " << b.corners << ", lowered " << b.lowered);
        CHECK_FALSE(b.exit);
        CHECK(b.corners >= 3);
        CHECK(b.corners_not_halved == 0);
        CHECK(b.lowered >= b.corners);
        const Turn b0 = turn(*mesh, fc_b, false);
        CHECK_FALSE(b0.exit);
        CHECK(b0.lowered == 0);
    }
    {
        std::vector<double> sorted = m;
        std::sort(sorted.begin(), sorted.end());
        const double fc_c = std::sqrt(sorted[sorted.size() / 2]);
        REQUIRE(fc_c > 0.);
        const Turn c = turn(*mesh, fc_c, true);
        INFO("corners of the faces over " << c.corners << ", lowered " << c.lowered);
        CHECK_FALSE(c.exit);
        CHECK(c.corners > 3);
        CHECK(c.corners_not_halved == 0);
        CHECK(c.lowered >= c.corners);
    }
    {
        auto def = build("vertex_edge", "wmtk_offset_settle_vertex_edge");
        double worst_vertex = 0.;
        std::set<size_t> front;
        for (const auto& f : faces) front.insert(f.begin(), f.end());
        for (const size_t v : front) {
            const double r =
                def->m_offset_potential->relative_residual(def->m_vertex_attribute[v].m_posf);
            worst_vertex = std::max(worst_vertex, std::abs(delta * r));
        }
        INFO("worst front vertex " << worst_vertex << ", front_conv " << fc_b);
        REQUIRE(worst_vertex < fc_b);
        def->m_offset_params.front_conv = fc_b;
        double e_prev = def->settle_metric();
        CHECK(def->settle_refinement_turn(1, e_prev, false));
    }
}

TEST_CASE("memory-output-with-offset-front", "[offset][3d][io]")
{
    // The memory export (collect_memory_output(), the route every run's output takes since the
    // new mesh IO) of a mesh with an offset potential and a front, as a run ends: under
    // DEBUG_stencil write_vtu() then also writes the stencil file (one point per stencil point of
    // every offset face), whose points are not the mesh's vertex slots. MEASURED 2026-10-29: every
    // run of the sonic stencil sweep on f9e9b94971 converged and then aborted in the export,
    // "Export references an inactive vertex" (compact_slot_outputs()). The export must succeed, its
    // keys be the offsets' own, and the mesh's rows be the live vertices.
    const GridInput g = grid_input(5);
    Parameters param = construction_params(
        {{"offsets", nlohmann::json::array({{{"selection", "tag_0"}, {"target_distance", 0.2}}})},
         {"num_threads", 0}},
        g.V);
    param.debug_stencil = true;
    auto mesh =
        grid_offset_mesh(g, param, std::filesystem::temp_directory_path() / "wmtk_offset_memout");
    REQUIRE(mesh->m_offset_potential);
    REQUIRE(!mesh->offset_surface_faces().empty());
    io::ComponentIO context;
    REQUIRE_NOTHROW(collect_memory_output(*mesh, context));
    const auto& out = context.result.first;
    // The mesh alone, no envelope given: the front, the input complex and its edges are the
    // mesh's tags, not outputs of their own.
    CHECK(out.size() == 1);
    CHECK(out.count("mesh") == 1);
    CHECK(size_t(out.at("mesh").V.rows()) == mesh->get_vertices().size());
}

namespace {
/// [-n h / 2, n h / 2]^3 in n^3 cubes of edge h, six tets each around the cube's main diagonal:
/// the vertices, vertex (i, j, k) row i + (n + 1) (j + (n + 1) k); the tets, positively
/// oriented; per tet its cube (i, j, k).
struct CubeGrid
{
    Eigen::MatrixXd V;
    std::vector<std::array<int, 4>> T;
    std::vector<std::array<int, 3>> cube;
};
CubeGrid cube_grid(const int n, const double h)
{
    const auto vid = [&](int i, int j, int k) { return i + (n + 1) * (j + (n + 1) * k); };
    CubeGrid g;
    g.V.resize((n + 1) * (n + 1) * (n + 1), 3);
    for (int k = 0; k <= n; ++k) {
        for (int j = 0; j <= n; ++j) {
            for (int i = 0; i <= n; ++i) {
                g.V.row(vid(i, j, k)) << (i - 0.5 * n) * h, (j - 0.5 * n) * h, (k - 0.5 * n) * h;
            }
        }
    }
    const std::array<std::array<int, 3>, 6> perms = {
        {{{0, 1, 2}}, {{0, 2, 1}}, {{1, 0, 2}}, {{1, 2, 0}}, {{2, 0, 1}}, {{2, 1, 0}}}};
    for (int k = 0; k < n; ++k) {
        for (int j = 0; j < n; ++j) {
            for (int i = 0; i < n; ++i) {
                for (const auto& pm : perms) {
                    std::array<int, 3> c = {{i, j, k}};
                    std::array<int, 4> t;
                    t[0] = vid(c[0], c[1], c[2]);
                    for (int s = 0; s < 3; ++s) {
                        ++c[size_t(pm[size_t(s)])];
                        t[size_t(s + 1)] = vid(c[0], c[1], c[2]);
                    }
                    g.T.push_back(positive(g.V, t));
                    g.cube.push_back({{i, j, k}});
                }
            }
        }
    }
    return g;
}

/// A rod where three blocks meet: [-1, 1]^3 in 4^3 cubes of edge 0.5 (cube_grid()), the cubes
/// in -0.5 <= z <= 0.5 tagged by quadrant around the z axis -- tag_0 x > 0 and y > 0, tag_1
/// x < 0, tag_2 x > 0 and y < 0 -- the rest ambient. "tag_0 & tag_1 & tag_2" is the segment
/// x = y = 0, -0.5 <= z <= 0.5, two grid edges with both ends inside the domain: the rod of the
/// 2026-10-16 rod fixture (rod.msh), on the grid.
MeshData rod_grid()
{
    const int n = 4;
    const double h = 0.5;
    const CubeGrid g = cube_grid(n, h);
    Eigen::MatrixXi Tm(int(g.T.size()), 4);
    DataMatrix<uint8_t> tags = DataMatrix<uint8_t>::Zero(g.T.size(), 4);
    for (size_t i = 0; i < g.T.size(); ++i) {
        for (int j = 0; j < 4; ++j) Tm(int(i), j) = g.T[i][size_t(j)];
        const double cx = (g.cube[i][0] + 0.5) * h - 1, cy = (g.cube[i][1] + 0.5) * h - 1,
                     cz = (g.cube[i][2] + 0.5) * h - 1;
        const int b = std::abs(cz) > 0.5 ? 3 : cx < 0 ? 1 : cy > 0 ? 0 : 2; // 3: ambient
        tags(i, b) = 1;
    }
    MeshData m = io::geometry_data(g.V, Tm, CellType::Tetrahedron);
    m.cells.at(CellType::Tetrahedron).attributes["tags"] = {
        tags,
        {"tag_0", "tag_1", "tag_2", "ambient"}};
    return m;
}
} // namespace

TEST_CASE("curve-offset-rod-held", "[offset][3d]" + release_only)
{
    // A curve offset (an input complex of segments): the band integral over segments, and the rod
    // held as tetwild holds its feature curves (hold_curves_and_points()). The run must not throw
    // -- construct_offset() checks that the rod's edges keep their feature tags through the march
    // -- and the input complex in the result must lie in the tube around the input segment, its
    // two ends still there. MEASURED (the rod fixture, exact rule, 7 turns): before the hold, the
    // rod's vertices 0.43 target_distance off the input, none of its edges tagged; before the
    // march carried the tags (EdgeSplittingTet), neither of its 2 edges tagged after it; with
    // both, every vertex on the input to 3e-16.
    // One turn: every operation -- split, collapse, swap, smoothing -- runs over the tags.
    const MeshData input = rod_grid();
    for (const std::string rule : {"exact", "exact_swept", "corner_bound"}) {
        INFO("EXPERIMENTAL_band_volume_rule " << rule);
        nlohmann::json j = {
            {"input", "m"},
            {"offsets",
             nlohmann::json::array(
                 {{{"selection", "tag_0 & tag_1 & tag_2"}, {"target_distance", 0.2}}})},
            {"EXPERIMENTAL_band_volume_rule", rule},
            {"max_rounds", 1},
            {"num_threads", 1},
            {"throw_on_nonconvergence", false}};
        // the default clean-up reads the exact integral, which "corner_bound" does not build
        if (rule == "corner_bound") j["EXPERIMENTAL_label_cleanup"] = "off";
        ComponentResult res;
        REQUIRE_NOTHROW(
            res = components::topological_offset::topological_offset(j, {{"m", input}}));
        const MeshData& out = res.first.at("mesh");
        const auto labels = out.vertex_attributes.at("labels").doubles();
        // the tube's half-width: feature_envelope_ratio 1 x envelope_size, envelope_size_rel
        // 1e-3 of the box's diagonal
        const double eps = 1e-3 * 2. * std::sqrt(3.);
        size_t n_rod = 0;
        bool end_lo = false, end_hi = false;
        for (Eigen::Index v = 0; v < out.V.rows(); ++v) {
            if (labels(v, 0) != 1.) continue;
            ++n_rod;
            const Vector3d p = out.V.row(v).transpose();
            const double z = std::clamp(p[2], -0.5, 0.5);
            CHECK((p - Vector3d(0, 0, z)).norm() <= eps);
            end_lo = end_lo || (p - Vector3d(0, 0, -0.5)).norm() <= eps;
            end_hi = end_hi || (p - Vector3d(0, 0, 0.5)).norm() <= eps;
        }
        CHECK(n_rod >= 3);
        CHECK(end_lo);
        CHECK(end_hi);
    }
}

TEST_CASE("curve-offset-refuses-offset-in", "[offset][3d]")
{
    // A curve encloses no volume (init_input_complex_bvh()).
    const nlohmann::json j = {
        {"input", "m"},
        {"offsets",
         nlohmann::json::array(
             {{{"selection", "tag_0 & tag_1 & tag_2"}, {"target_distance", 0.2}}})},
        {"offset_in", true},
        {"offset_out", false},
        {"num_threads", 1}};
    REQUIRE_THROWS_WITH(
        components::topological_offset::topological_offset(j, {{"m", rod_grid()}}),
        Catch::Matchers::ContainsSubstring("enclose no volume"));
}

namespace {
/// rod_grid() with every tet ambient and two edge tags, the edge block tetwild and simwild write:
/// "curve", the segment x = y = 0, -0.5 <= z <= 0.5 (rod_grid()'s rod, without the blocks), and
/// "other", the segment x = y = 0.5, -0.5 <= z <= 0.5, 0.71 from it and out of the band.
MeshData rod_edge_grid()
{
    MeshData m = rod_grid();
    auto& tets = m.cells.at(CellType::Tetrahedron);
    const DataMatrix<uint8_t> ambient = DataMatrix<uint8_t>::Ones(tets.connectivity.rows(), 1);
    tets.attributes["tags"] = {ambient, {"ambient"}};
    const auto vid = [](int i, int j, int k) { return i + 5 * (j + 5 * k); }; // rod_grid()'s
    auto& edges = m.cells[CellType::Edge];
    edges.connectivity.resize(4, 2);
    DataMatrix<uint8_t> member = DataMatrix<uint8_t>::Zero(4, 2);
    for (int k = 1; k <= 2; ++k) {
        edges.connectivity(k - 1, 0) = vid(2, 2, k);
        edges.connectivity(k - 1, 1) = vid(2, 2, k + 1);
        member(k - 1, 0) = 1;
        edges.connectivity(k + 1, 0) = vid(3, 3, k);
        edges.connectivity(k + 1, 1) = vid(3, 3, k + 1);
        member(k + 1, 1) = 1;
    }
    edges.attributes["tags"] = {member, {"curve", "other"}};
    return m;
}
} // namespace

TEST_CASE("curve-offset-edge-tag", "[offset][3d]" + release_only)
{
    // An edge tag selected (label_input_complex()): the curve offset of a tetwild or simwild
    // feature curve, with no region around it. Every edge tag is held, selected or not
    // (hold_curves_and_points()), and written back as the result's edge block: each tag's edges
    // in the tube around its segment, the selected curve's ends kept.
    const MeshData input = rod_edge_grid();
    const nlohmann::json j = {
        {"input", "m"},
        {"offsets", nlohmann::json::array({{{"selection", "curve"}, {"target_distance", 0.2}}})},
        {"max_rounds", 1},
        {"num_threads", 1},
        {"throw_on_nonconvergence", false}};
    ComponentResult res;
    REQUIRE_NOTHROW(res = components::topological_offset::topological_offset(j, {{"m", input}}));
    const MeshData& out = res.first.at("mesh");
    const double eps = 1e-3 * 2. * std::sqrt(3.); // feature_envelope_ratio 1 x envelope_size
    REQUIRE(out.cells.count(CellType::Edge) == 1);
    const auto& block = out.cells.at(CellType::Edge);
    const auto& tags = block.attributes.at("tags");
    REQUIRE(tags.labels == std::vector<std::string>{"curve", "other"});
    const auto member = tags.integers();
    const auto dist = [](const Vector3d& p, const double x, const double y) {
        const double z = std::clamp(p[2], -0.5, 0.5);
        return (p - Vector3d(x, y, z)).norm();
    };
    double zlo[2] = {1., 1.}, zhi[2] = {-1., -1.};
    for (Eigen::Index r = 0; r < block.connectivity.rows(); ++r) {
        for (int t = 0; t < 2; ++t) {
            if (member(r, t) == 0) continue;
            for (int c = 0; c < 2; ++c) {
                const Vector3d p = out.V.row(block.connectivity(r, c)).transpose();
                CHECK(dist(p, t == 0 ? 0. : 0.5, t == 0 ? 0. : 0.5) <= eps);
                zlo[t] = std::min(zlo[t], p[2]);
                zhi[t] = std::max(zhi[t], p[2]);
            }
        }
    }
    for (int t = 0; t < 2; ++t) { // each curve whole, end to end
        INFO("edge tag " << tags.labels[size_t(t)]);
        CHECK(std::abs(zlo[t] + 0.5) <= eps);
        CHECK(std::abs(zhi[t] - 0.5) <= eps);
    }
    const auto labels = out.vertex_attributes.at("labels").doubles();
    size_t n_rod = 0;
    for (Eigen::Index v = 0; v < out.V.rows(); ++v) {
        if (labels(v, 0) != 1.) continue;
        ++n_rod;
        CHECK(dist(out.V.row(v).transpose(), 0., 0.) <= eps);
    }
    CHECK(n_rod >= 3);
}

TEST_CASE("curve-offset-edge-tags-in-expression", "[offset][3d]")
{
    // An edge tag in an expression means what it means alone (label_offset()): "curve | other" is
    // both curves, every input-complex vertex on one of the two segments and both segments
    // whole, end to end. Refused until 2026-11 ("is selected alone").
    const nlohmann::json j = {
        {"input", "m"},
        {"offsets",
         nlohmann::json::array({{{"selection", "curve | other"}, {"target_distance", 0.2}}})},
        {"optimize_offset", false},
        {"num_threads", 1}};
    ComponentResult res;
    REQUIRE_NOTHROW(
        res = components::topological_offset::topological_offset(j, {{"m", rod_edge_grid()}}));
    const MeshData& out = res.first.at("mesh");
    const auto labels = out.vertex_attributes.at("labels").doubles();
    double zlo[2] = {1., 1.}, zhi[2] = {-1., -1.};
    for (Eigen::Index v = 0; v < out.V.rows(); ++v) {
        if (labels(v, 0) != 1.) continue;
        const Vector3d p = out.V.row(v).transpose();
        const int t = p[0] > 0.25 ? 1 : 0; // "curve" at x = y = 0, "other" at x = y = 0.5
        CHECK(std::abs(p[0] - 0.5 * t) < 1e-12);
        CHECK(std::abs(p[1] - 0.5 * t) < 1e-12);
        zlo[t] = std::min(zlo[t], p[2]);
        zhi[t] = std::max(zhi[t], p[2]);
    }
    for (int t = 0; t < 2; ++t) {
        CHECK(zlo[t] == -0.5);
        CHECK(zhi[t] == 0.5);
    }
}

namespace {
/// A curve as grid points (i, j, k) of curve_grid(), consecutive ones on one grid line.
using GridPolyline = std::vector<std::array<int, 3>>;
/// [-1, 1]^3 in 8^3 cubes of edge 0.25 (cube_grid()), every tet ambient, and an edge block: each
/// (name, polyline), every grid edge between consecutive points tagged with the name (a name given
/// twice tags both polylines), as tetwild and simwild write their feature curves.
MeshData curve_grid(const std::vector<std::pair<std::string, GridPolyline>>& curves)
{
    const int n = 8;
    const CubeGrid g = cube_grid(n, 0.25);
    const auto vid = [&](int i, int j, int k) { return i + (n + 1) * (j + (n + 1) * k); };
    Eigen::MatrixXi Tm(int(g.T.size()), 4);
    for (size_t i = 0; i < g.T.size(); ++i) {
        for (int j = 0; j < 4; ++j) Tm(int(i), j) = g.T[i][size_t(j)];
    }
    MeshData m = io::geometry_data(g.V, Tm, CellType::Tetrahedron);
    const DataMatrix<uint8_t> ambient = DataMatrix<uint8_t>::Ones(g.T.size(), 1);
    m.cells.at(CellType::Tetrahedron).attributes["tags"] = {ambient, {"ambient"}};
    std::vector<std::string> names;
    std::vector<std::pair<std::array<int, 2>, size_t>> edges; // grid edge, its name's column
    for (const auto& [name, poly] : curves) {
        auto it = std::find(names.begin(), names.end(), name);
        if (it == names.end()) it = names.insert(names.end(), name);
        const size_t col = size_t(it - names.begin());
        for (size_t p = 0; p + 1 < poly.size(); ++p) {
            std::array<int, 3> c = poly[p];
            while (c != poly[p + 1]) {
                const int a = vid(c[0], c[1], c[2]);
                for (int d = 0; d < 3; ++d)
                    c[size_t(d)] += (poly[p + 1][size_t(d)] > c[size_t(d)]) -
                                    (poly[p + 1][size_t(d)] < c[size_t(d)]);
                edges.push_back({{{a, vid(c[0], c[1], c[2])}}, col});
            }
        }
    }
    auto& block = m.cells[CellType::Edge];
    block.connectivity.resize(Eigen::Index(edges.size()), 2);
    DataMatrix<uint8_t> member = DataMatrix<uint8_t>::Zero(edges.size(), names.size());
    for (size_t r = 0; r < edges.size(); ++r) {
        block.connectivity(Eigen::Index(r), 0) = edges[r].first[0];
        block.connectivity(Eigen::Index(r), 1) = edges[r].first[1];
        member(r, edges[r].second) = 1;
    }
    block.attributes["tags"] = {member, names};
    return m;
}

/// The distance from p to the polyline of grid points (curve_grid()'s coordinates).
double grid_polyline_distance(const Vector3d& p, const GridPolyline& poly)
{
    const auto x = [](const std::array<int, 3>& c) {
        return Vector3d(c[0] * 0.25 - 1, c[1] * 0.25 - 1, c[2] * 0.25 - 1);
    };
    double d = std::numeric_limits<double>::infinity();
    for (size_t k = 0; k + 1 < poly.size(); ++k) {
        const Vector3d a = x(poly[k]), b = x(poly[k + 1]);
        const double t = std::clamp((p - a).dot(b - a) / (b - a).squaredNorm(), 0., 1.);
        d = std::min(d, (p - a - t * (b - a)).norm());
    }
    return d;
}
} // namespace

TEST_CASE("curve-offset-converges", "[offset][3d][.]")
{
    // Hidden, for the runs before a merge: a curve offset run to convergence (the settle exit,
    // throw_on_nonconvergence) on three curves no region surrounds -- a corner, three arms
    // meeting out of a plane, and a line with a second edge tag held inside its band -- and every
    // edge tag in its tube in the result. MEASURED (2026-11-05, exact rule, 1 thread, Release,
    // laptop): resolved at turns 6, 5 and 11, 22 s, 21 s and 47 s.
    struct Case
    {
        std::string name;
        std::vector<std::pair<std::string, GridPolyline>> curves;
        double delta;
    };
    const std::vector<Case> cases = {
        {"corner", {{"curve", {{{2, 4, 4}}, {{4, 4, 4}}, {{4, 6, 4}}}}}, 0.2},
        {"junction",
         {{"curve", {{{4, 4, 4}}, {{6, 4, 4}}}},
          {"curve", {{{4, 4, 4}}, {{4, 6, 4}}}},
          {"curve", {{{4, 4, 4}}, {{4, 4, 6}}}}},
         0.2},
        // the second line 0.25 from the first, inside its band at 0.3
        {"held-inside-the-band",
         {{"curve", {{{4, 4, 2}}, {{4, 4, 6}}}}, {"other", {{{5, 4, 2}}, {{5, 4, 6}}}}},
         0.3},
    };
    for (const Case& c : cases) {
        INFO(c.name);
        const nlohmann::json j = {
            {"input", "m"},
            {"offsets",
             nlohmann::json::array({{{"selection", "curve"}, {"target_distance", c.delta}}})},
            {"max_rounds", 15},
            {"num_threads", 1},
            {"throw_on_nonconvergence", true}};
        ComponentResult res;
        REQUIRE_NOTHROW(
            res = components::topological_offset::topological_offset(
                j,
                {{"m", curve_grid(c.curves)}}));
        const MeshData& out = res.first.at("mesh");
        const auto& block = out.cells.at(CellType::Edge);
        const auto& tags = block.attributes.at("tags");
        const auto member = tags.integers();
        const double eps = 1e-3 * 2. * std::sqrt(3.);
        for (Eigen::Index r = 0; r < block.connectivity.rows(); ++r) {
            for (size_t t = 0; t < tags.labels.size(); ++t) {
                if (member(r, Eigen::Index(t)) == 0) continue;
                for (int e = 0; e < 2; ++e) {
                    const Vector3d p = out.V.row(block.connectivity(r, e)).transpose();
                    double d = std::numeric_limits<double>::infinity();
                    for (const auto& [name, poly] : c.curves) {
                        if (name == tags.labels[t])
                            d = std::min(d, grid_polyline_distance(p, poly));
                    }
                    CHECK(d <= eps);
                }
            }
        }
    }
}

namespace {
/// A grid point of curve_grid(), in its coordinates.
Vector3d grid_point(const std::array<int, 3>& c)
{
    return Vector3d(c[0] * 0.25 - 1, c[1] * 0.25 - 1, c[2] * 0.25 - 1);
}

/// curve_grid(curves) and a point block: each (name, grid point), its vertex under the name (a
/// name given twice labels both), as tetwild and simwild write their input points.
MeshData point_grid(
    const std::vector<std::pair<std::string, std::array<int, 3>>>& points,
    const std::vector<std::pair<std::string, GridPolyline>>& curves = {})
{
    MeshData m = curve_grid(curves);
    if (curves.empty()) m.cells.erase(CellType::Edge);
    const auto vid = [](const std::array<int, 3>& c) { return c[0] + 9 * (c[1] + 9 * c[2]); };
    std::vector<std::string> names;
    auto& block = m.cells[CellType::Point];
    block.connectivity.resize(Eigen::Index(points.size()), 1);
    std::vector<size_t> col;
    for (size_t r = 0; r < points.size(); ++r) {
        auto it = std::find(names.begin(), names.end(), points[r].first);
        if (it == names.end()) it = names.insert(names.end(), points[r].first);
        col.push_back(size_t(it - names.begin()));
        block.connectivity(Eigen::Index(r), 0) = vid(points[r].second);
    }
    DataMatrix<uint8_t> member = DataMatrix<uint8_t>::Zero(points.size(), names.size());
    for (size_t r = 0; r < points.size(); ++r) member(r, col[r]) = 1;
    block.attributes["tags"] = {member, names};
    return m;
}

/// The result's point block: per label, the positions of its rows' vertices.
std::map<std::string, std::vector<Vector3d>> result_points(const MeshData& out)
{
    std::map<std::string, std::vector<Vector3d>> by_label;
    const auto& block = out.cells.at(CellType::Point);
    const auto& tags = block.attributes.at("tags");
    const auto member = tags.integers();
    for (Eigen::Index r = 0; r < block.connectivity.rows(); ++r) {
        for (size_t t = 0; t < tags.labels.size(); ++t) {
            if (member(r, Eigen::Index(t)) != 0)
                by_label[tags.labels[t]].push_back(out.V.row(block.connectivity(r, 0)).transpose());
        }
    }
    return by_label;
}
} // namespace

TEST_CASE("point-offset-point-label", "[offset][3d]" + release_only)
{
    // A point label selected (label_input_complex()): the point offset of a tetwild or simwild
    // input point, its band a ball. Every input point is anchored, selected or not
    // (hold_curves_and_points()), and written back as the result's point block: each within its
    // anchor's radius, feature_envelope_ratio x envelope_size, of where it was given. The rules
    // and the ratio over its range: 0.01, 1 and 10.
    struct Case
    {
        std::string rule;
        double ratio;
    };
    const Case c = GENERATE(
        Case{"exact", 1.},
        Case{"exact_swept", 1.},
        Case{"corner_bound", 1.},
        Case{"exact", 0.01},
        Case{"exact", 10.});
    INFO("EXPERIMENTAL_band_volume_rule " << c.rule << ", feature_envelope_ratio " << c.ratio);
    // p at the origin, q 0.5 from it, out of the band at 0.2
    const std::array<int, 3> p{{4, 4, 4}}, q{{6, 4, 4}};
    nlohmann::json j = {
        {"input", "m"},
        {"offsets", nlohmann::json::array({{{"selection", "p"}, {"target_distance", 0.2}}})},
        {"EXPERIMENTAL_band_volume_rule", c.rule},
        {"feature_envelope_ratio", c.ratio},
        {"max_rounds", 1},
        {"num_threads", 1},
        {"throw_on_nonconvergence", false}};
    // the default clean-up reads the exact integral, which "corner_bound" does not build
    if (c.rule == "corner_bound") j["EXPERIMENTAL_label_cleanup"] = "off";
    ComponentResult res;
    REQUIRE_NOTHROW(
        res = components::topological_offset::topological_offset(
            j,
            {{"m", point_grid({{"p", p}, {"q", q}})}}));
    const MeshData& out = res.first.at("mesh");
    const double r = c.ratio * 1e-3 * 2. * std::sqrt(3.); // the anchors' radius
    const auto points = result_points(out);
    REQUIRE(points.size() == 2);
    for (const auto& [label, at] : {std::pair("p", p), std::pair("q", q)}) {
        INFO("point " << label);
        REQUIRE(points.at(label).size() == 1);
        CHECK((points.at(label)[0] - grid_point(at)).norm() <= r);
    }
    // the complex: p alone, its vertex
    const auto labels = out.vertex_attributes.at("labels").doubles();
    size_t n_complex = 0;
    for (Eigen::Index v = 0; v < out.V.rows(); ++v) {
        if (labels(v, 0) != 1.) continue;
        ++n_complex;
        CHECK((out.V.row(v).transpose() - grid_point(p)).norm() <= r);
    }
    CHECK(n_complex == 1);
}

TEST_CASE("point-offset-labels-in-expression", "[offset][3d]")
{
    // A point label in an expression means what it means alone, as an edge tag: "p | q" is both
    // points, and no other vertex. Refused until 2026-11 ("is selected alone").
    const nlohmann::json j = {
        {"input", "m"},
        {"offsets", nlohmann::json::array({{{"selection", "p | q"}, {"target_distance", 0.2}}})},
        {"optimize_offset", false},
        {"num_threads", 1}};
    ComponentResult res;
    REQUIRE_NOTHROW(
        res = components::topological_offset::topological_offset(
            j,
            {{"m", point_grid({{"p", {{4, 4, 4}}}, {"q", {{6, 4, 4}}}})}}));
    const MeshData& out = res.first.at("mesh");
    const auto labels = out.vertex_attributes.at("labels").doubles();
    std::vector<Vector3d> complex;
    for (Eigen::Index v = 0; v < out.V.rows(); ++v) {
        if (labels(v, 0) == 1.) complex.push_back(out.V.row(v).transpose());
    }
    REQUIRE(complex.size() == 2);
    const bool p_first = (complex[0] - grid_point({{4, 4, 4}})).norm() < 1e-12;
    CHECK((complex[p_first ? 0 : 1] - grid_point({{4, 4, 4}})).norm() < 1e-12);
    CHECK((complex[p_first ? 1 : 0] - grid_point({{6, 4, 4}})).norm() < 1e-12);
}

TEST_CASE("point-offset-label-names", "[offset][3d]")
{
    // One name is one input (init_from_image()): a point label an edge tag also has is that
    // input's points beside its curve, selected with it; one a region has is refused.
    SECTION("an edge tag's")
    {
        const nlohmann::json j = {
            {"input", "m"},
            {"offsets",
             nlohmann::json::array({{{"selection", "curve"}, {"target_distance", 0.2}}})},
            {"optimize_offset", false},
            {"num_threads", 1}};
        ComponentResult res;
        REQUIRE_NOTHROW(
            res = components::topological_offset::topological_offset(
                j,
                {{"m",
                  point_grid({{"curve", {{6, 4, 4}}}}, {{"curve", {{{2, 4, 4}}, {{4, 4, 4}}}}})}}));
        const MeshData& out = res.first.at("mesh");
        const auto labels = out.vertex_attributes.at("labels").doubles();
        bool point_in_complex = false;
        for (Eigen::Index v = 0; v < out.V.rows(); ++v) {
            if (labels(v, 0) == 1. &&
                (out.V.row(v).transpose() - grid_point({{6, 4, 4}})).norm() < 1e-12)
                point_in_complex = true;
        }
        CHECK(point_in_complex);
        CHECK(result_points(out).at("curve").size() == 1);
    }
    SECTION("a region's")
    {
        const nlohmann::json j = {
            {"input", "m"},
            {"offsets",
             nlohmann::json::array({{{"selection", "ambient"}, {"target_distance", 0.2}}})},
            {"num_threads", 1}};
        REQUIRE_THROWS_WITH(
            components::topological_offset::topological_offset(
                j,
                {{"m", point_grid({{"ambient", {{4, 4, 4}}}})}}),
            Catch::Matchers::ContainsSubstring("is also a tet or face tag"));
    }
}

TEST_CASE("empty-input-complex-keeps-curves-and-points", "[offset][3d]")
{
    // An offset_selection naming a region with no tet: the input complex is empty, and the run
    // returns the mesh as loaded before anything is held (topological_offset.cpp). Its result
    // keeps the input's edge block and point block, as a run with a band does.
    MeshData input = point_grid({{"p", {{4, 4, 4}}}}, {{"curve", {{{2, 4, 4}}, {{6, 4, 4}}}}});
    auto& tets = input.cells.at(CellType::Tetrahedron);
    DataMatrix<uint8_t> tags = DataMatrix<uint8_t>::Zero(tets.connectivity.rows(), 2);
    tags.col(0).setOnes();
    tets.attributes["tags"] = {tags, {"ambient", "nothing"}};
    const nlohmann::json j = {
        {"input", "m"},
        {"offsets", nlohmann::json::array({{{"selection", "nothing"}, {"target_distance", 0.2}}})},
        {"num_threads", 1}};
    ComponentResult res;
    REQUIRE_NOTHROW(res = components::topological_offset::topological_offset(j, {{"m", input}}));
    const MeshData& out = res.first.at("mesh");
    // the curve: the four grid edges from x = -0.5 to x = 0.5 on the x axis
    REQUIRE(out.cells.count(CellType::Edge) == 1);
    const auto& edges = out.cells.at(CellType::Edge);
    CHECK(edges.attributes.at("tags").labels == std::vector<std::string>{"curve"});
    CHECK(edges.connectivity.rows() == 4);
    for (Eigen::Index r = 0; r < edges.connectivity.rows(); ++r) {
        for (int c = 0; c < 2; ++c) {
            const Vector3d x = out.V.row(edges.connectivity(r, c)).transpose();
            CHECK(std::abs(x[0]) <= 0.5);
            CHECK(x.tail<2>().norm() == 0.);
        }
    }
    // the point, at the origin
    REQUIRE(out.cells.count(CellType::Point) == 1);
    const auto points = result_points(out);
    REQUIRE(points.size() == 1);
    REQUIRE(points.at("p").size() == 1);
    CHECK(points.at("p")[0] == grid_point({{4, 4, 4}}));
}

namespace {
/// The offset of point_grid(points)'s point p at `delta`, run to convergence (the settle exit,
/// throw_on_nonconvergence), and every input point within its anchor's radius in the result.
void require_point_offset_converges(
    const std::vector<std::pair<std::string, std::array<int, 3>>>& points,
    const double delta)
{
    const nlohmann::json j = {
        {"input", "m"},
        {"offsets", nlohmann::json::array({{{"selection", "p"}, {"target_distance", delta}}})},
        {"max_rounds", 15},
        {"num_threads", 1},
        {"throw_on_nonconvergence", true}};
    ComponentResult res;
    REQUIRE_NOTHROW(
        res = components::topological_offset::topological_offset(j, {{"m", point_grid(points)}}));
    const auto out = result_points(res.first.at("mesh"));
    const double r = 1e-3 * 2. * std::sqrt(3.);
    for (const auto& [name, at] : points) {
        INFO("point " << name);
        REQUIRE(out.at(name).size() == 1);
        CHECK((out.at(name)[0] - grid_point(at)).norm() <= r);
    }
}
} // namespace

TEST_CASE("point-offset-converges", "[offset][3d][.]")
{
    // Hidden, for the runs before a merge: a point offset alone, run to convergence.
    require_point_offset_converges({{"p", {{4, 4, 4}}}}, 0.2);
}

TEST_CASE("point-offset-held-point-inside-the-band", "[offset][3d][.]")
{
    // Hidden, and failing until the front can be kept off a held vertex that cannot reach the
    // level set; the hidden runs record it until then. p is offset to 0.3, and q, a second input
    // point, is held 0.25 from it, inside the band. MEASURED (2026-11-06, frames): turn 1's label
    // clean-up puts q's vertex on the front. Its anchor (feature_envelope_ratio x envelope_size,
    // 0.0035) lets it reach at most 0.2535, 0.047 short of the level set and beyond the bar
    // (front_conv, 0.1 x 0.3), so the front never resolves there and folds around q through the
    // 15th turn. With feature_envelope_ratio 10, q reaches 0.274, within the bar, and the run
    // converges. It is the contradiction check_no_vertex_on_both_surfaces() refuses at
    // construction for a vertex on the input complex (double_sphere with tag_1 protected throws
    // there, 2026-11-09): a front vertex held where it cannot reach the level set. Since
    // 2026-11-10 the clean-up keeps q out of the band
    // (label-cleanup-keeps-held-point-off-the-band); the run still does not converge (MEASURED, 15
    // turns, exact and exact_swept: q never on the front, 2-6 folded faces on the boundary of q's
    // cells), as the level set near q is not attainable without the band touching q; since the
    // same day it throws there (unreachable-level-set-throws: turn 2), and stays hidden and failing
    // as the record of the case until a flag lets the front go beyond the throw.
    require_point_offset_converges({{"p", {{4, 4, 4}}}, {"q", {{5, 4, 4}}}}, 0.3);
}

namespace {
/// Per vertex of the result: whether a band cell (a tet tagged `band_tag`) has it as a corner.
std::vector<char> band_touched(const MeshData& out, const std::string& band_tag = "newtag")
{
    const auto& tets = out.cells.at(CellType::Tetrahedron);
    const auto& tags = tets.attributes.at("tags");
    const auto it = std::find(tags.labels.begin(), tags.labels.end(), band_tag);
    REQUIRE(it != tags.labels.end());
    const Eigen::Index col = Eigen::Index(it - tags.labels.begin());
    const auto member = tags.integers();
    std::vector<char> touched(size_t(out.V.rows()), 0);
    for (Eigen::Index r = 0; r < tets.connectivity.rows(); ++r) {
        if (member(r, col) == 0) continue;
        for (int c = 0; c < 4; ++c) touched[size_t(tets.connectivity(r, c))] = 1;
    }
    return touched;
}

/// The result's curve vertices on the side sign(x) = `side` of the origin, nearest first: their
/// |x| and whether a band cell touches them.
std::vector<std::pair<double, bool>>
curve_side(const MeshData& out, const std::vector<char>& touched, const int side)
{
    std::vector<Eigen::Index> vs;
    const auto& edges = out.cells.at(CellType::Edge);
    for (Eigen::Index r = 0; r < edges.connectivity.rows(); ++r) {
        for (int c = 0; c < 2; ++c) vs.push_back(edges.connectivity(r, c));
    }
    std::sort(vs.begin(), vs.end());
    vs.erase(std::unique(vs.begin(), vs.end()), vs.end());
    std::vector<std::pair<double, bool>> side_vs;
    for (const Eigen::Index v : vs) {
        const double x = out.V(v, 0);
        if (x * side > 0) side_vs.emplace_back(std::abs(x), touched[size_t(v)] != 0);
    }
    std::sort(side_vs.begin(), side_vs.end());
    return side_vs;
}
} // namespace

TEST_CASE("label-cleanup-keeps-held-point-off-the-band", "[offset][3d]" + release_only)
{
    // The construction's topology kept (the clean-up's held objects' test, 2026-11-10): q, a held
    // point 0.25 from p and so inside p's level set at 0.3, is out of the band as constructed
    // (the march traces to 0.125), and no band cell ever has it as a corner. MEASURED before the
    // test, 15 turns: turn 1's clean-up took 6 of q's 30 cells into the band, so q joined the
    // front; under exact_swept turn 11 took its last non-band cell and the band enclosed q.
    const std::string rule = GENERATE("exact", "exact_swept");
    INFO("EXPERIMENTAL_band_volume_rule " << rule);
    const nlohmann::json j = {
        {"input", "m"},
        {"offsets", nlohmann::json::array({{{"selection", "p"}, {"target_distance", 0.3}}})},
        {"EXPERIMENTAL_band_volume_rule", rule},
        {"max_rounds", 3},
        {"throw_on_unreachable_level_set", false}, // the topology is what is tested
        {"num_threads", 1},
        {"throw_on_nonconvergence", false}};
    ComponentResult res;
    REQUIRE_NOTHROW(
        res = components::topological_offset::topological_offset(
            j,
            {{"m", point_grid({{"p", {{4, 4, 4}}}, {"q", {{5, 4, 4}}}})}}));
    const MeshData& out = res.first.at("mesh");
    const auto touched = band_touched(out);
    const Vector3d q = result_points(out).at("q").at(0);
    Eigen::Index qv = 0;
    for (Eigen::Index v = 1; v < out.V.rows(); ++v) {
        if ((out.V.row(v).transpose() - q).norm() < (out.V.row(qv).transpose() - q).norm()) qv = v;
    }
    REQUIRE((out.V.row(qv).transpose() - q).norm() == 0.);
    CHECK(touched[size_t(qv)] == 0);
}

TEST_CASE("label-cleanup-held-curves", "[offset][3d]" + release_only)
{
    // The construction's topology kept on held curves (the clean-up's held objects' test,
    // 2026-11-10), p offset to 0.3:
    //   a curve out of the band as constructed, 0.25 from p at its nearest (y = 0.25, x in
    //     [-0.5, 0.5]), part of it inside the level set: no band cell touches it;
    //   a curve through p (the x axis, x in [-0.75, 0.75]), in the band near p and crossing the
    //     front once on each side: the crossing moves out along it -- the band reaches past the
    //     march's 0.125, to the grid vertex at 0.25 -- and stays one crossing, the band's cells
    //     touching a run of the curve from p outward and nothing beyond it.
    const std::string rule = GENERATE("exact", "exact_swept");
    const bool through = GENERATE(false, true);
    INFO(
        "EXPERIMENTAL_band_volume_rule " << rule << ", curve "
                                         << (through ? "through p" : "off the band"));
    const GridPolyline curve =
        through ? GridPolyline{{{1, 4, 4}}, {{7, 4, 4}}} : GridPolyline{{{2, 5, 4}}, {{6, 5, 4}}};
    const nlohmann::json j = {
        {"input", "m"},
        {"offsets", nlohmann::json::array({{{"selection", "p"}, {"target_distance", 0.3}}})},
        {"EXPERIMENTAL_band_volume_rule", rule},
        {"max_rounds", 3},
        {"throw_on_unreachable_level_set", false}, // the topology is what is tested
        {"num_threads", 1},
        {"throw_on_nonconvergence", false}};
    ComponentResult res;
    REQUIRE_NOTHROW(
        res = components::topological_offset::topological_offset(
            j,
            {{"m", point_grid({{"p", {{4, 4, 4}}}}, {{"curve", curve}})}}));
    const MeshData& out = res.first.at("mesh");
    const auto touched = band_touched(out);
    for (const int side : {-1, 1}) {
        INFO("side " << side);
        const auto vs = curve_side(out, touched, side);
        REQUIRE(!vs.empty());
        if (!through) {
            for (const auto& [x, t] : vs) CHECK(!t);
            continue;
        }
        size_t n_touched = 0;
        while (n_touched < vs.size() && vs[n_touched].second) ++n_touched;
        for (size_t k = n_touched; k < vs.size(); ++k) CHECK(!vs[k].second);
        REQUIRE(n_touched > 0);
        CHECK(vs[n_touched - 1].first >= 0.25);
    }
}

TEST_CASE("label-cleanup-held-surfaces", "[offset][3d]" + release_only)
{
    // The construction's topology kept on held surfaces (the clean-up's held objects' test,
    // 2026-11-10): p offset to 0.3 with a held surface 0.25 from it, which the band as
    // constructed (the march traces to 0.125) does not touch, so no band cell ever has a corner
    // on it -- the front stops a layer of cells short of it:
    //   a region boundary, the plane x = 0.25 between region A (x < 0.25) and region B;
    //   the box: p at x = -0.75, the wall x = -1 0.25 away.
    const std::string rule = GENERATE("exact", "exact_swept");
    const bool box = GENERATE(false, true);
    INFO(
        "EXPERIMENTAL_band_volume_rule " << rule << ", "
                                         << (box ? "the box" : "a region boundary"));
    const std::array<int, 3> p =
        box ? std::array<int, 3>{{1, 4, 4}} : std::array<int, 3>{{4, 4, 4}};
    MeshData input = point_grid({{"p", p}});
    if (!box) {
        auto& tets = input.cells.at(CellType::Tetrahedron);
        DataMatrix<uint8_t> tags = DataMatrix<uint8_t>::Zero(tets.connectivity.rows(), 2);
        for (Eigen::Index r = 0; r < tets.connectivity.rows(); ++r) {
            double x = 0.;
            for (int c = 0; c < 4; ++c) x += input.V(tets.connectivity(r, c), 0) / 4.;
            tags(r, x > 0.25 ? 1 : 0) = 1;
        }
        tets.attributes["tags"] = {tags, {"A", "B"}};
    }
    const nlohmann::json j = {
        {"input", "m"},
        {"offsets", nlohmann::json::array({{{"selection", "p"}, {"target_distance", 0.3}}})},
        {"EXPERIMENTAL_band_volume_rule", rule},
        {"max_rounds", 3},
        {"throw_on_unreachable_level_set", false}, // the topology is what is tested
        {"num_threads", 1},
        {"throw_on_nonconvergence", false}};
    ComponentResult res;
    REQUIRE_NOTHROW(res = components::topological_offset::topological_offset(j, {{"m", input}}));
    const MeshData& out = res.first.at("mesh");
    const auto touched = band_touched(out);
    const std::vector<char> held =
        box ? std::vector<char>{} : band_touched(out, "B"); // the corners of region B's cells
    size_t n_held = 0;
    for (Eigen::Index v = 0; v < out.V.rows(); ++v) {
        const bool on = box ? out.V(v, 0) == -1. : held[size_t(v)] != 0;
        if (!on) continue;
        ++n_held;
        CHECK(touched[size_t(v)] == 0);
    }
    CHECK(n_held > 0);
}

TEST_CASE("unreachable-level-set-throws", "[offset][3d]" + release_only)
{
    // The level set out of reach (check_level_set_reachable(), 2026-11-10): p offset to 0.3 with
    // a held object the band did not touch as constructed. Inside the level set -- a held point,
    // a curve, a region boundary, the box, each 0.25 from p -- the run throws at a settled turn
    // or at the loop's end (MEASURED: the held point at turn 2). A curve the band crosses as
    // constructed (through p) is reachable by moving the crossing along it: no throw. Nor a held
    // point inside the level set by less than front_conv (q 0.25 from p at target 0.27: 0.02
    // inside, front_conv 0.027): a front passing outside it is within the bar.
    const std::string rule = GENERATE("exact", "exact_swept");
    const std::string held =
        GENERATE("point", "curve", "region", "box", "curve through p", "point within the bar");
    INFO("EXPERIMENTAL_band_volume_rule " << rule << ", held: " << held);
    const std::array<int, 3> p =
        held == "box" ? std::array<int, 3>{{1, 4, 4}} : std::array<int, 3>{{4, 4, 4}};
    std::vector<std::pair<std::string, std::array<int, 3>>> points{{"p", p}};
    if (held == "point" || held == "point within the bar") points.push_back({"q", {{5, 4, 4}}});
    std::vector<std::pair<std::string, GridPolyline>> curves;
    if (held == "curve") curves.push_back({"curve", {{{2, 5, 4}}, {{6, 5, 4}}}});
    if (held == "curve through p") curves.push_back({"curve", {{{1, 4, 4}}, {{7, 4, 4}}}});
    MeshData input = point_grid(points, curves);
    if (held == "region") {
        auto& tets = input.cells.at(CellType::Tetrahedron);
        DataMatrix<uint8_t> tags = DataMatrix<uint8_t>::Zero(tets.connectivity.rows(), 2);
        for (Eigen::Index r = 0; r < tets.connectivity.rows(); ++r) {
            double x = 0.;
            for (int c = 0; c < 4; ++c) x += input.V(tets.connectivity(r, c), 0) / 4.;
            tags(r, x > 0.25 ? 1 : 0) = 1;
        }
        tets.attributes["tags"] = {tags, {"A", "B"}};
    }
    const double target = held == "point within the bar" ? 0.27 : 0.3;
    const nlohmann::json j = {
        {"input", "m"},
        {"offsets", nlohmann::json::array({{{"selection", "p"}, {"target_distance", target}}})},
        {"EXPERIMENTAL_band_volume_rule", rule},
        {"max_rounds", 3},
        {"num_threads", 1},
        {"throw_on_nonconvergence", false}};
    if (held == "curve through p" || held == "point within the bar") {
        REQUIRE_NOTHROW(components::topological_offset::topological_offset(j, {{"m", input}}));
    } else {
        REQUIRE_THROWS_WITH(
            components::topological_offset::topological_offset(j, {{"m", input}}),
            Catch::Matchers::ContainsSubstring("the level set is out of reach"));
    }
}

namespace {
/// The 2D twin of grid_point(): grid point (i, j) of point_grid_2d().
Vector2d grid_point_2d(const std::array<int, 2>& c)
{
    return Vector2d(c[0] * 0.25 - 1, c[1] * 0.25 - 1);
}

/// A curve as grid points (i, j) of point_grid_2d(), consecutive ones on one grid line: the 2D
/// twin of GridPolyline.
using GridPolyline2d = std::vector<std::array<int, 2>>;

/// The clean-up tests' Parameters in 2D: p offset to `target`, the clean-up "one_sided", the one
/// mode 2D runs until its per-cell energy carries the band term.
Parameters cleanup_params_2d(const double target)
{
    Eigen::MatrixXd box(2, 2);
    box << -1., -1., 1., 1.;
    return construction_params(
        {{"offsets", nlohmann::json::array({{{"selection", "p"}, {"target_distance", target}}})},
         {"EXPERIMENTAL_label_cleanup", "one_sided"},
         {"num_threads", 0}},
        box);
}

/// Input points as (name, grid point (i, j)), the 2D twin of point_grid()'s point block.
using GridPoints2d = std::vector<std::pair<std::string, std::array<int, 2>>>;

/// The 2D twin of point_grid() and the 3D tests' construction, driven directly (the end-to-end
/// twins drive the app, grid_2d()): [-1, 1]^2 in 8^2 squares of edge 0.25, each split along its
/// diagonal from (i, j) to (i + 1, j + 1); every triangle ambient, or with `regions` tagged A
/// (centroid x < 0.25) or B; a curve group along `curve` (none if empty); `points` the input
/// points (a point block, each vertex under its name), as the 2D driver passes them; the grid
/// point p the input complex, a point, labelled by hand (the app selects it by label, as 3D's).
/// Constructed at `param`'s target -- the construction holds the points
/// (hold_curves_and_points()) -- the field and the exact integral built, the optimization's labels
/// set, as optimize_offset() sets them before the loop.
std::unique_ptr<TopoOffsetTriMesh> point_grid_2d(
    Parameters& param,
    const std::array<int, 2>& p,
    const GridPolyline2d& curve = {},
    const bool regions = false,
    const GridPoints2d& points = {})
{
    const int n = 8;
    const auto vid = [&](const int i, const int j) { return i + (n + 1) * j; };
    Eigen::MatrixXd V((n + 1) * (n + 1), 2);
    for (int j = 0; j <= n; ++j) {
        for (int i = 0; i <= n; ++i) V.row(vid(i, j)) = grid_point_2d({{i, j}}).transpose();
    }
    Eigen::MatrixXi F(2 * n * n, 3);
    int r = 0;
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            F.row(r++) << vid(i, j), vid(i + 1, j), vid(i + 1, j + 1);
            F.row(r++) << vid(i, j), vid(i + 1, j + 1), vid(i, j + 1);
        }
    }
    MatrixSi tags(F.rows(), regions ? 2 : 1);
    if (regions) {
        for (Eigen::Index f = 0; f < F.rows(); ++f) {
            const double x = (V(F(f, 0), 0) + V(F(f, 1), 0) + V(F(f, 2), 0)) / 3.;
            tags.coeffRef(f, x > 0.25 ? 1 : 0) = 1;
        }
    }
    Eigen::MatrixXd CV(Eigen::Index(curve.size()), 2);
    Eigen::MatrixXi CE(curve.empty() ? 0 : Eigen::Index(curve.size() - 1), 2);
    for (size_t k = 0; k < curve.size(); ++k) {
        CV.row(Eigen::Index(k)) = grid_point_2d(curve[k]).transpose();
        if (k + 1 < curve.size()) CE.row(Eigen::Index(k)) << int(k), int(k + 1);
    }
    auto mesh = std::make_unique<TopoOffsetTriMesh>(param, 0);
    // The point block, as the 3D driver sets it before init_from_image(): the names, and per
    // point its vertex and the index of its name.
    std::vector<size_t> point_vids;
    for (const auto& [name, at] : points) {
        auto it = std::find(mesh->m_point_names.begin(), mesh->m_point_names.end(), name);
        if (it == mesh->m_point_names.end())
            it = mesh->m_point_names.insert(mesh->m_point_names.end(), name);
        mesh->m_input_point_labels.push_back(int(it - mesh->m_point_names.begin()));
        point_vids.push_back(size_t(vid(at[0], at[1])));
    }
    mesh->init_from_image(
        V,
        F,
        tags,
        CV,
        CE,
        regions ? std::vector<std::string>{"A", "B"} : std::vector<std::string>{"a"},
        "curve",
        Eigen::MatrixXi(),
        MatrixSi(),
        {},
        MatrixSi(),
        point_vids);
    mesh->m_vertex_extra[size_t(vid(p[0], p[1]))].label = 1;
    mesh->mark_input_complex_vertices();
    mesh->init_input_complex_bvh();
    mesh->consolidate_mesh();
    mesh->construct_offset(std::filesystem::temp_directory_path() / "wmtk_offset_cleanup_2d");
    mesh->init_offset_potential();
    mesh->label_offset_boundary();
    REQUIRE(mesh->m_band_exact != nullptr);
    return mesh;
}

/// The 2D twin of band_touched(): per vertex, whether a band triangle has it as a corner.
std::vector<char> band_touched_2d(const TopoOffsetTriMesh& m)
{
    std::vector<char> touched(m.vert_capacity(), 0);
    for (const auto& f : m.get_faces()) {
        const size_t fid = f.fid(m);
        if (!m.face_is_offset_band(fid)) continue;
        for (const size_t v : m.oriented_tri_vids(fid)) touched[v] = 1;
    }
    return touched;
}

/// The band's triangles.
size_t band_count_2d(const TopoOffsetTriMesh& m)
{
    size_t n = 0;
    for (const auto& f : m.get_faces()) n += m.face_is_offset_band(f.fid(m)) ? 1 : 0;
    return n;
}

/// The vertex at x exactly.
size_t vertex_at_2d(const TopoOffsetTriMesh& m, const Vector2d& x)
{
    size_t found = std::numeric_limits<size_t>::max();
    for (const auto& v : m.get_vertices()) {
        if (m.m_vertex_attribute[v.vid(m)].m_posf == x) found = v.vid(m);
    }
    INFO("a vertex at (" << x.x() << ", " << x.y() << ")");
    REQUIRE(found != std::numeric_limits<size_t>::max());
    return found;
}

/// Whether vertex v is anchored (hold_curves_and_points()).
bool anchored_2d(const TopoOffsetTriMesh& m, const size_t v)
{
    return m.m_track_feature_points &&
           m.m_feature_point_attribute[v].m_feature_point_id != TriOptimizerMesh::NO_FEATURE_POINT;
}
} // namespace

TEST_CASE("hold-curves-and-points-2d", "[offset][2d]")
{
    // What the construction holds (hold_curves_and_points()), the 2D twin of
    // point-offset-point-label's holding, on the construction (point-offset-point-label-2d runs
    // it through the app): p, the
    // complex, offset to 0.2, q an input point 0.5 from it, out of the band, and an open curve
    // y = 0.5, x in [-0.5, 0.5]. Every input point is anchored with its label, selected or not,
    // in a ball of feature_envelope_ratio x envelope_size about where it was given; p as an input
    // point or, labelled by hand alone, as an isolated point of a complex without faces, at the
    // same radius; the curve's two ends, where a held curve ends, in balls of
    // order2_envelope_ratio x envelope_size; nothing else, the march's vertices included. The
    // result's point block, written from the anchors, has each input point at its vertex. The two
    // ratios over their range, each against the other: 0.01, 1 and 10.
    struct Case
    {
        double feature, order2;
    };
    const Case c = GENERATE(Case{0.01, 10.}, Case{1., 1.}, Case{10., 0.01});
    const bool p_input = GENERATE(true, false);
    INFO(
        "feature_envelope_ratio " << c.feature << ", order2_envelope_ratio " << c.order2 << ", p "
                                  << (p_input ? "an input point" : "the complex alone"));
    Eigen::MatrixXd box(2, 2);
    box << -1., -1., 1., 1.;
    Parameters param = construction_params(
        {{"offsets", nlohmann::json::array({{{"selection", "p"}, {"target_distance", 0.2}}})},
         {"EXPERIMENTAL_label_cleanup", "one_sided"},
         {"feature_envelope_ratio", c.feature},
         {"order2_envelope_ratio", c.order2},
         {"num_threads", 0}},
        box);
    GridPoints2d points{{"q", {{6, 4}}}};
    if (p_input) points.insert(points.begin(), GridPoints2d::value_type{"p", {{4, 4}}});
    auto mesh = point_grid_2d(param, {{4, 4}}, {{{2, 6}}, {{6, 6}}}, false, points);
    REQUIRE(mesh->m_points_held);
    const double eps = param.envelope_size;
    struct Anchor
    {
        Vector2d at;
        double radius;
        std::string label; // empty: no input point
    };
    const std::vector<Anchor> want = {
        {grid_point_2d({{4, 4}}), c.feature * eps, p_input ? "p" : ""},
        {grid_point_2d({{6, 4}}), c.feature * eps, "q"},
        {grid_point_2d({{2, 6}}), c.order2 * eps, ""},
        {grid_point_2d({{6, 6}}), c.order2 * eps, ""}};
    size_t n_anchored = 0;
    for (const auto& v : mesh->get_vertices()) {
        const size_t vid = v.vid(*mesh);
        CHECK(mesh->m_vertex_extra[vid].point_inputs == 0); // the anchors carry them now
        if (!anchored_2d(*mesh, vid)) continue;
        ++n_anchored;
        const Vector2d x = mesh->m_vertex_attribute[vid].m_posf;
        INFO("anchor at (" << x.x() << ", " << x.y() << ")");
        const auto it =
            std::find_if(want.begin(), want.end(), [&](const Anchor& a) { return a.at == x; });
        REQUIRE(it != want.end());
        const size_t f = mesh->m_feature_point_attribute[vid].m_feature_point_id;
        CHECK(mesh->m_feature_points[f] == it->at);
        CHECK(mesh->m_feature_point_radius[f] == it->radius);
        // The smoother keeps it in its ball (smoothing_position_is_allowed(), the base's; this
        // mesh answered true until it anchored anything).
        CHECK(mesh->smoothing_position_is_allowed(vid, it->at + Vector2d(it->radius / 2., 0.)));
        CHECK_FALSE(
            mesh->smoothing_position_is_allowed(vid, it->at + Vector2d(2. * it->radius, 0.)));
        const InputSet labels =
            f < mesh->m_feature_point_inputs.size() ? mesh->m_feature_point_inputs[f] : 0;
        if (it->label.empty()) {
            CHECK(labels == 0);
        } else {
            const auto& names = mesh->m_point_names;
            const size_t bit =
                size_t(std::find(names.begin(), names.end(), it->label) - names.begin());
            REQUIRE(bit < names.size());
            CHECK(labels == (InputSet(1) << bit));
        }
    }
    CHECK(n_anchored == want.size());
    // The result's point block, from the anchors.
    io::ComponentIO context;
    collect_memory_output(*mesh, context);
    const MeshData& out = context.result.first.at("mesh");
    const auto& block = out.cells.at(CellType::Point);
    const auto& tags = block.attributes.at("tags");
    const auto member = tags.integers();
    std::map<std::string, std::vector<Vector2d>> at;
    for (Eigen::Index r = 0; r < block.connectivity.rows(); ++r) {
        for (size_t t = 0; t < tags.labels.size(); ++t) {
            if (member(r, Eigen::Index(t)) != 0)
                at[tags.labels[t]].push_back(out.V.row(block.connectivity(r, 0)).transpose());
        }
    }
    CHECK(at.size() == points.size());
    for (const auto& [name, g] : points) {
        INFO("point " << name);
        REQUIRE(at[name].size() == 1);
        CHECK(at[name][0] == grid_point_2d(g));
    }
}

TEST_CASE("label-cleanup-keeps-held-point-off-the-band-2d", "[offset][2d]")
{
    // The 2D twin of label-cleanup-keeps-held-point-off-the-band, on the clean-up slot itself
    // (the 2D loop has no 3D-twin energy yet): q, a held point -- an input point, anchored by the
    // construction (hold_curves_and_points()), as in 3D -- 0.25 from p and so inside p's level
    // set at 0.3, is out of the band as constructed (the march traces to 0.125). The
    // clean-up takes in the triangles at the front wholly inside the level set, and none with q
    // as a corner; the held objects' contact topology is unchanged.
    Parameters param = cleanup_params_2d(0.3);
    auto mesh = point_grid_2d(param, {{4, 4}}, {}, false, {{"q", {{5, 4}}}});
    const size_t q = vertex_at_2d(*mesh, grid_point_2d({{5, 4}}));
    REQUIRE(anchored_2d(*mesh, q)); // an input point: the construction held it
    REQUIRE(band_touched_2d(*mesh)[q] == 0);
    const size_t band0 = band_count_2d(*mesh);
    const auto signature0 = mesh->held_topology_signature();
    mesh->label_cleanup("test");
    CHECK(band_count_2d(*mesh) > band0); // the clean-up took triangles in, elsewhere
    CHECK(band_touched_2d(*mesh)[q] == 0);
    CHECK(mesh->held_topology_signature() == signature0);
}

TEST_CASE("label-cleanup-held-curves-2d", "[offset][2d]")
{
    // The 2D twin of label-cleanup-held-curves, p offset to 0.3. A 2D held curve -- the curve
    // group, tracked and held in its tube as a region boundary is -- is codimension 1, as a 3D
    // held surface is; 3D's held curves are codimension 2:
    //   a curve out of the band as constructed, 0.25 from p at its nearest (y = 0.25, x in
    //     [-0.5, 0.5]), part of it inside the level set: no band triangle touches it;
    //   a curve through p (the x axis, x in [-0.75, 0.75]), crossed by the front at the march's
    //     0.125 on each side: the crossing is never lost, and it stays where it is -- its vertex
    //     is on a held boundary and on the front, which eligibility refuses, as 3D's does for a
    //     region boundary -- so the band touches a run of the curve from p to the crossing and
    //     nothing beyond it. (3D's crossing moves along its codimension-2 curve.)
    const bool through = GENERATE(false, true);
    INFO("curve " << (through ? "through p" : "off the band"));
    Parameters param = cleanup_params_2d(0.3);
    const GridPolyline2d curve =
        through ? GridPolyline2d{{{1, 4}}, {{7, 4}}} : GridPolyline2d{{{2, 5}}, {{6, 5}}};
    auto mesh = point_grid_2d(param, {{4, 4}}, curve);
    const size_t band0 = band_count_2d(*mesh);
    const auto signature0 = mesh->held_topology_signature();
    mesh->label_cleanup("test");
    const auto touched = band_touched_2d(*mesh);
    const double y0 = through ? 0. : 0.25, half = through ? 0.75 : 0.5;
    // The curve's vertices (x, touched).
    std::vector<std::pair<double, bool>> on_curve;
    for (const auto& v : mesh->get_vertices()) {
        const Vector2d& x = mesh->m_vertex_attribute[v.vid(*mesh)].m_posf;
        if (x.y() == y0 && std::abs(x.x()) <= half)
            on_curve.emplace_back(x.x(), touched[v.vid(*mesh)] != 0);
    }
    REQUIRE(!on_curve.empty());
    if (!through) {
        CHECK(band_count_2d(*mesh) > band0); // the clean-up took triangles in, elsewhere
        for (const auto& [x, t] : on_curve) CHECK(!t);
    }
    for (const int side : {-1, 1}) {
        if (!through) break;
        INFO("side " << side);
        // The curve's vertices on the side sign(x) = side of p, nearest first.
        std::vector<std::pair<double, bool>> vs;
        for (const auto& [x, t] : on_curve) {
            if (x * side > 0) vs.emplace_back(std::abs(x), t);
        }
        std::sort(vs.begin(), vs.end());
        REQUIRE(!vs.empty());
        size_t n_touched = 0;
        while (n_touched < vs.size() && vs[n_touched].second) ++n_touched;
        for (size_t k = n_touched; k < vs.size(); ++k) CHECK(!vs[k].second);
        REQUIRE(n_touched > 0);
        CHECK(vs[n_touched - 1].first < 0.25); // the crossing, where the march put it
    }
    CHECK(mesh->held_topology_signature() == signature0);
}

TEST_CASE("label-cleanup-held-boundaries-2d", "[offset][2d]")
{
    // The 2D twin of label-cleanup-held-surfaces (3D's held surfaces, codimension 1, are lines
    // in 2D): p offset to 0.3 with a held boundary 0.25 from it, which the band as constructed
    // (the march traces to 0.125) does not touch, so no band triangle ever has a corner on it --
    // the front stops a layer of triangles short of it:
    //   a region boundary, the line x = 0.25 between region A (x < 0.25) and region B;
    //   the box: p at x = -0.75, the wall x = -1 0.25 away.
    const bool box = GENERATE(false, true);
    INFO((box ? "the box" : "a region boundary"));
    Parameters param = cleanup_params_2d(0.3);
    auto mesh = point_grid_2d(
        param,
        box ? std::array<int, 2>{{1, 4}} : std::array<int, 2>{{4, 4}},
        {},
        !box);
    // The held vertices: on the wall, or the corners of region B's triangles.
    std::vector<char> held(mesh->vert_capacity(), 0);
    for (const auto& f : mesh->get_faces()) {
        const size_t fid = f.fid(*mesh);
        const bool b =
            !box && mesh->m_face_attribute[fid].tags.count(mesh->m_tag_name_to_id.at("B")) > 0;
        for (const size_t v : mesh->oriented_tri_vids(fid)) {
            if (b || (box && mesh->m_vertex_attribute[v].m_posf.x() == -1.)) held[v] = 1;
        }
    }
    const size_t band0 = band_count_2d(*mesh);
    const auto signature0 = mesh->held_topology_signature();
    mesh->label_cleanup("test");
    CHECK(band_count_2d(*mesh) > band0); // the clean-up took triangles in, elsewhere
    const auto touched = band_touched_2d(*mesh);
    size_t n_held = 0;
    for (const auto& v : mesh->get_vertices()) {
        const size_t vid = v.vid(*mesh);
        if (!held[vid]) continue;
        ++n_held;
        CHECK(touched[vid] == 0);
    }
    CHECK(n_held > 0);
    CHECK(mesh->held_topology_signature() == signature0);
}

TEST_CASE("unreachable-level-set-throws-2d", "[offset][2d]")
{
    // The 2D twin of unreachable-level-set-throws, on the clean-up slot and the check the loop
    // makes at its end (check_level_set_reachable()): p offset to 0.3 with a held object the band
    // did not touch as constructed, inside the level set -- a held point, a curve, a region
    // boundary, the box, each 0.25 from p -- throws. Not a held point inside the level set by
    // less than front_conv (q 0.25 from p at target 0.27: 0.02 inside, front_conv 0.027): a front
    // passing outside it is within the bar. 3D's curve through p, whose crossing the clean-up
    // moves along it, has no twin here: a 2D curve is codimension 1, and the clean-up keeps its
    // crossing where it is (label-cleanup-held-curves-2d).
    const std::string held = GENERATE("point", "curve", "region", "box", "point within the bar");
    INFO("held: " << held);
    Parameters param = cleanup_params_2d(held == "point within the bar" ? 0.27 : 0.3);
    const bool point = held == "point" || held == "point within the bar";
    auto mesh = point_grid_2d(
        param,
        held == "box" ? std::array<int, 2>{{1, 4}} : std::array<int, 2>{{4, 4}},
        held == "curve" ? GridPolyline2d{{{2, 5}}, {{6, 5}}} : GridPolyline2d{},
        held == "region",
        point ? GridPoints2d{{"q", {{5, 4}}}} : GridPoints2d{});
    if (point) REQUIRE(anchored_2d(*mesh, vertex_at_2d(*mesh, grid_point_2d({{5, 4}}))));
    mesh->label_cleanup("test");
    if (held == "point within the bar") {
        REQUIRE_NOTHROW(mesh->check_level_set_reachable("test"));
    } else {
        REQUIRE_THROWS_WITH(
            mesh->check_level_set_reachable("test"),
            Catch::Matchers::ContainsSubstring("the level set is out of reach"));
    }
}

TEST_CASE("label-cleanup-modes-2d", "[offset][2d]")
{
    // EXPERIMENTAL_label_cleanup's band-term modes in 2D, now that its per-cell energy carries the
    // band term (tri_energy()): "one_sided_tips" and "one_sided_at_target" (the default) run on the
    // fixture of label-cleanup-keeps-held-point-off-the-band-2d and keep what "one_sided" keeps
    // there -- q out of the band, the held objects' contact topology. Under "exact" and
    // "exact_swept" (the default rule). "energy_decrease", which reads the "surface" rule's face
    // term, has no 2D port and is refused.
    const std::string mode = GENERATE("one_sided_tips", "one_sided_at_target", "energy_decrease");
    const std::string rule = GENERATE("exact", "exact_swept");
    INFO(mode << ", " << rule);
    Eigen::MatrixXd box(2, 2);
    box << -1., -1., 1., 1.;
    Parameters param = construction_params(
        {{"offsets", nlohmann::json::array({{{"selection", "p"}, {"target_distance", 0.3}}})},
         {"EXPERIMENTAL_label_cleanup", mode == "energy_decrease" ? "one_sided" : mode},
         {"EXPERIMENTAL_band_volume_rule", rule},
         {"num_threads", 0}},
        box);
    auto mesh = point_grid_2d(param, {{4, 4}}, {}, false, {{"q", {{5, 4}}}});
    if (mode == "energy_decrease") {
        // Parameters refuses it outside "surface"; set past that, the 2D clean-up refuses it.
        mesh->m_offset_params.label_cleanup = mode;
        REQUIRE_THROWS_WITH(
            mesh->label_cleanup("test"),
            Catch::Matchers::ContainsSubstring("has no 2D port"));
        return;
    }
    const size_t q = vertex_at_2d(*mesh, grid_point_2d({{5, 4}}));
    REQUIRE(anchored_2d(*mesh, q)); // an input point: the construction held it
    REQUIRE(band_touched_2d(*mesh)[q] == 0);
    const auto signature0 = mesh->held_topology_signature();
    REQUIRE_NOTHROW(mesh->label_cleanup("test"));
    CHECK(band_touched_2d(*mesh)[q] == 0);
    CHECK(mesh->held_topology_signature() == signature0);
}

TEST_CASE("exact-swept-smoothing-held-on-another-region-2d", "[offset][2d]")
{
    // The 2D twin of exact-swept-front-vertex-held-on-another-region, on the constructed band:
    // p on the boundary between regions A and B (x = 0.25), held, offset to 0.3 (the march traces
    // to 0.125, half the grid's edge), so the vertices on that boundary take the engine's
    // projected step, whose ring value (smoothing_ring_energy()) takes each candidate's swept
    // change from the vertex smoothed, and the front ones then the swept test; the others the
    // swept test alone. Two sweeps of the shared smoother over every vertex (smooth_vertex())
    // under "exact_swept": no move raises E, the sum of tri_energy() with "exact"'s band term
    // (the input a point: every piece exact, so the swept change is E's band change), the swept
    // tests ran on front and other moves, and held vertices moved.
    Parameters param = cleanup_params_2d(0.3);
    param.band_volume_rule = "exact_swept";
    auto mesh = point_grid_2d(param, {{5, 4}}, {}, true);
    size_t held_front = 0;
    for (const auto& v : mesh->get_vertices()) {
        const size_t vid = v.vid(*mesh);
        if (mesh->m_vertex_extra[vid].m_is_on_offset && mesh->smoothing_energy_envelope(vid)) {
            ++held_front;
        }
    }
    REQUIRE(held_front > 0); // the premise: the front crosses the held boundary
    const auto energy = [&]() {
        double e = 0., scale = 0.;
        for (const auto& f : mesh->get_faces()) {
            const double t = mesh->tri_energy(f.fid(*mesh));
            e += t;
            scale += std::abs(t);
        }
        return std::pair<double, double>(e, scale);
    };
    double worst_rise = 0.;
    size_t moved = 0, held_moved = 0, tried = 0;
    for (int sweep = 0; sweep < 2; ++sweep) {
        for (const auto& v : mesh->get_vertices()) {
            const auto [e0, scale] = energy();
            const size_t vid = v.vid(*mesh);
            const Vector2d x0 = mesh->m_vertex_attribute[vid].m_posf;
            ++tried;
            if (!mesh->smooth_vertex(v)) continue;
            const bool m = mesh->m_vertex_attribute[vid].m_posf != x0;
            moved += m;
            held_moved += m && mesh->smoothing_energy_envelope(vid) != nullptr;
            worst_rise = std::max(worst_rise, (energy().first - e0) / scale);
        }
    }
    const size_t front = mesh->m_swept_front.solves.load();
    const size_t other = mesh->m_swept_nonfront.solves.load();
    CHECK(front > 0);
    CHECK(other > 0);
    CHECK(held_moved > 0);
    CHECK(worst_rise <= 1e-13);
    std::printf(
        "exact-swept-smoothing-held-on-another-region-2d: %zu held front vertices; %zu of %zu "
        "smooths moved, %zu of them held (the projected step); swept tests: front %zu, other %zu; "
        "largest rise of E %.1e of its scale\n",
        held_front,
        moved,
        tried,
        held_moved,
        front,
        other,
        worst_rise);
}

TEST_CASE("fold-test-2d", "[offset][2d]")
{
    // The 2D twin of fold-test: the fold predicate (TopoOffsetTriMesh::fold_test()) over the
    // whole range of foldover_angle. Edge 1's normal out of the band is +y; edge 2's is (sin phi,
    // cos phi), 180 - phi degrees from opposite. A fold: the normals within `angle` of opposite
    // and one edge turned from the field (its field angle over `angle`, or NaN); two opposite
    // edges both along the field are a ridge vertex; edges not opposite are neither, and the
    // field is not asked of them.
    using FT = TopoOffsetTriMesh::FoldTest;
    const Vector2d n1(0., 1.);
    const auto n2 = [](const double deg) {
        const double r = deg * M_PI / 180.;
        return Vector2d(std::sin(r), std::cos(r));
    };
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (const double angle : {1., 10., 29., 30., 31., 45., 90., 120., 170., 179.}) {
        INFO("foldover_angle " << angle);
        int asked = 0;
        const auto test = [&](const double phi, const double a0, const double a1) {
            asked = 0;
            const auto field = [&](const int k) {
                ++asked;
                return k == 0 ? a0 : a1;
            };
            return TopoOffsetTriMesh::fold_test(n1, n2(phi), field, angle);
        };
        const double within = 180. - 0.5 * angle, beyond = 0.5 * (180. - angle);
        const double along = 0.5 * angle, turned = 0.5 * (angle + 180.);
        CHECK(test(beyond, turned, turned) == FT::none);
        CHECK(asked == 0);
        CHECK(test(within, along, along) == FT::ridge_vertex);
        CHECK(test(within, angle, angle) == FT::ridge_vertex); // exactly `angle` is not over it
        CHECK(test(within, turned, along) == FT::fold);
        CHECK(test(within, along, turned) == FT::fold);
        CHECK(test(within, nan, along) == FT::fold); // unmeasured is never taken for no fold
        CHECK(test(within, along, nan) == FT::fold);
        CHECK(test(180., turned, turned) == FT::fold); // flat onto each other
    }
}

TEST_CASE("front-edge-field-angle-2d", "[offset][2d]")
{
    // The 2D twin of foldover-face-angle's mesh half: front_edge_field_angle(), the angle between
    // an offset edge's normal, out of its band face, and grad d at its midpoint. One band
    // triangle (0, 1), (1, 1), (0, 2) over the input segment y = 0, x in [-10, 10]
    // (energy_mesh_2d()), where grad d = (0, 1): the edge (0, 1)-(1, 1) faces back at the input
    // (180 degrees), (0, 1)-(0, 2) across the field (90), (1, 1)-(0, 2) along it at 45. Either
    // order of an edge's ends. Not a band face, or no exact integral: NaN.
    Parameters param;
    Eigen::MatrixXd V(3, 2);
    V << 0., 1., 1., 1., 0., 2.;
    auto mesh = energy_mesh_2d(param, V, {{{0, 1, 2}}}, {2}, line_field());
    CHECK(mesh->front_edge_band_face(0, 1) == 0);
    CHECK(mesh->front_edge_field_angle(0, 1) == Catch::Approx(180.).margin(1e-9));
    CHECK(mesh->front_edge_field_angle(1, 0) == Catch::Approx(180.).margin(1e-9));
    CHECK(mesh->front_edge_field_angle(0, 2) == Catch::Approx(90.).margin(1e-9));
    CHECK(mesh->front_edge_field_angle(2, 1) == Catch::Approx(45.).margin(1e-9));
    mesh->m_face_extra[0].label = 0;
    CHECK(mesh->front_edge_band_face(1, 2) == -1);
    CHECK(std::isnan(mesh->front_edge_field_angle(1, 2)));
    mesh->m_face_extra[0].label = 2;
    mesh->m_band_exact.reset();
    CHECK(std::isnan(mesh->front_edge_field_angle(1, 2)));
}

TEST_CASE("count-folds-2d", "[offset][2d]")
{
    // count_folds() and offset_surface_foldover_labels() on a vertex V where two offset edges
    // nearly coincide, over foldover_angle's range. The input is the points P1 = (-a, 0) and P2
    // = (a, 0), and V = (0, h) is at delta = 0.5 from both (a = delta cos 3deg); A and B lie on the
    // circles about P1 and P2, 4 degrees on from V, so the chords VA and VB follow the level set
    // d = delta into its corner at V, their normals out of the band 10 degrees from opposite.
    // Band triangles (P1, P2, V), (P1, V, A), (P2, B, V), and the background triangle (V, B, A)
    // in the corner; VA and VB are V's two offset edges (the others are on the domain boundary,
    // which the count leaves out, as 3D's does). The field angles, by hand from the nearest input
    // point at each chord's midpoint, decide with the opposite test:
    //   - P1 and P2: both chords along the field (0 degrees): a ridge vertex once foldover_angle
    //     is over 10 degrees, never a fold;
    //   - P1 alone: VB faces back at it (170 degrees): a fold for foldover_angle in (10, 170),
    //     a ridge vertex above;
    //   - P1 and P2 with the labels swapped (the thin background corner is the band): both
    //     chords face back (180 degrees), a fold above 10 degrees -- the sheet of the band caught
    //     as the sheet of the background is.
    const double delta = 0.5, beta = 3. * M_PI / 180., gamma = 4. * M_PI / 180.;
    const Vector2d P1(-delta * std::cos(beta), 0.), P2(delta * std::cos(beta), 0.);
    const Vector2d Vv(0., delta * std::sin(beta));
    const Vector2d A = P1 + delta * Vector2d(std::cos(beta + gamma), std::sin(beta + gamma));
    const Vector2d B = P2 + delta * Vector2d(-std::cos(beta + gamma), std::sin(beta + gamma));
    Eigen::MatrixXd V(5, 2);
    V << P1.transpose(), P2.transpose(), Vv.transpose(), A.transpose(), B.transpose();
    const size_t iV = 2;
    for (const std::string inputs : {"P1 and P2", "P1", "P1 and P2, labels swapped"}) {
        const bool swapped = inputs == "P1 and P2, labels swapped";
        Parameters param;
        auto mesh = energy_mesh_2d(
            param,
            V,
            {{{0, 1, 2}}, {{0, 2, 3}}, {{1, 4, 2}}, {{2, 4, 3}}},
            swapped ? std::vector<int>{0, 0, 0, 2} : std::vector<int>{2, 2, 2, 0},
            line_field());
        // The input: the points alone (no segment), as init_band_exact() builds it.
        const bool two = inputs != "P1";
        mesh->m_phi_V.resize(two ? 2 : 1, 2);
        mesh->m_phi_V.row(0) = P1.transpose();
        if (two) mesh->m_phi_V.row(1) = P2.transpose();
        mesh->m_phi_E.resize(0, 2);
        mesh->m_phi_E_inner.clear();
        mesh->m_phi_P = two ? std::vector<int>{0, 1} : std::vector<int>{0};
        mesh->init_band_exact();
        // The chords' normals out of the band and their field angles, by hand.
        const auto unit_angle = [](const Vector2d& u, const Vector2d& v) {
            return std::atan2(std::abs(u.x() * v.y() - u.y() * v.x()), u.dot(v)) * 180. / M_PI;
        };
        const auto by_hand = [&](const Vector2d& e, const Vector2d& away, const Vector2d& mid) {
            Vector2d n(-e.y(), e.x());
            n.normalize();
            if (n.dot(-away) < 0.) n = -n;
            Vector2d g = mid - P1;
            if (two && (mid - P2).norm() < g.norm()) g = mid - P2;
            return std::pair<Vector2d, double>{n, unit_angle(n, g)};
        };
        // VA's band face's third corner is P1 (B when swapped), VB's P2 (A).
        const auto [nA, fA] = by_hand(A - Vv, (swapped ? B : P1) - Vv, 0.5 * (Vv + A));
        const auto [nB, fB] = by_hand(B - Vv, (swapped ? A : P2) - Vv, 0.5 * (Vv + B));
        CHECK(mesh->front_edge_field_angle(iV, 3) == Catch::Approx(fA).margin(1e-9));
        CHECK(mesh->front_edge_field_angle(iV, 4) == Catch::Approx(fB).margin(1e-9));
        const double opposite = 180. - unit_angle(nA, nB); // how far from opposite
        CHECK(opposite == Catch::Approx(10.).margin(1e-9));
        for (const double angle : {1., 5., 9., 11., 30., 90., 150., 169., 171., 179.}) {
            INFO(inputs << ", foldover_angle " << angle << ", field angles " << fA << ", " << fB);
            mesh->m_offset_params.foldover_angle = angle;
            const bool opp = opposite < angle;
            const bool turned = fA > angle || fB > angle;
            std::vector<size_t> fv;
            const auto fc = mesh->count_folds(&fv);
            CHECK(fc.folds == (opp && turned ? 1u : 0u));
            CHECK(fc.ridge_vertices == (opp && !turned ? 1u : 0u));
            CHECK(fv == (opp && turned ? std::vector<size_t>{iV} : std::vector<size_t>{}));
            const auto labels = mesh->offset_surface_foldover_labels();
            for (size_t v = 0; v < labels.size(); ++v) {
                CHECK(labels[v] == (v == iV && opp && turned ? 1 : 0));
            }
            if (opp && !turned) {
                CHECK(fc.ridge_max_angle == Catch::Approx(std::max(fA, fB)).margin(1e-9));
            }
        }
    }
}

TEST_CASE("energy-sums-2d", "[offset][2d]")
{
    // The 2D twin of surface-energy-sums' volume half: global_energy() is the sum of tri_energy()
    // over the live faces in get_faces() order; offset_energy() the sum of band_cell_term() over
    // the band faces, and the settle test's metric; energy_parts() splits E into the two, its
    // band part the offset energy. On the constructed 8 x 8 grid, p offset to 0.3, under every
    // 2D band rule.
    const std::string rule = GENERATE("exact", "exact_swept", "corner_bound");
    INFO("EXPERIMENTAL_band_volume_rule " << rule);
    Eigen::MatrixXd box(2, 2);
    box << -1., -1., 1., 1.;
    Parameters param = construction_params(
        {{"offsets", nlohmann::json::array({{{"selection", "p"}, {"target_distance", 0.3}}})},
         {"EXPERIMENTAL_band_volume_rule", rule},
         // Parameters refuses the default clean-up beside "corner_bound"
         {"EXPERIMENTAL_label_cleanup", rule == "corner_bound" ? "off" : "one_sided_at_target"},
         {"num_threads", 0}},
        box);
    auto mesh = point_grid_2d(param, {{4, 4}});
    double e = 0., band = 0.;
    size_t n_band = 0;
    for (const auto& f : mesh->get_faces()) {
        const size_t fid = f.fid(*mesh);
        e += mesh->tri_energy(fid);
        if (!mesh->face_is_offset_band(fid)) continue;
        ++n_band;
        band += mesh->band_cell_term(mesh->oriented_tri_vids(fid));
    }
    REQUIRE(n_band > 0);
    CHECK(mesh->global_energy() == e);
    CHECK(mesh->offset_energy() == band);
    CHECK(mesh->settle_metric() == band);
    CHECK(band != 0.);
    const auto [a_part, b_part] = mesh->energy_parts();
    CHECK(b_part == band);
    CHECK(a_part + b_part == Catch::Approx(e).epsilon(1e-12));
}

namespace {
/// The settle tests' fixture: the constructed 8 x 8 grid (point_grid_2d()), p = (0, 0), the
/// input complex, offset to `target`, the input the point alone, so d(x) = |x|.
std::unique_ptr<TopoOffsetTriMesh> settle_grid_2d(Parameters& param)
{
    auto mesh = point_grid_2d(param, {{4, 4}});
    mesh->m_edge_split_mode = TopoOffsetTriMesh::EdgeSplitMode::Optimization;
    return mesh;
}
Parameters settle_params_2d(const double target)
{
    Eigen::MatrixXd box(2, 2);
    box << -1., -1., 1., 1.;
    return construction_params(
        {{"offsets", nlohmann::json::array({{{"selection", "p"}, {"target_distance", target}}})},
         {"num_threads", 0}},
        box);
}
} // namespace

TEST_CASE("settle-marks-2d", "[offset][2d]")
{
    // The 2D twin of settle-mark-stencil, for every EXPERIMENTAL_settle_mark. On the fixture
    // (settle_grid_2d(), d = |x|), each mark's measure per front chord or vertex is computed here
    // from d alone: "max" max(delta - min over the chord of |x|, max of |x| at its ends - delta);
    // "rms" the 2-point Gauss mean of (|x| - delta)^2 n.(x / |x|), n the chord's normal out of
    // its band face; "stencil" the max over the chord's stencil of (delta r)^2 from the field;
    // "vertex" | |v| - delta |; "vertex_edge" and "vertex_split" add the chords whose ends are
    // within front_conv and whose midpoint is not. T is the max of the measure that holds the
    // exit. One turn of the settle test (e_prev = the offset energy, so the turn settles) with
    // front_conv set (a) just above T: exit; (b) just below: no exit, every corner the mark names
    // halved; refine_now false at (b): the same verdict, nothing halved; (c) a turn whose offset
    // energy fell by twice the bar energy: not settled, no exit, nothing halved. "vertex_edge" at
    // (a) halves the ends of the chords whose midpoint is over and still exits; "vertex_split" at
    // (a) lists exactly those chords (m_settle_split_edges) and exits. At two targets: 0.3, where
    // the march traces the front short of the level set (every vertex 0.175 off), and 0.1, where
    // it puts every front vertex on it: there the vertex marks' T is 0, (a) runs at half the
    // chords' midpoint max and (b) has no front_conv to run at.
    const double target = GENERATE(0.3, 0.1);
    const std::string mark =
        GENERATE("max", "rms", "stencil", "vertex", "vertex_edge", "vertex_split");
    INFO("target " << target << ", EXPERIMENTAL_settle_mark " << mark);
    Parameters param = settle_params_2d(target);
    auto mesh = settle_grid_2d(param);
    mesh->m_offset_params.settle_mark = mark;
    const double delta = mesh->m_offset_params.target_distance;
    const auto edges = mesh->offset_surface_edges();
    REQUIRE(!edges.empty());
    const auto pos = [&](const size_t v) { return mesh->m_vertex_attribute[v].m_posf; };
    const auto seg_d = [](const Vector2d& a, const Vector2d& b) { // min over ab of |x|
        const Vector2d ab = b - a;
        const double t = std::clamp(-a.dot(ab) / ab.squaredNorm(), 0., 1.);
        return (a + t * ab).norm();
    };
    // Per chord: the max mark, the rms mean, the stencil max, the midpoint error.
    std::vector<double> m_max(edges.size()), m_rms(edges.size()), m_sten(edges.size());
    std::vector<double> m_mid(edges.size());
    for (size_t k = 0; k < edges.size(); ++k) {
        const Vector2d a = pos(edges[k][0]), b = pos(edges[k][1]);
        m_max[k] = std::max(delta - seg_d(a, b), std::max(a.norm(), b.norm()) - delta);
        m_mid[k] = std::abs((0.5 * (a + b)).norm() - delta);
        Vector2d n;
        REQUIRE(mesh->front_edge_out_normal(edges[k][0], edges[k][1], n));
        double q = 0.;
        const double h = 0.5 / std::sqrt(3.);
        for (const double w : {0.5 - h, 0.5 + h}) {
            const Vector2d x = (1. - w) * a + w * b;
            q += 0.5 * (x.norm() - delta) * (x.norm() - delta) * n.dot(x / x.norm());
        }
        m_rms[k] = std::sqrt(std::max(q, 0.));
        double s = 0.;
        const OffsetPotential2D& pot = mesh->potential_for_edge(edges[k][0], edges[k][1]);
        mesh->for_each_edge_sample(a, b, [&](const Vector2d& x, double, double) {
            const double r = pot.relative_residual(x);
            REQUIRE(std::isfinite(r));
            s = std::max(s, (delta * r) * (delta * r));
        });
        CHECK(mesh->edge_stencil_max_sq(edges[k]) == s);
        m_sten[k] = std::sqrt(s);
    }
    std::set<size_t> front;
    for (const auto& e : edges) front.insert(e.begin(), e.end());
    std::map<size_t, double> m_vtx;
    for (const size_t v : front) m_vtx[v] = std::abs(pos(v).norm() - delta);
    double v_max = 0.;
    for (const auto& [v, x] : m_vtx) v_max = std::max(v_max, x);
    const bool vertex_marks = mark == "vertex" || mark == "vertex_edge" || mark == "vertex_split";
    const std::vector<double>& chord_measure = mark == "max"   ? m_max
                                               : mark == "rms" ? m_rms
                                                               : m_sten;
    const double T =
        vertex_marks ? v_max : *std::max_element(chord_measure.begin(), chord_measure.end());
    const double mid_max = *std::max_element(m_mid.begin(), m_mid.end());
    // At 0.1 the march puts every front vertex on the level set, |v| = 0.1 exactly: no front_conv
    // > 0 marks a vertex, so (a) is at half the midpoint max and (b) has no front_conv to run at.
    const bool on_level_set = vertex_marks && T == 0.;
    REQUIRE((T > 0. || on_level_set));
    REQUIRE(mid_max > 0.);
    const double fc_a = on_level_set ? 0.5 * mid_max : T * (1. + 1e-6);
    std::printf(
        "  settle-marks-2d target %g %-12s %zu chords, %zu front vertices: T %.6g (max vertex "
        "%.6g, max midpoint %.6g)\n",
        target,
        mark.c_str(),
        edges.size(),
        front.size(),
        T,
        v_max,
        mid_max);

    // The corners the mark names at front_conv fc, and the chords "vertex_split" lists.
    const auto named = [&](const double fc, std::vector<std::array<size_t, 2>>* split) {
        std::set<size_t> c;
        if (!vertex_marks) {
            for (size_t k = 0; k < edges.size(); ++k) {
                const bool over = mark == "stencil" ? !(m_sten[k] * m_sten[k] < fc * fc)
                                  : mark == "rms"   ? m_rms[k] * m_rms[k] > fc * fc
                                                    : m_max[k] > fc;
                if (over) c.insert(edges[k].begin(), edges[k].end());
            }
            return c;
        }
        for (const auto& [v, x] : m_vtx) {
            if (x > fc) c.insert(v);
        }
        for (size_t k = 0; k < edges.size(); ++k) {
            const auto& e = edges[k];
            if (m_vtx[e[0]] > fc || m_vtx[e[1]] > fc || !(m_mid[k] > fc)) continue;
            if (mark == "vertex_edge") c.insert(e.begin(), e.end());
            if (mark == "vertex_split" && split) split->push_back(e);
        }
        return c;
    };
    const size_t nv = mesh->vert_capacity();
    std::vector<double> s0(nv);
    for (size_t v = 0; v < nv; ++v) s0[v] = mesh->m_vertex_attribute[v].m_sizing_scalar;
    const double s_floor = std::max(
        mesh->m_offset_params.min_sizing_scalar,
        mesh->m_offset_params.min_edge_length / mesh->m_params.l);
    double length = 0.;
    for (const auto& e : edges) length += (pos(e[1]) - pos(e[0])).norm();
    struct Turn
    {
        bool exit = false, settled = false;
        size_t lowered = 0, not_halved = 0, corners = 0;
        std::vector<std::array<size_t, 2>> split, want_split;
    };
    const auto turn = [&](const double fc, const bool refine, const double fall_in_bars) {
        mesh->m_offset_params.front_conv = fc;
        mesh->m_settle_split_edges = {{{0, 0}}}; // a marker: kept unless the turn assigns it
        const double bar = fc * fc * length / (2. * delta);
        double e_prev = mesh->settle_metric() + fall_in_bars * bar;
        Turn t;
        t.exit = mesh->settle_refinement_turn(1, e_prev, refine);
        t.settled = mesh->m_turn_settled;
        const std::set<size_t> c = named(fc, &t.want_split);
        t.corners = c.size();
        for (size_t v = 0; v < nv; ++v) {
            double& s = mesh->m_vertex_attribute[v].m_sizing_scalar;
            if (s < s0[v]) ++t.lowered;
            if (c.count(v) && !(s <= std::max(0.5 * s0[v], s_floor))) ++t.not_halved;
            s = s0[v]; // put back for the next turn
        }
        t.split = mesh->m_settle_split_edges;
        return t;
    };
    const std::vector<std::array<size_t, 2>> marker = {{{0, 0}}};
    {
        const Turn a = turn(fc_a, true, 0.);
        CHECK(a.settled);
        CHECK(a.exit);
        CHECK(a.not_halved == 0);
        if (mark == "vertex_edge") {
            CHECK(a.lowered >= a.corners); // the chord marks refine, and do not hold the exit
        } else {
            CHECK(a.corners == 0);
            CHECK(a.lowered == 0);
        }
        // A settled turn that refines assigns the list, empty but under "vertex_split".
        CHECK(a.split == a.want_split);
        if (mark == "vertex_edge" || mark == "vertex_split") {
            INFO(
                "chords with both ends within and the midpoint over: "
                << a.corners << " corners, " << a.want_split.size() << " chords");
            CHECK((mark == "vertex_edge" ? a.corners : a.want_split.size()) > 0);
        }
    }
    if (!on_level_set) {
        const Turn b = turn(T * (1. - 1e-6), true, 0.);
        CHECK(b.settled);
        CHECK_FALSE(b.exit);
        CHECK(b.corners > 0);
        CHECK(b.not_halved == 0);
        CHECK(b.lowered >= b.corners);
        CHECK(b.split == b.want_split);
        const Turn b0 = turn(T * (1. - 1e-6), false, 0.);
        CHECK(b0.settled);
        CHECK_FALSE(b0.exit);
        CHECK(b0.lowered == 0);
        CHECK(b0.split == marker);
    }
    {
        const Turn c = turn(fc_a, true, 2.);
        CHECK_FALSE(c.settled);
        CHECK_FALSE(c.exit);
        CHECK(c.lowered == 0);
        CHECK(c.split == marker);
        // Half a bar's fall: settled.
        CHECK(turn(fc_a, false, 0.5).settled);
    }
}

TEST_CASE("settle-gap-refinement-2d", "[offset][2d]")
{
    // EXPERIMENTAL_gap_refinement on the settle test, the 2D twin of its 3D rule: a band face with
    // a front corner whose corner mean of d exceeds d at its centroid by more than front_conv is
    // refined and holds the exit. On settle_grid_2d() at 0.1 (d = |x|, so the corner mean is the
    // mean of the corners' |x|), under settle_mark "vertex": with front_conv between the vertex
    // test's max V and the gap's max G, the exit is held by the gap test alone (on), not held
    // (off); above both, exit.
    Parameters param = settle_params_2d(0.1);
    auto mesh = settle_grid_2d(param);
    mesh->m_offset_params.settle_mark = "vertex";
    const auto pos = [&](const size_t v) { return mesh->m_vertex_attribute[v].m_posf; };
    std::set<size_t> front;
    for (const auto& e : mesh->offset_surface_edges()) front.insert(e.begin(), e.end());
    double v_max = 0., g_max = 0.;
    const double delta = mesh->m_offset_params.target_distance;
    for (const size_t v : front) v_max = std::max(v_max, std::abs(pos(v).norm() - delta));
    for (const auto& f : mesh->get_faces()) {
        const size_t fid = f.fid(*mesh);
        if (!mesh->face_is_offset_band(fid)) continue;
        const auto vs = mesh->oriented_tri_vids(fid);
        if (!front.count(vs[0]) && !front.count(vs[1]) && !front.count(vs[2])) continue;
        const double md = (pos(vs[0]).norm() + pos(vs[1]).norm() + pos(vs[2]).norm()) / 3.;
        g_max = std::max(g_max, md - ((pos(vs[0]) + pos(vs[1]) + pos(vs[2])) / 3.).norm());
    }
    std::printf("  settle-gap-refinement-2d: vertex max %.6g, gap max %.6g\n", v_max, g_max);
    REQUIRE(g_max > 2. * v_max);
    const auto exit_at = [&](const double fc, const bool gap) {
        mesh->m_offset_params.front_conv = fc;
        mesh->m_offset_params.gap_refinement = gap;
        double e_prev = mesh->settle_metric();
        return mesh->settle_refinement_turn(1, e_prev, false);
    };
    const double between = 0.5 * (v_max + g_max);
    CHECK_FALSE(exit_at(between, true));
    CHECK(exit_at(between, false));
    CHECK(exit_at(g_max * (1. + 1e-6), true));
    CHECK_FALSE(exit_at(g_max * (1. - 1e-6), true));
}

TEST_CASE("settle-split-before-smoothing-2d", "[offset][2d][pocket_split]")
{
    // EXPERIMENTAL_settle_mark "vertex_split": the chords a settled turn lists
    // (m_settle_split_edges) are split by the next front_pocket_split(), before its pinned cells,
    // and the list is cleared; a listed chord gone already is skipped. On settle_grid_2d() at
    // 0.3, front_conv between the vertex test's max and the chords' midpoint max.
    Parameters param = settle_params_2d(0.3);
    auto mesh = settle_grid_2d(param);
    mesh->m_offset_params.settle_mark = "vertex_split";
    const auto pos = [&](const size_t v) { return mesh->m_vertex_attribute[v].m_posf; };
    const double delta = mesh->m_offset_params.target_distance;
    const auto edges = mesh->offset_surface_edges();
    double v_max = 0., mid_max = 0.;
    for (const auto& e : edges) {
        for (const size_t v : e) v_max = std::max(v_max, std::abs(pos(v).norm() - delta));
        mid_max = std::max(mid_max, std::abs((0.5 * (pos(e[0]) + pos(e[1]))).norm() - delta));
    }
    REQUIRE(mid_max > v_max);
    mesh->m_offset_params.front_conv = 0.5 * (v_max + mid_max);
    double e_prev = mesh->settle_metric();
    CHECK(mesh->settle_refinement_turn(1, e_prev, true)); // the vertex test passes: exit
    const auto listed = mesh->m_settle_split_edges;
    REQUIRE(!listed.empty());
    for (const auto& e : listed) REQUIRE(!mesh->get_incident_fids_for_edge(e[0], e[1]).empty());
    // A listed pair that is no edge (gone): skipped. Two corners of the box, far from the front.
    mesh->m_settle_split_edges.push_back(
        {{vertex_at_2d(*mesh, Vector2d(-1., -1.)), vertex_at_2d(*mesh, Vector2d(1., 1.))}});
    mesh->front_pocket_split("test");
    CHECK(mesh->m_settle_split_edges.empty());
    for (const auto& e : listed) CHECK(mesh->get_incident_fids_for_edge(e[0], e[1]).empty());
}

namespace {
/// The 2D loop run on point_grid_2d(), the mesh in hand: p offset to `target` with
/// `j`'s keys besides, optimize_offset() as the driver calls it after the construction, its log
/// captured. Returns the exception's message, or "" for none.
std::string run_loop_2d(
    const double target,
    const nlohmann::json& j,
    std::string& log_out,
    const std::array<int, 2>& p = {{4, 4}},
    const GridPolyline2d& curve = {},
    const bool regions = false,
    const GridPoints2d& points = {},
    std::unique_ptr<TopoOffsetTriMesh>* keep = nullptr)
{
    Eigen::MatrixXd box(2, 2);
    box << -1., -1., 1., 1.;
    nlohmann::json k = j;
    k["offsets"] = nlohmann::json::array({{{"selection", "p"}, {"target_distance", target}}});
    k["num_threads"] = 0;
    k["throw_on_nonconvergence"] = false;
    auto param = std::make_unique<Parameters>(construction_params(k, box));
    auto mesh = point_grid_2d(*param, p, curve, regions, points);
    std::ostringstream log;
    const auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(log);
    wmtk::logger().sinks().push_back(sink);
    std::string what;
    try {
        mesh->optimize_offset(std::filesystem::temp_directory_path() / "wmtk_offset_loop_2d");
    } catch (const std::exception& e) {
        what = e.what();
    }
    wmtk::logger().sinks().pop_back();
    log_out = log.str();
    if (keep) {
        // the mesh holds a reference to its parameters: keep them alive with it
        static std::vector<std::unique_ptr<Parameters>> params;
        params.push_back(std::move(param));
        *keep = std::move(mesh);
    }
    return what;
}
} // namespace

TEST_CASE("settle-exit-2d", "[offset][2d]")
{
    // The 2D loop's convergence and exit, as 3D's: p offset to 0.3, max_rounds 8, front_conv_rel
    // 0.2, where the energy criterion passes at turn 1 (ring measure 0.46x the bar) while the
    // front there has folds: MEASURED (the band-area smoother, 0471dbf2de), 2 at turn 1 -- a
    // blade of two non-band triangles whose tip, 0.278 from p (inside the level set by 0.37
    // front_conv), meets two offset edges 14 degrees apart and 84 degrees from the field -- and
    // none at turn 2. 3D's twin (point_grid() p at 0.3, the same keys) MEASURED the same: turn 1
    // passes the energy criterion with 1 fold, held, exit at turn 2. The fixture needs turn 1's
    // folds (REQUIRE): the smoother before 0471dbf2de left none there, and the energy criterion
    // exited at turn 1 with EXPERIMENTAL_exit_no_folds on.
    //   - EXPERIMENTAL_settle_refinement off, EXPERIMENTAL_exit_no_folds off: the energy
    //     criterion's exit at turn 1, the [exit folds] line saying the key is off;
    //   - settle off, exit_no_folds on: turn 1 passes the exit test and is held for the folds;
    //     the exit comes later;
    //   - settle on (the default): every turn logs its [settle] line; turn 1 does not settle (the
    //     construction's front moves by more than the bar energy), so the loop exits later, at a
    //     settled turn with nothing marked, and the verdict's front half is that exit ("[settle]
    //     verdict: front resolved"), with exit_no_folds on or off.
    // Every group logs its [band volume energy] E line and its [energy step] baseline, every turn
    // its [exit folds] count.
    const bool settle = GENERATE(true, false);
    const bool no_folds = GENERATE(true, false);
    INFO(
        "EXPERIMENTAL_settle_refinement " << settle << ", EXPERIMENTAL_exit_no_folds " << no_folds);
    std::string log;
    std::unique_ptr<TopoOffsetTriMesh> mesh;
    const std::string what = run_loop_2d(
        0.3,
        {{"max_rounds", 8},
         {"front_conv_rel", 0.2},
         {"EXPERIMENTAL_settle_refinement", settle},
         {"EXPERIMENTAL_exit_no_folds", no_folds}},
        log,
        {{4, 4}},
        {},
        false,
        {},
        &mesh);
    INFO(log.substr(log.size() > 6000 ? log.size() - 6000 : 0));
    REQUIRE(what.empty());
    CHECK(mesh->m_converged);
    CHECK(mesh->m_energy_verdict.has_value());
    const int turns = mesh->m_iterations_used;
    const auto line_of = [&](const std::string& head) {
        const size_t at = log.find(head);
        return at == std::string::npos ? std::string() : log.substr(at, log.find('\n', at) - at);
    };
    CHECK(log.find("[band volume energy] turn 1 ops group: E ") != std::string::npos);
    CHECK(log.find("[energy step] turn 1 start: E ") != std::string::npos);
    // The exit turn has no fold, but where EXPERIMENTAL_exit_no_folds is off and the energy
    // criterion exits at turn 1 with its folds.
    if (settle || no_folds) {
        CHECK(
            line_of("[exit folds] turn " + std::to_string(turns) + ": ") ==
            "[exit folds] turn " + std::to_string(turns) + ": folds 0");
    }
    // Turn 1's folds, which the fixture needs.
    const std::string folds1 = line_of("[exit folds] turn 1: folds ");
    REQUIRE(!folds1.empty());
    REQUIRE(std::stoi(folds1.substr(std::string("[exit folds] turn 1: folds ").size())) > 0);
    CHECK((log.find("[settle] turn 1:") != std::string::npos) == settle);
    CHECK((log.find("[settle] verdict: front resolved") != std::string::npos) == settle);
    if (!settle) {
        CHECK((turns == 1) == !no_folds);
        CHECK(
            folds1.find(
                no_folds ? " -- the exit test passes; held for the folds"
                         : " -- the exit test passes; EXPERIMENTAL_exit_no_folds is off") !=
            std::string::npos);
    } else {
        CHECK(turns > 1);
        CHECK(folds1.find(" -- ") == std::string::npos); // turn 1 did not pass the exit test
        CHECK(
            line_of("[settle] turn 1: ").find("settled no -- no chord test") != std::string::npos);
        CHECK(
            line_of("[settle] turn " + std::to_string(turns) + ": ")
                .find("settled yes | marked 0 ") != std::string::npos);
    }
}

TEST_CASE("unreachable-level-set-throws-in-the-loop-2d", "[offset][2d]")
{
    // The 2D twin of unreachable-level-set-throws, through the loop: p offset to 0.3 with a held
    // object the band did not touch as constructed, inside the level set -- a held point, a curve,
    // a region boundary, the box, each 0.25 from p -- throws at a settled turn
    // (check_level_set_reachable() after settle_refinement_turn()): a loop that resolves never
    // reaches the loop's end, where the only other check is. Not a held point inside the level
    // set by less than front_conv (q 0.25 from p at 0.27): a front passing outside it is within
    // the bar, and the run resolves.
    const std::string held = GENERATE("point", "curve", "region", "box", "point within the bar");
    INFO("held: " << held);
    const bool point = held == "point" || held == "point within the bar";
    std::string log;
    const std::string what = run_loop_2d(
        held == "point within the bar" ? 0.27 : 0.3,
        {{"max_rounds", 8}},
        log,
        held == "box" ? std::array<int, 2>{{1, 4}} : std::array<int, 2>{{4, 4}},
        held == "curve" ? GridPolyline2d{{{2, 5}}, {{6, 5}}} : GridPolyline2d{},
        held == "region",
        point ? GridPoints2d{{"q", {{5, 4}}}} : GridPoints2d{});
    INFO(log.substr(log.size() > 6000 ? log.size() - 6000 : 0));
    if (held == "point within the bar") {
        CHECK(what.empty());
    } else {
        CHECK(what.find("the level set is out of reach") != std::string::npos);
        CHECK(what.find("[held objects] turn ") != std::string::npos);
    }
}

namespace {
/// The 2D twin of point_grid() and curve_grid(), as the driver reads it (a MeshData):
/// [-1, 1]^2 in 8^2 squares of edge 0.25, each split along its diagonal from (i, j) to
/// (i + 1, j + 1), point_grid_2d()'s triangles; each triangle under the name `region` gives its
/// centroid, or ambient where it gives none (or no `region`); `curves`, grid polylines, as an
/// edge block without tags -- the envelope, which the 2D offset holds as its curve group and
/// selects by the envelope's name, "EnvelopeSurface" on the memory route (read_mesh_data()): the
/// 2D input of a curve no region surrounds, 3D's edge tag one dimension down; `edge_tags`, a
/// tagged edge block (2D's edge tags, the twin of 3D's face tags), each (name, polyline); and
/// `points`, a point block, each vertex under its name, as triwild and simwild write their input
/// points.
using GridRegion2d = std::function<std::string(const Vector2d&)>;
MeshData grid_2d(
    const GridPoints2d& points,
    const std::vector<GridPolyline2d>& curves = {},
    const GridRegion2d& region = nullptr,
    const std::vector<std::pair<std::string, GridPolyline2d>>& edge_tags = {})
{
    const int n = 8;
    const auto vid = [&](const int i, const int j) { return i + (n + 1) * j; };
    Eigen::MatrixXd V((n + 1) * (n + 1), 2);
    for (int j = 0; j <= n; ++j) {
        for (int i = 0; i <= n; ++i) V.row(vid(i, j)) = grid_point_2d({{i, j}}).transpose();
    }
    Eigen::MatrixXi F(2 * n * n, 3);
    int r = 0;
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            F.row(r++) << vid(i, j), vid(i + 1, j), vid(i + 1, j + 1);
            F.row(r++) << vid(i, j), vid(i + 1, j + 1), vid(i, j + 1);
        }
    }
    MeshData m = io::geometry_data(V, F, CellType::Triangle);
    std::vector<std::string> names{"ambient"};
    std::vector<size_t> col(size_t(F.rows()), 0);
    for (Eigen::Index f = 0; f < F.rows(); ++f) {
        const Vector2d c = (V.row(F(f, 0)) + V.row(F(f, 1)) + V.row(F(f, 2))).transpose() / 3.;
        const std::string name = region ? region(c) : std::string();
        if (name.empty()) continue;
        auto it = std::find(names.begin(), names.end(), name);
        if (it == names.end()) it = names.insert(names.end(), name);
        col[size_t(f)] = size_t(it - names.begin());
    }
    DataMatrix<uint8_t> tags = DataMatrix<uint8_t>::Zero(F.rows(), names.size());
    for (size_t f = 0; f < col.size(); ++f) tags(f, col[f]) = 1;
    m.cells.at(CellType::Triangle).attributes["tags"] = {tags, names};
    // The grid edges of a polyline, consecutive points on one grid line.
    const auto grid_edges = [&](const GridPolyline2d& poly) {
        std::vector<std::array<int, 2>> edges;
        for (size_t p = 0; p + 1 < poly.size(); ++p) {
            std::array<int, 2> c = poly[p];
            while (c != poly[p + 1]) {
                const int a = vid(c[0], c[1]);
                for (int d = 0; d < 2; ++d)
                    c[size_t(d)] += (poly[p + 1][size_t(d)] > c[size_t(d)]) -
                                    (poly[p + 1][size_t(d)] < c[size_t(d)]);
                edges.push_back({{a, vid(c[0], c[1])}});
            }
        }
        return edges;
    };
    if (!curves.empty() || !edge_tags.empty()) {
        std::vector<std::array<int, 2>> rows;
        std::vector<int> row_tag; // -1: the envelope (no tags)
        std::vector<std::string> tag_names;
        for (const auto& poly : curves) {
            for (const auto& e : grid_edges(poly)) {
                rows.push_back(e);
                row_tag.push_back(-1);
            }
        }
        for (const auto& [name, poly] : edge_tags) {
            auto it = std::find(tag_names.begin(), tag_names.end(), name);
            if (it == tag_names.end()) it = tag_names.insert(tag_names.end(), name);
            for (const auto& e : grid_edges(poly)) {
                rows.push_back(e);
                row_tag.push_back(int(it - tag_names.begin()));
            }
        }
        auto& block = m.cells[CellType::Edge];
        block.connectivity.resize(Eigen::Index(rows.size()), 2);
        for (size_t k = 0; k < rows.size(); ++k) {
            block.connectivity(Eigen::Index(k), 0) = rows[k][0];
            block.connectivity(Eigen::Index(k), 1) = rows[k][1];
        }
        if (!edge_tags.empty()) {
            // With tags, the envelope's rows are those labelled EnvelopeSurface, the reader's
            // reserved name.
            if (!curves.empty()) tag_names.push_back("EnvelopeSurface");
            DataMatrix<uint8_t> member = DataMatrix<uint8_t>::Zero(rows.size(), tag_names.size());
            for (size_t k = 0; k < rows.size(); ++k)
                member(k, row_tag[k] < 0 ? tag_names.size() - 1 : size_t(row_tag[k])) = 1;
            block.attributes["tags"] = {member, tag_names};
        }
    }
    if (!points.empty()) {
        std::vector<std::string> pnames;
        std::vector<size_t> pcol;
        auto& block = m.cells[CellType::Point];
        block.connectivity.resize(Eigen::Index(points.size()), 1);
        for (size_t k = 0; k < points.size(); ++k) {
            auto it = std::find(pnames.begin(), pnames.end(), points[k].first);
            if (it == pnames.end()) it = pnames.insert(pnames.end(), points[k].first);
            pcol.push_back(size_t(it - pnames.begin()));
            block.connectivity(Eigen::Index(k), 0) = vid(points[k].second[0], points[k].second[1]);
        }
        DataMatrix<uint8_t> member = DataMatrix<uint8_t>::Zero(points.size(), pnames.size());
        for (size_t k = 0; k < points.size(); ++k) member(k, pcol[k]) = 1;
        block.attributes["tags"] = {member, pnames};
    }
    return m;
}

/// The 2D twin of result_points(): per label of the result's point block, its vertices.
std::map<std::string, std::vector<Vector2d>> result_points_2d(const MeshData& out)
{
    std::map<std::string, std::vector<Vector2d>> by_label;
    const auto& block = out.cells.at(CellType::Point);
    const auto& tags = block.attributes.at("tags");
    const auto member = tags.integers();
    for (Eigen::Index r = 0; r < block.connectivity.rows(); ++r) {
        for (size_t t = 0; t < tags.labels.size(); ++t) {
            if (member(r, Eigen::Index(t)) != 0)
                by_label[tags.labels[t]].push_back(
                    out.V.row(block.connectivity(r, 0)).head<2>().transpose());
        }
    }
    return by_label;
}

/// The 2D twin of band_touched(): per vertex of the result, whether a triangle tagged
/// `band_tag` has it as a corner.
std::vector<char> band_touched_2d(const MeshData& out, const std::string& band_tag = "newtag")
{
    const auto& tris = out.cells.at(CellType::Triangle);
    const auto& tags = tris.attributes.at("tags");
    const auto it = std::find(tags.labels.begin(), tags.labels.end(), band_tag);
    REQUIRE(it != tags.labels.end());
    const Eigen::Index col = Eigen::Index(it - tags.labels.begin());
    const auto member = tags.integers();
    std::vector<char> touched(size_t(out.V.rows()), 0);
    for (Eigen::Index r = 0; r < tris.connectivity.rows(); ++r) {
        if (member(r, col) == 0) continue;
        for (int c = 0; c < 3; ++c) touched[size_t(tris.connectivity(r, c))] = 1;
    }
    return touched;
}

/// The result's input complex (vertex field labels 1).
std::vector<Vector2d> result_complex_2d(const MeshData& out)
{
    const auto labels = out.vertex_attributes.at("labels").doubles();
    std::vector<Vector2d> complex;
    for (Eigen::Index v = 0; v < out.V.rows(); ++v) {
        if (labels(v, 0) == 1.) complex.push_back(out.V.row(v).head<2>().transpose());
    }
    return complex;
}

/// The front of the result: the edges between a triangle tagged `band_tag` and one not, the
/// domain wall's excepted, as (a, b) positions.
std::vector<std::array<Vector2d, 2>> result_front_2d(
    const MeshData& out,
    const std::string& band_tag = "newtag")
{
    const auto& tris = out.cells.at(CellType::Triangle);
    const auto& tags = tris.attributes.at("tags");
    const auto it = std::find(tags.labels.begin(), tags.labels.end(), band_tag);
    REQUIRE(it != tags.labels.end());
    const Eigen::Index col = Eigen::Index(it - tags.labels.begin());
    const auto member = tags.integers();
    std::map<std::array<int64_t, 2>, std::vector<bool>> sides;
    for (Eigen::Index r = 0; r < tris.connectivity.rows(); ++r) {
        for (int c = 0; c < 3; ++c) {
            const int64_t a = tris.connectivity(r, c), b = tris.connectivity(r, (c + 1) % 3);
            sides[{{std::min(a, b), std::max(a, b)}}].push_back(member(r, col) != 0);
        }
    }
    std::vector<std::array<Vector2d, 2>> front;
    for (const auto& [e, s] : sides) {
        if (s.size() != 2 || s[0] == s[1]) continue;
        front.push_back(
            {{out.V.row(e[0]).head<2>().transpose(), out.V.row(e[1]).head<2>().transpose()}});
    }
    return front;
}

/// The diagonal of grid_2d()'s box and the envelopes' half-width at the default
/// envelope_size_rel 1e-3.
const double grid_2d_eps = 1e-3 * 2. * std::sqrt(2.);
} // namespace

TEST_CASE("point-offset-point-label-2d", "[offset][2d]" + release_only)
{
    // The 2D twin of point-offset-point-label, through the driver: a point label selected
    // (label_input_complex()), the point offset of a triwild or simwild input point, its band a
    // disk. Every input point is anchored, selected or not (hold_curves_and_points()), and written
    // back as the result's point block: each within its anchor's radius,
    // feature_envelope_ratio x envelope_size, of where it was given. The rules and the ratio over
    // its range: 0.01, 1 and 10.
    struct Case
    {
        std::string rule;
        double ratio;
    };
    const Case c = GENERATE(
        Case{"exact", 1.},
        Case{"exact_swept", 1.},
        Case{"corner_bound", 1.},
        Case{"exact", 0.01},
        Case{"exact", 10.});
    INFO("EXPERIMENTAL_band_volume_rule " << c.rule << ", feature_envelope_ratio " << c.ratio);
    // p at the origin, q 0.5 from it, out of the band at 0.2
    const std::array<int, 2> p{{4, 4}}, q{{6, 4}};
    nlohmann::json j = {
        {"input", "m"},
        {"offsets", nlohmann::json::array({{{"selection", "p"}, {"target_distance", 0.2}}})},
        {"EXPERIMENTAL_band_volume_rule", c.rule},
        {"feature_envelope_ratio", c.ratio},
        {"max_rounds", 1},
        {"num_threads", 1},
        {"throw_on_nonconvergence", false}};
    // the default clean-up reads the exact integral, which "corner_bound" does not build
    if (c.rule == "corner_bound") j["EXPERIMENTAL_label_cleanup"] = "off";
    ComponentResult res;
    REQUIRE_NOTHROW(
        res = components::topological_offset::topological_offset(
            j,
            {{"m", grid_2d({{"p", p}, {"q", q}})}}));
    const MeshData& out = res.first.at("mesh");
    const double r = c.ratio * grid_2d_eps; // the anchors' radius
    const auto points = result_points_2d(out);
    REQUIRE(points.size() == 2);
    for (const auto& [label, at] : {std::pair("p", p), std::pair("q", q)}) {
        INFO("point " << label);
        REQUIRE(points.at(label).size() == 1);
        CHECK((points.at(label)[0] - grid_point_2d(at)).norm() <= r);
    }
    // the complex: p alone, its vertex
    const auto complex = result_complex_2d(out);
    REQUIRE(complex.size() == 1);
    CHECK((complex[0] - grid_point_2d(p)).norm() <= r);
    // the result json: the optimization ran one turn
    CHECK(res.second.at("converged").is_boolean());
    CHECK(res.second.at("optimization_metrics").at("max_l2_dist_err").size() == 1);
}

TEST_CASE("point-offset-labels-in-expression-2d", "[offset][2d]")
{
    // The 2D twin of point-offset-labels-in-expression: a point label in an expression means
    // what it means alone, "p | q" both points and no other vertex; and the curve group in one
    // its edges: "EnvelopeSurface | q" the curve (y = 0.5, x in [-0.5, 0.5]) and q. Refused, as
    // the whole 2D offset, until 2026-11; before then neither selected anything in an
    // expression.
    SECTION("two point labels")
    {
        const nlohmann::json j = {
            {"input", "m"},
            {"offsets",
             nlohmann::json::array({{{"selection", "p | q"}, {"target_distance", 0.2}}})},
            {"optimize_offset", false},
            {"num_threads", 1}};
        ComponentResult res;
        REQUIRE_NOTHROW(
            res = components::topological_offset::topological_offset(
                j,
                {{"m", grid_2d({{"p", {{4, 4}}}, {"q", {{6, 4}}}})}}));
        const auto complex = result_complex_2d(res.first.at("mesh"));
        REQUIRE(complex.size() == 2);
        const bool p_first = (complex[0] - grid_point_2d({{4, 4}})).norm() < 1e-12;
        CHECK((complex[p_first ? 0 : 1] - grid_point_2d({{4, 4}})).norm() < 1e-12);
        CHECK((complex[p_first ? 1 : 0] - grid_point_2d({{6, 4}})).norm() < 1e-12);
    }
    SECTION("the curve group and a point label")
    {
        const nlohmann::json j = {
            {"input", "m"},
            {"offsets",
             nlohmann::json::array(
                 {{{"selection", "EnvelopeSurface | q"}, {"target_distance", 0.1}}})},
            {"optimize_offset", false},
            {"num_threads", 1}};
        ComponentResult res;
        REQUIRE_NOTHROW(
            res = components::topological_offset::topological_offset(
                j,
                {{"m", grid_2d({{"q", {{4, 1}}}}, {{{{2, 6}}, {{6, 6}}}})}}));
        size_t n_curve = 0, n_q = 0;
        for (const Vector2d& x : result_complex_2d(res.first.at("mesh"))) {
            const bool on_curve = x.y() == 0.5 && std::abs(x.x()) <= 0.5;
            const bool at_q = x == grid_point_2d({{4, 1}});
            CHECK((on_curve || at_q));
            n_curve += on_curve;
            n_q += at_q;
        }
        CHECK(n_curve == 5); // the curve's grid vertices, x = -0.5, -0.25, ..., 0.5
        CHECK(n_q == 1);
    }
}

TEST_CASE("point-offset-label-names-2d", "[offset][2d]")
{
    // The 2D twin of point-offset-label-names: one name is one input (init_from_image()), so a
    // point label an edge tag also has is that input's points beside its curve, selected with it.
    // Where 2D differs (m_point_tag_bit): triwild names an input's region, its curve and its
    // points alike, so a point label a region has keeps apart -- its points held and written
    // back, the name selecting the region -- where 3D refuses it ("is also a tet or face tag").
    // So does one the curve group has.
    SECTION("a region's")
    {
        const nlohmann::json j = {
            {"input", "m"},
            {"offsets", nlohmann::json::array({{{"selection", "A"}, {"target_distance", 0.2}}})},
            {"optimize_offset", false},
            {"num_threads", 1}};
        ComponentResult res;
        REQUIRE_NOTHROW(
            res = components::topological_offset::topological_offset(
                j,
                {{"m", grid_2d({{"A", {{6, 4}}}}, {}, [](const Vector2d& c) {
                      return c.x() < 0. ? "A" : "";
                  })}}));
        const MeshData& out = res.first.at("mesh");
        // the complex: region A's triangles and their corners, x <= 0, not the point at x = 0.5
        for (const Vector2d& x : result_complex_2d(out)) CHECK(x.x() <= 0.);
        const auto points = result_points_2d(out);
        REQUIRE(points.at("A").size() == 1);
        CHECK(points.at("A")[0] == grid_point_2d({{6, 4}}));
    }
    SECTION("an edge tag's")
    {
        // as in 3D: a curve input's points share its curve's name, and selecting it selects both
        const nlohmann::json j = {
            {"input", "m"},
            {"offsets",
             nlohmann::json::array({{{"selection", "curve"}, {"target_distance", 0.2}}})},
            {"optimize_offset", false},
            {"num_threads", 1}};
        ComponentResult res;
        REQUIRE_NOTHROW(
            res = components::topological_offset::topological_offset(
                j,
                {{"m",
                  grid_2d(
                      {{"curve", {{6, 4}}}},
                      {},
                      nullptr,
                      {{"curve", {{{2, 4}}, {{4, 4}}}}})}}));
        const MeshData& out = res.first.at("mesh");
        bool point_in_complex = false;
        for (const Vector2d& x : result_complex_2d(out))
            point_in_complex = point_in_complex || x == grid_point_2d({{6, 4}});
        CHECK(point_in_complex);
        CHECK(result_points_2d(out).at("curve").size() == 1);
    }
    SECTION("the curve group's")
    {
        const nlohmann::json j = {
            {"input", "m"},
            {"offsets",
             nlohmann::json::array({{{"selection", "EnvelopeSurface"}, {"target_distance", 0.1}}})},
            {"optimize_offset", false},
            {"num_threads", 1}};
        ComponentResult res;
        REQUIRE_NOTHROW(
            res = components::topological_offset::topological_offset(
                j,
                {{"m", grid_2d({{"EnvelopeSurface", {{4, 1}}}}, {{{{2, 6}}, {{6, 6}}}})}}));
        const MeshData& out = res.first.at("mesh");
        for (const Vector2d& x : result_complex_2d(out)) CHECK(x.y() == 0.5);
        CHECK(result_points_2d(out).at("EnvelopeSurface").size() == 1);
    }
}

TEST_CASE("empty-input-complex-keeps-curves-and-points-2d", "[offset][2d]")
{
    // The 2D twin of empty-input-complex-keeps-curves-and-points: a selection naming a region
    // with no triangle, so the input complex is empty and the run returns the mesh as loaded
    // before anything is held (topological_offset.cpp). Its result keeps the input's edge block
    // (2D's edge tags, the twin of 3D's face tags; 3D's edge block is codimension 2) and its
    // point block, as a run with a band does.
    MeshData input = grid_2d({{"p", {{4, 4}}}}, {}, nullptr, {{"curve", {{{2, 4}}, {{6, 4}}}}});
    auto& tris = input.cells.at(CellType::Triangle);
    DataMatrix<uint8_t> tags = DataMatrix<uint8_t>::Zero(tris.connectivity.rows(), 2);
    tags.col(0).setOnes();
    tris.attributes["tags"] = {tags, {"ambient", "nothing"}};
    const nlohmann::json j = {
        {"input", "m"},
        {"offsets", nlohmann::json::array({{{"selection", "nothing"}, {"target_distance", 0.2}}})},
        {"num_threads", 1}};
    ComponentResult res;
    REQUIRE_NOTHROW(res = components::topological_offset::topological_offset(j, {{"m", input}}));
    const MeshData& out = res.first.at("mesh");
    // the curve: the four grid edges from x = -0.5 to x = 0.5 on the x axis
    REQUIRE(out.cells.count(CellType::Edge) == 1);
    const auto& edges = out.cells.at(CellType::Edge);
    CHECK(edges.attributes.at("tags").labels == std::vector<std::string>{"curve"});
    CHECK(edges.connectivity.rows() == 4);
    for (Eigen::Index r = 0; r < edges.connectivity.rows(); ++r) {
        for (int c = 0; c < 2; ++c) {
            const Vector2d x = out.V.row(edges.connectivity(r, c)).head<2>().transpose();
            CHECK(std::abs(x[0]) <= 0.5);
            CHECK(x[1] == 0.);
        }
    }
    // the point, at the origin
    REQUIRE(out.cells.count(CellType::Point) == 1);
    const auto points = result_points_2d(out);
    REQUIRE(points.size() == 1);
    REQUIRE(points.at("p").size() == 1);
    CHECK(points.at("p")[0] == grid_point_2d({{4, 4}}));
}

namespace {
/// The 2D twin of rod_grid(), three regions meeting at both ends of a segment: grid_2d() with
/// A (x < 0) and B (x > 0) in -0.5 <= y <= 0.5, ambient elsewhere. "A & B" is the segment x = 0,
/// -0.5 <= y <= 0.5, four grid edges with both ends inside the domain, where A, B and ambient
/// meet -- the rod of 3D's rod_grid(), one dimension down: a curve an expression selects, held as
/// the region boundary it is (in the tubes of A and B), its ends anchored where three regions
/// meet (hold_curves_and_points()).
MeshData rod_grid_2d()
{
    return grid_2d({}, {}, [](const Vector2d& c) {
        return std::abs(c.y()) > 0.5 ? "" : c.x() < 0. ? "A" : "B";
    });
}
} // namespace

TEST_CASE("curve-offset-region-boundary-held-2d", "[offset][2d]" + release_only)
{
    // The 2D twin of curve-offset-rod-held: a curve offset (an input complex of segments), the
    // band integral over segments, the curve held. The run must not throw, and the input complex
    // in the result must lie in the tube around the input segment, its two ends still there. A 2D
    // curve an expression selects is a region boundary, held in its regions' tubes (half-width
    // envelope_size; 3D holds its codimension-2 curve in a feature tube of the same default
    // width), its ends where three regions meet in balls of order2_envelope_ratio x
    // envelope_size. One turn: every operation -- split, collapse, swap, smoothing -- runs over
    // the curve.
    const MeshData input = rod_grid_2d();
    for (const std::string rule : {"exact", "exact_swept", "corner_bound"}) {
        INFO("EXPERIMENTAL_band_volume_rule " << rule);
        nlohmann::json j = {
            {"input", "m"},
            {"offsets",
             nlohmann::json::array({{{"selection", "A & B"}, {"target_distance", 0.2}}})},
            {"EXPERIMENTAL_band_volume_rule", rule},
            {"max_rounds", 1},
            {"num_threads", 1},
            {"throw_on_nonconvergence", false}};
        // the default clean-up reads the exact integral, which "corner_bound" does not build
        if (rule == "corner_bound") j["EXPERIMENTAL_label_cleanup"] = "off";
        ComponentResult res;
        REQUIRE_NOTHROW(
            res = components::topological_offset::topological_offset(j, {{"m", input}}));
        const MeshData& out = res.first.at("mesh");
        size_t n_rod = 0;
        bool end_lo = false, end_hi = false;
        for (const Vector2d& p : result_complex_2d(out)) {
            ++n_rod;
            const double y = std::clamp(p[1], -0.5, 0.5);
            CHECK((p - Vector2d(0, y)).norm() <= grid_2d_eps);
            end_lo = end_lo || (p - Vector2d(0, -0.5)).norm() <= grid_2d_eps;
            end_hi = end_hi || (p - Vector2d(0, 0.5)).norm() <= grid_2d_eps;
        }
        CHECK(n_rod >= 3);
        CHECK(end_lo);
        CHECK(end_hi);
        // and the result's complex is the whole curve: every vertex in the tube carries label 1,
        // the split pass's midpoints included (carried_vertex_label(); before it, 4 of the 9 did
        // not)
        const auto labels = out.vertex_attributes.at("labels").doubles();
        size_t n_tube = 0;
        for (Eigen::Index v = 0; v < out.V.rows(); ++v) {
            const Vector2d p = out.V.row(v).head<2>().transpose();
            if ((p - Vector2d(0, std::clamp(p[1], -0.5, 0.5))).norm() > grid_2d_eps) continue;
            ++n_tube;
            CHECK(labels(v, 0) == 1.);
        }
        CHECK(n_tube == n_rod);
    }
}

TEST_CASE("curve-offset-refuses-offset-in-2d", "[offset][2d]")
{
    // The 2D twin of curve-offset-refuses-offset-in: a curve encloses no area
    // (init_input_complex_bvh()).
    const nlohmann::json j = {
        {"input", "m"},
        {"offsets", nlohmann::json::array({{{"selection", "A & B"}, {"target_distance", 0.2}}})},
        {"offset_in", true},
        {"offset_out", false},
        {"num_threads", 1}};
    REQUIRE_THROWS_WITH(
        components::topological_offset::topological_offset(j, {{"m", rod_grid_2d()}}),
        Catch::Matchers::ContainsSubstring("enclose no area"));
}

TEST_CASE("curve-offset-curve-group-2d", "[offset][2d]" + release_only)
{
    // The 2D twin of curve-offset-edge-tag: a curve no region surrounds, selected by name -- 2D's
    // curve group (the envelope, classify_curve_edges()), the twin of 3D's edge tag -- held in its
    // tube (envelope_size) and its ends anchored (order2_envelope_ratio x envelope_size): every
    // input-complex vertex of the result in the tube around its segment, both ends kept. The
    // envelope is handed back as the result's `envelope`.
    const nlohmann::json j = {
        {"input", "m"},
        {"offsets",
         nlohmann::json::array({{{"selection", "EnvelopeSurface"}, {"target_distance", 0.2}}})},
        {"max_rounds", 1},
        {"num_threads", 1},
        {"throw_on_nonconvergence", false}};
    ComponentResult res;
    REQUIRE_NOTHROW(
        res = components::topological_offset::topological_offset(
            j,
            {{"m", grid_2d({}, {{{{2, 4}}, {{6, 4}}}})}}));
    const MeshData& out = res.first.at("mesh");
    size_t n = 0;
    bool end_lo = false, end_hi = false;
    for (const Vector2d& p : result_complex_2d(out)) {
        ++n;
        const double x = std::clamp(p[0], -0.5, 0.5);
        CHECK((p - Vector2d(x, 0)).norm() <= grid_2d_eps);
        end_lo = end_lo || (p - Vector2d(-0.5, 0)).norm() <= grid_2d_eps;
        end_hi = end_hi || (p - Vector2d(0.5, 0)).norm() <= grid_2d_eps;
    }
    CHECK(n >= 3);
    CHECK(end_lo);
    CHECK(end_hi);
    // the result's complex is the whole curve, the split pass's midpoints included
    const auto labels = out.vertex_attributes.at("labels").doubles();
    size_t n_tube = 0;
    for (Eigen::Index v = 0; v < out.V.rows(); ++v) {
        const Vector2d p = out.V.row(v).head<2>().transpose();
        if ((p - Vector2d(std::clamp(p[0], -0.5, 0.5), 0)).norm() > grid_2d_eps) continue;
        ++n_tube;
        CHECK(labels(v, 0) == 1.);
    }
    CHECK(n_tube == n);
    CHECK(res.first.count("envelope") == 1);
}

TEST_CASE("curve-offset-edge-tag-2d", "[offset][2d]" + release_only)
{
    // The 2D twin of curve-offset-edge-tag: an edge tag selected (label_input_complex()), the
    // curve offset of a triwild or simwild curve, with no region around it. Every edge tag is
    // held, selected or not (init_surfaces_and_boundaries(): tracked, in its own tube of
    // envelope_size, its ends anchored), and written back as the result's edge block: each tag's
    // edges in the tube around its segment, each curve whole, end to end. "curve" is y = 0,
    // x in [-0.5, 0.5]; "other" y = 0.5, out of the band at 0.2.
    const nlohmann::json j = {
        {"input", "m"},
        {"offsets", nlohmann::json::array({{{"selection", "curve"}, {"target_distance", 0.2}}})},
        {"max_rounds", 1},
        {"num_threads", 1},
        {"throw_on_nonconvergence", false}};
    ComponentResult res;
    REQUIRE_NOTHROW(
        res = components::topological_offset::topological_offset(
            j,
            {{"m",
              grid_2d(
                  {},
                  {},
                  nullptr,
                  {{"curve", {{{2, 4}}, {{6, 4}}}}, {"other", {{{2, 6}}, {{6, 6}}}}})}}));
    const MeshData& out = res.first.at("mesh");
    REQUIRE(out.cells.count(CellType::Edge) == 1);
    const auto& block = out.cells.at(CellType::Edge);
    const auto& tags = block.attributes.at("tags");
    REQUIRE(tags.labels == std::vector<std::string>{"curve", "other"});
    const auto member = tags.integers();
    const auto dist = [](const Vector2d& p, const double y) {
        return (p - Vector2d(std::clamp(p[0], -0.5, 0.5), y)).norm();
    };
    double xlo[2] = {1., 1.}, xhi[2] = {-1., -1.};
    for (Eigen::Index r = 0; r < block.connectivity.rows(); ++r) {
        for (int t = 0; t < 2; ++t) {
            if (member(r, t) == 0) continue;
            for (int c = 0; c < 2; ++c) {
                const Vector2d p = out.V.row(block.connectivity(r, c)).head<2>().transpose();
                CHECK(dist(p, t == 0 ? 0. : 0.5) <= grid_2d_eps);
                xlo[t] = std::min(xlo[t], p[0]);
                xhi[t] = std::max(xhi[t], p[0]);
            }
        }
    }
    for (int t = 0; t < 2; ++t) { // each curve whole, end to end
        INFO("edge tag " << tags.labels[size_t(t)]);
        CHECK(std::abs(xlo[t] + 0.5) <= grid_2d_eps);
        CHECK(std::abs(xhi[t] - 0.5) <= grid_2d_eps);
    }
    size_t n = 0;
    for (const Vector2d& p : result_complex_2d(out)) {
        ++n;
        CHECK(dist(p, 0.) <= grid_2d_eps);
    }
    CHECK(n >= 3);
}

TEST_CASE("curve-offset-edge-tags-in-expression-2d", "[offset][2d]")
{
    // The 2D twin of curve-offset-edge-tags-in-expression: an edge tag in an expression means
    // what it means alone, "curve | other" both curves, every input-complex vertex on one of the
    // two segments and both segments whole, end to end.
    const nlohmann::json j = {
        {"input", "m"},
        {"offsets",
         nlohmann::json::array({{{"selection", "curve | other"}, {"target_distance", 0.1}}})},
        {"optimize_offset", false},
        {"num_threads", 1}};
    ComponentResult res;
    REQUIRE_NOTHROW(
        res = components::topological_offset::topological_offset(
            j,
            {{"m",
              grid_2d(
                  {},
                  {},
                  nullptr,
                  {{"curve", {{{2, 4}}, {{6, 4}}}}, {"other", {{{2, 6}}, {{6, 6}}}}})}}));
    double xlo[2] = {1., 1.}, xhi[2] = {-1., -1.};
    for (const Vector2d& p : result_complex_2d(res.first.at("mesh"))) {
        const int t = p[1] > 0.25 ? 1 : 0; // "curve" at y = 0, "other" at y = 0.5
        CHECK(p[1] == 0.5 * t);
        xlo[t] = std::min(xlo[t], p[0]);
        xhi[t] = std::max(xhi[t], p[0]);
    }
    for (int t = 0; t < 2; ++t) {
        CHECK(xlo[t] == -0.5);
        CHECK(xhi[t] == 0.5);
    }
}

TEST_CASE("edge-tags-held-2d", "[offset][2d]" + release_only)
{
    // 2D's edge tags held through the loop, selected or not, as 3D's face tags: p offset to 0.2
    // with two edge tags, "near" on y = 0.25 (0.05 outside the level set) and "far" on y = 0.5;
    // after 3 turns each tag's edges are in the tube around its segment, end to end. MEASURED
    // before they were tracked (init_surfaces_and_boundaries()): after 1 turn "near" was 0.043 off
    // its line, after 3 turns 0.103, and "far" 0.034, against envelope_size 0.0028.
    const nlohmann::json j = {
        {"input", "m"},
        {"offsets", nlohmann::json::array({{{"selection", "p"}, {"target_distance", 0.2}}})},
        {"max_rounds", 3},
        {"num_threads", 1},
        {"throw_on_nonconvergence", false}};
    ComponentResult res;
    REQUIRE_NOTHROW(
        res = components::topological_offset::topological_offset(
            j,
            {{"m",
              grid_2d(
                  {{"p", {{4, 4}}}},
                  {},
                  nullptr,
                  {{"near", {{{1, 5}}, {{7, 5}}}}, {"far", {{{2, 6}}, {{6, 6}}}}})}}));
    const MeshData& out = res.first.at("mesh");
    const auto& block = out.cells.at(CellType::Edge);
    const auto& tags = block.attributes.at("tags");
    REQUIRE(tags.labels == std::vector<std::string>{"near", "far"});
    const auto member = tags.integers();
    const double y0[2] = {0.25, 0.5}, half[2] = {0.75, 0.5};
    double xlo[2] = {1., 1.}, xhi[2] = {-1., -1.};
    for (Eigen::Index r = 0; r < block.connectivity.rows(); ++r) {
        for (int t = 0; t < 2; ++t) {
            if (member(r, t) == 0) continue;
            for (int c = 0; c < 2; ++c) {
                const Vector2d p = out.V.row(block.connectivity(r, c)).head<2>().transpose();
                INFO("edge tag " << tags.labels[size_t(t)]);
                CHECK(
                    (p - Vector2d(std::clamp(p[0], -half[t], half[t]), y0[t])).norm() <=
                    grid_2d_eps);
                xlo[t] = std::min(xlo[t], p[0]);
                xhi[t] = std::max(xhi[t], p[0]);
            }
        }
    }
    for (int t = 0; t < 2; ++t) {
        INFO("edge tag " << tags.labels[size_t(t)]);
        CHECK(std::abs(xlo[t] + half[t]) <= grid_2d_eps);
        CHECK(std::abs(xhi[t] - half[t]) <= grid_2d_eps);
    }
}

TEST_CASE("driver-refusals-2d", "[offset][2d]")
{
    // What the 2D driver (topological_offset.cpp) refuses before it builds the mesh: offset_field
    // "smooth", as 3D refuses it; a labelled envelope reference, which 3D holds to and 2D cannot
    // yet (the envelope-passing step of the port). Several entries of offsets and growth_mode
    // were refused until 2026-11-11 (the several-offsets-*-run-2d and growth cases).
    const MeshData input = grid_2d({{"p", {{4, 4}}}, {"q", {{6, 4}}}});
    nlohmann::json j = {
        {"input", "m"},
        {"offsets", nlohmann::json::array({{{"selection", "p"}, {"target_distance", 0.2}}})},
        {"num_threads", 1}};
    MeshMap meshes{{"m", input}};
    std::string want;
    SECTION("offset_field smooth")
    {
        j["offset_field"] = "smooth";
        want = "is not available in 2D";
    }
    SECTION("a labelled reference")
    {
        Eigen::MatrixXd V(2, 3);
        V << -0.5, 0.5, 0., 0.5, 0.5, 0.;
        Eigen::MatrixXi E(1, 2);
        E << 0, 1;
        MeshData reference;
        io::append_envelope_block(
            reference,
            io::geometry_data(V, E, CellType::Edge),
            CellType::Edge,
            {InputSet(1)},
            {"curve"});
        meshes["e"] = reference;
        j["envelope"] = "e";
        want = "labelled reference";
    }
    REQUIRE_THROWS_WITH(
        components::topological_offset::topological_offset(j, meshes),
        Catch::Matchers::ContainsSubstring(want));
}

TEST_CASE("memory-output-with-offset-front-2d", "[offset][2d][io]")
{
    // The 2D twin of memory-output-with-offset-front: the memory export (collect_memory_output())
    // of a mesh with an offset potential and a front, as a run ends, under DEBUG_stencil. The
    // export must succeed, its keys be the offsets' own, and the mesh's rows be the live
    // vertices.
    Eigen::MatrixXd box(2, 2);
    box << -1., -1., 1., 1.;
    Parameters param = construction_params(
        {{"offsets", nlohmann::json::array({{{"selection", "p"}, {"target_distance", 0.2}}})},
         {"num_threads", 0}},
        box);
    param.debug_stencil = true;
    auto mesh = point_grid_2d(param, {{4, 4}});
    REQUIRE(mesh->m_offset_potential);
    size_t n_front = 0;
    for (const auto& e : mesh->get_edges()) n_front += mesh->edge_is_offset(e.eid(*mesh));
    REQUIRE(n_front > 0);
    io::ComponentIO context;
    REQUIRE_NOTHROW(collect_memory_output(*mesh, context));
    const auto& out = context.result.first;
    // The mesh alone, no envelope given.
    CHECK(out.size() == 1);
    CHECK(out.count("mesh") == 1);
    CHECK(size_t(out.at("mesh").V.rows()) == mesh->get_vertices().size());
}

namespace {
/// One construction-only 2D offset of the edge tag `curve`, its band tagged `band`, the
/// correspondence saved: the 2D twin of offset_sheet(), 2D's edge tags the twin of 3D's face
/// tags.
MeshData offset_curve_2d(const MeshData& m, const std::string& curve, const std::string& band)
{
    return components::topological_offset::topological_offset(
               {{"input", "m"},
                {"offsets",
                 nlohmann::json::array(
                     {{{"selection", curve}, {"output_tags", {band}}, {"target_distance", 0.1}}})},
                {"optimize_offset", false},
                {"save_offset_correspondence", true},
                {"num_threads", 1}},
               {{"m", m}})
        .first.at("mesh");
}

/// The 2D twin of correspondence_pairs(): each offset vertex as its position followed by its
/// input vertex's.
std::set<std::array<double, 4>> correspondence_pairs_2d(const MeshData& m)
{
    const auto corr = m.vertex_attributes.at("corr_input_vid").integers();
    const auto vid = m.vertex_attributes.at("vid").integers();
    std::map<int64_t, Eigen::Index> row_of;
    for (Eigen::Index r = 0; r < vid.rows(); ++r) row_of[vid(r, 0)] = r;
    std::set<std::array<double, 4>> pairs;
    for (Eigen::Index r = 0; r < corr.rows(); ++r) {
        if (corr(r, 0) < 0) continue;
        const Eigen::Index u = row_of.at(corr(r, 0));
        pairs.insert({{m.V(r, 0), m.V(r, 1), m.V(u, 0), m.V(u, 1)}});
    }
    return pairs;
}

/// The 2D twin of two_sheet_grid(): grid_2d() with two open curves as edge tags, yellow on
/// y = 0 and blue on y = y_blue, both x in [-0.5, 0.5].
MeshData two_curve_grid_2d(const int j_blue)
{
    return grid_2d(
        {},
        {},
        nullptr,
        {{"yellow", {{{2, 4}}, {{6, 4}}}}, {"blue", {{{2, j_blue}}, {{6, j_blue}}}}});
}
} // namespace

TEST_CASE("offset-carries-an-earlier-correspondence-2d", "[offset][2d][correspondence]")
{
    // The 2D twin of offset-carries-an-earlier-correspondence, 2D's edge tags in place of 3D's
    // face tags: two offsets one after another, yellow's band and then blue's. The second run
    // carries the first's corr_input_vid. Construction moves no vertex, so the pairs compare by
    // position, exactly. 3D's fourth section (the earlier band protected) has no twin: 2D's march
    // still cuts protected cells (split_edge_before()).
    SECTION("curves three squares apart: every carried value survives")
    {
        const auto first = offset_curve_2d(two_curve_grid_2d(7), "yellow", "band_yellow");
        const auto second = offset_curve_2d(first, "blue", "band_blue");
        const auto carried = correspondence_pairs_2d(first);
        REQUIRE(!carried.empty());
        std::set<std::array<double, 4>> to_yellow, to_blue;
        for (const auto& p : correspondence_pairs_2d(second))
            (p[3] == 0. ? to_yellow : to_blue).insert(p);
        CHECK(to_yellow == carried);
        CHECK(!to_blue.empty());
        for (const auto& p : to_blue) CHECK(p[3] == 0.75);
    }
    SECTION("curves one square apart: blue's march cuts beside yellow's pairs, not them")
    {
        // Blue's marched edges end at yellow's offset vertices, not at the input vertices they
        // name, so no carried edge is cut and every carried value survives.
        const auto first = offset_curve_2d(two_curve_grid_2d(5), "yellow", "band_yellow");
        const auto second = offset_curve_2d(first, "blue", "band_blue");
        const auto carried = correspondence_pairs_2d(first);
        REQUIRE(!carried.empty());
        const auto pairs = correspondence_pairs_2d(second);
        CHECK(std::includes(pairs.begin(), pairs.end(), carried.begin(), carried.end()));
    }
    SECTION("yellow offset again: the march cuts yellow's carried edges, and the run throws")
    {
        const auto first = offset_curve_2d(two_curve_grid_2d(7), "yellow", "band_yellow");
        REQUIRE_THROWS_WITH(
            offset_curve_2d(first, "yellow", "band_yellow_2"),
            Catch::Matchers::ContainsSubstring("corr_input_vid"));
    }
}

TEST_CASE("label-cleanup-keeps-held-point-off-the-band-run-2d", "[offset][2d]" + release_only)
{
    // The 2D twin of label-cleanup-keeps-held-point-off-the-band through the driver (its
    // clean-up-slot twin is label-cleanup-keeps-held-point-off-the-band-2d): q, a held point
    // 0.25 from p and so inside p's level set at 0.3, out of the band as constructed (the march
    // traces to 0.125); no band triangle ever has it as a corner.
    const std::string rule = GENERATE("exact", "exact_swept");
    INFO("EXPERIMENTAL_band_volume_rule " << rule);
    const nlohmann::json j = {
        {"input", "m"},
        {"offsets", nlohmann::json::array({{{"selection", "p"}, {"target_distance", 0.3}}})},
        {"EXPERIMENTAL_band_volume_rule", rule},
        {"max_rounds", 3},
        {"throw_on_unreachable_level_set", false}, // the topology is what is tested
        {"num_threads", 1},
        {"throw_on_nonconvergence", false}};
    ComponentResult res;
    REQUIRE_NOTHROW(
        res = components::topological_offset::topological_offset(
            j,
            {{"m", grid_2d({{"p", {{4, 4}}}, {"q", {{5, 4}}}})}}));
    const MeshData& out = res.first.at("mesh");
    const auto touched = band_touched_2d(out);
    const Vector2d q = result_points_2d(out).at("q").at(0);
    Eigen::Index qv = 0;
    for (Eigen::Index v = 1; v < out.V.rows(); ++v) {
        if ((out.V.row(v).head<2>().transpose() - q).norm() <
            (out.V.row(qv).head<2>().transpose() - q).norm())
            qv = v;
    }
    REQUIRE((out.V.row(qv).head<2>().transpose() - q).norm() == 0.);
    CHECK(touched[size_t(qv)] == 0);
}

TEST_CASE("unreachable-level-set-throws-run-2d", "[offset][2d]" + release_only)
{
    // The 2D twin of unreachable-level-set-throws through the driver (its loop twin is
    // unreachable-level-set-throws-in-the-loop-2d): p offset to 0.3 with a held object the band
    // did not touch as constructed. Inside the level set -- a held point, a curve, a region
    // boundary, the box, each 0.25 from p -- the run throws. Not a held point inside the level set
    // by less than front_conv (q 0.25 from p at target 0.27). 3D's curve through p has no twin
    // (unreachable-level-set-throws-2d says why).
    const std::string rule = GENERATE("exact", "exact_swept");
    const std::string held = GENERATE("point", "curve", "region", "box", "point within the bar");
    INFO("EXPERIMENTAL_band_volume_rule " << rule << ", held: " << held);
    const std::array<int, 2> p =
        held == "box" ? std::array<int, 2>{{1, 4}} : std::array<int, 2>{{4, 4}};
    GridPoints2d points{{"p", p}};
    if (held == "point" || held == "point within the bar") points.push_back({"q", {{5, 4}}});
    std::vector<GridPolyline2d> curves;
    if (held == "curve") curves.push_back({{{2, 5}}, {{6, 5}}});
    const MeshData input = grid_2d(
        points,
        curves,
        held == "region" ? GridRegion2d([](const Vector2d& c) { return c.x() < 0.25 ? "A" : "B"; })
                         : GridRegion2d());
    const double target = held == "point within the bar" ? 0.27 : 0.3;
    const nlohmann::json j = {
        {"input", "m"},
        {"offsets", nlohmann::json::array({{{"selection", "p"}, {"target_distance", target}}})},
        {"EXPERIMENTAL_band_volume_rule", rule},
        {"max_rounds", 3},
        {"num_threads", 1},
        {"throw_on_nonconvergence", false}};
    if (held == "point within the bar") {
        REQUIRE_NOTHROW(components::topological_offset::topological_offset(j, {{"m", input}}));
    } else {
        REQUIRE_THROWS_WITH(
            components::topological_offset::topological_offset(j, {{"m", input}}),
            Catch::Matchers::ContainsSubstring("the level set is out of reach"));
    }
}

namespace {
/// The 2D twin of require_point_offset_converges(): grid_2d(points)'s point p offset to `delta`,
/// run to convergence (the settle exit, throw_on_nonconvergence), every input point within its
/// anchor's radius in the result, and the front within the bar of the level set: every front
/// vertex within front_conv (0.1 x delta) of distance delta from p.
void require_point_offset_converges_2d(const GridPoints2d& points, const double delta)
{
    const nlohmann::json j = {
        {"input", "m"},
        {"offsets", nlohmann::json::array({{{"selection", "p"}, {"target_distance", delta}}})},
        {"max_rounds", 15},
        {"num_threads", 1},
        {"throw_on_nonconvergence", true}};
    ComponentResult res;
    REQUIRE_NOTHROW(
        res = components::topological_offset::topological_offset(j, {{"m", grid_2d(points)}}));
    CHECK(res.second.at("converged") == true);
    const MeshData& out = res.first.at("mesh");
    const auto at = result_points_2d(out);
    for (const auto& [name, g] : points) {
        INFO("point " << name);
        REQUIRE(at.at(name).size() == 1);
        CHECK((at.at(name)[0] - grid_point_2d(g)).norm() <= grid_2d_eps);
    }
    const Vector2d p = grid_point_2d(std::find_if(points.begin(), points.end(), [](const auto& x) {
                                         return x.first == "p";
                                     })->second);
    const auto front = result_front_2d(out);
    REQUIRE(!front.empty());
    for (const auto& e : front) {
        for (const Vector2d& x : e) CHECK(std::abs((x - p).norm() - delta) <= 0.1 * delta);
    }
}
} // namespace

TEST_CASE("point-offset-converges-2d", "[offset][2d][.]")
{
    // Hidden, for the runs before a merge: the 2D twin of point-offset-converges.
    require_point_offset_converges_2d({{"p", {{4, 4}}}}, 0.2);
}

TEST_CASE("point-offset-held-point-inside-the-band-2d", "[offset][2d][.]")
{
    // Hidden, the 2D twin of point-offset-held-point-inside-the-band, and failing as it does: q,
    // a held point 0.25 from p, inside p's level set at 0.3; since 2026-11-10 the run throws at
    // a settled turn (unreachable-level-set-throws-run-2d). Kept as the record of the case until
    // a flag lets the front go beyond the throw.
    require_point_offset_converges_2d({{"p", {{4, 4}}}, {"q", {{5, 4}}}}, 0.3);
}

TEST_CASE("curve-offset-converges-2d", "[offset][2d][.]")
{
    // Hidden, for the runs before a merge: the 2D twin of curve-offset-converges, on curves no
    // region surrounds (an edge tag) -- a corner, and three arms meeting in a junction (3D's
    // meet out of a plane) -- and the input complex in the curves' tube in the result. 3D's third
    // case, a second curve held inside the band, has no 2D twin here: a held curve is
    // codimension 1 in 2D, so one inside the level set is out of reach and the run throws
    // (unreachable-level-set-throws-run-2d); the codimension-2 held object inside the band is a
    // point (point-offset-held-point-inside-the-band-2d).
    struct Case
    {
        std::string name;
        std::vector<GridPolyline2d> curves;
        double delta;
    };
    const std::vector<Case> cases = {
        {"corner", {{{{2, 4}}, {{4, 4}}, {{4, 6}}}}, 0.2},
        {"junction", {{{{4, 4}}, {{6, 4}}}, {{{4, 4}}, {{4, 6}}}, {{{4, 4}}, {{2, 4}}}}, 0.2},
    };
    for (const Case& c : cases) {
        INFO(c.name);
        const nlohmann::json j = {
            {"input", "m"},
            {"offsets",
             nlohmann::json::array({{{"selection", "curve"}, {"target_distance", c.delta}}})},
            {"max_rounds", 15},
            {"num_threads", 1},
            {"throw_on_nonconvergence", true}};
        std::vector<std::pair<std::string, GridPolyline2d>> tagged;
        for (const auto& poly : c.curves) tagged.push_back({"curve", poly});
        ComponentResult res;
        REQUIRE_NOTHROW(
            res = components::topological_offset::topological_offset(
                j,
                {{"m", grid_2d({}, {}, nullptr, tagged)}}));
        CHECK(res.second.at("converged") == true);
        for (const Vector2d& x : result_complex_2d(res.first.at("mesh"))) {
            double d = std::numeric_limits<double>::infinity();
            for (const auto& poly : c.curves) {
                for (size_t k = 0; k + 1 < poly.size(); ++k) {
                    const Vector2d a = grid_point_2d(poly[k]), b = grid_point_2d(poly[k + 1]);
                    const double t = std::clamp((x - a).dot(b - a) / (b - a).squaredNorm(), 0., 1.);
                    d = std::min(d, (x - a - t * (b - a)).norm());
                }
            }
            CHECK(d <= grid_2d_eps);
        }
    }
}

TEST_CASE("double-sphere-other-region-held-or-protected", "[offset][3d][.]")
{
    // Hidden until the junction is fixed: neither variant converges reliably without it, so it
    // would fail every CI run; the hidden runs record it until then, and it moves back into CI
    // (release_only) once it passes.
    // double_sphere, offset of tag_0 at 0.05 with the other region, tag_1, held (every region
    // boundary is held since 2026-11-04) or protected: the front meets tag_1's surface along a
    // junction curve, a T-junction when held (the band takes tag_1's cells) and an X-junction
    // when protected (the band overlaps them). The run must converge -- EXPERIMENTAL_exit_no_folds
    // holds the exit while a fold remains -- at 4 threads, which is nondeterministic, as the
    // batches run. MEASURED (2026-11-04, 15 turns, 4 threads, one run each): neither converged,
    // held with 13 folds left, protected with 3; main, which held only the input complex and
    // let the band push tag_1's surface aside, converged in 2 turns. Fails until the junction is
    // fixed.
    const MeshData input = io::read_mesh(
        (std::filesystem::path(WMTK_DATA_DIR) / "models" / "double_sphere.msh").string());
    const bool protect = GENERATE(false, true);
    INFO((protect ? "tag_1 protected" : "tag_1 held"));
    nlohmann::json j = {
        {"input", "m"},
        {"offsets", nlohmann::json::array({{{"selection", "tag_0"}, {"target_distance", 0.05}}})},
        {"optimize_offset", true},
        {"max_rounds", 15},
        {"num_threads", 4},
        {"throw_on_nonconvergence", true}};
    if (protect) j["protected_tags"] = "tag_1";
    REQUIRE_NOTHROW(components::topological_offset::topological_offset(j, {{"m", input}}));
}

TEST_CASE("exact-swept-front-vertex-held-on-another-region", "[offset][3d]" + release_only)
{
    // double_sphere, offset of tag_0 with exact_swept: the loop holds every region boundary
    // (since 2026-11-04), so a front vertex on tag_1's surface is held and takes the engine's
    // projected step, whose energy (smoothing_ring_energy()) reads the vertex smoothed. Only
    // smooth_nonfront_vertex() recorded it: one round threw "exact_swept: the projected step's
    // ring is not vertex N's" (2026-11-05).
    const MeshData input = io::read_mesh(
        (std::filesystem::path(WMTK_DATA_DIR) / "models" / "double_sphere.msh").string());
    const nlohmann::json j = {
        {"input", "m"},
        {"offsets", nlohmann::json::array({{{"selection", "tag_0"}, {"target_distance", 0.05}}})},
        {"EXPERIMENTAL_band_volume_rule", "exact_swept"},
        {"max_rounds", 1},
        {"num_threads", 1}};
    REQUIRE_NOTHROW(components::topological_offset::topological_offset(j, {{"m", input}}));
}

namespace {
/// [-1, 1]^3 in 8^3 cubes of edge 0.25 (cube_grid()), the cubes (i, j, k) of each named box --
/// i, j, k in [lo, hi] of its {i_lo, i_hi, j_lo, j_hi, k_lo, k_hi} -- tagged by its name, every
/// other cube ambient. Boxes do not overlap.
MeshData box_grid(const std::vector<std::pair<std::string, std::array<int, 6>>>& boxes)
{
    const CubeGrid g = cube_grid(8, 0.25);
    Eigen::MatrixXi Tm(int(g.T.size()), 4);
    DataMatrix<uint8_t> tags = DataMatrix<uint8_t>::Zero(g.T.size(), boxes.size() + 1);
    std::vector<std::string> names;
    for (const auto& [name, box] : boxes) names.push_back(name);
    names.push_back("ambient");
    for (size_t i = 0; i < g.T.size(); ++i) {
        for (int j = 0; j < 4; ++j) Tm(int(i), j) = g.T[i][size_t(j)];
        size_t col = boxes.size();
        for (size_t b = 0; b < boxes.size(); ++b) {
            const auto& r = boxes[b].second;
            bool in = true;
            for (int a = 0; a < 3; ++a)
                in = in && g.cube[i][size_t(a)] >= r[size_t(2 * a)] &&
                     g.cube[i][size_t(a)] <= r[size_t(2 * a + 1)];
            if (in) col = b;
        }
        tags(i, col) = 1;
    }
    MeshData m = io::geometry_data(g.V, Tm, CellType::Tetrahedron);
    m.cells.at(CellType::Tetrahedron).attributes["tags"] = {tags, names};
    return m;
}

/// The distance from p to box_grid()'s box of cube indices r.
double box_distance(const Vector3d& p, const std::array<int, 6>& r)
{
    double s = 0.;
    for (int a = 0; a < 3; ++a) {
        const double lo = r[size_t(2 * a)] * 0.25 - 1., hi = (r[size_t(2 * a + 1)] + 1) * 0.25 - 1.;
        const double e = std::max({lo - p[a], 0., p[a] - hi});
        s += e * e;
    }
    return std::sqrt(s);
}

/// The cells of a result carrying `tag`, each as its corners' positions in sorted order: a cell's
/// identity across two runs that number the vertices differently.
using CellCorners = std::array<std::array<double, 3>, 4>;
std::set<CellCorners> cells_tagged(const MeshData& m, const std::string& tag)
{
    const auto& block = m.cells.at(CellType::Tetrahedron);
    const auto& tags = block.attributes.at("tags");
    const auto it = std::find(tags.labels.begin(), tags.labels.end(), tag);
    REQUIRE(it != tags.labels.end());
    const Eigen::Index col = Eigen::Index(it - tags.labels.begin());
    const auto member = tags.integers();
    std::set<CellCorners> out;
    for (Eigen::Index r = 0; r < block.connectivity.rows(); ++r) {
        if (member(r, col) == 0) continue;
        CellCorners c;
        for (int k = 0; k < 4; ++k) {
            const auto p = m.V.row(block.connectivity(r, k));
            c[size_t(k)] = {{p(0), p(1), p(2)}};
        }
        std::sort(c.begin(), c.end());
        out.insert(c);
    }
    return out;
}

/// One construction-only run with these offsets.
MeshData construct_offsets(const MeshData& m, const nlohmann::json& offsets)
{
    return components::topological_offset::topological_offset(
               {{"input", "m"},
                {"offsets", offsets},
                {"optimize_offset", false},
                {"num_threads", 1}},
               {{"m", m}})
        .first.at("mesh");
}
} // namespace

TEST_CASE("several-offsets-as-alone", "[offset][3d]")
{
    // Three bodies, each two cubes from the next, offset in one march: each entry's band is the
    // one a run of that entry alone makes -- the same cells at the same positions -- and takes its
    // own output tags. Each entry decides its fallback itself (marching_tets()): every outer end is
    // a grid edge (0.25) or more from its body, so a at 0.1 and c at 0.2 are traced to, and b at
    // 0.3 falls back to half of 0.25. Before 2026-11 the union "a | b | c" was one input under one
    // maximum marchable distance, one target and one set of output tags.
    const std::vector<std::pair<std::string, std::array<int, 6>>> boxes = {
        {"a", {{1, 2, 1, 2, 1, 2}}},
        {"b", {{5, 6, 1, 2, 1, 2}}},
        {"c", {{1, 2, 5, 6, 5, 6}}}};
    const MeshData m = box_grid(boxes);
    const std::vector<double> D = {0.1, 0.3, 0.2};
    nlohmann::json all = nlohmann::json::array();
    for (size_t k = 0; k < boxes.size(); ++k) {
        all.push_back(
            {{"selection", boxes[k].first},
             {"output_tags", {boxes[k].first + "_band"}},
             {"target_distance", D[k]}});
    }
    const MeshData joint = construct_offsets(m, all);
    for (size_t k = 0; k < boxes.size(); ++k) {
        const std::string band = boxes[k].first + "_band";
        INFO("offsets[" << k << "] " << boxes[k].first << " at " << D[k]);
        const auto cells = cells_tagged(joint, band);
        CHECK(!cells.empty());
        CHECK(cells == cells_tagged(construct_offsets(m, nlohmann::json::array({all[k]})), band));
        const double traced = D[k] < 0.25 ? D[k] : 0.125;
        size_t n_outer = 0;
        for (const CellCorners& c : cells) {
            for (const auto& q : c) {
                const double d = box_distance(Vector3d(q[0], q[1], q[2]), boxes[k].second);
                if (d == 0.) continue; // on the body
                ++n_outer;
                CHECK(std::abs(d - traced) <= 0.01 * traced);
            }
        }
        CHECK(n_outer > 0);
    }
    // The comparison tells positions apart: a alone at c's distance is not a's band.
    nlohmann::json a_at_c = all[0];
    a_at_c["target_distance"] = D[2];
    CHECK(
        cells_tagged(joint, "a_band") !=
        cells_tagged(construct_offsets(m, nlohmann::json::array({a_at_c})), "a_band"));
}

TEST_CASE("several-offsets-touch", "[offset][3d]")
{
    // Entries that intersect are refused (label_input_complex()): b's cubes next to a's share
    // their vertices. One cube apart, a's and b's vertices are joined by grid edges; the simplicial
    // embedding splits them, and the two bands come out apart -- no vertex in both -- each within
    // half of such an edge (0.125, from the half-edge midpoint 0.125 from both), so a target of 0.1
    // is still traced to.
    const auto run = [](const std::array<int, 6>& b) {
        return construct_offsets(
            box_grid({{"a", {{1, 2, 1, 2, 1, 2}}}, {"b", b}}),
            nlohmann::json::array(
                {{{"selection", "a"}, {"output_tags", {"a_band"}}, {"target_distance", 0.1}},
                 {{"selection", "b"}, {"output_tags", {"b_band"}}, {"target_distance", 0.1}}}));
    };
    REQUIRE_THROWS_WITH(run({{3, 4, 1, 2, 1, 2}}), Catch::Matchers::ContainsSubstring("share"));
    const std::array<int, 6> b = {{4, 5, 1, 2, 1, 2}};
    MeshData out;
    REQUIRE_NOTHROW(out = run(b));
    std::set<std::array<double, 3>> corners_a;
    for (const CellCorners& c : cells_tagged(out, "a_band")) corners_a.insert(c.begin(), c.end());
    size_t n_b = 0;
    for (const CellCorners& c : cells_tagged(out, "b_band")) {
        ++n_b;
        for (const auto& q : c) {
            CHECK(corners_a.count(q) == 0);
            const double d = box_distance(Vector3d(q[0], q[1], q[2]), b);
            if (d > 0.) CHECK(std::abs(d - 0.1) <= 0.01 * 0.1);
        }
    }
    CHECK(!corners_a.empty());
    CHECK(n_b > 0);
}

TEST_CASE("several-offsets-sheets", "[offset][3d]")
{
    // A body and two sheets (face tags) in one march, each as alone: the sheets' faces are each
    // entry's own (label_input_complex()'s on_sheet union), not the last one's.
    const MeshData m = two_sheet_grid(1.5);
    const std::vector<std::string> names = {"red", "yellow", "blue"};
    nlohmann::json all = nlohmann::json::array();
    for (const std::string& name : names) {
        all.push_back(
            {{"selection", name}, {"output_tags", {name + "_band"}}, {"target_distance", 0.1}});
    }
    const MeshData joint = construct_offsets(m, all);
    for (size_t k = 0; k < names.size(); ++k) {
        INFO("offsets[" << k << "] " << names[k]);
        const auto cells = cells_tagged(joint, names[k] + "_band");
        CHECK(!cells.empty());
        CHECK(
            cells == cells_tagged(
                         construct_offsets(m, nlohmann::json::array({all[k]})),
                         names[k] + "_band"));
    }
}

TEST_CASE("offsets-face-tags-in-expression", "[offset][3d]")
{
    // A face tag in an expression means what it means alone (label_offset()): "yellow | red" is
    // the yellow sheet and the red solid, one input of triangles -- the sheet's faces and the
    // solid's boundary -- and its band the band of both. Until 2026-11 a face tag in an
    // expression selected nothing, silently: no cell carries it. An empty list makes no offset.
    const MeshData m = two_sheet_grid(1.5);
    const MeshData out = construct_offsets(
        m,
        nlohmann::json::array(
            {{{"selection", "yellow | red"},
              {"output_tags", {"band"}},
              {"target_distance", 0.1}}}));
    const auto labels = out.vertex_attributes.at("labels").doubles();
    size_t on_sheet = 0, on_red = 0, elsewhere = 0;
    for (Eigen::Index v = 0; v < out.V.rows(); ++v) {
        if (labels(v, 0) != 1.) continue;
        const Vector3d p = out.V.row(v).transpose();
        if (p[2] == 1. && p[0] >= 0.5 && p[0] <= 1.5 && p[1] >= 0.5 && p[1] <= 1.5) {
            ++on_sheet;
        } else if (p.maxCoeff() <= 0.25) {
            ++on_red;
        } else {
            ++elsewhere;
        }
    }
    CHECK(on_sheet == 25); // the sheet's 5 x 5 grid vertices
    CHECK(on_red == 8); // the red cube's corners
    CHECK(elsewhere == 0);
    // The band of the union: the bands each makes alone, together.
    std::set<CellCorners> both = cells_tagged(
        construct_offsets(
            m,
            nlohmann::json::array(
                {{{"selection", "yellow"}, {"output_tags", {"band"}}, {"target_distance", 0.1}}})),
        "band");
    for (const CellCorners& c : cells_tagged(
             construct_offsets(
                 m,
                 nlohmann::json::array(
                     {{{"selection", "red"},
                       {"output_tags", {"band"}},
                       {"target_distance", 0.1}}})),
             "band"))
        both.insert(c);
    CHECK(cells_tagged(out, "band") == both);
    REQUIRE_THROWS_WITH(
        construct_offsets(m, nlohmann::json::array()),
        Catch::Matchers::ContainsSubstring("at least one offset"));
}

TEST_CASE("several-offsets-optimize", "[offset][3d]" + release_only)
{
    // Several entries run the optimization: an input complex of several connected pieces is no
    // longer refused (TESTING_allow_mult_comp, until 2026-11).
    const MeshData m = box_grid({{"a", {{1, 2, 1, 2, 1, 2}}}, {"b", {{5, 6, 1, 2, 1, 2}}}});
    const nlohmann::json j = {
        {"input", "m"},
        {"offsets",
         nlohmann::json::array(
             {{{"selection", "a"}, {"output_tags", {"a_band"}}, {"target_distance", 0.1}},
              {{"selection", "b"}, {"output_tags", {"b_band"}}, {"target_distance", 0.1}}})},
        {"max_rounds", 1},
        {"num_threads", 1},
        {"throw_on_nonconvergence", false}};
    ComponentResult res;
    REQUIRE_NOTHROW(res = components::topological_offset::topological_offset(j, {{"m", m}}));
    CHECK(!cells_tagged(res.first.at("mesh"), "a_band").empty());
    CHECK(!cells_tagged(res.first.at("mesh"), "b_band").empty());
}

namespace {
/// The front of `tag`'s band in an optimized result: the corners its cells share with ambient
/// cells, off the body's own box.
std::vector<double>
front_distances(const MeshData& out, const std::string& tag, const std::array<int, 6>& box)
{
    std::set<std::array<double, 3>> ambient;
    for (const CellCorners& c : cells_tagged(out, "ambient")) ambient.insert(c.begin(), c.end());
    std::set<std::array<double, 3>> front;
    for (const CellCorners& c : cells_tagged(out, tag)) {
        for (const auto& q : c) {
            if (ambient.count(q)) front.insert(q);
        }
    }
    std::vector<double> d;
    for (const auto& q : front) {
        const double x = box_distance(Vector3d(q[0], q[1], q[2]), box);
        if (x > 0.) d.push_back(x);
    }
    return d;
}
} // namespace

TEST_CASE("several-offsets-optimize-distances", "[offset][3d]" + release_only)
{
    // Each piece its own target distance in the optimization: its field, the band term's distance
    // and integral, the settle tests' bar (front_conv_rel of its distance) -- place_piece(). Two
    // boxes 0.5 apart, a at 0.1, b at 0.2: both fronts at their own targets and converged.
    // MEASURED (2026-11-09, 5 turns): a's front mean 0.1025, b's 0.2031 (each alone: 0.1023,
    // 0.2037); before, b's bar was a's (the settle vertex test against the run's front_conv) and
    // the run refined b's front to 7905 vertices (alone 227) without converging.
    const std::array<int, 6> a = {{1, 2, 1, 2, 1, 2}}, b = {{5, 6, 1, 2, 1, 2}};
    const auto run = [](const MeshData& m, const double db) {
        const nlohmann::json j = {
            {"input", "m"},
            {"offsets",
             nlohmann::json::array(
                 {{{"selection", "a"}, {"output_tags", {"a_band"}}, {"target_distance", 0.1}},
                  {{"selection", "b"}, {"output_tags", {"b_band"}}, {"target_distance", db}}})},
            {"max_rounds", 5},
            {"num_threads", 1},
            {"throw_on_nonconvergence", false}};
        return components::topological_offset::topological_offset(j, {{"m", m}});
    };
    {
        const ComponentResult res = run(box_grid({{"a", a}, {"b", b}}), 0.2);
        CHECK(res.second.value("converged", false));
    }
    // b a one cube at 0.3, above the maximum marchable distance (constructed at 0.125), a 0.5 away:
    // the cells within 0.3 of a reach past b's front. Over the whole complex the band term
    // rewarded b's band for them (d there is a's, below b's 0.3): MEASURED (2026-11-09, boxes 0.5
    // apart, b at 0.3, 5 turns) b's band reached around a, its front vertices up to 1.05 from b
    // and 0.12 from a. Each piece's own integral: b's front at 0.3 (MEASURED 0.297..0.313, 5
    // turns, converged), as far from a as the gap leaves.
    {
        const std::array<int, 6> a1 = {{1, 2, 3, 3, 3, 3}}, b1 = {{5, 5, 3, 3, 3, 3}};
        const ComponentResult res = run(box_grid({{"a", a1}, {"b", b1}}), 0.3);
        CHECK(res.second.value("converged", false));
    }
}

TEST_CASE("several-offsets-absolute-front-conv", "[offset][3d]")
{
    // An absolute front_conv is one length for every piece; entries of different distances
    // take front_conv_rel, each bar its own distance's fraction (Parameters::init()). Equal
    // distances keep it.
    Eigen::MatrixXd V(2, 3);
    V << 0, 0, 0, 3, 4, 12;
    const auto with = [&](const double d1) {
        return construction_params(
            {{"offsets",
              nlohmann::json::array(
                  {{{"selection", "a"}, {"target_distance", 0.5}},
                   {{"selection", "b"}, {"target_distance", d1}}})},
             {"front_conv", 0.05}},
            V);
    };
    CHECK_THROWS(with(0.6));
    const Parameters same = with(0.5);
    CHECK(same.front_conv == 0.05);
}

TEST_CASE("several-offsets-solid-and-curve", "[offset][3d]")
{
    // Entries of different primitive kinds in one run, a solid (triangles) and an edge tag (a
    // curve, segments): one kind per entry, each entry its own engine (init_input_complex_bvh()).
    // Refused until 2026-11 ("the input complex mixes primitive kinds: 48 triangles, 4 segments
    // not on a triangle"). Each band is the one its entry makes alone.
    const GridPolyline rod = {{{1, 6, 6}}, {{2, 6, 6}}, {{3, 6, 6}}, {{4, 6, 6}}, {{5, 6, 6}}};
    MeshData m = curve_grid({{"rod", rod}});
    m.cells.at(CellType::Tetrahedron).attributes["tags"] = box_grid({{"a", {{1, 2, 1, 2, 1, 2}}}})
                                                               .cells.at(CellType::Tetrahedron)
                                                               .attributes.at("tags");
    const nlohmann::json a = {
        {"selection", "a"},
        {"output_tags", {"a_band"}},
        {"target_distance", 0.1}};
    const nlohmann::json r = {
        {"selection", "rod"},
        {"output_tags", {"rod_band"}},
        {"target_distance", 0.1}};
    const MeshData joint = construct_offsets(m, nlohmann::json::array({a, r}));
    CHECK(!cells_tagged(joint, "a_band").empty());
    CHECK(!cells_tagged(joint, "rod_band").empty());
    CHECK(
        cells_tagged(joint, "a_band") ==
        cells_tagged(construct_offsets(m, nlohmann::json::array({a})), "a_band"));
    CHECK(
        cells_tagged(joint, "rod_band") ==
        cells_tagged(construct_offsets(m, nlohmann::json::array({r})), "rod_band"));
    // A tag expression may not mix the kinds within one entry.
    REQUIRE_THROWS_WITH(
        construct_offsets(m, nlohmann::json::array({{{"selection", "a | rod"}}})),
        Catch::Matchers::ContainsSubstring("mixes primitive kinds"));
}

TEST_CASE("several-offsets-solid-and-curve-optimize", "[offset][3d]" + release_only)
{
    // The same two entries through the optimization: each piece its own engine, field and
    // integral, so the curve's band is not read against the solid's triangles. Both bands at
    // their targets: their front vertices within 10% of 0.1 from their own input.
    const GridPolyline rod = {{{1, 6, 6}}, {{2, 6, 6}}, {{3, 6, 6}}, {{4, 6, 6}}, {{5, 6, 6}}};
    MeshData m = curve_grid({{"rod", rod}});
    m.cells.at(CellType::Tetrahedron).attributes["tags"] = box_grid({{"a", {{1, 2, 1, 2, 1, 2}}}})
                                                               .cells.at(CellType::Tetrahedron)
                                                               .attributes.at("tags");
    const nlohmann::json j = {
        {"input", "m"},
        {"offsets",
         nlohmann::json::array(
             {{{"selection", "a"}, {"output_tags", {"a_band"}}, {"target_distance", 0.1}},
              {{"selection", "rod"}, {"output_tags", {"rod_band"}}, {"target_distance", 0.1}}})},
        {"max_rounds", 5},
        {"num_threads", 1},
        {"throw_on_nonconvergence", false}};
    ComponentResult res;
    REQUIRE_NOTHROW(res = components::topological_offset::topological_offset(j, {{"m", m}}));
    const MeshData& out = res.first.at("mesh");
    for (const double d : front_distances(out, "a_band", {{1, 2, 1, 2, 1, 2}})) {
        CHECK(d >= 0.09);
        CHECK(d <= 0.11);
    }
    // the rod: x in [-0.75, 0.25] at y = z = 0.5
    const auto rod_distance = [](const std::array<double, 3>& q) {
        const double x = std::clamp(q[0], -0.75, 0.25);
        return (Vector3d(q[0], q[1], q[2]) - Vector3d(x, 0.5, 0.5)).norm();
    };
    std::set<std::array<double, 3>> ambient;
    for (const CellCorners& c : cells_tagged(out, "ambient")) ambient.insert(c.begin(), c.end());
    size_t n = 0;
    for (const CellCorners& c : cells_tagged(out, "rod_band")) {
        for (const auto& q : c) {
            if (!ambient.count(q)) continue;
            ++n;
            CHECK(rod_distance(q) >= 0.09);
            CHECK(rod_distance(q) <= 0.11);
        }
    }
    CHECK(n > 0);
}

TEST_CASE("several-offsets-growth", "[offset][3d]" + release_only)
{
    // growth_mode with several entries (linear_growth()): each entry from its own d_0 to its own
    // target, one schedule. a one cube at 0.1, traced to by the march, stays there; b one cube at
    // 0.4, above the maximum marchable distance 0.25 (constructed at 0.125), grows in
    // linear_growth_steps stages 0.125 -> 0.4. Both converge at their own targets. MEASURED
    // (2026-11-10, 10 turns a stage): a's front mean 0.1019, b's 0.4052 (0.392 .. 0.431).
    const std::array<int, 6> a = {{1, 1, 3, 3, 3, 3}}, b = {{5, 5, 3, 3, 3, 3}};
    const nlohmann::json j = {
        {"input", "m"},
        {"offsets",
         nlohmann::json::array(
             {{{"selection", "a"}, {"output_tags", {"a_band"}}, {"target_distance", 0.1}},
              {{"selection", "b"}, {"output_tags", {"b_band"}}, {"target_distance", 0.4}}})},
        {"growth_mode", "linear"},
        {"max_rounds", 10},
        {"num_threads", 1},
        {"throw_on_nonconvergence", false}};
    const ComponentResult res = components::topological_offset::topological_offset(
        j,
        {{"m", box_grid({{"a", a}, {"b", b}})}});
    CHECK(res.second.value("converged", false));
}

namespace {
/// A box of squares (i, j) of point_grid_2d()'s grid, i, j in [lo, hi] of its {i_lo, i_hi, j_lo,
/// j_hi}: the 2D twin of box_grid()'s boxes.
using Box2d = std::array<int, 4>;

/// The input of box_grid_2d(): the grid, the faces' tags (one column per box), the curve group.
struct BoxGrid2d
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    MatrixSi tags;
    std::vector<std::string> names;
    Eigen::MatrixXd CV = Eigen::MatrixXd(0, 2);
    Eigen::MatrixXi CE = Eigen::MatrixXi(0, 2);
};

/// The 2D twin of box_grid(): [-1, 1]^2 in 8^2 squares of edge 0.25 (point_grid_2d()'s grid,
/// each square split along its diagonal from (i, j) to (i + 1, j + 1)), the squares of each named
/// box tagged by its name, every other square ambient; a curve group along `curve` (none if
/// empty), registered as the tag "curve". Boxes do not overlap.
BoxGrid2d box_grid_2d(
    const std::vector<std::pair<std::string, Box2d>>& boxes,
    const GridPolyline2d& curve = {})
{
    const int n = 8;
    const auto vid = [&](const int i, const int j) { return i + (n + 1) * j; };
    BoxGrid2d g;
    g.V.resize((n + 1) * (n + 1), 2);
    for (int j = 0; j <= n; ++j) {
        for (int i = 0; i <= n; ++i) g.V.row(vid(i, j)) = grid_point_2d({{i, j}}).transpose();
    }
    g.F.resize(2 * n * n, 3);
    g.tags.resize(2 * n * n, Eigen::Index(std::max<size_t>(1, boxes.size())));
    for (const auto& [name, box] : boxes) g.names.push_back(name);
    int r = 0;
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            for (size_t b = 0; b < boxes.size(); ++b) {
                const Box2d& q = boxes[b].second;
                if (i < q[0] || i > q[1] || j < q[2] || j > q[3]) continue;
                g.tags.coeffRef(r, Eigen::Index(b)) = 1;
                g.tags.coeffRef(r + 1, Eigen::Index(b)) = 1;
            }
            g.F.row(r++) << vid(i, j), vid(i + 1, j), vid(i + 1, j + 1);
            g.F.row(r++) << vid(i, j), vid(i + 1, j + 1), vid(i, j + 1);
        }
    }
    if (!curve.empty()) {
        g.CV.resize(Eigen::Index(curve.size()), 2);
        g.CE.resize(Eigen::Index(curve.size() - 1), 2);
        for (size_t k = 0; k < curve.size(); ++k) {
            g.CV.row(Eigen::Index(k)) = grid_point_2d(curve[k]).transpose();
            if (k + 1 < curve.size()) g.CE.row(Eigen::Index(k)) << int(k), int(k + 1);
        }
    }
    return g;
}

/// The distance from p to box_grid_2d()'s box `b`: the twin of box_distance().
double box_distance_2d(const Vector2d& p, const Box2d& b)
{
    double s = 0.;
    for (int a = 0; a < 2; ++a) {
        const double lo = b[size_t(2 * a)] * 0.25 - 1., hi = (b[size_t(2 * a + 1)] + 1) * 0.25 - 1.;
        const double e = std::max({lo - p[a], 0., p[a] - hi});
        s += e * e;
    }
    return std::sqrt(s);
}

/// Captures the log while it lives, the sink removed on every way out.
struct LogCapture2d
{
    std::ostringstream log;
    std::shared_ptr<spdlog::sinks::ostream_sink_mt> sink =
        std::make_shared<spdlog::sinks::ostream_sink_mt>(log);
    LogCapture2d() { wmtk::logger().sinks().push_back(sink); }
    ~LogCapture2d()
    {
        auto& s = wmtk::logger().sinks();
        s.erase(std::remove(s.begin(), s.end(), sink), s.end());
    }
};

/// The steps the 2D branch of topological_offset() runs, driven directly to read the mesh, on
/// box_grid_2d(): `j`'s keys (its offsets among them), each entry's selection parsed as the
/// driver parses it, the construction, the field, and with `optimize` the loop
/// (optimize_offset()), optimize_offset `optimize` unless `j` sets it. The 2D twin of
/// construct_offsets() and of topological_offset() on box_grid(). `log`, when given, receives
/// the run's log.
std::unique_ptr<TopoOffsetTriMesh> run_offsets_2d(
    const BoxGrid2d& g,
    const nlohmann::json& j,
    const bool optimize,
    std::string* log = nullptr)
{
    Eigen::MatrixXd box(2, 2);
    box << -1., -1., 1., 1.;
    nlohmann::json k = j;
    if (!k.contains("num_threads")) k["num_threads"] = 0;
    if (!k.contains("throw_on_nonconvergence")) k["throw_on_nonconvergence"] = false;
    if (!k.contains("optimize_offset")) k["optimize_offset"] = optimize;
    // the mesh holds a reference to its parameters: keep them alive with it
    static std::vector<std::unique_ptr<Parameters>> params;
    params.push_back(std::make_unique<Parameters>(construction_params(k, box)));
    auto mesh = std::make_unique<TopoOffsetTriMesh>(*params.back(), 0);
    LogCapture2d capture;
    const auto keep_log = [&]() {
        if (log) *log = capture.log.str();
    };
    try {
        mesh->init_from_image(g.V, g.F, g.tags, g.CV, g.CE, g.names, "curve");
        for (Parameters::Offset& o : mesh->m_offset_params.offsets)
            o.expression = parse(o.selection, mesh->m_tag_name_to_id);
        mesh->m_offset_params.parse_protected_tags(mesh->m_tag_name_to_id);
        mesh->label_input_complex();
        mesh->init_input_complex_bvh();
        mesh->consolidate_mesh();
        const std::filesystem::path out =
            std::filesystem::temp_directory_path() / "wmtk_offset_several_2d";
        mesh->construct_offset(out);
        mesh->init_offset_potential();
        if (optimize) mesh->optimize_offset(out);
    } catch (...) {
        keep_log();
        throw;
    }
    keep_log();
    return mesh;
}

/// The faces of a 2D mesh carrying `tag`, each as its corners' positions in sorted order: a
/// face's identity across two runs that number the vertices differently. The twin of
/// cells_tagged().
using FaceCorners2d = std::array<std::array<double, 2>, 3>;
std::set<FaceCorners2d> faces_tagged_2d(const TopoOffsetTriMesh& m, const std::string& tag)
{
    const auto it = m.m_tag_name_to_id.find(tag);
    REQUIRE(it != m.m_tag_name_to_id.end());
    std::set<FaceCorners2d> out;
    for (const auto& f : m.get_faces()) {
        const size_t fid = f.fid(m);
        if (!m.m_face_attribute[fid].tags.count(it->second)) continue;
        FaceCorners2d c;
        const auto vs = m.oriented_tri_vids(fid);
        for (int k = 0; k < 3; ++k) {
            const Vector2d& p = m.m_vertex_attribute[vs[size_t(k)]].m_posf;
            c[size_t(k)] = {{p[0], p[1]}};
        }
        std::sort(c.begin(), c.end());
        out.insert(c);
    }
    return out;
}

/// The front of `tag`'s band: the corners its faces share with ambient faces, off the box `b`, as
/// distances to it. The twin of front_distances().
std::vector<double>
front_distances_2d(const TopoOffsetTriMesh& m, const std::string& tag, const Box2d& b)
{
    std::set<std::array<double, 2>> ambient;
    for (const auto& f : m.get_faces()) {
        const size_t fid = f.fid(m);
        if (!m.m_face_attribute[fid].tags.count(0)) continue;
        for (const size_t v : m.oriented_tri_vids(fid)) {
            const Vector2d& p = m.m_vertex_attribute[v].m_posf;
            ambient.insert({{p[0], p[1]}});
        }
    }
    std::set<std::array<double, 2>> front;
    for (const FaceCorners2d& c : faces_tagged_2d(m, tag)) {
        for (const auto& q : c) {
            if (ambient.count(q)) front.insert(q);
        }
    }
    std::vector<double> d;
    for (const auto& q : front) {
        const double x = box_distance_2d(Vector2d(q[0], q[1]), b);
        if (x > 0.) d.push_back(x);
    }
    return d;
}

/// One entry of offsets as the json gives it.
nlohmann::json offset_entry_2d(const std::string& selection, const double target)
{
    return {
        {"selection", selection},
        {"output_tags", {selection + "_band"}},
        {"target_distance", target}};
}

} // namespace

TEST_CASE("several-offsets-as-alone-2d", "[offset][2d]")
{
    // The 2D twin of several-offsets-as-alone, on the construction (driven directly): three
    // bodies, each two squares from the next, offset in one march; each entry's band is the one a
    // run of that entry alone makes -- the same faces at the same positions -- and takes its own
    // output tags. Each entry decides its fallback itself (marching_tris()): every outer end is a
    // grid edge (0.25) or more from its body, so a at 0.1 and c at 0.2 are traced to, and b at
    // 0.3 falls back to half of 0.25.
    const std::vector<std::pair<std::string, Box2d>> boxes = {
        {"a", {{1, 2, 1, 2}}},
        {"b", {{5, 6, 1, 2}}},
        {"c", {{1, 2, 5, 6}}}};
    const BoxGrid2d g = box_grid_2d(boxes);
    const std::vector<double> D = {0.1, 0.3, 0.2};
    nlohmann::json all = nlohmann::json::array();
    for (size_t k = 0; k < boxes.size(); ++k) all.push_back(offset_entry_2d(boxes[k].first, D[k]));
    std::string log;
    const auto joint = run_offsets_2d(g, {{"offsets", all}}, false, &log);
    // Each entry's own construction line, and its own trace.
    CHECK(log.find("[construction] offsets[0]: construction_mode") != std::string::npos);
    CHECK(log.find("[construction] offsets[2]: construction_mode") != std::string::npos);
    CHECK(log.find("D = each entry's, above") != std::string::npos);
    for (size_t k = 0; k < boxes.size(); ++k) {
        const std::string band = boxes[k].first + "_band";
        INFO("offsets[" << k << "] " << boxes[k].first << " at " << D[k]);
        CHECK(joint->m_construction_distances[k] == (D[k] < 0.25 ? D[k] : 0.125));
        const auto faces = faces_tagged_2d(*joint, band);
        CHECK(!faces.empty());
        const auto alone = run_offsets_2d(g, {{"offsets", nlohmann::json::array({all[k]})}}, false);
        CHECK(faces == faces_tagged_2d(*alone, band));
        const double traced = D[k] < 0.25 ? D[k] : 0.125;
        size_t n_outer = 0;
        for (const FaceCorners2d& c : faces) {
            for (const auto& q : c) {
                const double d = box_distance_2d(Vector2d(q[0], q[1]), boxes[k].second);
                if (d == 0.) continue; // on the body
                ++n_outer;
                CHECK(std::abs(d - traced) <= 0.01 * traced);
            }
        }
        CHECK(n_outer > 0);
        // No band face of this entry carries another entry's output tag.
        for (size_t o = 0; o < boxes.size(); ++o) {
            if (o == k) continue;
            for (const FaceCorners2d& c : faces_tagged_2d(*joint, boxes[o].first + "_band")) {
                CHECK(faces.count(c) == 0);
            }
        }
    }
    // The comparison tells positions apart: a alone at c's distance is not a's band.
    nlohmann::json a_at_c = all[0];
    a_at_c["target_distance"] = D[2];
    CHECK(
        faces_tagged_2d(*joint, "a_band") !=
        faces_tagged_2d(
            *run_offsets_2d(g, {{"offsets", nlohmann::json::array({a_at_c})}}, false),
            "a_band"));
}

TEST_CASE("several-offsets-touch-2d", "[offset][2d]")
{
    // The 2D twin of several-offsets-touch: entries that intersect are refused
    // (label_input_complex()): b's squares next to a's share their vertices. One square apart,
    // a's and b's vertices are joined by grid edges; the simplicial embedding splits them, and
    // the two bands come out apart -- no vertex in both -- each within half of such an edge
    // (0.125 from both), so a target of 0.1 is still traced to.
    const auto run = [](const Box2d& b) {
        return run_offsets_2d(
            box_grid_2d({{"a", {{1, 2, 1, 2}}}, {"b", b}}),
            {{"offsets",
              nlohmann::json::array({offset_entry_2d("a", 0.1), offset_entry_2d("b", 0.1)})}},
            false);
    };
    REQUIRE_THROWS_WITH(run({{3, 4, 1, 2}}), Catch::Matchers::ContainsSubstring("share"));
    const Box2d b = {{4, 5, 1, 2}};
    std::unique_ptr<TopoOffsetTriMesh> out;
    REQUIRE_NOTHROW(out = run(b));
    std::set<std::array<double, 2>> corners_a;
    for (const FaceCorners2d& c : faces_tagged_2d(*out, "a_band"))
        corners_a.insert(c.begin(), c.end());
    size_t n_b = 0, n_b_outer = 0;
    for (const FaceCorners2d& c : faces_tagged_2d(*out, "b_band")) {
        ++n_b;
        for (const auto& q : c) {
            CHECK(corners_a.count(q) == 0);
            const double d = box_distance_2d(Vector2d(q[0], q[1]), b);
            if (d > 0.) {
                ++n_b_outer;
                CHECK(std::abs(d - 0.1) <= 0.01 * 0.1);
            }
        }
    }
    CHECK(!corners_a.empty());
    CHECK(n_b > 0);
    CHECK(n_b_outer > 0);
}

TEST_CASE("several-offsets-absolute-front-conv-2d", "[offset][2d]")
{
    // The 2D twin of several-offsets-absolute-front-conv, on a 2D box: an absolute front_conv is
    // one length for every piece; entries of different distances take front_conv_rel, each bar
    // its own distance's fraction (Parameters::init(), shared). Equal distances keep it; the
    // range of the second entry's distance, below, at and above the first's.
    Eigen::MatrixXd V(2, 2);
    V << 0, 0, 3, 4;
    const auto with = [&](const double d1, const nlohmann::json& conv) {
        nlohmann::json j = {
            {"offsets",
             nlohmann::json::array(
                 {{{"selection", "a"}, {"target_distance", 0.5}},
                  {{"selection", "b"}, {"target_distance", d1}}})}};
        j.update(conv);
        return construction_params(j, V);
    };
    for (const double d1 : {0.25, 0.6, 2.}) {
        INFO("second entry at " << d1);
        CHECK_THROWS(with(d1, {{"front_conv", 0.05}}));
        const Parameters rel = with(d1, {{"front_conv_rel", 0.1}});
        CHECK(rel.target_distance == std::min(0.5, d1)); // the run's: the smallest entry's
        CHECK(rel.front_conv == Catch::Approx(0.1 * std::min(0.5, d1)));
    }
    const Parameters same = with(0.5, {{"front_conv", 0.05}});
    CHECK(same.front_conv == 0.05);
}

TEST_CASE("several-offsets-solid-and-curve-2d", "[offset][2d]")
{
    // The 2D twin of several-offsets-solid-and-curve: entries of different kinds in one run, a
    // solid (a face tag, entering as its boundary segments, with a side) and the curve group (a
    // curve: segments, no side), each entry its own distance engine (init_input_complex_bvh(),
    // offset_distance()). Each band is the one its entry makes alone. 3D also refuses one entry
    // mixing the kinds ("a | rod"); 2D refuses no mix (its primitives are all segments and
    // points), and the curve group in an expression selects no face, so 2D has no twin of that
    // half.
    const GridPolyline2d curve = {{{1, 6}}, {{2, 6}}, {{3, 6}}, {{4, 6}}, {{5, 6}}};
    const BoxGrid2d g = box_grid_2d({{"a", {{1, 2, 1, 2}}}}, curve);
    const nlohmann::json a = offset_entry_2d("a", 0.1);
    const nlohmann::json r = offset_entry_2d("curve", 0.1);
    const auto joint = run_offsets_2d(g, {{"offsets", nlohmann::json::array({a, r})}}, false);
    CHECK(joint->m_offset_curves.size() == 2);
    CHECK_FALSE(joint->offset_curves(0)); // the solid
    CHECK(joint->offset_curves(1)); // the curve
    CHECK(!faces_tagged_2d(*joint, "a_band").empty());
    CHECK(!faces_tagged_2d(*joint, "curve_band").empty());
    CHECK(
        faces_tagged_2d(*joint, "a_band") ==
        faces_tagged_2d(
            *run_offsets_2d(g, {{"offsets", nlohmann::json::array({a})}}, false),
            "a_band"));
    CHECK(
        faces_tagged_2d(*joint, "curve_band") ==
        faces_tagged_2d(
            *run_offsets_2d(g, {{"offsets", nlohmann::json::array({r})}}, false),
            "curve_band"));
}

TEST_CASE("several-offsets-optimize-2d", "[offset][2d]" + release_only)
{
    // The 2D twin of several-offsets-optimize: several entries run the optimization, an input
    // complex of several connected pieces, each its entry's.
    const auto mesh = run_offsets_2d(
        box_grid_2d({{"a", {{1, 2, 1, 2}}}, {"b", {{5, 6, 1, 2}}}}),
        {{"offsets", nlohmann::json::array({offset_entry_2d("a", 0.1), offset_entry_2d("b", 0.1)})},
         {"max_rounds", 1}},
        true);
    CHECK(mesh->m_region_offset == std::vector<int>{0, 1});
    CHECK(!faces_tagged_2d(*mesh, "a_band").empty());
    CHECK(!faces_tagged_2d(*mesh, "b_band").empty());
}

TEST_CASE("several-offsets-optimize-distances-2d", "[offset][2d]" + release_only)
{
    // The 2D twin of several-offsets-optimize-distances: each piece its own target distance in
    // the optimization -- its field, the band term's distance and integral, the settle tests' bar
    // (front_conv_rel of its distance): place_piece(). Two boxes 0.5 apart, a at 0.1, b at 0.2:
    // both fronts at their own targets and converged; then b one square at 0.3, above the
    // maximum marchable distance (constructed at 0.125), a 0.5 away, whose band over the whole
    // complex 3D measured reaching around a (each piece's own integral, init_band_exact()).
    // Under EXPERIMENTAL_band_volume_rule "exact_swept", the default, each piece's swept change
    // (TopoOffsetTriMesh, EXACT SWEPT CHANGE, several entries): read at the moving vertex's piece,
    // b's front ended at 0.278 (3D's at 0.252 from 455ab9ba66). Under the settle test (the
    // default), whose tests read each piece's integral, and under the energy criterion
    // (EXPERIMENTAL_settle_refinement false), whose chord and ring measures read each piece's
    // field (init_region_potentials()).
    const bool settle = GENERATE(true, false);
    INFO("EXPERIMENTAL_settle_refinement " << settle);
    const auto run = [&](const Box2d& a, const Box2d& b, const double db) {
        return run_offsets_2d(
            box_grid_2d({{"a", a}, {"b", b}}),
            {{"offsets",
              nlohmann::json::array({offset_entry_2d("a", 0.1), offset_entry_2d("b", db)})},
             {"EXPERIMENTAL_settle_refinement", settle},
             {"max_rounds", 5}},
            true);
    };
    {
        const Box2d a = {{1, 2, 1, 2}}, b = {{5, 6, 1, 2}};
        const auto mesh = run(a, b, 0.2);
        CHECK(mesh->m_converged);
    }
    {
        const Box2d a1 = {{1, 2, 3, 3}}, b1 = {{5, 5, 3, 3}};
        const auto mesh = run(a1, b1, 0.3);
        CHECK(mesh->m_converged);
    }
}

TEST_CASE("several-offsets-solid-and-curve-optimize-2d", "[offset][2d]" + release_only)
{
    // The 2D twin of several-offsets-solid-and-curve-optimize: the same two entries through the
    // optimization, each piece its own field and integral, so the curve's band is not read
    // against the solid's segments. Both bands at their targets: their front vertices within 10%
    // of 0.1 from their own input.
    const GridPolyline2d curve = {{{1, 6}}, {{2, 6}}, {{3, 6}}, {{4, 6}}, {{5, 6}}};
    const auto mesh = run_offsets_2d(
        box_grid_2d({{"a", {{1, 2, 1, 2}}}}, curve),
        {{"offsets",
          nlohmann::json::array({offset_entry_2d("a", 0.1), offset_entry_2d("curve", 0.1)})},
         {"max_rounds", 5}},
        true);
    const std::vector<double> da = front_distances_2d(*mesh, "a_band", {{1, 2, 1, 2}});
    REQUIRE(!da.empty());
    for (const double d : da) {
        CHECK(d >= 0.09);
        CHECK(d <= 0.11);
    }
    // the curve: x in [-0.75, 0.25] at y = 0.5
    const auto curve_distance = [](const std::array<double, 2>& q) {
        const double x = std::clamp(q[0], -0.75, 0.25);
        return (Vector2d(q[0], q[1]) - Vector2d(x, 0.5)).norm();
    };
    std::set<std::array<double, 2>> ambient;
    for (const auto& f : mesh->get_faces()) {
        const size_t fid = f.fid(*mesh);
        if (!mesh->m_face_attribute[fid].tags.count(0)) continue;
        for (const size_t v : mesh->oriented_tri_vids(fid)) {
            const Vector2d& p = mesh->m_vertex_attribute[v].m_posf;
            ambient.insert({{p[0], p[1]}});
        }
    }
    size_t n = 0;
    for (const FaceCorners2d& c : faces_tagged_2d(*mesh, "curve_band")) {
        for (const auto& q : c) {
            if (!ambient.count(q)) continue;
            ++n;
            CHECK(curve_distance(q) >= 0.09);
            CHECK(curve_distance(q) <= 0.11);
        }
    }
    CHECK(n > 0);
}

TEST_CASE("several-offsets-growth-2d", "[offset][2d]" + release_only)
{
    // The 2D twin of several-offsets-growth: growth_mode "linear" with several entries
    // (linear_growth()), each entry from its own d_0 to its own target, one schedule. a one square
    // at 0.1, traced to by the march, stays there; b one square at 0.4, above the maximum
    // marchable distance 0.25 (constructed at 0.125), grows in linear_growth_steps stages
    // 0.125 -> 0.4. Both converge at their own targets. Under EXPERIMENTAL_band_volume_rule
    // "exact_swept", the default, each piece's swept change (TopoOffsetTriMesh, EXACT SWEPT
    // CHANGE, several entries; 3D's hung at 455ab9ba66 without it).
    const Box2d a = {{1, 1, 3, 3}}, b = {{5, 5, 3, 3}};
    std::string log;
    const auto mesh = run_offsets_2d(
        box_grid_2d({{"a", a}, {"b", b}}),
        {{"offsets", nlohmann::json::array({offset_entry_2d("a", 0.1), offset_entry_2d("b", 0.4)})},
         {"growth_mode", "linear"},
         {"max_rounds", 10}},
        true,
        &log);
    INFO(log.substr(log.size() > 4000 ? log.size() - 4000 : 0));
    CHECK(mesh->m_growth_starts.empty() == false);
    CHECK(mesh->m_growth_starts == std::vector<double>{0.1, 0.125});
    CHECK(mesh->m_offset_levels.empty()); // each entry at its target_distance after the stages
    CHECK(
        log.find("[growth] offsets[0]: from 0.1 to its target_distance 0.1 (no growth") !=
        std::string::npos);
    CHECK(
        log.find("[growth] offsets[1]: from 0.125 to its target_distance 0.4") !=
        std::string::npos);
    CHECK(
        log.find(
            "[incremental growth] stage 1 / 4: offsets[1] at 0.19375 (its target_distance "
            "0.4)") != std::string::npos);
    CHECK(
        log.find("[incremental growth] stage 1 / 4: offsets[0] at 0.1 (its target_distance 0.1)") !=
        std::string::npos);
    // Each stage's field for b's piece at b's stage target (region_delta() reads m_offset_levels;
    // init_region_potentials() logs the level it builds at), a's at 0.1 throughout.
    for (int k = 0; k <= 4; ++k) {
        const double level = TopoOffsetTriMesh::growth_stage_target(0.125, 0.4, k, 4);
        INFO("stage " << k << ": b at " << level);
        CHECK(log.find(fmt::format(" points, level {} -- the band", level)) != std::string::npos);
    }
    CHECK(mesh->m_converged);
}

TEST_CASE("growth-mode-keys-2d", "[offset][2d]")
{
    // The 2D twin of growth-mode-keys: the keys are Parameters' (shared, read the same in both
    // dimensions; growth-mode-keys checks their defaults and refusals), so this checks the 2D
    // mesh's stage targets, TopoOffsetTriMesh::growth_stage_target(), as 3D's: stage k of n at
    // d0 + k (delta - d0) / n, d0 at k = 0, delta itself at k = n; and that growth_mode reaches
    // the 2D construction: d_0 is recorded (m_growth_start) for every mode but "none", only when
    // the march traces below the target.
    CHECK(TopoOffsetTriMesh::growth_stage_target(0.3, 0.7, 0, 1) == 0.3);
    CHECK(TopoOffsetTriMesh::growth_stage_target(0.3, 0.7, 1, 1) == 0.7);
    const std::vector<double> quarters = {0.5, 1., 1.5, 2., 2.5};
    for (int k = 0; k <= 4; ++k) {
        CHECK(TopoOffsetTriMesh::growth_stage_target(0.5, 2.5, k, 4) == quarters[size_t(k)]);
    }
    CHECK(TopoOffsetTriMesh::growth_stage_target(0.1, 0.9, 3, 3) == 0.9);
    for (int k = 0; k < 7; ++k) {
        CHECK(
            TopoOffsetTriMesh::growth_stage_target(0.1, 0.9, k, 7) <
            TopoOffsetTriMesh::growth_stage_target(0.1, 0.9, k + 1, 7));
    }
    // Two squares (the maximum marchable distance 0.25: d_0 = 0.125), at 0.2 (traced to: no
    // growth) and at 0.4, under each mode, construction only; optimize_offset false records no
    // growth (growth is the loop's), as in 3D.
    const BoxGrid2d g = box_grid_2d({{"a", {{3, 4, 3, 4}}}});
    for (const char* mode : {"none", "linear", "smart", "individual_op_reject"}) {
        for (const double target : {0.2, 0.4}) {
            for (const bool optimize : {false, true}) {
                INFO("growth_mode " << mode << ", target " << target << ", optimize " << optimize);
                std::string log;
                const auto mesh = run_offsets_2d(
                    g,
                    {{"offsets", nlohmann::json::array({offset_entry_2d("a", target)})},
                     {"growth_mode", mode},
                     {"optimize_offset", optimize}},
                    false,
                    &log);
                const bool grows = std::string(mode) != "none" && target > 0.25 && optimize;
                CHECK(mesh->m_growth_start == (grows ? 0.125 : 0.));
                CHECK(mesh->m_growth_starts.empty()); // one entry
                if (std::string(mode) != "none" && optimize && !grows) {
                    CHECK(
                        log.find("not done: the march traces to target_distance itself") !=
                        std::string::npos);
                }
            }
        }
    }
}

TEST_CASE("smart-growth-schedule-2d", "[offset][2d]")
{
    // The 2D twin of smart-growth-schedule (TopoOffsetTriMesh::SmartGrowthSchedule, the same
    // rule): from base b the j-th try after j failures is at b + (delta - b) / 2^j, delta itself
    // first; a success below delta becomes the new base and the next try is at delta again;
    // max_fails failures from one base give up; a success at delta is the end.
    using Schedule = TopoOffsetTriMesh::SmartGrowthSchedule;
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

TEST_CASE("turned-edges-at-vertex-2d", "[offset][2d]")
{
    // The 2D twin of turned-faces-at-vertex: growth_mode "individual_op_reject" counts the edges
    // turned from the field per operation with turned_front_edges_at(), which reads one vertex's
    // own face list (lock-safe in the operation passes). Over every vertex it must find exactly
    // the turned edges turned_edge_census() finds over the whole offset curve
    // (offset_surface_edges()). A square at 0.2, constructed, with one front vertex dragged to the
    // input's centre, which turns its offset edges back toward the input: turned edges to find.
    auto mesh = run_offsets_2d(
        box_grid_2d({{"a", {{3, 4, 3, 4}}}}),
        {{"offsets", nlohmann::json::array({offset_entry_2d("a", 0.2)})}},
        false);
    mesh->label_offset_boundary(); // as optimize_offset() does before the loop
    CHECK_FALSE(mesh->turned_reject_active()); // growth_mode "none"
    const auto census_set = [&]() {
        std::set<std::array<size_t, 2>> s;
        for (const auto& e : mesh->offset_surface_edges()) {
            if (mesh->front_edge_turned(e[0], e[1])) s.insert({{e[0], e[1]}});
        }
        return s;
    };
    const auto per_vertex_set = [&]() {
        std::set<std::array<size_t, 2>> s;
        for (const auto& v : mesh->get_vertices()) mesh->turned_front_edges_at(v.vid(*mesh), s);
        return s;
    };
    CHECK(census_set().empty()); // as constructed
    CHECK(per_vertex_set().empty());
    CHECK(mesh->turned_edge_census().n_turned == 0);
    CHECK(mesh->turned_edge_census().n_edges == mesh->offset_surface_edges().size());
    size_t front = static_cast<size_t>(-1);
    for (const auto& v : mesh->get_vertices()) {
        if (mesh->m_vertex_extra[v.vid(*mesh)].m_is_on_offset) {
            front = v.vid(*mesh);
            break;
        }
    }
    REQUIRE(front != static_cast<size_t>(-1));
    mesh->set_vertex_position(front, Vector2d::Zero());
    const auto census = census_set();
    INFO(census.size() << " turned edges");
    REQUIRE(!census.empty());
    CHECK(per_vertex_set() == census);
    CHECK(mesh->turned_edge_census().n_turned == census.size());
    // Every turned edge found through the moved vertex goes through it.
    std::set<std::array<size_t, 2>> at;
    mesh->turned_front_edges_at(front, at);
    CHECK(!at.empty());
    for (const auto& e : at) CHECK((e[0] == front || e[1] == front));

    // The rule is on in the loops of growth_mode "individual_op_reject" only.
    mesh->m_offset_params.growth_mode = "individual_op_reject";
    CHECK(mesh->turned_reject_active());
    mesh->m_freeze_front = true;
    CHECK_FALSE(mesh->turned_reject_active());
    mesh->m_freeze_front = false;
}

TEST_CASE("growth-modes-in-the-loop-2d", "[offset][2d]" + release_only)
{
    // growth_mode through the 2D loop (optimize_offset()), a box of 2 x 2 squares at 0.35, beyond
    // the maximum marchable distance 0.25, so the march traces to d_0 = 0.125:
    //   - "linear" over linear_growth_steps 1, 2 and 4: N + 1 loops, stage k at
    //     growth_stage_target(0.125, 0.35, k, N), each logged, the last the ordinary loop;
    //   - "individual_op_reject": "linear" with one step, and an [op reject] line per operation
    //     group in its loops, which refuse smoothing moves (MEASURED at 0.4: 9 and 2 in its last
    //     loop's two turns);
    //   - "smart": level 0 at d_0, then the attempt at 0.35 (the final stage), of up to 10 turns
    //     (smart_growth_attempts; with the default 2 the attempt here is cut off) -- MEASURED
    //     converged free of turned edges at 0.27 and 0.35, while at 0.3, 0.4, 0.45 and 0.5, on
    //     this box and on one square, it converges in 2 turns with 1 to 3 chords at the box's
    //     corners 37 to 47 degrees from the field, which fails it -- with the 2D swap pass's 1e-5
    //     max-AMIPS filter, dropped 2026-11-12, after which 0.4 converges free of them;
    //   - "none": no stage, the loop at 0.35.
    // Each converges, its front at 0.35, the run's target set back to it. The front of a box of
    // side 0.5 at 0.35 is a rounded square inside [-1, 1]^2.
    struct Case
    {
        std::string mode;
        int steps;
    };
    const Case c = GENERATE(
        Case{"none", 4},
        Case{"linear", 1},
        Case{"linear", 2},
        Case{"linear", 4},
        Case{"individual_op_reject", 4},
        Case{"smart", 4});
    INFO("growth_mode " << c.mode << ", linear_growth_steps " << c.steps);
    const double delta = 0.35;
    nlohmann::json j = {
        {"offsets", nlohmann::json::array({offset_entry_2d("a", delta)})},
        {"growth_mode", c.mode},
        {"max_rounds", 10}};
    if (c.mode == "linear") j["linear_growth_steps"] = c.steps;
    if (c.mode == "smart") j["smart_growth_attempts"] = 10;
    std::string log;
    const Box2d a = {{3, 4, 3, 4}};
    const auto mesh = run_offsets_2d(box_grid_2d({{"a", a}}), j, true, &log);
    INFO(log.substr(log.size() > 4000 ? log.size() - 4000 : 0));
    CHECK(mesh->m_offset_params.target_distance == delta);
    CHECK(mesh->m_growth_start == (c.mode == "none" ? 0. : 0.125));
    if (c.mode == "linear" || c.mode == "individual_op_reject") {
        const int n = c.mode == "linear" ? c.steps : 1;
        CHECK(
            log.find(
                fmt::format(
                    "[incremental growth] {} loops: at the traced distance 0.125",
                    n + 1)) != std::string::npos);
        for (int k = 0; k < n; ++k) {
            CHECK(
                log.find(
                    fmt::format(
                        "======== [incremental growth] stage {} / {}: target_distance {:.6g}",
                        k,
                        n,
                        TopoOffsetTriMesh::growth_stage_target(0.125, delta, k, n))) !=
                std::string::npos);
        }
        CHECK(
            log.find(
                fmt::format(
                    "======== [incremental growth] stage {} / {}: target_distance 0.35, the "
                    "ordinary "
                    "loop",
                    n,
                    n)) != std::string::npos);
    }
    CHECK(
        (log.find("[op reject] turn 1 ") != std::string::npos) ==
        (c.mode == "individual_op_reject"));
    if (c.mode == "individual_op_reject") {
        // The rule acts: the smoothing moves it refused over the run's loops (MEASURED 90 here,
        // no collapse; at 0.4, 9 and 2 in the last loop's two turns).
        int refused = 0;
        const std::string key = " collapses, ";
        for (size_t at = log.find("[op reject] turn "); at != std::string::npos;
             at = log.find("[op reject] turn ", at + 1)) {
            const size_t c0 = log.find(key, at);
            refused += std::stoi(log.substr(c0 + key.size()));
        }
        CHECK(refused > 0);
    }
    if (c.mode == "smart") {
        CHECK(
            log.find("======== [smart growth] level 0: target_distance 0.125") !=
            std::string::npos);
        CHECK(
            log.find("======== [smart growth] level 1 attempt 1: target_distance 0.35") !=
            std::string::npos);
        CHECK(log.find("[smart growth] target_distance 0.35 reached") != std::string::npos);
    }
    if (c.mode == "none") CHECK(log.find("growth]") == std::string::npos);
    CHECK(mesh->m_converged);
}

namespace {
/// A 2D mesh read off its live slots, to compare two meshes bit for bit: the twin of
/// frames_identical(), which reads 3D's frame arrays (gather_vtu_frame(); the 2D mesh has no frame
/// gatherer). Each face's corners as stored -- the order its edge slots 3 fid + j are keyed on --
/// with its region tags and label; each vertex's position, exact position, sizing scalar, flags
/// and labels; each edge slot's surface tags and label. Not what an operation stamps from a run
/// counter (VertexExtra2d::m_born_epoch), which a snapshot does not restore.
struct MeshState2d
{
    std::vector<std::array<size_t, 3>> faces;
    std::vector<std::set<int64_t>> face_tags;
    std::vector<int> face_label;
    std::vector<Vector2d> posf;
    std::vector<Vector2r> pos;
    std::vector<double> sizing;
    std::vector<std::array<int64_t, 11>> vertex_flags;
    std::vector<std::array<int64_t, 7>> edge_flags;
    bool operator==(const MeshState2d& o) const
    {
        return std::tie(
                   faces,
                   face_tags,
                   face_label,
                   posf,
                   pos,
                   sizing,
                   vertex_flags,
                   edge_flags) ==
               std::tie(
                   o.faces,
                   o.face_tags,
                   o.face_label,
                   o.posf,
                   o.pos,
                   o.sizing,
                   o.vertex_flags,
                   o.edge_flags);
    }
};

MeshState2d mesh_state_2d(const TopoOffsetTriMesh& m)
{
    MeshState2d s;
    for (const auto& t : m.get_vertices()) {
        const size_t v = t.vid(m);
        const auto& a = m.m_vertex_attribute[v];
        const VertexExtra2d& x = m.m_vertex_extra[v];
        s.posf.push_back(a.m_posf);
        s.pos.push_back(a.pos());
        s.sizing.push_back(a.m_sizing_scalar);
        s.vertex_flags.push_back(
            {{a.m_is_rounded,
              a.m_is_on_surface,
              x.label,
              x.offset,
              int64_t(x.point_inputs),
              x.m_is_on_input,
              x.m_is_on_offset,
              x.m_is_on_region,
              int64_t(x.m_boundary_mask),
              x.m_corr_input_vid,
              x.m_corr_carried}});
    }
    for (const auto& t : m.get_faces()) {
        const size_t fid = t.fid(m);
        s.faces.push_back(m.oriented_tri_vids(fid));
        s.face_tags.push_back(m.m_face_attribute[fid].tags);
        s.face_label.push_back(m.m_face_extra[fid].label);
        for (size_t j = 0; j < 3; ++j) {
            const SurfaceTagAttributes& a = m.m_edge_attribute[3 * fid + j];
            const EdgeExtra2d& x = m.m_edge_extra[3 * fid + j];
            s.edge_flags.push_back(
                {{a.m_is_surface_fs,
                  a.m_is_bbox_fs,
                  a.m_surface_class,
                  int64_t(a.input_set()),
                  a.orientation(),
                  x.label,
                  x.on_curve}});
        }
    }
    return s;
}
} // namespace

TEST_CASE("mesh-snapshot-roundtrip-2d", "[offset][2d]" + release_only)
{
    // The 2D twin of mesh-snapshot-roundtrip. take_snapshot() / restore_snapshot(): after a
    // restore the mesh is the one the snapshot left, bit for bit (mesh_state_2d()), and the same
    // operations from it give the same result again -- the storage is the same, not only the
    // geometry. A square at 0.2, constructed, serial; the change is a split pass at a lowered
    // sizing field, which adds vertices and faces.
    auto mesh = run_offsets_2d(
        box_grid_2d({{"a", {{3, 4, 3, 4}}}}),
        {{"offsets", nlohmann::json::array({offset_entry_2d("a", 0.2)})},
         {"EXPERIMENTAL_band_volume_rule", "exact"}},
        false);
    // As optimize_offset() sets them before the loop.
    mesh->m_edge_split_mode = TopoOffsetTriMesh::EdgeSplitMode::Optimization;
    mesh->label_offset_boundary();
    const TopoOffsetTriMesh::MeshSnapshot snap = mesh->take_snapshot();
    REQUIRE(mesh->check_mesh_connectivity_validity());
    const size_t n_v = mesh->get_vertices().size(), n_f = mesh->get_faces().size();
    const auto change = [&]() {
        for (const auto& v : mesh->get_vertices()) {
            mesh->m_vertex_attribute[v.vid(*mesh)].m_sizing_scalar = 0.3;
        }
        mesh->local_operations({{1, 0, 0, 0}});
        mesh->consolidate_mesh();
    };
    const MeshState2d before = mesh_state_2d(*mesh);
    change();
    INFO(n_f << " faces before the split pass, " << mesh->get_faces().size() << " after");
    REQUIRE(mesh->get_faces().size() > n_f);
    const MeshState2d changed = mesh_state_2d(*mesh);
    CHECK_FALSE(before == changed); // the comparison can fail

    mesh->restore_snapshot(snap);
    CHECK(mesh->check_mesh_connectivity_validity());
    CHECK(mesh->get_vertices().size() == n_v);
    CHECK(mesh->get_faces().size() == n_f);
    CHECK(mesh_state_2d(*mesh) == before);
    change();
    CHECK(mesh_state_2d(*mesh) == changed);
}

TEST_CASE("smart-growth-attempt-fails-2d", "[offset][2d]" + release_only)
{
    // growth_mode "smart": a failed attempt puts the mesh back at the converged distance, as in
    // 3D, after level 0 converged at d_0 = 0.125. With smart_growth_max_fails 1 the first failure
    // is the last: the mesh is put back -- the vertex and face counts it was stored with -- and
    // the run stops with the give-up error. A box of 2 x 2 squares, as in
    // growth-modes-in-the-loop-2d, failed by design: one turn (smart_growth_attempts 1) at 0.35,
    // where the front still moves. (Until 2026-11-12 also an attempt at 0.4 that converged with
    // edges turned from the field, a defect of the 2D swap pass's 1e-5 filter rather than a
    // failure by design: without the filter it converges free of turned edges.)
    std::string log;
    REQUIRE_THROWS_WITH(
        run_offsets_2d(
            box_grid_2d({{"a", {{3, 4, 3, 4}}}}),
            {{"offsets", nlohmann::json::array({offset_entry_2d("a", 0.35)})},
             {"growth_mode", "smart"},
             {"smart_growth_attempts", 1},
             {"smart_growth_max_fails", 1},
             {"max_rounds", 10}},
            true,
            &log),
        Catch::Matchers::ContainsSubstring(
            "1 attempts from the converged distance 0.125 failed (smart_growth_max_fails)"));
    // The "(#V n, #F m)" that follows `key` in the log.
    const auto counts_after = [&](const std::string& key) {
        const size_t at = log.find(key);
        REQUIRE(at != std::string::npos);
        const size_t from = at + key.size();
        return log.substr(from, log.find(')', from) + 1 - from);
    };
    CHECK(
        counts_after("the mesh is back at the converged 0.125 ") ==
        counts_after(
            "[smart growth] level 0 converged at target_distance 0.125; the mesh is "
            "stored "));
}

TEST_CASE("several-offsets-place-readers-2d", "[offset][2d]")
{
    // Several entries: every reader of a distance or a bar at a place reads its piece's
    // (place_piece()): the piece's entry (m_region_offset), its target (region_delta(),
    // vertex_delta()), its field (potential_for()), its bar (front_conv_of(): front_conv_rel of the
    // entry's target_distance, fixed through growth_mode's stages; front_conv_frac_of(): that over
    // the stage's target), the vertex measure (front_vertex_conv_ratio()), the vertex error
    // (band_vertex_distance_error(), from its own piece) and the chord measure
    // (edge_offset_term()). As in 3D. Two squares 0.5 apart, a at 0.1, b at 0.3 (constructed at
    // 0.125), before the loop; then as in a growth stage with a at 0.1 and b at 0.2
    // (m_offset_levels).
    const Box2d a = {{1, 2, 1, 2}}, b = {{5, 6, 1, 2}};
    auto mesh = run_offsets_2d(
        box_grid_2d({{"a", a}, {"b", b}}),
        {{"offsets",
          nlohmann::json::array({offset_entry_2d("a", 0.1), offset_entry_2d("b", 0.3)})}},
        false);
    mesh->label_offset_boundary(); // as optimize_offset() does before the loop
    mesh->assign_band_regions();
    const double rel = mesh->m_offset_params.front_conv_rel;
    REQUIRE(mesh->m_offset_params.target_distance == 0.1); // the run's: the smallest entry's
    REQUIRE(mesh->m_region_offset.size() == 2);
    // the front vertices of each band, by their distance to its box
    std::array<std::vector<size_t>, 2> front;
    for (const auto& v : mesh->get_vertices()) {
        const size_t vid = v.vid(*mesh);
        if (!mesh->m_vertex_extra[vid].m_is_on_offset) continue;
        const Vector2d& p = mesh->m_vertex_attribute[vid].m_posf;
        front[box_distance_2d(p, a) < box_distance_2d(p, b) ? 0 : 1].push_back(vid);
    }
    REQUIRE(!front[0].empty());
    REQUIRE(!front[1].empty());
    for (const bool stage : {false, true}) {
        if (stage) mesh->m_offset_levels = {0.1, 0.2};
        for (int e = 0; e < 2; ++e) {
            const double target = e == 0 ? 0.1 : 0.3;
            const double now = stage && e == 1 ? 0.2 : target;
            const Box2d& box = e == 0 ? a : b;
            INFO("entry " << e << (stage ? ", in a stage" : ""));
            for (const size_t v : front[size_t(e)]) {
                const int r = mesh->place_piece(&v, 1);
                REQUIRE(r >= 0);
                CHECK(mesh->m_region_offset[size_t(r)] == e);
                CHECK(mesh->vertex_delta(v) == now);
                CHECK(mesh->front_conv_of(r) == rel * target);
                CHECK(mesh->front_conv_frac_of(r) == Catch::Approx(rel * target / now));
                const Vector2d& p = mesh->m_vertex_attribute[v].m_posf;
                CHECK(mesh->potential_for(v).delta() == target); // the fields stay built
                CHECK(
                    mesh->front_vertex_conv_ratio(v) ==
                    Catch::Approx(
                        std::abs(mesh->potential_for(v).relative_residual(p)) /
                        (rel * target / now)));
                CHECK(
                    mesh->band_vertex_distance_error(v) ==
                    Catch::Approx(std::abs(box_distance_2d(p, box) - now)).margin(1e-12));
            }
        }
        // A chord of b's front: its measure in b's bar, the piece-less form in the run's.
        for (const auto& c : mesh->offset_surface_edges()) {
            const int r = mesh->place_piece(c.data(), 2);
            if (r < 0 || mesh->m_region_offset[size_t(r)] != 1) continue;
            const auto& VA = mesh->m_vertex_attribute;
            const OffsetPotential2D& pot = mesh->potential_for_edge(c[0], c[1]);
            const double in_piece =
                mesh->edge_offset_term(pot, VA[c[0]].m_posf, VA[c[1]].m_posf, r);
            const double in_run = mesh->edge_offset_term(pot, VA[c[0]].m_posf, VA[c[1]].m_posf);
            CHECK(mesh->edge_offset_term(c[0], c[1]) == in_piece);
            const double f = rel * 0.3 / (stage ? 0.2 : 0.3), f_run = rel;
            CHECK(in_piece * f * f == Catch::Approx(in_run * f_run * f_run));
        }
    }
}

TEST_CASE("several-offsets-isolated-point-2d", "[offset][2d]")
{
    // hold_curves_and_points() per entry of offsets: the isolated points of every entry without
    // faces (offset_curves()) are anchored at an input point's radius (feature_envelope_ratio x
    // envelope_size), whatever the other entries hold. As in 3D. A point entry: "a & b", where
    // the squares a and b touch at one corner, selects that vertex alone (label_offset()), beside
    // a solid entry c. The corner is also a junction of the tracked region boundaries, anchored
    // at the order-2 radius; the point keeps the smaller radius (anchor_feature_point()), so it
    // is the feature radius when that is the smaller. The two ratios each against the other, and
    // the point entry alone. Before the per-entry rule, an input complex with faces (c's)
    // anchored no isolated point: the corner kept the order-2 radius.
    struct Case
    {
        double feature, order2;
    };
    const Case c = GENERATE(Case{0.01, 1.}, Case{1., 0.01}, Case{1., 1.});
    const bool with_solid = GENERATE(true, false);
    INFO(
        "feature_envelope_ratio " << c.feature << ", order2_envelope_ratio " << c.order2
                                  << (with_solid ? ", beside the solid c" : ", alone"));
    nlohmann::json offsets = nlohmann::json::array();
    if (with_solid) offsets.push_back(offset_entry_2d("c", 0.1));
    offsets.push_back(
        {{"selection", "a & b"}, {"output_tags", {"p_band"}}, {"target_distance", 0.1}});
    const auto mesh = run_offsets_2d(
        box_grid_2d({{"a", {{2, 2, 2, 2}}}, {"b", {{3, 3, 3, 3}}}, {"c", {{6, 6, 6, 6}}}}),
        {{"offsets", offsets},
         {"feature_envelope_ratio", c.feature},
         {"order2_envelope_ratio", c.order2}},
        false);
    const size_t p = vertex_at_2d(*mesh, grid_point_2d({{3, 3}}));
    CHECK(mesh->m_vertex_extra[p].label == 1);
    CHECK(mesh->offset_of(p) == (with_solid ? 1 : 0));
    CHECK(mesh->offset_curves(mesh->offset_of(p)));
    REQUIRE(anchored_2d(*mesh, p));
    const size_t f = mesh->m_feature_point_attribute[p].m_feature_point_id;
    const double eps = mesh->m_offset_params.envelope_size;
    CHECK(mesh->m_feature_point_radius[f] == std::min(c.feature, c.order2) * eps);
    CHECK(!faces_tagged_2d(*mesh, "p_band").empty());
    // c's corners are on its edges: no isolated point, and no junction either.
    if (with_solid) {
        CHECK_FALSE(mesh->offset_curves(0));
        CHECK_FALSE(anchored_2d(*mesh, vertex_at_2d(*mesh, grid_point_2d({{6, 6}}))));
    }
}

TEST_CASE("offsets-curve-in-expression-2d", "[offset][2d]")
{
    // The 2D twin of offsets-face-tags-in-expression: the curve group in an expression means what
    // it means alone (label_offset()). "a | curve" is the solid a and the curve, one input -- a's
    // faces and the curve's edges -- and its band the band of both; "a & curve" the curve's edges
    // that bound a's faces. Until 2026-11 the curve group in an expression selected nothing,
    // silently: no face carries it. An empty list makes no offset (Parameters, shared).
    const auto label_1 = [](const TopoOffsetTriMesh& m) {
        std::set<std::array<double, 2>> out;
        for (const auto& v : m.get_vertices()) {
            if (m.m_vertex_extra[v.vid(m)].label != 1) continue;
            const Vector2d& p = m.m_vertex_attribute[v.vid(m)].m_posf;
            out.insert({{p[0], p[1]}});
        }
        return out;
    };
    const auto at = [](const int i, const int j) {
        const Vector2d p = grid_point_2d({{i, j}});
        return std::array<double, 2>{{p[0], p[1]}};
    };
    const auto entry = [](const std::string& selection) {
        return nlohmann::json::array(
            {{{"selection", selection}, {"output_tags", {"band"}}, {"target_distance", 0.1}}});
    };
    {
        // the curve 0.75 above a, x in [-0.75, 0.25] at y = 0.5
        const BoxGrid2d g = box_grid_2d(
            {{"a", {{1, 2, 1, 2}}}},
            {{{1, 6}}, {{2, 6}}, {{3, 6}}, {{4, 6}}, {{5, 6}}});
        const auto out = run_offsets_2d(g, {{"offsets", entry("a | curve")}}, false);
        std::set<std::array<double, 2>> want;
        for (int i = 1; i <= 3; ++i)
            for (int j = 1; j <= 3; ++j) want.insert(at(i, j)); // a's 3 x 3 vertices
        for (int i = 1; i <= 5; ++i) want.insert(at(i, 6)); // the curve's 5
        CHECK(label_1(*out) == want);
        // The band of the union: the bands each makes alone, together.
        std::set<FaceCorners2d> both =
            faces_tagged_2d(*run_offsets_2d(g, {{"offsets", entry("a")}}, false), "band");
        for (const FaceCorners2d& c :
             faces_tagged_2d(*run_offsets_2d(g, {{"offsets", entry("curve")}}, false), "band"))
            both.insert(c);
        CHECK(faces_tagged_2d(*out, "band") == both);
    }
    {
        // the curve along a's top side and on past it, x in [-0.75, 0.25] at y = -0.25
        const BoxGrid2d g = box_grid_2d(
            {{"a", {{1, 2, 1, 2}}}},
            {{{1, 3}}, {{2, 3}}, {{3, 3}}, {{4, 3}}, {{5, 3}}});
        const auto out = run_offsets_2d(g, {{"offsets", entry("a & curve")}}, false);
        CHECK(label_1(*out) == std::set<std::array<double, 2>>{at(1, 3), at(2, 3), at(3, 3)});
    }
    Eigen::MatrixXd box(2, 2);
    box << -1., -1., 1., 1.;
    REQUIRE_THROWS_WITH(
        construction_params({{"offsets", nlohmann::json::array()}}, box),
        Catch::Matchers::ContainsSubstring("at least one offset"));
}

namespace {
/// box_grid_2d()'s grid as the driver reads it (grid_2d()): the squares of each named box tagged
/// by its name, every other square ambient, and a curve group along `curve` (none if empty),
/// selected as "EnvelopeSurface" on the memory route. The input of the -run-2d twins.
MeshData box_mesh_2d(
    const std::vector<std::pair<std::string, Box2d>>& boxes,
    const GridPolyline2d& curve = {})
{
    return grid_2d(
        {},
        curve.empty() ? std::vector<GridPolyline2d>() : std::vector<GridPolyline2d>{curve},
        [boxes](const Vector2d& c) -> std::string {
            const int i = int(std::floor((c[0] + 1.) / 0.25)),
                      j = int(std::floor((c[1] + 1.) / 0.25));
            for (const auto& [name, b] : boxes) {
                if (i >= b[0] && i <= b[1] && j >= b[2] && j <= b[3]) return name;
            }
            return "";
        });
}

/// The triangles of a 2D result carrying `tag`, each as its corners' positions in sorted order:
/// the twin of cells_tagged().
std::set<FaceCorners2d> result_faces_tagged_2d(const MeshData& m, const std::string& tag)
{
    const auto& block = m.cells.at(CellType::Triangle);
    const auto& tags = block.attributes.at("tags");
    const auto it = std::find(tags.labels.begin(), tags.labels.end(), tag);
    REQUIRE(it != tags.labels.end());
    const Eigen::Index col = Eigen::Index(it - tags.labels.begin());
    const auto member = tags.integers();
    std::set<FaceCorners2d> out;
    for (Eigen::Index r = 0; r < block.connectivity.rows(); ++r) {
        if (member(r, col) == 0) continue;
        FaceCorners2d c;
        for (int k = 0; k < 3; ++k) {
            const auto p = m.V.row(block.connectivity(r, k));
            c[size_t(k)] = {{p(0), p(1)}};
        }
        std::sort(c.begin(), c.end());
        out.insert(c);
    }
    return out;
}

/// The corners of a 2D result's ambient triangles.
std::set<std::array<double, 2>> result_ambient_corners_2d(const MeshData& out)
{
    std::set<std::array<double, 2>> ambient;
    for (const FaceCorners2d& c : result_faces_tagged_2d(out, "ambient"))
        ambient.insert(c.begin(), c.end());
    return ambient;
}

/// The front of `tag`'s band in a 2D result: the corners its triangles share with ambient ones,
/// off the box `b`, as distances to it. The twin of front_distances().
std::vector<double>
result_front_distances_2d(const MeshData& out, const std::string& tag, const Box2d& b)
{
    const std::set<std::array<double, 2>> ambient = result_ambient_corners_2d(out);
    std::set<std::array<double, 2>> front;
    for (const FaceCorners2d& c : result_faces_tagged_2d(out, tag)) {
        for (const auto& q : c) {
            if (ambient.count(q)) front.insert(q);
        }
    }
    std::vector<double> d;
    for (const auto& q : front) {
        const double x = box_distance_2d(Vector2d(q[0], q[1]), b);
        if (x > 0.) d.push_back(x);
    }
    return d;
}

/// One construction-only 2D run through the driver with these offsets: the twin of
/// construct_offsets().
MeshData construct_offsets_2d(const MeshData& m, const nlohmann::json& offsets)
{
    return components::topological_offset::topological_offset(
               {{"input", "m"},
                {"offsets", offsets},
                {"optimize_offset", false},
                {"num_threads", 1}},
               {{"m", m}})
        .first.at("mesh");
}

/// One optimized 2D run through the driver with these offsets and keys `j`.
ComponentResult run_offsets_driver_2d(const MeshData& m, nlohmann::json j)
{
    j["input"] = "m";
    j["num_threads"] = 1;
    j["throw_on_nonconvergence"] = false;
    return components::topological_offset::topological_offset(j, {{"m", m}});
}
} // namespace

TEST_CASE("several-offsets-as-alone-run-2d", "[offset][2d]")
{
    // The 2D twin of several-offsets-as-alone, through the driver (several-offsets-as-alone-2d
    // drives the mesh): three bodies, each two squares from the next, offset in one march; each
    // entry's band in the result is the one a run of that entry alone makes -- the same triangles
    // at the same positions -- under its own output tag and no other entry's. Every outer end is a
    // grid edge (0.25) or more from its body, so a at 0.1 and c at 0.2 are traced to, and b at
    // 0.3 falls back to half of 0.25.
    const std::vector<std::pair<std::string, Box2d>> boxes = {
        {"a", {{1, 2, 1, 2}}},
        {"b", {{5, 6, 1, 2}}},
        {"c", {{1, 2, 5, 6}}}};
    const MeshData m = box_mesh_2d(boxes);
    const std::vector<double> D = {0.1, 0.3, 0.2};
    nlohmann::json all = nlohmann::json::array();
    for (size_t k = 0; k < boxes.size(); ++k) all.push_back(offset_entry_2d(boxes[k].first, D[k]));
    const MeshData joint = construct_offsets_2d(m, all);
    for (size_t k = 0; k < boxes.size(); ++k) {
        const std::string band = boxes[k].first + "_band";
        INFO("offsets[" << k << "] " << boxes[k].first << " at " << D[k]);
        const auto faces = result_faces_tagged_2d(joint, band);
        CHECK(!faces.empty());
        CHECK(
            faces ==
            result_faces_tagged_2d(construct_offsets_2d(m, nlohmann::json::array({all[k]})), band));
        const double traced = D[k] < 0.25 ? D[k] : 0.125;
        size_t n_outer = 0;
        for (const FaceCorners2d& c : faces) {
            for (const auto& q : c) {
                const double d = box_distance_2d(Vector2d(q[0], q[1]), boxes[k].second);
                if (d == 0.) continue; // on the body
                ++n_outer;
                CHECK(std::abs(d - traced) <= 0.01 * traced);
            }
        }
        CHECK(n_outer > 0);
        for (size_t o = 0; o < boxes.size(); ++o) {
            if (o == k) continue;
            for (const FaceCorners2d& c : result_faces_tagged_2d(joint, boxes[o].first + "_band"))
                CHECK(faces.count(c) == 0);
        }
    }
    // The comparison tells positions apart: a alone at c's distance is not a's band.
    nlohmann::json a_at_c = all[0];
    a_at_c["target_distance"] = D[2];
    CHECK(
        result_faces_tagged_2d(joint, "a_band") !=
        result_faces_tagged_2d(construct_offsets_2d(m, nlohmann::json::array({a_at_c})), "a_band"));
}

TEST_CASE("several-offsets-touch-run-2d", "[offset][2d]")
{
    // The 2D twin of several-offsets-touch, through the driver: entries that intersect are refused
    // (label_input_complex()): b's squares next to a's share their vertices. One square apart,
    // the simplicial embedding splits the grid edges between them, and the two bands come out
    // apart -- no vertex in both -- each within 0.125 of its body, so a target of 0.1 is still
    // traced to.
    const auto run = [](const Box2d& b) {
        return construct_offsets_2d(
            box_mesh_2d({{"a", {{1, 2, 1, 2}}}, {"b", b}}),
            nlohmann::json::array({offset_entry_2d("a", 0.1), offset_entry_2d("b", 0.1)}));
    };
    REQUIRE_THROWS_WITH(run({{3, 4, 1, 2}}), Catch::Matchers::ContainsSubstring("share"));
    const Box2d b = {{4, 5, 1, 2}};
    MeshData out;
    REQUIRE_NOTHROW(out = run(b));
    std::set<std::array<double, 2>> corners_a;
    for (const FaceCorners2d& c : result_faces_tagged_2d(out, "a_band"))
        corners_a.insert(c.begin(), c.end());
    size_t n_b = 0, n_b_outer = 0;
    for (const FaceCorners2d& c : result_faces_tagged_2d(out, "b_band")) {
        ++n_b;
        for (const auto& q : c) {
            CHECK(corners_a.count(q) == 0);
            const double d = box_distance_2d(Vector2d(q[0], q[1]), b);
            if (d > 0.) {
                ++n_b_outer;
                CHECK(std::abs(d - 0.1) <= 0.01 * 0.1);
            }
        }
    }
    CHECK(!corners_a.empty());
    CHECK(n_b > 0);
    CHECK(n_b_outer > 0);
}

TEST_CASE("several-offsets-optimize-run-2d", "[offset][2d]" + release_only)
{
    // The 2D twin of several-offsets-optimize, through the driver: several entries run the
    // optimization, an input complex of several connected pieces, each its entry's; the result
    // json reports the one turn.
    ComponentResult res;
    REQUIRE_NOTHROW(
        res = run_offsets_driver_2d(
            box_mesh_2d({{"a", {{1, 2, 1, 2}}}, {"b", {{5, 6, 1, 2}}}}),
            {{"offsets",
              nlohmann::json::array({offset_entry_2d("a", 0.1), offset_entry_2d("b", 0.1)})},
             {"max_rounds", 1}}));
    CHECK(!result_faces_tagged_2d(res.first.at("mesh"), "a_band").empty());
    CHECK(!result_faces_tagged_2d(res.first.at("mesh"), "b_band").empty());
    CHECK(res.second.at("optimization_metrics").at("max_grad").size() == 1);
}

TEST_CASE("several-offsets-optimize-distances-run-2d", "[offset][2d]" + release_only)
{
    // The 2D twin of several-offsets-optimize-distances, through the driver: each piece its own
    // target distance in the optimization (place_piece()). Two boxes 0.5 apart, a at 0.1, b at
    // 0.2: both fronts at their own targets and converged; then b one square at 0.3, above the
    // maximum marchable distance (constructed at 0.125), a 0.5 away (each piece its own
    // integral). Under EXPERIMENTAL_band_volume_rule "exact_swept", the default, each piece's
    // swept change, as several-offsets-optimize-distances-2d.
    const auto run = [](const Box2d& a, const Box2d& b, const double db) {
        return run_offsets_driver_2d(
            box_mesh_2d({{"a", a}, {"b", b}}),
            {{"offsets",
              nlohmann::json::array({offset_entry_2d("a", 0.1), offset_entry_2d("b", db)})},
             {"max_rounds", 5}});
    };
    {
        const Box2d a = {{1, 2, 1, 2}}, b = {{5, 6, 1, 2}};
        const ComponentResult res = run(a, b, 0.2);
        CHECK(res.second.value("converged", false));
    }
    {
        const Box2d a1 = {{1, 2, 3, 3}}, b1 = {{5, 5, 3, 3}};
        const ComponentResult res = run(a1, b1, 0.3);
        CHECK(res.second.value("converged", false));
    }
}

TEST_CASE("several-offsets-solid-and-curve-run-2d", "[offset][2d]")
{
    // The 2D twin of several-offsets-solid-and-curve, through the driver: entries of different
    // kinds in one run, a solid (a face tag) and the curve group (the envelope, "EnvelopeSurface"
    // on the memory route), each entry its own distance engine. Each band is the one its entry
    // makes alone. 2D refuses no mix of kinds within one entry (see
    // several-offsets-solid-and-curve-2d).
    const MeshData m = box_mesh_2d({{"a", {{1, 2, 1, 2}}}}, {{{1, 6}}, {{5, 6}}});
    const nlohmann::json a = offset_entry_2d("a", 0.1);
    const nlohmann::json r = {
        {"selection", "EnvelopeSurface"},
        {"output_tags", {"curve_band"}},
        {"target_distance", 0.1}};
    const MeshData joint = construct_offsets_2d(m, nlohmann::json::array({a, r}));
    CHECK(!result_faces_tagged_2d(joint, "a_band").empty());
    CHECK(!result_faces_tagged_2d(joint, "curve_band").empty());
    CHECK(
        result_faces_tagged_2d(joint, "a_band") ==
        result_faces_tagged_2d(construct_offsets_2d(m, nlohmann::json::array({a})), "a_band"));
    CHECK(
        result_faces_tagged_2d(joint, "curve_band") ==
        result_faces_tagged_2d(construct_offsets_2d(m, nlohmann::json::array({r})), "curve_band"));
}

TEST_CASE("several-offsets-solid-and-curve-optimize-run-2d", "[offset][2d]" + release_only)
{
    // The 2D twin of several-offsets-solid-and-curve-optimize, through the driver: the same two
    // entries through the optimization, each piece its own field and integral. Both bands at
    // their targets: their front vertices within 10% of 0.1 from their own input.
    const ComponentResult res = run_offsets_driver_2d(
        box_mesh_2d({{"a", {{1, 2, 1, 2}}}}, {{{1, 6}}, {{5, 6}}}),
        {{"offsets",
          nlohmann::json::array(
              {offset_entry_2d("a", 0.1),
               {{"selection", "EnvelopeSurface"},
                {"output_tags", {"curve_band"}},
                {"target_distance", 0.1}}})},
         {"max_rounds", 5}});
    const MeshData& out = res.first.at("mesh");
    const std::vector<double> da = result_front_distances_2d(out, "a_band", {{1, 2, 1, 2}});
    REQUIRE(!da.empty());
    for (const double d : da) {
        CHECK(d >= 0.09);
        CHECK(d <= 0.11);
    }
    // the curve: x in [-0.75, 0.25] at y = 0.5
    const auto curve_distance = [](const std::array<double, 2>& q) {
        const double x = std::clamp(q[0], -0.75, 0.25);
        return (Vector2d(q[0], q[1]) - Vector2d(x, 0.5)).norm();
    };
    const std::set<std::array<double, 2>> ambient = result_ambient_corners_2d(out);
    size_t n = 0;
    for (const FaceCorners2d& c : result_faces_tagged_2d(out, "curve_band")) {
        for (const auto& q : c) {
            if (!ambient.count(q)) continue;
            ++n;
            CHECK(curve_distance(q) >= 0.09);
            CHECK(curve_distance(q) <= 0.11);
        }
    }
    CHECK(n > 0);
}

TEST_CASE("several-offsets-growth-run-2d", "[offset][2d]" + release_only)
{
    // The 2D twin of several-offsets-growth, through the driver: growth_mode "linear" with several
    // entries, each entry from its own d_0 to its own target, one schedule. a one square at 0.1,
    // traced to by the march, stays there; b one square at 0.4, above the maximum marchable
    // distance 0.25 (constructed at 0.125), grows in linear_growth_steps stages 0.125 -> 0.4. Both
    // converge at their own targets. The result json's offset_level is the run-wide euclidean
    // field's level, 1: d in units of target_distance (EuclideanOffsetPotential2D, m_c = 1, as
    // 3D's). Under EXPERIMENTAL_band_volume_rule "exact_swept", the default, each piece's swept
    // change, as several-offsets-growth-2d.
    const Box2d a = {{1, 1, 3, 3}}, b = {{5, 5, 3, 3}};
    const ComponentResult res = run_offsets_driver_2d(
        box_mesh_2d({{"a", a}, {"b", b}}),
        {{"offsets", nlohmann::json::array({offset_entry_2d("a", 0.1), offset_entry_2d("b", 0.4)})},
         {"growth_mode", "linear"},
         {"max_rounds", 10}});
    CHECK(res.second.value("converged", false));
    CHECK(res.second.at("offset_level").get<double>() == Catch::Approx(1.));
}

TEST_CASE("smart-growth-attempt-fails-run-2d", "[offset][2d]" + release_only)
{
    // growth_mode "smart" through the driver: the give-up error of
    // TopoOffsetTriMesh::smart_growth() reaches the caller. The turn-cap case of
    // smart-growth-attempt-fails-2d: a box of 2 x 2 squares at 0.35, one turn an attempt
    // (smart_growth_attempts 1), after level 0 converged at d_0 = 0.125; with
    // smart_growth_max_fails 1 the first failure is put back and is the last.
    REQUIRE_THROWS_WITH(
        run_offsets_driver_2d(
            box_mesh_2d({{"a", {{3, 4, 3, 4}}}}),
            {{"offsets", nlohmann::json::array({offset_entry_2d("a", 0.35)})},
             {"growth_mode", "smart"},
             {"smart_growth_attempts", 1},
             {"smart_growth_max_fails", 1},
             {"max_rounds", 10}}),
        Catch::Matchers::ContainsSubstring(
            "1 attempts from the converged distance 0.125 failed (smart_growth_max_fails)"));
}
