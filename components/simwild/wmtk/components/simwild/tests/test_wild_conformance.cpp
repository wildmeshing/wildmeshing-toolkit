#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/simwild/SimWildMeshTri.hpp>
#include <wmtk/components/simwild/SimWildMesh.h>
#include <wmtk/components/tetwild/TetWildMesh.h>
#include <wmtk/components/triwild/TriWildMesh.h>

#include <algorithm>
#include <array>
#include <map>
#include <memory>
#include <vector>

namespace {

using wmtk::TriOptimizerMesh;
using wmtk::Vector2d;

const std::vector<std::array<size_t, 3>> kQuadFaces = {{{0, 1, 2}}, {{0, 2, 3}}};
const std::array<Vector2d, 4> kQuadVertices = {
    Vector2d(0, 0), Vector2d(1, 0), Vector2d(1, 1), Vector2d(0, 0.2)};
const std::set<int64_t> kHomogeneousTag = {7};

template <typename Mesh>
void init_homogeneous_quad(Mesh& mesh)
{
    mesh.init(4, kQuadFaces);
    mesh.m_vertex_attribute.resize(4);
    mesh.m_edge_attribute.resize(6);
    mesh.m_face_attribute.resize(2);

    for (size_t vid = 0; vid < kQuadVertices.size(); ++vid) {
        auto& attr = mesh.m_vertex_attribute[vid];
        attr.m_posf = kQuadVertices[vid];
        attr.m_pos = wmtk::to_rational(attr.m_posf);
        attr.m_is_rounded = true;
        attr.m_is_on_surface = false;
        attr.m_sizing_scalar = 1.;
    }
    for (const auto& f : mesh.get_faces()) {
        auto& attr = mesh.m_face_attribute[f.fid(mesh)];
        attr.m_quality = mesh.get_quality(f);
        attr.tags = kHomogeneousTag;
    }
}

template <typename Mesh>
std::vector<std::array<size_t, 3>> canonical_faces(const Mesh& mesh)
{
    std::vector<std::array<size_t, 3>> faces;
    for (const auto& f : mesh.get_faces()) {
        auto vids = mesh.oriented_tri_vids(f);
        std::sort(vids.begin(), vids.end());
        faces.push_back(vids);
    }
    std::sort(faces.begin(), faces.end());
    return faces;
}

template <typename Mesh>
std::map<std::array<size_t, 3>, double> qualities_by_face(const Mesh& mesh)
{
    std::map<std::array<size_t, 3>, double> result;
    for (const auto& f : mesh.get_faces()) {
        auto vids = mesh.oriented_tri_vids(f);
        std::sort(vids.begin(), vids.end());
        result.emplace(vids, mesh.m_face_attribute[f.fid(mesh)].m_quality);
    }
    return result;
}

template <typename Mesh>
size_t newest_vertex(const Mesh& mesh)
{
    size_t newest = 0;
    for (const auto& v : mesh.get_vertices()) newest = std::max(newest, v.vid(mesh));
    return newest;
}

} // namespace

TEST_CASE(
    "tag-homogeneous SimWild follows TriWild's serial edge-swap pass",
    "[simwild][triwild][conformance][serial]")
{
    wmtk::components::triwild::Parameters tri_params;
    tri_params.init(Vector2d(-1, -1), Vector2d(2, 2));
    wmtk::components::simwild::Parameters sim_params;
    sim_params.init(Vector2d(-1, -1), Vector2d(2, 2));

    wmtk::components::triwild::TriWildMesh tri(tri_params, tri_params.eps, 0);
    wmtk::components::simwild::tri::SimWildMeshTri sim(sim_params, sim_params.eps, 0);
    init_homogeneous_quad(tri);
    init_homogeneous_quad(sim);

    REQUIRE(canonical_faces(tri) == canonical_faces(sim));
    const auto tri_diagonal = std::get<0>(tri.tuple_from_edge(std::array<size_t, 2>{{0, 2}}));
    const auto sim_diagonal = std::get<0>(sim.tuple_from_edge(std::array<size_t, 2>{{0, 2}}));
    REQUIRE(sim.swap_weight(sim_diagonal) == tri.swap_weight(tri_diagonal));

    // Intentionally serial and ordered: TriWild is the oracle, then SimWild starts from its
    // untouched identical mesh. Running concurrently would turn a disagreement into a race.
    const size_t tri_swaps = tri.swap_all_edges();
    const size_t sim_swaps = sim.swap_all_edges();

    CHECK(tri_swaps == 1);
    CHECK(sim_swaps == tri_swaps);
    CHECK(canonical_faces(sim) == canonical_faces(tri));

    const auto tri_qualities = qualities_by_face(tri);
    const auto sim_qualities = qualities_by_face(sim);
    REQUIRE(sim_qualities.size() == tri_qualities.size());
    for (const auto& [face, quality] : tri_qualities) {
        CHECK(sim_qualities.at(face) == quality);
    }
    for (size_t vid = 0; vid < kQuadVertices.size(); ++vid) {
        CHECK(
            (sim.m_vertex_attribute[vid].m_posf - tri.m_vertex_attribute[vid].m_posf)
                .squaredNorm() == 0.);
        CHECK(bool(sim.m_vertex_attribute[vid].m_pos == tri.m_vertex_attribute[vid].m_pos));
    }
    for (const auto& f : sim.get_faces()) {
        CHECK(sim.m_face_attribute[f.fid(sim)].tags == kHomogeneousTag);
    }
}

TEST_CASE(
    "tag-homogeneous SimWild follows TriWild's serial split and collapse",
    "[simwild][triwild][conformance][serial]")
{
    wmtk::components::triwild::Parameters tri_params;
    tri_params.init(Vector2d(-1, -1), Vector2d(2, 2));
    wmtk::components::simwild::Parameters sim_params;
    sim_params.init(Vector2d(-1, -1), Vector2d(2, 2));

    wmtk::components::triwild::TriWildMesh tri(tri_params, tri_params.eps, 0);
    wmtk::components::simwild::tri::SimWildMeshTri sim(sim_params, sim_params.eps, 0);
    init_homogeneous_quad(tri);
    init_homogeneous_quad(sim);

    std::vector<wmtk::TriMesh::Tuple> tri_children;
    std::vector<wmtk::TriMesh::Tuple> sim_children;
    const auto tri_diagonal = std::get<0>(tri.tuple_from_edge({{0, 2}}));
    const auto sim_diagonal = std::get<0>(sim.tuple_from_edge({{0, 2}}));

    // Run the oracle operation first, then SimWild from its still-untouched identical mesh.
    REQUIRE(tri.split_edge(tri_diagonal, tri_children));
    REQUIRE(sim.split_edge(sim_diagonal, sim_children));

    const size_t tri_mid = newest_vertex(tri);
    const size_t sim_mid = newest_vertex(sim);
    REQUIRE(tri_mid == sim_mid);
    CHECK(canonical_faces(sim) == canonical_faces(tri));
    CHECK((sim.m_vertex_attribute[sim_mid].m_posf - tri.m_vertex_attribute[tri_mid].m_posf)
              .squaredNorm() == 0.);
    CHECK(bool(sim.m_vertex_attribute[sim_mid].m_pos == tri.m_vertex_attribute[tri_mid].m_pos));
    for (const auto& f : sim.get_faces()) {
        CHECK(sim.m_face_attribute[f.fid(sim)].tags == kHomogeneousTag);
    }

    std::vector<wmtk::TriMesh::Tuple> tri_after_collapse;
    std::vector<wmtk::TriMesh::Tuple> sim_after_collapse;
    const auto tri_half = std::get<0>(tri.tuple_from_edge({{tri_mid, 0}}));
    const auto sim_half = std::get<0>(sim.tuple_from_edge({{sim_mid, 0}}));

    REQUIRE(tri.collapse_edge(tri_half, tri_after_collapse));
    REQUIRE(sim.collapse_edge(sim_half, sim_after_collapse));
    CHECK(canonical_faces(sim) == canonical_faces(tri));
    CHECK(qualities_by_face(sim) == qualities_by_face(tri));
    for (const auto& f : sim.get_faces()) {
        CHECK(sim.m_face_attribute[f.fid(sim)].tags == kHomogeneousTag);
    }
}

namespace {

using wmtk::Vector3d;

const std::vector<std::array<size_t, 4>> kTetRing = {
    {{0, 1, 2, 3}}, {{0, 1, 3, 4}}, {{0, 1, 4, 2}}};
const std::array<Vector3d, 5> kTetRingVertices = {
    Vector3d(0, 0, -1000),
    Vector3d(0, 0, 1000),
    Vector3d(1, 0, 0),
    Vector3d(-0.5, 0.8660254037844386, 0),
    Vector3d(-0.5, -0.8660254037844386, 0)};

template <typename Mesh>
void init_homogeneous_tet_ring(Mesh& mesh)
{
    mesh.init(5, kTetRing);
    mesh.m_vertex_attribute.resize(5);
    mesh.m_face_attribute.resize(12);
    mesh.m_tet_attribute.resize(3);

    for (size_t vid = 0; vid < kTetRingVertices.size(); ++vid) {
        auto& attr = mesh.m_vertex_attribute[vid];
        attr.m_posf = kTetRingVertices[vid];
        attr.m_pos = wmtk::to_rational(attr.m_posf);
        attr.m_is_rounded = true;
        attr.m_is_on_surface = false;
        attr.m_order = 0;
        attr.m_sizing_scalar = 1.;
    }
    // The operation recomputes the new qualities. Inflating the identical old qualities makes
    // this fixture isolate topology/tag conformance from the particular AMIPS threshold.
    for (size_t tid = 0; tid < kTetRing.size(); ++tid) {
        mesh.m_tet_attribute[tid].m_quality = 1e40;
    }
}

void set_surface_face(
    wmtk::components::tetwild::TetWildMesh& mesh,
    const std::array<size_t, 3>& vids)
{
    const auto [_, fid] = mesh.tuple_from_face(vids);
    REQUIRE(fid != static_cast<size_t>(-1));
    mesh.m_face_attribute[fid].m_is_surface_fs = true;
}

void check_tag_interface_invariant(const wmtk::components::simwild::SimWildMesh& mesh)
{
    for (const auto& face : mesh.get_faces()) {
        const auto opposite = face.switch_tetrahedron(mesh);
        const bool expected =
            opposite.has_value() &&
            mesh.m_tet_attribute[face.tid(mesh)].tags !=
                mesh.m_tet_attribute[opposite->tid(mesh)].tags;
        CHECK(mesh.m_face_attribute[face.fid(mesh)].m_is_surface_fs == expected);
    }
}

template <typename Mesh>
std::vector<std::array<size_t, 4>> canonical_tets(const Mesh& mesh)
{
    std::vector<std::array<size_t, 4>> result;
    for (const auto& t : mesh.get_tets()) {
        auto vids = mesh.oriented_tet_vids(t);
        std::sort(vids.begin(), vids.end());
        result.push_back(vids);
    }
    std::sort(result.begin(), result.end());
    return result;
}

template <typename Mesh>
std::map<std::array<size_t, 4>, double> qualities_by_tet(const Mesh& mesh)
{
    std::map<std::array<size_t, 4>, double> result;
    for (const auto& t : mesh.get_tets()) {
        auto vids = mesh.oriented_tet_vids(t);
        std::sort(vids.begin(), vids.end());
        result.emplace(vids, mesh.cell_quality(t.tid(mesh)));
    }
    return result;
}

} // namespace

TEST_CASE(
    "tag-homogeneous SimWild follows TetWild's serial 3-to-2 swap pass",
    "[simwild][tetwild][conformance][serial]")
{
    wmtk::components::tetwild::Parameters tet_params;
    tet_params.init(Vector3d(-2, -2, -2001), Vector3d(2, 2, 2001));
    wmtk::components::simwild::Parameters sim_params;
    sim_params.epsr = tet_params.epsr;
    sim_params.stop_energy = tet_params.stop_energy;
    sim_params.preserve_topology = tet_params.preserve_topology;
    sim_params.init(Vector3d(-2, -2, -2001), Vector3d(2, 2, 2001));

    wmtk::components::tetwild::TetWildMesh tet(tet_params, nullptr, 0);
    wmtk::components::simwild::SimWildMesh sim(sim_params, sim_params.eps, 0);
    init_homogeneous_tet_ring(tet);
    init_homogeneous_tet_ring(sim);
    for (size_t tid = 0; tid < kTetRing.size(); ++tid) {
        sim.m_tet_attribute[tid].tags = kHomogeneousTag;
    }

    REQUIRE(canonical_tets(tet) == canonical_tets(sim));

    // The oracle always runs first; SimWild starts only after TetWild has completed its pass.
    const size_t tet_swaps = tet.swap_all_edges_32();
    const size_t sim_swaps = sim.swap_all_edges_32();

    CHECK(tet_swaps == 1);
    CHECK(sim_swaps == tet_swaps);
    CHECK(canonical_tets(sim) == canonical_tets(tet));
    CHECK(canonical_tets(sim).size() == 2);

    const auto tet_qualities = qualities_by_tet(tet);
    const auto sim_qualities = qualities_by_tet(sim);
    REQUIRE(sim_qualities.size() == tet_qualities.size());
    for (const auto& [cell, quality] : tet_qualities) CHECK(sim_qualities.at(cell) == quality);

    for (size_t vid = 0; vid < 5; ++vid) {
        CHECK((sim.m_vertex_attribute[vid].m_posf - tet.m_vertex_attribute[vid].m_posf)
                  .squaredNorm() == 0.);
        CHECK(bool(sim.m_vertex_attribute[vid].m_pos == tet.m_vertex_attribute[vid].m_pos));
    }
    for (const auto& t : sim.get_tets()) {
        CHECK(sim.m_tet_attribute[t.tid(sim)].tags == kHomogeneousTag);
    }
}

TEST_CASE(
    "tag-homogeneous SimWild follows TetWild's serial split and collapse",
    "[simwild][tetwild][conformance][serial]")
{
    wmtk::components::tetwild::Parameters tet_params;
    tet_params.init(Vector3d(-2, -2, -2001), Vector3d(2, 2, 2001));
    wmtk::components::simwild::Parameters sim_params;
    sim_params.epsr = tet_params.epsr;
    sim_params.stop_energy = tet_params.stop_energy;
    sim_params.preserve_topology = tet_params.preserve_topology;
    sim_params.init(Vector3d(-2, -2, -2001), Vector3d(2, 2, 2001));

    wmtk::components::tetwild::TetWildMesh tet(tet_params, nullptr, 0);
    wmtk::components::simwild::SimWildMesh sim(sim_params, sim_params.eps, 0);
    init_homogeneous_tet_ring(tet);
    init_homogeneous_tet_ring(sim);
    for (size_t tid = 0; tid < kTetRing.size(); ++tid) {
        sim.m_tet_attribute[tid].tags = kHomogeneousTag;
    }

    std::vector<wmtk::TetMesh::Tuple> tet_children;
    std::vector<wmtk::TetMesh::Tuple> sim_children;
    const auto tet_axis = tet.tuple_from_edge({{0, 1}});
    const auto sim_axis = sim.tuple_from_edge({{0, 1}});

    // TetWild completes first; only then is the same operation applied to SimWild.
    REQUIRE(tet.split_edge(tet_axis, tet_children));
    REQUIRE(sim.split_edge(sim_axis, sim_children));

    const size_t tet_mid = newest_vertex(tet);
    const size_t sim_mid = newest_vertex(sim);
    REQUIRE(tet_mid == sim_mid);
    CHECK(canonical_tets(sim) == canonical_tets(tet));
    CHECK((sim.m_vertex_attribute[sim_mid].m_posf - tet.m_vertex_attribute[tet_mid].m_posf)
              .squaredNorm() == 0.);
    CHECK(bool(sim.m_vertex_attribute[sim_mid].m_pos == tet.m_vertex_attribute[tet_mid].m_pos));
    // This is the behavior that used to differ: SimWild now takes the exact TetWild edge
    // order for the new vertex instead of assigning its own 1/2 heuristic.
    CHECK(sim.m_vertex_attribute[sim_mid].m_order == tet.m_vertex_attribute[tet_mid].m_order);
    for (const auto& t : sim.get_tets()) {
        CHECK(sim.m_tet_attribute[t.tid(sim)].tags == kHomogeneousTag);
    }

    // Isolate collapse behavior from this deliberately extreme ring's AMIPS value. Both
    // implementations see the same inflated pre-collapse qualities, as in the swap fixture.
    for (const auto& t : tet.get_tets()) tet.m_tet_attribute[t.tid(tet)].m_quality = 1e40;
    for (const auto& t : sim.get_tets()) sim.m_tet_attribute[t.tid(sim)].m_quality = 1e40;

    std::vector<wmtk::TetMesh::Tuple> tet_after_collapse;
    std::vector<wmtk::TetMesh::Tuple> sim_after_collapse;
    auto tet_half = tet.tuple_from_edge({{tet_mid, 0}});
    auto sim_half = sim.tuple_from_edge({{sim_mid, 0}});
    if (tet_half.vid(tet) != tet_mid) tet_half = tet_half.switch_vertex(tet);
    if (sim_half.vid(sim) != sim_mid) sim_half = sim_half.switch_vertex(sim);

    REQUIRE(tet.collapse_edge(tet_half, tet_after_collapse));
    REQUIRE(sim.collapse_edge(sim_half, sim_after_collapse));
    CHECK(canonical_tets(sim) == canonical_tets(tet));
    CHECK(qualities_by_tet(sim) == qualities_by_tet(tet));
    for (const auto& t : sim.get_tets()) {
        CHECK(sim.m_tet_attribute[t.tid(sim)].tags == kHomogeneousTag);
    }
}

TEST_CASE(
    "SimWild copies TetWild's exact new-vertex order while preserving tag interfaces",
    "[simwild][tetwild][conformance][split][topology][tags]")
{
    wmtk::components::tetwild::Parameters tet_params;
    tet_params.preserve_topology = true;
    tet_params.init(Vector3d(-2, -2, -2001), Vector3d(2, 2, 2001));
    wmtk::components::simwild::Parameters sim_params;
    sim_params.preserve_topology = true;
    sim_params.init(Vector3d(-2, -2, -2001), Vector3d(2, 2, 2001));

    const std::vector<Eigen::Vector3i> interface_faces = {
        Eigen::Vector3i(0, 1, 2), Eigen::Vector3i(0, 1, 3), Eigen::Vector3i(0, 1, 4)};
    auto tet_envelope = std::make_shared<wmtk::SampleEnvelope>();
    tet_envelope->init(
        std::vector<Vector3d>(kTetRingVertices.begin(), kTetRingVertices.end()),
        interface_faces,
        10.);

    wmtk::components::tetwild::TetWildMesh tet(tet_params, tet_envelope, 0);
    wmtk::components::simwild::SimWildMesh sim(sim_params, 10., 0);
    init_homogeneous_tet_ring(tet);
    init_homogeneous_tet_ring(sim);

    set_surface_face(tet, {{0, 1, 2}});
    set_surface_face(tet, {{0, 1, 3}});
    set_surface_face(tet, {{0, 1, 4}});
    for (size_t vid = 0; vid < kTetRingVertices.size(); ++vid) {
        tet.m_vertex_attribute[vid].m_is_on_surface = true;
    }
    tet.init_vertex_order();

    for (size_t tid = 0; tid < kTetRing.size(); ++tid) {
        sim.m_tet_attribute[tid].tags = {static_cast<int64_t>(tid + 1)};
    }
    Eigen::MatrixXd V(5, 3);
    for (size_t vid = 0; vid < kTetRingVertices.size(); ++vid) V.row(vid) = kTetRingVertices[vid];
    Eigen::MatrixXi F(3, 3);
    for (size_t i = 0; i < interface_faces.size(); ++i) F.row(i) = interface_faces[i];
    sim.init_envelope(V, F, false);
    sim.init_surfaces_and_boundaries();
    sim.init_vertex_order();

    const auto tet_axis = tet.tuple_from_edge({{0, 1}});
    const auto sim_axis = sim.tuple_from_edge({{0, 1}});
    const size_t expected_order = tet.get_order_of_edge({{0, 1}});
    REQUIRE(expected_order > 1);

    std::vector<wmtk::TetMesh::Tuple> tet_children;
    std::vector<wmtk::TetMesh::Tuple> sim_children;
    REQUIRE(tet.split_edge(tet_axis, tet_children));
    REQUIRE(sim.split_edge(sim_axis, sim_children));

    const size_t tet_mid = newest_vertex(tet);
    const size_t sim_mid = newest_vertex(sim);
    CHECK(tet.m_vertex_attribute[tet_mid].m_order == expected_order);
    CHECK(sim.m_vertex_attribute[sim_mid].m_order == expected_order);
    CHECK(canonical_tets(sim) == canonical_tets(tet));
    check_tag_interface_invariant(sim);

    for (const auto& t : tet.get_tets()) tet.m_tet_attribute[t.tid(tet)].m_quality = 1e40;
    for (const auto& t : sim.get_tets()) sim.m_tet_attribute[t.tid(sim)].m_quality = 1e40;
    auto tet_half = tet.tuple_from_edge({{tet_mid, 0}});
    auto sim_half = sim.tuple_from_edge({{sim_mid, 0}});
    if (tet_half.vid(tet) != tet_mid) tet_half = tet_half.switch_vertex(tet);
    if (sim_half.vid(sim) != sim_mid) sim_half = sim_half.switch_vertex(sim);
    std::vector<wmtk::TetMesh::Tuple> tet_after_collapse;
    std::vector<wmtk::TetMesh::Tuple> sim_after_collapse;
    REQUIRE(tet.collapse_edge(tet_half, tet_after_collapse));
    REQUIRE(sim.collapse_edge(sim_half, sim_after_collapse));
    CHECK(canonical_tets(sim) == canonical_tets(tet));
    check_tag_interface_invariant(sim);
}

TEST_CASE(
    "tag-homogeneous SimWild follows TetWild's serial smoothing pass",
    "[simwild][tetwild][conformance][serial]")
{
    wmtk::components::tetwild::Parameters tet_params;
    tet_params.init(Vector3d(-2, -2, -2001), Vector3d(2, 2, 2001));
    wmtk::components::simwild::Parameters sim_params;
    sim_params.epsr = tet_params.epsr;
    sim_params.stop_energy = tet_params.stop_energy;
    sim_params.w_amips = tet_params.w_amips;
    sim_params.init(Vector3d(-2, -2, -2001), Vector3d(2, 2, 2001));

    wmtk::components::tetwild::TetWildMesh tet(tet_params, nullptr, 0);
    wmtk::components::simwild::SimWildMesh sim(sim_params, sim_params.eps, 0);
    init_homogeneous_tet_ring(tet);
    init_homogeneous_tet_ring(sim);
    for (size_t tid = 0; tid < kTetRing.size(); ++tid) {
        sim.m_tet_attribute[tid].tags = kHomogeneousTag;
    }

    // As above, run the oracle to completion before touching the identical SimWild mesh.
    tet.smooth_all_vertices(1);
    sim.smooth_all_vertices(1);

    CHECK(canonical_tets(sim) == canonical_tets(tet));
    const auto tet_qualities = qualities_by_tet(tet);
    const auto sim_qualities = qualities_by_tet(sim);
    REQUIRE(sim_qualities.size() == tet_qualities.size());
    for (const auto& [cell, quality] : tet_qualities) CHECK(sim_qualities.at(cell) == quality);
    for (size_t vid = 0; vid < 5; ++vid) {
        CHECK((sim.m_vertex_attribute[vid].m_posf - tet.m_vertex_attribute[vid].m_posf)
                  .squaredNorm() == 0.);
        CHECK(bool(sim.m_vertex_attribute[vid].m_pos == tet.m_vertex_attribute[vid].m_pos));
    }
    for (const auto& t : sim.get_tets()) {
        CHECK(sim.m_tet_attribute[t.tid(sim)].tags == kHomogeneousTag);
    }
}

TEST_CASE("SimWild 2D honors stop_at_float", "[simwild][2D][stop_at_float]")
{
    wmtk::components::simwild::Parameters params;
    params.init(Vector2d(-1, -1), Vector2d(2, 2));
    params.stop_at_float = true;
    wmtk::components::simwild::tri::SimWildMeshTri mesh(params, params.eps, 0);
    init_homogeneous_quad(mesh);
    const auto before = canonical_faces(mesh);

    mesh.mesh_improvement(1);

    CHECK(canonical_faces(mesh) == before);
}

TEST_CASE("SimWild 3D replace_tags validates vector lengths", "[simwild][3D][tags]")
{
    wmtk::components::simwild::Parameters params;
    params.init(Vector3d(-1, -1, -1), Vector3d(1, 1, 1));
    wmtk::components::simwild::SimWildMesh mesh(params, params.eps, 0);

    CHECK_THROWS(mesh.replace_tags({kHomogeneousTag}, {}));
}
