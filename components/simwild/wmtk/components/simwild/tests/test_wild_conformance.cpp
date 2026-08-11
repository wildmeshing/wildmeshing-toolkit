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

namespace {

using wmtk::Vector3d;

const std::vector<std::array<size_t, 4>> kTetRing = {
    {{0, 1, 2, 3}}, {{0, 1, 3, 4}}, {{0, 1, 4, 2}}};

template <typename Mesh>
void init_homogeneous_tet_ring(Mesh& mesh)
{
    mesh.init(5, kTetRing);
    mesh.m_vertex_attribute.resize(5);
    mesh.m_face_attribute.resize(12);
    mesh.m_tet_attribute.resize(3);

    const std::array<Vector3d, 5> positions = {
        Vector3d(0, 0, -1000),
        Vector3d(0, 0, 1000),
        Vector3d(1, 0, 0),
        Vector3d(-0.5, 0.8660254037844386, 0),
        Vector3d(-0.5, -0.8660254037844386, 0)};
    for (size_t vid = 0; vid < positions.size(); ++vid) {
        auto& attr = mesh.m_vertex_attribute[vid];
        attr.m_posf = positions[vid];
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
