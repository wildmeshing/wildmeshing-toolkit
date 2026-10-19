#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/simwild/SimWildMesh.h>
#include <wmtk/components/simwild/SimWildMeshTri.hpp>

#include <algorithm>
#include <array>
#include <map>
#include <set>
#include <vector>

// Feature tracking enabled AFTER init().
//
// enable_feature_edge_tracking() and enable_feature_point_tracking() may run after init(): the
// collection they register is then sized to the current capacity with default values, and from
// there on it must be grown, moved and compacted with every other collection. The failure this
// hunts is silent -- a collection sized short is read past its end, and one the engine does not
// move leaves a tag or an anchor on a dead slot -- so each check re-reads the values from the
// live elements after a split pass (which grows the storage) and after a consolidation (which
// moves the slots).
//
// SimWild is the vehicle because it is the one application that turns on neither tracking
// itself: tetwild and triwild enable point tracking in their constructors.

namespace {

using wmtk::Vector2d;
using wmtk::Vector3d;
const std::set<int64_t> kHomogeneousTag = {7};

// A ring of four tets around the axis (0,0,-1)-(0,0,1).
void init_tet_ring(wmtk::components::simwild::SimWildMesh& mesh)
{
    constexpr int n = 4;
    std::vector<std::array<size_t, 4>> tets;
    for (int i = 0; i < n; ++i) {
        tets.push_back({{0, 1, size_t(2 + i), size_t(2 + (i + 1) % n)}});
    }
    mesh.init(size_t(n + 2), tets);
    mesh.m_vertex_attribute[0].m_posf = Vector3d(0, 0, -1);
    mesh.m_vertex_attribute[1].m_posf = Vector3d(0, 0, 1);
    mesh.m_vertex_attribute[2].m_posf = Vector3d(1, 0, 0);
    mesh.m_vertex_attribute[3].m_posf = Vector3d(0, 1, 0);
    mesh.m_vertex_attribute[4].m_posf = Vector3d(-1, 0, 0);
    mesh.m_vertex_attribute[5].m_posf = Vector3d(0, -1, 0);
    for (size_t vid = 0; vid < size_t(n + 2); ++vid) {
        auto& attr = mesh.m_vertex_attribute[vid];
        attr.set_pos_to_posf();
        attr.m_is_rounded = true;
        attr.m_is_on_surface = false;
        attr.m_order = 0;
        attr.m_sizing_scalar = 1.;
    }
    for (const auto& t : mesh.get_tets()) {
        mesh.set_cell_quality(t.tid(mesh), mesh.get_quality(mesh.oriented_tet_vids(t)));
        mesh.m_tet_attribute[t.tid(mesh)].tags = kHomogeneousTag;
    }
}

// The unit square as two triangles.
void init_quad(wmtk::components::simwild::tri::SimWildMeshTri& mesh)
{
    const std::vector<std::array<size_t, 3>> faces = {{{0, 1, 2}}, {{0, 2, 3}}};
    const std::array<Vector2d, 4> verts = {
        Vector2d(0, 0),
        Vector2d(1, 0),
        Vector2d(1, 1),
        Vector2d(0, 1)};
    mesh.init(4, faces);
    for (size_t vid = 0; vid < verts.size(); ++vid) {
        auto& attr = mesh.m_vertex_attribute[vid];
        attr.m_posf = verts[vid];
        attr.set_pos_to_posf();
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

/// Tagged edges as sorted vertex pairs, re-read from the live slots.
std::set<std::array<size_t, 2>> tagged_edges(const wmtk::components::simwild::SimWildMesh& m)
{
    std::set<std::array<size_t, 2>> out;
    for (const auto& e : m.get_edges()) {
        if (!m.m_feature_edge_attribute[e.eid(m)].m_is_feature_edge) continue;
        size_t a = e.vid(m);
        size_t b = e.switch_vertex(m).vid(m);
        if (a > b) std::swap(a, b);
        out.insert({{a, b}});
    }
    return out;
}

/// Tagged edges as sorted pairs of endpoint positions: comparable across a renumbering.
std::set<std::array<double, 6>> tagged_positions(const wmtk::components::simwild::SimWildMesh& m)
{
    std::set<std::array<double, 6>> out;
    for (const auto& [a, b] : tagged_edges(m)) {
        const Vector3d& p = m.m_vertex_attribute[a].m_posf;
        const Vector3d& q = m.m_vertex_attribute[b].m_posf;
        std::array<double, 3> u{{p[0], p[1], p[2]}}, w{{q[0], q[1], q[2]}};
        if (w < u) std::swap(u, w);
        out.insert({{u[0], u[1], u[2], w[0], w[1], w[2]}});
    }
    return out;
}

/// The tagged edges are exactly the edges of the mesh that lie on the axis, and they form one
/// chain from `a` to `b`: a tag left on a dead slot would show up off the axis or break the chain.
void check_axis_chain(
    const wmtk::components::simwild::SimWildMesh& m,
    const std::set<std::array<size_t, 2>>& tagged,
    const size_t a,
    const size_t b)
{
    const auto on_axis = [&](size_t v) {
        const auto p = m.m_vertex_attribute[v].pos();
        return p[0] == 0 && p[1] == 0;
    };
    std::map<size_t, int> degree;
    for (const auto& e : m.get_edges()) {
        size_t u = e.vid(m);
        size_t w = e.switch_vertex(m).vid(m);
        if (u > w) std::swap(u, w);
        const bool is_tagged = tagged.count({{u, w}}) > 0;
        CHECK(is_tagged == (on_axis(u) && on_axis(w)));
        if (is_tagged) {
            ++degree[u];
            ++degree[w];
        }
    }
    for (const auto& [v, d] : degree) CHECK(d == ((v == a || v == b) ? 1 : 2));
    CHECK(degree.count(a) == 1);
    CHECK(degree.count(b) == 1);
}

/// Every live vertex's feature-point id, by vertex id.
template <typename Mesh>
std::map<size_t, size_t> anchors(const Mesh& m)
{
    std::map<size_t, size_t> out;
    for (const auto& v : m.get_vertices()) {
        const size_t vid = v.vid(m);
        const size_t f = m.m_feature_point_attribute[vid].m_feature_point_id;
        if (f != Mesh::NO_FEATURE_POINT) out.emplace(vid, f);
    }
    return out;
}

/// Each feature is carried by exactly one live vertex, sitting at its anchor; no other vertex
/// carries an id.
template <typename Mesh>
void check_anchors_at_their_points(const Mesh& m)
{
    const auto a = anchors(m);
    std::map<size_t, int> carriers;
    for (const auto& [vid, f] : a) {
        REQUIRE(f < m.m_feature_points.size());
        ++carriers[f];
        CHECK((m.m_vertex_attribute[vid].m_posf - m.m_feature_points[f]).norm() == 0.);
    }
    CHECK(carriers.size() == m.m_feature_points.size());
    for (const auto& [f, c] : carriers) CHECK(c == 1);
}

template <typename Mesh>
size_t vertex_at(const Mesh& m, const size_t f)
{
    for (const auto& [vid, g] : anchors(m)) {
        if (g == f) return vid;
    }
    return Mesh::NO_FEATURE_POINT;
}

} // namespace

TEST_CASE("feature-edge tracking enabled after init (3D)", "[simwild][feature_tracking]")
{
    wmtk::components::simwild::Parameters params;
    params.init(Vector3d(-2, -2, -2), Vector3d(2, 2, 2));
    wmtk::components::simwild::SimWildMesh mesh(params, params.eps, 0);
    init_tet_ring(mesh);
    REQUIRE(mesh.p_edge_attrs == nullptr);

    mesh.enable_feature_edge_tracking();
    REQUIRE(mesh.p_edge_attrs != nullptr);
    const size_t cap0 = mesh.tet_storage_capacity();
    REQUIRE(cap0 > 0);
    REQUIRE(mesh.m_feature_edge_attribute.size() == 6 * cap0);
    for (size_t i = 0; i < mesh.m_feature_edge_attribute.size(); ++i) {
        CHECK(!mesh.m_feature_edge_attribute[i].m_is_feature_edge);
    }

    const auto axis = mesh.tuple_from_edge({{0, 1}});
    mesh.m_feature_edge_attribute[axis.eid(mesh)].m_is_feature_edge = true;
    REQUIRE(tagged_edges(mesh) == std::set<std::array<size_t, 2>>{{{0, 1}}});

    // Refine until the storage has to grow past what init() reserved.
    for (size_t vid = 0; vid < 6; ++vid) mesh.m_vertex_attribute[vid].m_sizing_scalar = 0.3;
    mesh.split_all_edges();
    REQUIRE(mesh.tet_storage_capacity() > cap0);
    CHECK(mesh.m_feature_edge_attribute.size() >= 6 * mesh.tet_storage_capacity());
    const auto tagged_after_split = tagged_edges(mesh);
    REQUIRE(tagged_after_split.size() > 1); // the axis itself was split
    check_axis_chain(mesh, tagged_after_split, 0, 1);

    // A split keeps its tet slots, so nothing is removed yet. Coarsen: collapses remove tets
    // and vertices, consolidation then moves the surviving edge slots and renumbers the
    // vertices, and the tagged edges, compared by their endpoint positions, must come out
    // identical.
    for (const auto& v : mesh.get_vertices()) {
        mesh.m_vertex_attribute[v.vid(mesh)].m_sizing_scalar = 1.;
    }
    mesh.collapse_all_edges();
    REQUIRE(mesh.tet_capacity() > mesh.get_tets().size());
    const auto tagged_before = tagged_positions(mesh);
    REQUIRE(!tagged_before.empty());
    mesh.consolidate_mesh();
    REQUIRE(mesh.tet_capacity() == mesh.get_tets().size());
    CHECK(mesh.m_feature_edge_attribute.size() >= 6 * mesh.tet_storage_capacity());
    CHECK(tagged_positions(mesh) == tagged_before);
}

TEST_CASE("feature-point tracking enabled after init (3D)", "[simwild][feature_tracking]")
{
    using Mesh = wmtk::components::simwild::SimWildMesh;
    wmtk::components::simwild::Parameters params;
    params.init(Vector3d(-2, -2, -2), Vector3d(2, 2, 2));
    Mesh mesh(params, params.eps, 0);
    init_tet_ring(mesh);
    REQUIRE(mesh.m_feature_point_attribute.size() == 0);

    mesh.enable_feature_point_tracking(/*preserve=*/true);
    const size_t cap0 = mesh.vert_storage_capacity();
    REQUIRE(cap0 > 0);
    REQUIRE(mesh.m_feature_point_attribute.size() == cap0);
    for (size_t i = 0; i < cap0; ++i) {
        CHECK(mesh.m_feature_point_attribute[i].m_feature_point_id == Mesh::NO_FEATURE_POINT);
    }

    mesh.m_feature_eps = 1e-9;
    mesh.m_feature_points = {mesh.m_vertex_attribute[2].m_posf};
    mesh.m_feature_point_attribute[2].m_feature_point_id = 0;

    for (size_t vid = 0; vid < 6; ++vid) mesh.m_vertex_attribute[vid].m_sizing_scalar = 0.3;
    mesh.split_all_edges();
    REQUIRE(mesh.vert_storage_capacity() > cap0);
    CHECK(mesh.m_feature_point_attribute.size() >= mesh.vert_storage_capacity());
    CHECK(vertex_at(mesh, 0) == 2);
    check_anchors_at_their_points(mesh);

    // A second anchor on the newest vertex, then coarsen: collapses remove vertices below it,
    // so consolidation has to move its id with it.
    size_t newest = 0;
    for (const auto& v : mesh.get_vertices()) newest = std::max(newest, v.vid(mesh));
    mesh.m_feature_points.push_back(mesh.m_vertex_attribute[newest].m_posf);
    mesh.m_feature_point_attribute[newest].m_feature_point_id = 1;
    for (const auto& v : mesh.get_vertices()) {
        mesh.m_vertex_attribute[v.vid(mesh)].m_sizing_scalar = 4.;
    }
    mesh.collapse_all_edges();
    check_anchors_at_their_points(mesh);
    REQUIRE(vertex_at(mesh, 1) == newest);

    mesh.consolidate_mesh();
    REQUIRE(mesh.vert_capacity() == mesh.get_vertices().size());
    CHECK(mesh.m_feature_point_attribute.size() >= mesh.vert_storage_capacity());
    REQUIRE(vertex_at(mesh, 1) != Mesh::NO_FEATURE_POINT);
    CHECK(vertex_at(mesh, 1) < newest); // the id moved with its vertex
    check_anchors_at_their_points(mesh);
}

TEST_CASE("feature-point tracking enabled after init (2D)", "[simwild][feature_tracking]")
{
    using Mesh = wmtk::components::simwild::tri::SimWildMeshTri;
    wmtk::components::simwild::Parameters params;
    params.init(Vector2d(-1, -1), Vector2d(2, 2));
    Mesh mesh(params, params.eps, 0);
    init_quad(mesh);
    REQUIRE(mesh.m_feature_point_attribute.size() == 0);

    mesh.enable_feature_point_tracking(/*preserve=*/true);
    const size_t cap0 = mesh.vert_storage_capacity();
    REQUIRE(cap0 > 0);
    REQUIRE(mesh.m_feature_point_attribute.size() == cap0);
    for (size_t i = 0; i < cap0; ++i) {
        CHECK(mesh.m_feature_point_attribute[i].m_feature_point_id == Mesh::NO_FEATURE_POINT);
    }

    mesh.m_feature_eps = 1e-9;
    mesh.m_feature_points = {mesh.m_vertex_attribute[2].m_posf};
    mesh.m_feature_point_attribute[2].m_feature_point_id = 0;

    for (size_t vid = 0; vid < 4; ++vid) mesh.m_vertex_attribute[vid].m_sizing_scalar = 0.25;
    mesh.split_all_edges();
    REQUIRE(mesh.vert_storage_capacity() > cap0);
    CHECK(mesh.m_feature_point_attribute.size() >= mesh.vert_storage_capacity());
    CHECK(vertex_at(mesh, 0) == 2);
    check_anchors_at_their_points(mesh);

    size_t newest = 0;
    for (const auto& v : mesh.get_vertices()) newest = std::max(newest, v.vid(mesh));
    mesh.m_feature_points.push_back(mesh.m_vertex_attribute[newest].m_posf);
    mesh.m_feature_point_attribute[newest].m_feature_point_id = 1;
    for (const auto& v : mesh.get_vertices()) {
        mesh.m_vertex_attribute[v.vid(mesh)].m_sizing_scalar = 4.;
    }
    mesh.collapse_all_edges();
    check_anchors_at_their_points(mesh);
    REQUIRE(vertex_at(mesh, 1) == newest);

    mesh.consolidate_mesh();
    REQUIRE(mesh.vert_capacity() == mesh.get_vertices().size());
    CHECK(mesh.m_feature_point_attribute.size() >= mesh.vert_storage_capacity());
    REQUIRE(vertex_at(mesh, 1) != Mesh::NO_FEATURE_POINT);
    CHECK(vertex_at(mesh, 1) < newest); // the id moved with its vertex
    check_anchors_at_their_points(mesh);
}
