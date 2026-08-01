#include <wmtk/TriMesh.h>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <set>

using namespace wmtk;
using Tuple = TriMesh::Tuple;

namespace {

/**
 * Meshes used across this file. Each one isolates a single kind of non-manifoldness so a
 * failure points at a specific piece of the data structure rather than at "something in the
 * connectivity".
 */
namespace meshes {

/**
 * Three triangles sharing edge (0,1). The archetypal non-manifold edge.
 *
 *        2
 *        |
 *   3 -- 0=1 -- 4        (0=1 seen end-on: the shared edge)
 */
inline std::vector<std::array<size_t, 3>> fan3()
{
    return {{{0, 1, 2}}, {{0, 3, 1}}, {{0, 1, 4}}};
}

/// Five triangles sharing edge (0,1).
inline std::vector<std::array<size_t, 3>> fan5()
{
    return {{{0, 1, 2}}, {{0, 3, 1}}, {{0, 1, 4}}, {{0, 5, 1}}, {{0, 1, 6}}};
}

/**
 * Two triangles meeting at vertex 0 only: every edge is manifold (in fact boundary), but
 * vertex 0 has two edge-connected components.
 *
 *   2---1     4---3
 *    \ /       \ /
 *     0---------0      (the same vertex 0, drawn twice)
 */
inline std::vector<std::array<size_t, 3>> bowtie()
{
    return {{{0, 1, 2}}, {{0, 3, 4}}};
}

/**
 * Three triangles meeting at vertex 0, in three separate components. Exercises a component
 * cycle longer than 2.
 */
inline std::vector<std::array<size_t, 3>> bowtie3()
{
    return {{{0, 1, 2}}, {{0, 3, 4}}, {{0, 5, 6}}};
}

/**
 * A closed fan of four triangles around vertex 0 (one component, manifold vertex) plus a
 * detached triangle at the same vertex (second component). Mixes a manifold-looking
 * neighbourhood with a pinch.
 *
 *     2---1
 *     |\ /|
 *     | 0 |      + triangle (0,5,6) elsewhere
 *     |/ \|
 *     3---4
 */
inline std::vector<std::array<size_t, 3>> disk_plus_pinch()
{
    return {{{0, 1, 2}}, {{0, 2, 3}}, {{0, 3, 4}}, {{0, 4, 1}}, {{0, 5, 6}}};
}

/// Two triangles sharing edge (0,1): the manifold control case.
inline std::vector<std::array<size_t, 3>> two_triangles()
{
    return {{{0, 1, 2}}, {{0, 3, 1}}};
}

/// A single triangle: every edge is a boundary edge.
inline std::vector<std::array<size_t, 3>> single_triangle()
{
    return {{{0, 1, 2}}};
}

} // namespace meshes

size_t max_vid(const std::vector<std::array<size_t, 3>>& tris)
{
    size_t m = 0;
    for (const auto& t : tris) {
        for (const size_t v : t) {
            m = std::max(m, v);
        }
    }
    return m + 1;
}

/// Number of distinct faces incident to the edge the tuple points at, counting the tuple's
/// own face. Computed from scratch, independently of whatever the mesh caches.
size_t brute_force_edge_valence(const TriMesh& m, const Tuple& t)
{
    return m.get_incident_fids_for_edge(t).size();
}

/// Number of edge-connected components of the fan of `vid`, computed from scratch.
size_t brute_force_vertex_components(const TriMesh& m, size_t vid)
{
    const std::vector<size_t>& fan = m.get_one_ring_fids_for_vertex(vid);
    if (fan.empty()) {
        return 0;
    }

    // Union-find over the fan; two faces are joined when they share an edge that contains
    // vid. Small fans, so a flat parent array indexed by position is plenty.
    std::vector<size_t> parent(fan.size());
    for (size_t i = 0; i < parent.size(); ++i) {
        parent[i] = i;
    }
    const std::function<size_t(size_t)> find = [&](size_t x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    };

    for (size_t i = 0; i < fan.size(); ++i) {
        const std::array<size_t, 3> vi = m.oriented_tri_vids(fan[i]);
        for (size_t j = i + 1; j < fan.size(); ++j) {
            const std::array<size_t, 3> vj = m.oriented_tri_vids(fan[j]);
            // shared vertices other than vid
            size_t shared = 0;
            for (const size_t a : vi) {
                if (a == vid) continue;
                if (std::find(vj.begin(), vj.end(), a) != vj.end()) ++shared;
            }
            // sharing one more vertex means sharing the edge (vid, that vertex)
            if (shared >= 1) {
                const size_t ri = find(i);
                const size_t rj = find(j);
                if (ri != rj) parent[ri] = rj;
            }
        }
    }

    std::set<size_t> roots;
    for (size_t i = 0; i < fan.size(); ++i) {
        roots.insert(find(i));
    }
    return roots.size();
}

} // namespace

TEST_CASE("nonmanifold_cycles_agree_with_brute_force", "[nonmanifold]")
{
    // The stored cycles are a cache over connectivity that is computed independently here.
    // Any disagreement means an operation, or init, failed to maintain them.
    auto check_all = [](const std::vector<std::array<size_t, 3>>& tris) {
        TriMesh m;
        m.init(max_vid(tris), tris);
        REQUIRE(m.check_mesh_connectivity_validity());

        for (const Tuple& e : m.get_edges()) {
            CHECK(m.edge_valence(e) == brute_force_edge_valence(m, e));
            CHECK(m.is_boundary_edge(e) == (brute_force_edge_valence(m, e) == 1));
            CHECK(m.is_manifold_edge(e) == (brute_force_edge_valence(m, e) == 2));
        }
        for (const Tuple& v : m.get_vertices()) {
            const size_t vid = v.vid(m);
            CHECK(m.vertex_component_count(vid) == brute_force_vertex_components(m, vid));
        }
    };

    SECTION("two_triangles")
    {
        check_all(meshes::two_triangles());
    }
    SECTION("single_triangle")
    {
        check_all(meshes::single_triangle());
    }
    SECTION("fan3")
    {
        check_all(meshes::fan3());
    }
    SECTION("fan5")
    {
        check_all(meshes::fan5());
    }
    SECTION("bowtie")
    {
        check_all(meshes::bowtie());
    }
    SECTION("bowtie3")
    {
        check_all(meshes::bowtie3());
    }
    SECTION("disk_plus_pinch")
    {
        check_all(meshes::disk_plus_pinch());
    }
}

TEST_CASE("nonmanifold_switch_face_cycles", "[nonmanifold]")
{
    // Walking switch_face k times around a k-fan edge is the identity, and every step lands
    // on a distinct face. This is the property the radial cycle exists to provide.
    auto walk = [](const std::vector<std::array<size_t, 3>>& tris, size_t v0, size_t v1) {
        TriMesh m;
        m.init(max_vid(tris), tris);

        const Tuple start = m.tuple_from_edge(v0, v1, m.get_incident_fids_for_edge(v0, v1)[0]);
        const size_t k = m.edge_valence(start);

        std::set<size_t> seen;
        Tuple t = start;
        for (size_t i = 0; i < k; ++i) {
            CHECK(seen.insert(t.fid(m)).second); // no face visited twice
            CHECK(t.vid(m) == start.vid(m)); // the vertex is fixed
            CHECK(t.switch_vertex(m).vid(m) == start.switch_vertex(m).vid(m));
            CHECK(t.eid(m) == start.eid(m)); // and so is the edge
            const auto next = t.switch_face(m);
            REQUIRE(next.has_value());
            t = next.value();
        }
        CHECK(t.fid(m) == start.fid(m)); // k steps is the identity
        CHECK(seen.size() == k);
    };

    SECTION("manifold edge, k = 2")
    {
        walk(meshes::two_triangles(), 0, 1);
    }
    SECTION("fan of three, k = 3")
    {
        walk(meshes::fan3(), 0, 1);
    }
    SECTION("fan of five, k = 5")
    {
        walk(meshes::fan5(), 0, 1);
    }

    SECTION("boundary edge has no other side")
    {
        TriMesh m;
        const auto tris = meshes::single_triangle();
        m.init(max_vid(tris), tris);
        for (const Tuple& e : m.get_edges()) {
            CHECK(m.edge_valence(e) == 1);
            CHECK_FALSE(e.switch_face(m).has_value());
        }
    }
}

TEST_CASE("nonmanifold_switch_component", "[nonmanifold]")
{
    SECTION("manifold vertex has nowhere to jump")
    {
        TriMesh m;
        const auto tris = meshes::two_triangles();
        m.init(max_vid(tris), tris);

        for (const Tuple& v : m.get_vertices()) {
            CHECK(m.is_manifold_vertex(v.vid(m)));
            CHECK_FALSE(m.switch_component(v).has_value());
        }
    }

    SECTION("a non-manifold edge does not split the fan")
    {
        // All three faces are joined through the shared edge, so vertices 0 and 1 have a
        // single component even though the edge is not manifold.
        TriMesh m;
        const auto tris = meshes::fan3();
        m.init(max_vid(tris), tris);

        for (const size_t vid : {size_t(0), size_t(1)}) {
            CHECK(m.vertex_component_count(vid) == 1);
            CHECK_FALSE(m.switch_component(m.tuple_from_vertex(vid)).has_value());
        }
    }

    auto walk_components = [](const std::vector<std::array<size_t, 3>>& tris, size_t vid) {
        TriMesh m;
        m.init(max_vid(tris), tris);

        const size_t k = m.vertex_component_count(vid);
        REQUIRE(k > 1);

        Tuple t = m.tuple_from_vertex(vid);
        std::set<size_t> seen;
        for (size_t i = 0; i < k; ++i) {
            CHECK(t.vid(m) == vid); // the vertex is fixed
            CHECK(seen.insert(t.fid(m)).second);
            const auto next = m.switch_component(t);
            REQUIRE(next.has_value());
            t = next.value();
        }
        // k jumps return to the component we started the walk in
        CHECK(seen.size() == k);
        CHECK(m.switch_component(t).has_value());
    };

    SECTION("bowtie, 2 components")
    {
        walk_components(meshes::bowtie(), 0);
    }
    SECTION("bowtie3, 3 components")
    {
        walk_components(meshes::bowtie3(), 0);
    }
    SECTION("disk plus pinch, 2 components")
    {
        walk_components(meshes::disk_plus_pinch(), 0);
    }

    SECTION("the jump really does leave the component")
    {
        // The destination must be unreachable by switch_edge/switch_face from the start,
        // which is what "different edge-connected component" means.
        TriMesh m;
        const auto tris = meshes::disk_plus_pinch();
        m.init(max_vid(tris), tris);

        const Tuple start = m.tuple_from_vertex(0);
        const auto jumped = m.switch_component(start);
        REQUIRE(jumped.has_value());

        // flood the start's component through edges at vertex 0
        std::set<size_t> reachable;
        std::vector<Tuple> stack = {start};
        while (!stack.empty()) {
            Tuple t = stack.back();
            stack.pop_back();
            if (!reachable.insert(t.fid(m)).second) continue;
            for (const Tuple& e : {t, t.switch_edge(m)}) {
                for (const Tuple& opp : e.switch_faces(m)) {
                    stack.push_back(opp.vid(m) == 0 ? opp : opp.switch_vertex(m));
                }
            }
        }
        CHECK(reachable.count(jumped.value().fid(m)) == 0);
    }
}

TEST_CASE("nonmanifold_edge_valence", "[nonmanifold]")
{
    SECTION("manifold edge has two faces")
    {
        TriMesh m;
        const auto tris = meshes::two_triangles();
        m.init(max_vid(tris), tris);

        const Tuple e = m.tuple_from_vids(0, 1, 2);
        CHECK(brute_force_edge_valence(m, e) == 2);
    }

    SECTION("boundary edge has one face")
    {
        TriMesh m;
        const auto tris = meshes::single_triangle();
        m.init(max_vid(tris), tris);

        for (const Tuple& e : m.get_edges()) {
            CHECK(brute_force_edge_valence(m, e) == 1);
            CHECK(m.is_boundary_edge(e));
        }
    }

    SECTION("fan of three")
    {
        TriMesh m;
        const auto tris = meshes::fan3();
        m.init(max_vid(tris), tris);

        const Tuple e = m.tuple_from_vids(0, 1, 2);
        CHECK(brute_force_edge_valence(m, e) == 3);
        CHECK_FALSE(m.is_boundary_edge(e));
        // switch_faces reports the other two, and is already non-manifold-correct today
        CHECK(e.switch_faces(m).size() == 2);
    }

    SECTION("fan of five")
    {
        TriMesh m;
        const auto tris = meshes::fan5();
        m.init(max_vid(tris), tris);

        const Tuple e = m.tuple_from_vids(0, 1, 2);
        CHECK(brute_force_edge_valence(m, e) == 5);
        CHECK(e.switch_faces(m).size() == 4);
    }
}

TEST_CASE("nonmanifold_vertex_components", "[nonmanifold]")
{
    SECTION("manifold vertex is a single component")
    {
        TriMesh m;
        const auto tris = meshes::two_triangles();
        m.init(max_vid(tris), tris);

        for (size_t vid = 0; vid < 4; ++vid) {
            CHECK(brute_force_vertex_components(m, vid) == 1);
        }
    }

    SECTION("bowtie vertex has two components")
    {
        TriMesh m;
        const auto tris = meshes::bowtie();
        m.init(max_vid(tris), tris);

        CHECK(brute_force_vertex_components(m, 0) == 2);
        for (size_t vid = 1; vid < 5; ++vid) {
            CHECK(brute_force_vertex_components(m, vid) == 1);
        }
    }

    SECTION("three components at one vertex")
    {
        TriMesh m;
        const auto tris = meshes::bowtie3();
        m.init(max_vid(tris), tris);

        CHECK(brute_force_vertex_components(m, 0) == 3);
    }

    SECTION("closed disk plus a pinch")
    {
        TriMesh m;
        const auto tris = meshes::disk_plus_pinch();
        m.init(max_vid(tris), tris);

        // the four disk triangles form one component, the detached one another
        CHECK(brute_force_vertex_components(m, 0) == 2);
        CHECK(m.get_one_ring_fids_for_vertex(0).size() == 5);
    }

    SECTION("a fan of three does not split the shared vertices")
    {
        TriMesh m;
        const auto tris = meshes::fan3();
        m.init(max_vid(tris), tris);

        // vertices 0 and 1 are on a non-manifold *edge*, but all three faces are joined
        // through it, so the vertex fan is still a single edge-connected component
        CHECK(brute_force_vertex_components(m, 0) == 1);
        CHECK(brute_force_vertex_components(m, 1) == 1);
    }
}

TEST_CASE("nonmanifold_swap_is_refused", "[nonmanifold]")
{
    SECTION("non-manifold edge")
    {
        TriMesh m;
        const auto tris = meshes::fan3();
        m.init(max_vid(tris), tris);

        const Tuple e = m.tuple_from_vids(0, 1, 2);
        std::vector<Tuple> dummy;
        CHECK_FALSE(m.swap_edge(e, dummy));
    }

    SECTION("boundary edge")
    {
        TriMesh m;
        const auto tris = meshes::two_triangles();
        m.init(max_vid(tris), tris);

        const Tuple e = m.tuple_from_edge(0, 2, 0); // (0,2) is on the boundary of face 0
        REQUIRE(m.is_boundary_edge(e));
        std::vector<Tuple> dummy;
        CHECK_FALSE(m.swap_edge(e, dummy));
    }
}

TEST_CASE("nonmanifold_split_preserves_fan_size", "[nonmanifold]")
{
    TriMesh m;
    const auto tris = meshes::fan5();
    m.init(max_vid(tris), tris);

    const Tuple e = m.tuple_from_vids(0, 1, 2);
    REQUIRE(brute_force_edge_valence(m, e) == 5);

    std::vector<Tuple> dummy;
    REQUIRE(m.split_edge(e, dummy));
    REQUIRE(m.check_mesh_connectivity_validity());

    // The split vertex is the one added last.
    const size_t new_vid = m.vert_capacity() - 1;
    // Both halves of the split edge keep the full fan...
    CHECK(m.get_incident_fids_for_edge(0, new_vid).size() == 5);
    CHECK(m.get_incident_fids_for_edge(new_vid, 1).size() == 5);
    // ...and each of the 5 spokes to the opposite vertices is a new manifold edge.
    for (const size_t apex : {2u, 3u, 4u, 5u, 6u}) {
        CHECK(m.get_incident_fids_for_edge(new_vid, apex).size() == 2);
    }
    CHECK(m.get_faces().size() == 10);
}

TEST_CASE("nonmanifold_collapse_of_the_shared_edge", "[nonmanifold]")
{
    // Collapsing the non-manifold edge itself would remove all three faces at once, leaving
    // vertices 2, 3 and 4 with nothing incident.
    //
    // The link condition refuses it, but for a reason worth recording: every edge at 0 and
    // at 1 other than (0,1) is a boundary edge, so the sentinel `dummy` vertex lands in both
    // vertex links; (0,1) itself has three faces and so is *not* a boundary edge, and no
    // dummy goes into the edge link. lk(0) n lk(1) = {2,3,4,dummy} against lk(01) = {2,3,4},
    // and the sizes differ. It is the sentinel that rejects this, not the fan.
    TriMesh m;
    const auto tris = meshes::fan3();
    m.init(max_vid(tris), tris);

    const Tuple e = m.tuple_from_vids(0, 1, 2);
    REQUIRE_FALSE(m.check_link_condition(e));

    std::vector<Tuple> dummy;
    CHECK_FALSE(m.collapse_edge(e, dummy));
}

TEST_CASE("nonmanifold_collapse_next_to_the_shared_edge", "[nonmanifold]")
{
    // Collapse (0,2), an ordinary manifold boundary edge of the fan. Face (0,1,2) goes
    // away; the two remaining faces keep the now-manifold edge (0,1).
    TriMesh m;
    const auto tris = meshes::fan3();
    m.init(max_vid(tris), tris);

    const Tuple e = m.tuple_from_edge(0, 2, 0);
    REQUIRE(m.check_link_condition(e));

    std::vector<Tuple> dummy;
    REQUIRE(m.collapse_edge(e, dummy));
    REQUIRE(m.check_mesh_connectivity_validity());

    CHECK(m.get_faces().size() == 2);
    const Tuple survivor = m.tuple_from_vids(0, 3, 1);
    CHECK(brute_force_edge_valence(m, survivor) == 1);
}

TEST_CASE("nonmanifold_consolidate_round_trip", "[nonmanifold]")
{
    auto round_trip = [](const std::vector<std::array<size_t, 3>>& tris) {
        TriMesh m;
        m.init(max_vid(tris), tris);

        const size_t nv_before = m.get_vertices().size();
        const size_t nf_before = m.get_faces().size();

        m.consolidate_mesh();

        REQUIRE(m.check_mesh_connectivity_validity());
        CHECK(m.get_vertices().size() == nv_before);
        CHECK(m.get_faces().size() == nf_before);
    };

    SECTION("fan3")
    {
        round_trip(meshes::fan3());
    }
    SECTION("fan5")
    {
        round_trip(meshes::fan5());
    }
    SECTION("bowtie")
    {
        round_trip(meshes::bowtie());
    }
    SECTION("bowtie3")
    {
        round_trip(meshes::bowtie3());
    }
    SECTION("disk_plus_pinch")
    {
        round_trip(meshes::disk_plus_pinch());
    }
}
