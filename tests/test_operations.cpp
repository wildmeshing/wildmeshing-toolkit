#include <wmtk/TetMesh.h>
#include <wmtk/utils/AMIPS.h>
#include <catch2/catch_test_macros.hpp>
#include <wmtk/OperationDryRun.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/examples/TetMesh_examples.hpp>

using namespace wmtk;

struct VertexAttributes
{
    Vector3d m_posf;
};

class TetMeshWithPosition : public wmtk::TetMesh
{
public:
    TetMeshWithPosition() { p_vertex_attrs = &m_vertex_attribute; }

    bool is_inverted_f(const Tuple& loc) const
    {
        auto vs = oriented_tet_vertices(loc);

        return is_inverted_f(
            {{m_vertex_attribute[vs[0].vid(*this)].m_posf,
              m_vertex_attribute[vs[1].vid(*this)].m_posf,
              m_vertex_attribute[vs[2].vid(*this)].m_posf,
              m_vertex_attribute[vs[3].vid(*this)].m_posf}});
    }

    bool is_inverted_f(const std::array<Vector3d, 4>& ps) const
    {
        wmtk::utils::predicates::exactinit();
        auto res = wmtk::utils::predicates::orient3d(ps[0], ps[1], ps[2], ps[3]);
        int result;
        if (res == wmtk::utils::predicates::Orientation::POSITIVE)
            result = 1;
        else if (res == wmtk::utils::predicates::Orientation::NEGATIVE)
            result = -1;
        else
            result = 0;

        if (result < 0) // neg result == pos tet (tet origin from geogram delaunay)
            return false;
        return true;
    }

    double get_quality(const std::array<Vector3d, 4>& ps) const
    {
        std::array<double, 12> T;
        for (auto k = 0; k < 4; k++) {
            for (auto j = 0; j < 3; j++) {
                T[k * 3 + j] = ps[k][j];
            }
        }
        return wmtk::AMIPS_energy_stable_p3<wmtk::Rational>(T);
    }

    double swap_edge_56_energy(const std::vector<std::array<size_t, 4>>& tets, const int op_case)
        override
    {
        double max_energy = -1;
        for (const auto& vids : tets) {
            std::array<Vector3d, 4> p;
            p[0] = m_vertex_attribute[vids[0]].m_posf;
            p[1] = m_vertex_attribute[vids[1]].m_posf;
            p[2] = m_vertex_attribute[vids[2]].m_posf;
            p[3] = m_vertex_attribute[vids[3]].m_posf;
            if (is_inverted_f(p)) {
                return std::numeric_limits<double>::max();
            }
            const double e = get_quality(p);
            max_energy = std::max(max_energy, e);
        }
        return max_energy;
    }

public:
    using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    VertAttCol m_vertex_attribute;
};

TEST_CASE("edge_splitting", "[tuple_operation]")
{
    auto mesh = TetMesh();
    mesh.init(5, {{{0, 1, 2, 3}}, {{0, 1, 2, 4}}});
    const auto tuple = mesh.tuple_from_face(0, 0);
    std::vector<TetMesh::Tuple> dummy;
    REQUIRE(mesh.split_edge(tuple, dummy));
    REQUIRE_FALSE(tuple.is_valid(mesh));
    REQUIRE(mesh.check_mesh_connectivity_validity());
}

TEST_CASE("edge_collapsing_impossible", "[tuple_operation]")
{
    auto mesh = TetMesh();
    mesh.init(5, {{{0, 1, 2, 3}}, {{0, 1, 2, 4}}});
    const auto tuple = mesh.tuple_from_face(0, 0);
    std::vector<TetMesh::Tuple> dummy;

    // Cannot collapse the edge
    REQUIRE_FALSE(mesh.collapse_edge(tuple, dummy));
    REQUIRE(tuple.is_valid(mesh));
    REQUIRE(mesh.check_mesh_connectivity_validity());
}

TEST_CASE("edge_collapsing", "[tuple_operation]")
{
    auto mesh = TetMesh();
    mesh.init(5, {{{0, 1, 2, 3}}, {{0, 2, 1, 4}}, {{0, 1, 3, 4}}});
    const auto tuple = mesh.tuple_from_face(0, 0);
    std::vector<TetMesh::Tuple> new_edge, dummy;
    REQUIRE(mesh.split_edge(tuple, new_edge));
    REQUIRE_FALSE(tuple.is_valid(mesh));

    // Cannot collapse the edge
    REQUIRE(mesh.collapse_edge(new_edge[1], dummy));
    for (const auto& e : new_edge) REQUIRE_FALSE(e.is_valid(mesh));
    REQUIRE(mesh.check_mesh_connectivity_validity());
}

TEST_CASE("edge_collapsing_invalid", "[tuple_operation]")
{
    auto mesh = TetMesh();
    mesh.init(6, {{{0, 1, 2, 3}}, {{1, 2, 3, 4}}, {{1, 3, 4, 5}}});
    const auto tuple = mesh.tuple_from_edge(0, 1);
    std::vector<TetMesh::Tuple> new_edge, dummy;
    // Cannot collapse the edge
    REQUIRE(!mesh.collapse_edge(tuple, dummy));
}

TEST_CASE("tet_mesh_swap", "[tuple_operation]")
{
    TetMesh mesh;
    mesh.init(5, {{{0, 1, 2, 3}}, {{0, 2, 1, 4}}, {{0, 1, 3, 4}}});

    // SECTION ("3-2 swap")
    {
        const auto edges = mesh.get_edges();

        REQUIRE(edges.size() == 10);
        auto cnt_swap = 0;
        for (auto e : edges) {
            if (!e.is_valid(mesh)) continue;
            std::vector<TetMesh::Tuple> newt;
            if (mesh.swap_edge(e, newt)) {
                cnt_swap++;
                REQUIRE_FALSE(e.is_valid(mesh));
                REQUIRE(newt.front().is_valid(mesh));
            }
        }
        REQUIRE(mesh.check_mesh_connectivity_validity());
        REQUIRE(cnt_swap == 1);
        REQUIRE(mesh.get_edges().size() == 9);
        REQUIRE(mesh.get_tets().size() == 2);
    }
    //
    // SECTION ("2-3 swap")
    {
        const auto faces = mesh.get_faces();
        REQUIRE(faces.size() == 7);
        auto cnt_swap = 0;
        for (auto e : faces) {
            if (!e.is_valid(mesh)) continue;
            std::vector<TetMesh::Tuple> newt;
            if (mesh.swap_face(e, newt)) {
                cnt_swap++;
                REQUIRE_FALSE(e.is_valid(mesh));
                REQUIRE(newt.front().is_valid(mesh));
            }
        }
        REQUIRE(cnt_swap == 1);
        REQUIRE(mesh.check_mesh_connectivity_validity());
        REQUIRE(mesh.get_tets().size() == 3);
    }

    REQUIRE(mesh.tet_capacity() == 4);
    mesh.consolidate_mesh();
    REQUIRE(mesh.tet_capacity() == 3);
}

TEST_CASE("rollback_operation", "[tuple_operation]")
{
    class NoOperationMesh : public TetMesh
    {
    public:
        bool split_edge_after(const TetMesh::Tuple& locs) override { return false; };
        bool collapse_edge_after(const TetMesh::Tuple& locs) override { return false; };
        bool swap_edge_after(const TetMesh::Tuple& locs) override { return false; };
        bool swap_face_after(const TetMesh::Tuple& locs) override { return false; };
    };
    auto mesh = NoOperationMesh();
    mesh.init(5, {{{0, 1, 2, 3}}, {{0, 1, 2, 4}}});
    const auto tuple = mesh.tuple_from_face(0, 0);
    std::vector<TetMesh::Tuple> dummy;
    SECTION("split")
    {
        REQUIRE_FALSE(mesh.split_edge(tuple, dummy));
        REQUIRE(tuple.is_valid(mesh));
    }
    SECTION("swap")
    {
        REQUIRE_FALSE(mesh.swap_face(tuple, dummy));
        REQUIRE(tuple.is_valid(mesh));
        REQUIRE_FALSE(mesh.swap_edge(tuple, dummy));
        REQUIRE(tuple.is_valid(mesh));
    }
    SECTION("collapse")
    {
        REQUIRE_FALSE(mesh.collapse_edge(tuple, dummy));
        REQUIRE(tuple.is_valid(mesh));
    }
}

TEST_CASE("forbidden-face-swap", "[tuple_operation]")
{
    /// https://i.imgur.com/aVCsOvf.png and 0,2,3 should not be swapped.
    /// Visualize V as
    //   [[ 0,  0, -1],
    //    [ 0,  0,  1],
    //    [ 1,  1,  0],
    //    [ 1, -1,  0],
    //    [ 2,  0,  0]]
    auto mesh = TetMesh();
    mesh.init(5, {{{0, 3, 2, 4}}, {{1, 2, 3, 4}}, {{0, 1, 2, 3}}});
    auto t = mesh.tuple_from_tet(0);
    REQUIRE(t.vid(mesh) == 0);
    auto oppo = t.switch_vertex(mesh);
    REQUIRE(oppo.vid(mesh) == 3);
    REQUIRE(oppo.switch_edge(mesh).switch_vertex(mesh).vid(mesh) == 2);
    std::vector<TetMesh::Tuple> new_t;
    mesh.swap_face(t, new_t);
    REQUIRE(t.is_valid(mesh)); // operation rejected.
    REQUIRE(mesh.check_mesh_connectivity_validity());
}

TEST_CASE("tet_mesh_swap44", "[tuple_operation]")
{
    TetMesh mesh;
    using Tuple = TetMesh::Tuple;
    // 01 is the common edge
    mesh.init(6, {{{0, 1, 2, 3}}, {{0, 1, 3, 4}}, {{0, 1, 4, 5}}, {{0, 1, 5, 2}}});

    {
        const auto edges = mesh.get_edges();

        REQUIRE(edges.size() == 13);
        auto cnt_swap = 0;
        for (const Tuple& e : edges) {
            if (!e.is_valid(mesh)) {
                continue;
            }
            std::vector<Tuple> newt;
            if (mesh.swap_edge_44(e, newt)) {
                cnt_swap++;
                REQUIRE_FALSE(e.is_valid(mesh));
                REQUIRE(newt.front().is_valid(mesh));
            }
        }
        REQUIRE(mesh.check_mesh_connectivity_validity());
        REQUIRE(cnt_swap == 1);
        REQUIRE(mesh.get_edges().size() == 13);
        REQUIRE(mesh.get_tets().size() == 4);
    }
}

TEST_CASE("tet_mesh_swap56_with_position", "[tuple_operation]")
{
    TetMeshWithPosition mesh;
    using Tuple = TetMesh::Tuple;
    // 01 is the common edge
    mesh.init(7, {{{0, 1, 2, 3}}, {{0, 1, 3, 4}}, {{0, 1, 4, 5}}, {{0, 1, 5, 6}}, {{0, 1, 6, 2}}});
    {
        mesh.m_vertex_attribute[0].m_posf = Vector3d(0, 0, -1000);
        mesh.m_vertex_attribute[1].m_posf = Vector3d(0, 0, 1000);
        mesh.m_vertex_attribute[2].m_posf = Vector3d(-1, -1, 0);
        mesh.m_vertex_attribute[3].m_posf = Vector3d(1, -1, 0);
        mesh.m_vertex_attribute[4].m_posf = Vector3d(1, 1, 0);
        mesh.m_vertex_attribute[5].m_posf = Vector3d(0, 2, 0);
        mesh.m_vertex_attribute[6].m_posf = Vector3d(-1, 1, 0);
    }

    REQUIRE(mesh.invariants(mesh.get_tets()));
    REQUIRE(mesh.get_edges().size() == 16);
    std::vector<Tuple> newt;

    SECTION("basic")
    {
        const Tuple e = mesh.tuple_from_edge({{0, 1}});
        REQUIRE(e.is_valid(mesh));
        REQUIRE(mesh.swap_edge_56(e, newt));
        REQUIRE(mesh.check_mesh_connectivity_validity());
        CHECK(newt.size() == 6);
        for (const Tuple& t : newt) {
            CHECK(t.is_valid(mesh));
        }
        CHECK(mesh.get_edges().size() == 17);
        CHECK(mesh.get_tets().size() == 6);
    }
    SECTION("first-inverted")
    {
        mesh.m_vertex_attribute[3].m_posf = Vector3d(0.5, -0.5, 0);
        mesh.m_vertex_attribute[4].m_posf = Vector3d(5, 1, 0);
        REQUIRE(mesh.invariants(mesh.get_tets()));

        const Tuple e = mesh.tuple_from_edge({{0, 1}});
        REQUIRE(e.is_valid(mesh));
        REQUIRE(mesh.swap_edge_56(e, newt));
        REQUIRE(mesh.check_mesh_connectivity_validity());
        CHECK(newt.size() == 6);
        for (const Tuple& t : newt) {
            CHECK(t.is_valid(mesh));
        }
        CHECK(mesh.get_edges().size() == 17);
        CHECK(mesh.get_tets().size() == 6);
    }
}

TEST_CASE("tet_face_split", "[TetMesh][tuple_operation]")
{
    using namespace wmtk::utils::examples::tet;
    using Tuple = TetMesh::Tuple;

    SECTION("single_tet_ccw")
    {
        TetMeshVT VT = single_tet();
        TetMesh m;
        m.init(VT.T);

        const Tuple t = m.tuple_from_vids(0, 1, 2, 3);

        std::vector<Tuple> new_tets;
        REQUIRE(m.split_face(t, new_tets));
        CHECK(new_tets.size() == 3);
        CHECK(m.get_vertices().size() == 5);
        CHECK(m.get_edges().size() == 10);
        CHECK(m.get_faces().size() == 9);
        REQUIRE(m.get_tets().size() == 3);
        CHECK(m.oriented_tet_vids(0) == std::array<size_t, 4>{4, 1, 2, 3});
        CHECK(m.oriented_tet_vids(1) == std::array<size_t, 4>{0, 4, 2, 3});
        CHECK(m.oriented_tet_vids(2) == std::array<size_t, 4>{0, 1, 4, 3});
    }
    SECTION("single_tet_not_ccw")
    {
        TetMeshVT VT = single_tet();
        TetMesh m;
        m.init(VT.T);

        const Tuple t = m.tuple_from_vids(0, 2, 1, 3);

        std::vector<Tuple> new_tets;
        REQUIRE(m.split_face(t, new_tets));
        CHECK(new_tets.size() == 3);
        CHECK(m.get_vertices().size() == 5);
        CHECK(m.get_edges().size() == 10);
        CHECK(m.get_faces().size() == 9);
        REQUIRE(m.get_tets().size() == 3);
        CHECK(m.oriented_tet_vids(0) == std::array<size_t, 4>{4, 1, 2, 3});
        CHECK(m.oriented_tet_vids(2) == std::array<size_t, 4>{0, 4, 2, 3});
        CHECK(m.oriented_tet_vids(1) == std::array<size_t, 4>{0, 1, 4, 3});
    }
    SECTION("two_tets_boundary")
    {
        TetMeshVT VT = two_tets();
        TetMesh m;
        m.init(VT.T);

        const Tuple t = m.tuple_from_vids(0, 1, 2, 3);

        std::vector<Tuple> new_tets;
        REQUIRE(m.split_face(t, new_tets));
        CHECK(new_tets.size() == 3);
        CHECK(m.get_vertices().size() == 6);
        CHECK(m.get_edges().size() == 13);
        CHECK(m.get_faces().size() == 12);
        REQUIRE(m.get_tets().size() == 4);
        CHECK(m.oriented_tet_vids(0) == std::array<size_t, 4>{5, 1, 2, 3});
        CHECK(m.oriented_tet_vids(2) == std::array<size_t, 4>{0, 5, 2, 3});
        CHECK(m.oriented_tet_vids(3) == std::array<size_t, 4>{0, 1, 5, 3});
    }
    SECTION("two_tets_interior")
    {
        TetMeshVT VT = two_tets();
        TetMesh m;
        m.init(VT.T);

        const Tuple t = m.tuple_from_vids(0, 2, 3, 1);

        std::vector<Tuple> new_tets;
        REQUIRE(m.split_face(t, new_tets));
        CHECK(new_tets.size() == 6);
        CHECK(m.get_vertices().size() == 6);
        CHECK(m.get_edges().size() == 14);
        CHECK(m.get_faces().size() == 15);
        REQUIRE(m.get_tets().size() == 6);
        // 0, 1, 2, 3
        CHECK(m.oriented_tet_vids(0) == std::array<size_t, 4>{5, 1, 2, 3});
        CHECK(m.oriented_tet_vids(2) == std::array<size_t, 4>{0, 1, 5, 3});
        CHECK(m.oriented_tet_vids(3) == std::array<size_t, 4>{0, 1, 2, 5});
        // 0, 2, 3, 4
        CHECK(m.oriented_tet_vids(1) == std::array<size_t, 4>{5, 2, 3, 4});
        CHECK(m.oriented_tet_vids(4) == std::array<size_t, 4>{0, 5, 3, 4});
        CHECK(m.oriented_tet_vids(5) == std::array<size_t, 4>{0, 2, 5, 4});
    }
}

TEST_CASE("tet_tet_split", "[TetMesh][tuple_operation]")
{
    using namespace wmtk::utils::examples::tet;
    using Tuple = TetMesh::Tuple;

    SECTION("single_tet_ccw")
    {
        TetMeshVT VT = single_tet();
        TetMesh m;
        m.init(VT.T);

        const Tuple t = m.tuple_from_vids(0, 1, 2, 3);

        std::vector<Tuple> new_tets;
        REQUIRE(m.split_tet(t, new_tets));
        CHECK(new_tets.size() == 4);
        CHECK_FALSE(t.is_valid(m));
        CHECK(m.get_vertices().size() == 5);
        CHECK(m.get_edges().size() == 10);
        CHECK(m.get_faces().size() == 10);
        REQUIRE(m.get_tets().size() == 4);
        CHECK(m.oriented_tet_vids(0) == std::array<size_t, 4>{4, 1, 2, 3});
        CHECK(m.oriented_tet_vids(1) == std::array<size_t, 4>{0, 4, 2, 3});
        CHECK(m.oriented_tet_vids(2) == std::array<size_t, 4>{0, 1, 4, 3});
        CHECK(m.oriented_tet_vids(3) == std::array<size_t, 4>{0, 1, 2, 4});
        CHECK(m.get_one_ring_tids_for_vertex(0) == std::vector<size_t>{1, 2, 3});
        CHECK(m.get_one_ring_tids_for_vertex(1) == std::vector<size_t>{0, 2, 3});
        CHECK(m.get_one_ring_tids_for_vertex(2) == std::vector<size_t>{0, 1, 3});
        CHECK(m.get_one_ring_tids_for_vertex(3) == std::vector<size_t>{0, 1, 2});
    }
    SECTION("single_tet_not_ccw")
    {
        TetMeshVT VT = single_tet();
        TetMesh m;
        m.init(VT.T);

        const Tuple t = m.tuple_from_vids(0, 2, 1, 3);

        std::vector<Tuple> new_tets;
        REQUIRE(m.split_tet(t, new_tets));
        CHECK(new_tets.size() == 4);
        CHECK_FALSE(t.is_valid(m));
        CHECK(m.get_vertices().size() == 5);
        CHECK(m.get_edges().size() == 10);
        CHECK(m.get_faces().size() == 10);
        REQUIRE(m.get_tets().size() == 4);
        CHECK(m.oriented_tet_vids(0) == std::array<size_t, 4>{4, 1, 2, 3});
        CHECK(m.oriented_tet_vids(2) == std::array<size_t, 4>{0, 4, 2, 3});
        CHECK(m.oriented_tet_vids(1) == std::array<size_t, 4>{0, 1, 4, 3});
        CHECK(m.oriented_tet_vids(3) == std::array<size_t, 4>{0, 1, 2, 4});
    }
    SECTION("two_tets")
    {
        TetMeshVT VT = two_tets();
        TetMesh m;
        m.init(VT.T);

        const Tuple t = m.tuple_from_vids(0, 1, 2, 3);

        std::vector<Tuple> new_tets;
        REQUIRE(m.split_tet(t, new_tets));
        CHECK(new_tets.size() == 4);
        CHECK_FALSE(t.is_valid(m));
        CHECK(m.get_vertices().size() == 6);
        CHECK(m.get_edges().size() == 13);
        CHECK(m.get_faces().size() == 13);
        REQUIRE(m.get_tets().size() == 5);

        CHECK(m.oriented_tet_vids(1) == std::array<size_t, 4>{0, 2, 3, 4});

        CHECK(m.oriented_tet_vids(0) == std::array<size_t, 4>{5, 1, 2, 3});
        CHECK(m.oriented_tet_vids(2) == std::array<size_t, 4>{0, 5, 2, 3});
        CHECK(m.oriented_tet_vids(3) == std::array<size_t, 4>{0, 1, 5, 3});
        CHECK(m.oriented_tet_vids(4) == std::array<size_t, 4>{0, 1, 2, 5});
    }
}

TEST_CASE("operation_dry_run_scope_nests", "[operations]")
{
    // A scope restores what it found, so an inner scope -- or one opened where the flag was
    // already set -- does not end the outer dry run.
    REQUIRE_FALSE(wmtk::operation_dry_run());
    {
        wmtk::OperationDryRunScope outer;
        CHECK(wmtk::operation_dry_run());
        {
            wmtk::OperationDryRunScope inner;
            CHECK(wmtk::operation_dry_run());
        }
        CHECK(wmtk::operation_dry_run());
    }
    CHECK_FALSE(wmtk::operation_dry_run());
}

namespace {

// Everything a dry run must leave as it found it: every cell, every vertex's star, and the slot
// counts (a slot taken and not given back shows in the live count).
std::vector<std::vector<size_t>> tet_mesh_state(const TetMesh& m)
{
    std::vector<std::vector<size_t>> s;
    for (const auto& t : m.get_tets()) {
        const auto v = m.oriented_tet_vids(t.tid(m));
        s.push_back({t.tid(m), v[0], v[1], v[2], v[3]});
    }
    for (const auto& v : m.get_vertices()) {
        std::vector<size_t> star;
        for (const auto& t : m.get_one_ring_tets_for_vertex(v)) star.push_back(t.tid(m));
        std::sort(star.begin(), star.end());
        star.insert(star.begin(), v.vid(m));
        s.push_back(star);
    }
    s.push_back({m.tet_capacity(), m.vert_capacity()});
    return s;
}

/**
 * For each simplex `simplices` lists, on a fresh mesh from `make` each time: a dry run of `op`
 * changes nothing, and refuses only what the real operation refuses -- screening never throws
 * away an operation that would have succeeded. `exact`: the dry run stops after the operation's
 * last check, so it passes exactly what the real operation does. Returns how many passed.
 */
template <class Make, class Simplices, class Op>
int check_dry_runs(const Make& make, const Simplices& simplices, const Op& op, bool exact)
{
    const size_t n = simplices(*make()).size();
    REQUIRE(n > 0);
    int passed = 0;
    for (size_t i = 0; i < n; ++i) {
        auto dry = make();
        const auto before = tet_mesh_state(*dry);
        bool d = false;
        {
            OperationDryRunScope scope;
            d = op(*dry, simplices(*dry)[i]);
        }
        CHECK(tet_mesh_state(*dry) == before);
        // As the scheduler does before every serial operation: storage for the worst case.
        auto real = make();
        real->reserve_free_slots(real->cell_slot_bound(simplices(*real)[i]), 1);
        const bool r = op(*real, simplices(*real)[i]);
        if (r) CHECK(d);
        if (exact) CHECK(d == r);
        passed += d ? 1 : 0;
    }
    return passed;
}

/// Counts what TetMesh reports through op_event().
class CountingTetMesh : public TetMesh
{
public:
    mutable std::array<std::array<int, size_t(OpEvent::COUNT)>, size_t(OpKind::COUNT)> events{};
    void op_event(OpKind k, OpEvent e) const override { ++events[size_t(k)][size_t(e)]; }
    int count(OpKind k, OpEvent e) const { return events[size_t(k)][size_t(e)]; }
};

} // namespace

TEST_CASE("dry runs change nothing and refuse only what the operation refuses", "[operations]")
{
    using Tuple = TetMesh::Tuple;
    const auto edges = [](const TetMesh& m) { return m.get_edges(); };
    const auto faces = [](const TetMesh& m) { return m.get_faces(); };
    const auto tets = [](const TetMesh& m) { return m.get_tets(); };
    const auto vertices = [](const TetMesh& m) { return m.get_vertices(); };
    // Three tets around the edge (0,1), so it has a 3-2 swap; its other edges and faces do not.
    const auto three = [] {
        auto m = std::make_unique<CountingTetMesh>();
        m->init(5, {{{0, 1, 2, 3}}, {{0, 2, 1, 4}}, {{0, 1, 3, 4}}});
        return m;
    };
    // Two tets on the face (0,1,2), which has a 2-3 swap.
    const auto two = [] {
        auto m = std::make_unique<CountingTetMesh>();
        m->init(5, {{{0, 1, 2, 3}}, {{0, 2, 1, 4}}});
        return m;
    };
    // Four tets around (0,1): a 4-4 swap.
    const auto four = [] {
        auto m = std::make_unique<CountingTetMesh>();
        m->init(6, {{{0, 1, 2, 3}}, {{0, 1, 3, 4}}, {{0, 1, 4, 5}}, {{0, 1, 5, 2}}});
        return m;
    };
    // Five tets around (0,1), placed so that a 5-6 swap improves them.
    const auto five = [] {
        auto m = std::make_unique<TetMeshWithPosition>();
        m->init(
            7,
            {{{0, 1, 2, 3}}, {{0, 1, 3, 4}}, {{0, 1, 4, 5}}, {{0, 1, 5, 6}}, {{0, 1, 6, 2}}});
        m->m_vertex_attribute[0].m_posf = Vector3d(0, 0, -1000);
        m->m_vertex_attribute[1].m_posf = Vector3d(0, 0, 1000);
        m->m_vertex_attribute[2].m_posf = Vector3d(-1, -1, 0);
        m->m_vertex_attribute[3].m_posf = Vector3d(1, -1, 0);
        m->m_vertex_attribute[4].m_posf = Vector3d(1, 1, 0);
        m->m_vertex_attribute[5].m_posf = Vector3d(0, 2, 0);
        m->m_vertex_attribute[6].m_posf = Vector3d(-1, 1, 0);
        return m;
    };
    // A fresh output vector per call, as the scheduler passes: the operations append to it.
    const auto split_edge = [](TetMesh& m, const Tuple& t) {
        std::vector<Tuple> out;
        return m.split_edge(t, out);
    };
    const auto collapse_edge = [](TetMesh& m, const Tuple& t) {
        std::vector<Tuple> out;
        return m.collapse_edge(t, out);
    };
    const auto swap_32 = [](TetMesh& m, const Tuple& t) {
        std::vector<Tuple> out;
        return m.swap_edge(t, out);
    };
    const auto swap_23 = [](TetMesh& m, const Tuple& t) {
        std::vector<Tuple> out;
        return m.swap_face(t, out);
    };
    const auto swap_44 = [](TetMesh& m, const Tuple& t) {
        std::vector<Tuple> out;
        return m.swap_edge_44(t, out);
    };
    const auto swap_56 = [](TetMesh& m, const Tuple& t) {
        std::vector<Tuple> out;
        return m.swap_edge_56(t, out);
    };
    const auto split_face = [](TetMesh& m, const Tuple& t) {
        std::vector<Tuple> out;
        return m.split_face(t, out);
    };
    const auto split_tet = [](TetMesh& m, const Tuple& t) {
        std::vector<Tuple> out;
        return m.split_tet(t, out);
    };
    const auto smooth = [](TetMesh& m, const Tuple& t) { return m.smooth_vertex(t); };

    const int passed_1 = check_dry_runs(three, edges, split_edge, true);
    CHECK(passed_1 > 0);
    // The link condition is part of the change itself (collapse_edge_conn), so a collapse's dry
    // run stops before it and may pass what the real collapse refuses.
    const int passed_2 = check_dry_runs(three, edges, collapse_edge, false);
    CHECK(passed_2 > 0);
    const int passed_3 = check_dry_runs(three, edges, swap_32, true);
    CHECK(passed_3 == 1);
    const int passed_4 = check_dry_runs(two, faces, swap_23, true);
    CHECK(passed_4 == 1);
    const int passed_5 = check_dry_runs(four, edges, swap_44, true);
    CHECK(passed_5 == 1);
    const int passed_6 = check_dry_runs(five, edges, swap_56, true);
    CHECK(passed_6 == 1);
    const int passed_7 = check_dry_runs(three, faces, split_face, true);
    CHECK(passed_7 > 0);
    const int passed_8 = check_dry_runs(three, tets, split_tet, true);
    CHECK(passed_8 > 0);
    const int passed_9 = check_dry_runs(three, vertices, smooth, true);
    CHECK(passed_9 > 0);

    SECTION("a dry run that passes is reported as screened, not committed")
    {
        auto m = three();
        const Tuple e = m->tuple_from_edge({{0, 1}});
        {
            OperationDryRunScope scope;
            std::vector<Tuple> out;
            REQUIRE(m->swap_edge(e, out));
        }
        using K = TetMesh::OpKind;
        using E = TetMesh::OpEvent;
        CHECK(m->count(K::swap_32, E::attempt) == 1);
        CHECK(m->count(K::swap_32, E::screened) == 1);
        CHECK(m->count(K::swap_32, E::committed) == 0);
    }
}
