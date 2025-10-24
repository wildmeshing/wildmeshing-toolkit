#include <catch2/catch_test_macros.hpp>

#include <igl/Timer.h>
#include <wmtk/TetMesh.h>
#include <wmtk/utils/examples/TetMesh_examples.hpp>

using namespace wmtk;
using namespace wmtk::utils::examples::tet;

TEST_CASE("tet_get_simplex_functions", "[test_tuple][TetMesh]")
{
    TetMeshVT VT;
    int64_t n_verts = -1;
    int64_t n_edges = -1;
    int64_t n_tris = -1;
    int64_t n_tets = -1;
    SECTION("single_tet")
    {
        VT = single_tet();
        n_verts = 4;
        n_edges = 6;
        n_tris = 4;
        n_tets = 1;
    }
    SECTION("two_tets")
    {
        VT = two_tets();
        n_verts = 5;
        n_edges = 9;
        n_tris = 7;
        n_tets = 2;
    }
    SECTION("two_tets")
    {
        VT = two_tets();
        n_verts = 5;
        n_edges = 9;
        n_tris = 7;
        n_tets = 2;
    }
    SECTION("six_cycle_tets")
    {
        VT = six_cycle_tets();
        n_verts = 8;
        n_edges = 19;
        n_tris = 18;
        n_tets = 6;
    }

    TetMesh mesh;
    mesh.init(VT.T);
    const auto edges = mesh.get_edges();

    CHECK(mesh.get_vertices().size() == n_verts);
    CHECK(mesh.get_edges().size() == n_edges);
    CHECK(mesh.get_faces().size() == n_tris);
    CHECK(mesh.get_tets().size() == n_tets);
}

TEST_CASE("switch_vertex", "[test_tuple][TetMesh]")
{
    TetMesh mesh;
    mesh.init(4, {{{0, 1, 2, 3}}});
    const auto tuple = mesh.tuple_from_edge(0, 0);
    REQUIRE(tuple.vid(mesh) == 0);

    const auto t1 = mesh.switch_vertex(tuple);
    REQUIRE(t1.vid(mesh) == 1);
    int eid1 = tuple.eid(mesh);
    int eid2 = t1.eid(mesh);
    REQUIRE(eid1 == eid2);

    const auto t2 = mesh.switch_vertex(t1);
    REQUIRE(tuple.vid(mesh) == t2.vid(mesh));
}

TEST_CASE("switch_edge", "[test_tuple][TetMesh]")
{
    TetMesh mesh;
    mesh.init(4, {{{0, 1, 2, 3}}});
    const auto tuple = mesh.tuple_from_vertex(0);

    int eid1 = tuple.eid(mesh);
    const auto t1_tmp = mesh.switch_edge(tuple);
    const auto t1 = mesh.switch_vertex(t1_tmp);
    const auto t2_tmp = mesh.switch_edge(t1);
    const auto t2 = mesh.switch_vertex(t2_tmp);
    const auto t3 = mesh.switch_edge(t2);
    int eid2 = t3.eid(mesh);
    REQUIRE(eid1 == eid2);
}

TEST_CASE("switch_face", "[test_tuple][TetMesh]")
{
    TetMesh mesh;
    mesh.init(4, {{{0, 1, 2, 3}}});
    const auto tuple = mesh.tuple_from_vertex(0);

    int fid1 = tuple.fid(mesh);
    const auto t1 = mesh.switch_face(tuple);
    const auto t2 = mesh.switch_face(t1);
    const auto t3 = mesh.switch_face(t2);
    const auto t4 = mesh.switch_face(t3);
    int fid2 = t4.fid(mesh);
    REQUIRE(fid1 == fid2);
}

TEST_CASE("switch_tet", "[test_tuple][TetMesh]")
{
    TetMesh mesh;
    mesh.init(5, {{{0, 1, 2, 3}}, {{0, 1, 2, 4}}});
    const auto tuple = mesh.tuple_from_face(0, 0);

    int tid1 = tuple.tid(mesh);
    const auto t1 = mesh.switch_tetrahedron(tuple);
    REQUIRE(t1.has_value());
    const auto t2 = mesh.switch_tetrahedron(t1.value());
    REQUIRE(t2.has_value());
    int tid2 = t2.value().tid(mesh);
    REQUIRE(tid1 == tid2);
}


TEST_CASE("switch_face_tet", "[test_tuple][TetMesh]")
{
    TetMesh m;
    m.init(5, {{{0, 1, 2, 3}}, {{0, 1, 4, 2}}, {{0, 1, 3, 4}}});
    auto e = m.tuple_from_edge(0, 3);

    e = e.switch_face(m);
    auto edge0 = e.eid(m);
    e = e.switch_tetrahedron(m).value();
    auto edge1 = e.eid(m);
    REQUIRE(edge0 == edge1);
}

TEST_CASE("count_edge_on_boundary", "[test_tuple][TetMesh]")
{
    TetMesh mesh;
    mesh.init(5, {{{0, 1, 2, 3}}, {{0, 1, 4, 2}}, {{0, 1, 3, 4}}});
    const auto edges = mesh.get_edges();
    auto cnt = 0;
    for (auto& e : edges) {
        if (e.is_boundary_edge(mesh)) cnt++;
    }
    REQUIRE(edges.size() == 10);
    REQUIRE(cnt == 9);
}

TEST_CASE("tuple_iterator", "[test_tuple][TetMesh]")
{
    class IterableMesh : public TetMesh
    {
    public:
        struct EdgeIterator
        {
            using value_type = TetMesh::Tuple;
            using pointer = value_type*;

            EdgeIterator(const TetMesh& m, value_type ptr)
                : m_tuple(ptr)
                , mesh(m) {};

            EdgeIterator operator++()
            {
                auto e = m_tuple.eid(mesh);
                for (auto ei = e + 1; ei < mesh.tet_capacity() * 6; ei++) {
                    auto t = ei / 6, j = ei % 6;
                    auto tup = mesh.tuple_from_edge(t, j);
                    if (tup.is_valid(mesh) && tup.eid(mesh) == ei) {
                        m_tuple = tup;
                        return *this;
                    };
                }
                m_tuple = TetMesh::Tuple();
                return *this;
            };

            bool operator==(const EdgeIterator& b) const { return m_tuple == b.m_tuple; }
            bool operator!=(const EdgeIterator& b) const { return !(*this == b); }

            value_type operator*() { return m_tuple; };
            pointer operator->() { return &m_tuple; };

        private:
            value_type m_tuple;
            const TetMesh& mesh;
        };

        EdgeIterator begin() const
        {
            for (auto i = 0; i < vert_capacity(); i++) {
                auto tup = tuple_from_vertex(i);
                if (tup.is_valid(*this)) return EdgeIterator(*this, tup);
            }
            return EdgeIterator(*this, Tuple());
        };
        EdgeIterator end() const { return EdgeIterator(*this, Tuple()); };
    };

    IterableMesh mesh;
    mesh.init(5, {{{0, 1, 2, 3}}, {{0, 1, 4, 2}}, {{0, 1, 3, 4}}});
    auto cnt = 0;
    REQUIRE(mesh.begin() != mesh.end());

    for (auto t : mesh) {
        cnt++;
    }
    REQUIRE(cnt == 10);
}

TEST_CASE("switch_tet_performance", "[test_tuple][TetMesh][performance][.]")
{
    using Tuple = TetMesh::Tuple;

    // compare the two different implementations
    TetMeshVT VT = six_cycle_tets();

    TetMesh mesh;
    mesh.init(VT.T);

    const auto faces = mesh.get_faces();

    const size_t reps = 1e6;

    logger().info("Ensure equalness");
    for (const Tuple& t : faces) {
        const auto t_opp1 = t.switch_tetrahedron_slow(mesh);
        const auto t_opp2 = t.switch_tetrahedron(mesh);

        REQUIRE(t_opp1.has_value() == t_opp2.has_value());

        if (t_opp1) {
            const Tuple t1 = t_opp1.value();
            const Tuple t2 = t_opp2.value();
            CHECK(t1 == t2);
        }
    }

    logger().info("Test start");
    igl::Timer timer;
    timer.start();
    size_t checksum_old = 0;
    for (size_t n = 0; n < reps; n++) {
        for (const Tuple& t : faces) {
            const auto t_opp = t.switch_tetrahedron_slow(mesh);
            if (t_opp) {
                checksum_old += t_opp.value().tid(mesh);
            }
        }
    }
    timer.stop();
    const double time_old = timer.getElapsedTimeInMilliSec();
    logger().info("Test end {}", checksum_old);
    logger().info("time {}ms", time_old);

    logger().info("Test start");
    timer.start();
    size_t checksum_new = 0;
    for (size_t n = 0; n < reps; n++) {
        for (const Tuple& t : faces) {
            const auto t_opp = t.switch_tetrahedron(mesh);
            if (t_opp) {
                checksum_new += t_opp.value().tid(mesh);
            }
        }
    }
    timer.stop();
    const double time_new = timer.getElapsedTimeInMilliSec();
    logger().info("Test end {}", checksum_new);
    logger().info("time {}ms", time_new);
    logger().info("improvement: {}", time_old / time_new);
}