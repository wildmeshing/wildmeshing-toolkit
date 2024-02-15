#include <wmtk/TetMesh.hpp>

#include <catch2/catch_test_macros.hpp>
#include "tools/DEBUG_TetMesh.hpp"

using namespace wmtk;
using namespace wmtk::tests_3d;

TEST_CASE("create TetMesh", "[tuple_3d]")
{
    DEBUG_TetMesh m;
    {
        RowVectors4l tets;
        tets.resize(1, 4);
        tets.row(0) = Eigen::Matrix<int64_t, 4, 1>{0, 1, 2, 3};
        m.initialize(tets);
    }


    const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
    REQUIRE(vertices.size() == 4);
    const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
    REQUIRE(edges.size() == 6);
    const std::vector<Tuple> faces = m.get_all(PrimitiveType::Triangle);
    REQUIRE(faces.size() == 4);
    const std::vector<Tuple> tets = m.get_all(PrimitiveType::Tetrahedron);
    REQUIRE(tets.size() == 1);
}

TEST_CASE("TetMesh with 1 tet", "[tuple_3d][.]")
{
    DEBUG_TetMesh m;
    {
        RowVectors4l tets;
        tets.resize(1, 4);
        tets.row(0) = Eigen::Matrix<int64_t, 4, 1>{0, 1, 2, 3};
        m.initialize(tets);
    }

    SECTION("vertices")
    {
        const std::vector<Tuple> vertices = m.get_all(PrimitiveType::Vertex);
        REQUIRE(vertices.size() == 4);
        CHECK(m.id(vertices[0], PrimitiveType::Vertex) == 0);
        CHECK(m.id(vertices[1], PrimitiveType::Vertex) == 1);
        CHECK(m.id(vertices[2], PrimitiveType::Vertex) == 2);
        CHECK(m.id(vertices[3], PrimitiveType::Vertex) == 3);
        CHECK(m.id(vertices[0], PrimitiveType::Tetrahedron) == 0);
        CHECK(m.id(vertices[1], PrimitiveType::Tetrahedron) == 0);
        CHECK(m.id(vertices[2], PrimitiveType::Tetrahedron) == 0);
        CHECK(m.id(vertices[3], PrimitiveType::Tetrahedron) == 0);
    }
    SECTION("edges")
    {
        const std::vector<Tuple> edges = m.get_all(PrimitiveType::Edge);
        REQUIRE(edges.size() == 6);
        //  TODO add test for edge ids
        CHECK(false);
        CHECK(false);
        CHECK(false);
        CHECK(false);
    }
    SECTION("faces")
    {
        const std::vector<Tuple> faces = m.get_all(PrimitiveType::Triangle);
        REQUIRE(faces.size() == 4);
        //  TODO add test for face ids
        CHECK(false);
        CHECK(false);
        CHECK(false);
    }
    SECTION("3Drahedron")
    {
        const std::vector<Tuple> tets = m.get_all(PrimitiveType::Tetrahedron);
        REQUIRE(tets.size() == 1);
        CHECK(m.id(tets[0], PrimitiveType::Tetrahedron) == 0);
    }
}

TEST_CASE("3D_switch_vertex", "[tuple_3d]")
{
    TetMesh m;
    {
        RowVectors4l tets;
        tets.resize(1, 4);
        tets.row(0) = Eigen::Matrix<int64_t, 4, 1>{0, 1, 2, 3};
        m.initialize(tets);
    }


    // TetMesh mesh;
    // mesh.init(4, {{{0, 1, 2, 3}}});
    // const auto tuple = mesh.tuple_from_edge(0, 0);
    // REQUIRE(tuple.vid(mesh) == 0);
    //
    // const auto t1 = mesh.switch_vertex(tuple);
    // REQUIRE(t1.vid(mesh) == 1);
    // int eid1 = tuple.eid(mesh);
    // int eid2 = t1.eid(mesh);
    // REQUIRE(eid1 == eid2);
    //
    // const auto t2 = mesh.switch_vertex(t1);
    // REQUIRE(tuple.vid(mesh) == t2.vid(mesh));
}

TEST_CASE("3D_switch_edge", "[tuple_3d][.]")
{
    REQUIRE(false);

    // TetMesh mesh;
    // mesh.init(4, {{{0, 1, 2, 3}}});
    // const auto tuple = mesh.tuple_from_vertex(0);
    //
    // int eid1 = tuple.eid(mesh);
    // const auto t1_tmp = mesh.switch_edge(tuple);
    // const auto t1 = mesh.switch_vertex(t1_tmp);
    // const auto t2_tmp = mesh.switch_edge(t1);
    // const auto t2 = mesh.switch_vertex(t2_tmp);
    // const auto t3 = mesh.switch_edge(t2);
    // int eid2 = t3.eid(mesh);
    // REQUIRE(eid1 == eid2);
}

TEST_CASE("3D_switch_face", "[tuple_3d][.]")
{
    REQUIRE(false);

    // TetMesh mesh;
    // mesh.init(4, {{{0, 1, 2, 3}}});
    // const auto tuple = mesh.tuple_from_vertex(0);
    //
    // int fid1 = tuple.fid(mesh);
    // const auto t1 = mesh.switch_face(tuple);
    // const auto t2 = mesh.switch_face(t1);
    // const auto t3 = mesh.switch_face(t2);
    // const auto t4 = mesh.switch_face(t3);
    // int fid2 = t4.fid(mesh);
    // REQUIRE(fid1 == fid2);
}

TEST_CASE("3D_switch_tet", "[tuple_3d][.]")
{
    REQUIRE(false);

    // TetMesh mesh;
    // mesh.init(5, {{{0, 1, 2, 3}}, {{0, 1, 2, 4}}});
    // const auto tuple = mesh.tuple_from_face(0, 0);
    //
    // int tid1 = tuple.tid(mesh);
    // const auto t1 = mesh.switch_tetrahedron(tuple);
    // REQUIRE(t1.has_value());
    // const auto t2 = mesh.switch_tetrahedron(t1.value());
    // REQUIRE(t2.has_value());
    // int tid2 = t2.value().tid(mesh);
    // REQUIRE(tid1 == tid2);
}


TEST_CASE("3D_switch_face_tet", "[tuple_3d][.]")
{
    REQUIRE(false);

    // TetMesh m;
    // m.init(5, {{{0, 1, 2, 3}}, {{0, 1, 4, 2}}, {{0, 1, 3, 4}}});
    // auto e = m.tuple_from_edge(0, 3);
    //
    // e = e.switch_face(m);
    // auto edge0 = e.eid(m);
    // e = e.switch_tetrahedron(m).value();
    // auto edge1 = e.eid(m);
    // REQUIRE(edge0 == edge1);
}

TEST_CASE("3D_count_edge_on_boundary", "[tuple_3d][.]")
{
    REQUIRE(false);

    // TetMesh mesh;
    // mesh.init(5, {{{0, 1, 2, 3}}, {{0, 1, 4, 2}}, {{0, 1, 3, 4}}});
    // const auto edges = mesh.get_edges();
    // auto cnt = 0;
    // for (auto& e : edges) {
    //     if (e.is_boundary_edge(mesh)) cnt++;
    // }
    // REQUIRE(edges.size() == 10);
    // REQUIRE(cnt == 9);
}

TEST_CASE("3D_tuple_iterator", "[tuple_3d][.]")
{
    REQUIRE(false);

    // class IterableMesh : public TetMesh
    //{
    // public:
    //     struct EdgeIterator
    //     {
    //         using value_type = TetMesh::Tuple;
    //         using pointer = value_type*;
    //
    //         EdgeIterator(const TetMesh& m, value_type ptr)
    //             : m_tuple(ptr)
    //             , mesh(m){};
    //
    //         EdgeIterator operator++()
    //         {
    //             auto e = m_tuple.eid(mesh);
    //             for (auto ei = e + 1; ei < mesh.tet_capacity() * 6; ei++) {
    //                auto t = ei / 6, j = ei % 6;
    //                auto tup = mesh.tuple_from_edge(t, j);
    //                if (tup.is_valid(mesh) && tup.eid(mesh) == ei) {
    //                    m_tuple = tup;
    //                    return *this;
    //                };
    //            }
    //            m_tuple = TetMesh::Tuple();
    //            return *this;
    //        };
    //
    //        bool operator==(const EdgeIterator& b) const { return m_tuple == b.m_tuple; }
    //        bool operator!=(const EdgeIterator& b) const { return !(*this == b); }
    //
    //        value_type operator*() { return m_tuple; };
    //        pointer operator->() { return &m_tuple; };
    //
    //    private:
    //        value_type m_tuple;
    //        const TetMesh& mesh;
    //    };
    //
    //    EdgeIterator begin() const
    //    {
    //        for (auto i = 0; i < vert_capacity(); i++) {
    //            auto tup = tuple_from_vertex(i);
    //            if (tup.is_valid(*this)) return EdgeIterator(*this, tup);
    //        }
    //        return EdgeIterator(*this, Tuple());
    //    };
    //    EdgeIterator end() const { return EdgeIterator(*this, Tuple()); };
    //};
    //
    // IterableMesh mesh;
    // mesh.init(5, {{{0, 1, 2, 3}}, {{0, 1, 4, 2}}, {{0, 1, 3, 4}}});
    // auto cnt = 0;
    // REQUIRE(mesh.begin() != mesh.end());
    //
    // for (auto t : mesh) {
    //    cnt++;
    //}
    // REQUIRE(cnt == 10);
}
