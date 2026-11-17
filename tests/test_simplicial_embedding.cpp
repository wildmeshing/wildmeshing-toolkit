#include <catch2/catch_test_macros.hpp>

#include <wmtk/TetMesh.h>
#include <wmtk/TriMesh.h>
#include <wmtk/utils/SimplicialEmbedding.hpp>

#include <algorithm>
#include <array>
#include <set>

using namespace wmtk;
using namespace wmtk::utils;

// The shared simplicial embedding on the base meshes (connectivity only): K is a set of simplices
// by their sorted vertex ids, so the simplices a split makes (they all hold the new vertex) are
// never in K.
namespace {

template <size_t N>
std::array<size_t, N> sorted(std::array<size_t, N> a)
{
    std::sort(a.begin(), a.end());
    return a;
}

struct SetComplex
{
    std::set<size_t> v;
    std::set<std::array<size_t, 2>> e;
    std::set<std::array<size_t, 3>> f;
    std::set<std::array<size_t, 4>> t;

    /// K is the closure of the given simplices
    void add_edge(size_t a, size_t b)
    {
        e.insert(sorted<2>({{a, b}}));
        v.insert(a);
        v.insert(b);
    }
    void add_face(size_t a, size_t b, size_t c)
    {
        f.insert(sorted<3>({{a, b, c}}));
        add_edge(a, b);
        add_edge(b, c);
        add_edge(c, a);
    }

    TriComplex tri(const TriMesh& m) const
    {
        return {
            [this](const size_t vid) { return v.count(vid) > 0; },
            [this, &m](const TriMesh::Tuple& t) {
                return e.count(sorted<2>({{t.vid(m), t.switch_vertex(m).vid(m)}})) > 0;
            },
            [this, &m](const TriMesh::Tuple& t) {
                return f.count(sorted<3>(m.oriented_tri_vids(t))) > 0;
            }};
    }
    TetComplex tet(const TetMesh& m) const
    {
        return {
            [this](const size_t vid) { return v.count(vid) > 0; },
            [this, &m](const TetMesh::Tuple& x) {
                return e.count(sorted<2>({{x.vid(m), m.switch_vertex(x).vid(m)}})) > 0;
            },
            [this, &m](const TetMesh::Tuple& x) {
                return f.count(sorted<3>(m.get_face_vids(x))) > 0;
            },
            [this, &m](const TetMesh::Tuple& x) {
                return t.count(sorted<4>(m.oriented_tet_vids(x))) > 0;
            }};
    }
};

} // namespace

TEST_CASE("simplicial_embedding_tri_edge", "[simplicial_embedding][2d]")
{
    // a square cut along the diagonal 0-2; K = the square's boundary, so the diagonal joins two
    // vertices of K outside it
    TriMesh m;
    m.init(4, {{{0, 1, 2}}, {{0, 2, 3}}});
    SetComplex K;
    K.add_edge(0, 1);
    K.add_edge(1, 2);
    K.add_edge(2, 3);
    K.add_edge(3, 0);
    CHECK(count_embedding_violations(m, K.tri(m)) == 2);

    const SimplicialEmbeddingStats s = simplicial_embedding(m, K.tri(m));
    CHECK(s.faces_split == 0);
    CHECK(s.edges_split == 1);
    CHECK(s.refused == 0);
    CHECK(m.get_faces().size() == 4);
    CHECK(count_embedding_violations(m, K.tri(m)) == 0);
    CHECK(m.check_mesh_connectivity_validity());
}

TEST_CASE("simplicial_embedding_tri_face", "[simplicial_embedding][2d]")
{
    // K = the three edges of face 0-1-2 but not the face, with a neighbour on each side
    TriMesh m;
    m.init(6, {{{0, 1, 2}}, {{1, 0, 3}}, {{2, 1, 4}}, {{0, 2, 5}}});
    SetComplex K;
    K.add_edge(0, 1);
    K.add_edge(1, 2);
    K.add_edge(2, 0);
    CHECK(count_embedding_violations(m, K.tri(m)) == 1);

    const SimplicialEmbeddingStats s = simplicial_embedding(m, K.tri(m));
    CHECK(s.faces_split == 1);
    CHECK(s.edges_split == 0);
    CHECK(s.refused == 0);
    CHECK(m.get_faces().size() == 6);
    CHECK(count_embedding_violations(m, K.tri(m)) == 0);

    // already embedded: nothing to do
    const SimplicialEmbeddingStats again = simplicial_embedding(m, K.tri(m));
    CHECK(again.faces_split + again.edges_split == 0);
}

TEST_CASE("simplicial_embedding_tet", "[simplicial_embedding][3d]")
{
    SECTION("tet whose faces are all in K")
    {
        TetMesh m;
        m.init(4, {{{0, 1, 2, 3}}});
        SetComplex K;
        K.add_face(0, 1, 2);
        K.add_face(0, 1, 3);
        K.add_face(0, 2, 3);
        K.add_face(1, 2, 3);
        CHECK(count_embedding_violations(m, K.tet(m)) == 1);
        const SimplicialEmbeddingStats s = simplicial_embedding(m, K.tet(m));
        CHECK(s.tets_split == 1);
        CHECK(s.faces_split == 0);
        CHECK(s.edges_split == 0);
        CHECK(m.get_tets().size() == 4);
        CHECK(count_embedding_violations(m, K.tet(m)) == 0);
        CHECK(m.check_mesh_connectivity_validity());
    }
    SECTION("face whose edges are all in K")
    {
        TetMesh m;
        m.init(5, {{{0, 1, 2, 3}}, {{0, 2, 1, 4}}});
        SetComplex K;
        K.add_edge(0, 1);
        K.add_edge(1, 2);
        K.add_edge(2, 0);
        CHECK(count_embedding_violations(m, K.tet(m)) == 2);
        const SimplicialEmbeddingStats s = simplicial_embedding(m, K.tet(m));
        CHECK(s.tets_split == 0);
        CHECK(s.faces_split == 1);
        CHECK(s.edges_split == 0);
        CHECK(m.get_tets().size() == 6);
        CHECK(count_embedding_violations(m, K.tet(m)) == 0);
        CHECK(m.check_mesh_connectivity_validity());
    }
    SECTION("edge whose vertices are in K")
    {
        TetMesh m;
        m.init(5, {{{0, 1, 2, 3}}, {{0, 2, 1, 4}}});
        SetComplex K;
        K.v = {0, 3};
        CHECK(count_embedding_violations(m, K.tet(m)) == 1);
        const SimplicialEmbeddingStats s = simplicial_embedding(m, K.tet(m));
        CHECK(s.tets_split + s.faces_split == 0);
        CHECK(s.edges_split == 1);
        CHECK(m.get_tets().size() == 3);
        CHECK(count_embedding_violations(m, K.tet(m)) == 0);
        CHECK(m.check_mesh_connectivity_validity());
    }
}
