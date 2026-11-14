#include <wmtk/TetMesh.h>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <map>
#include <vector>

using namespace wmtk;

namespace {

// add_vertex_slot and replace_tets are protected: a mesh class's operations use them
class ReplacingMesh : public TetMesh
{
public:
    using TetMesh::add_vertex_slot;
    using TetMesh::replace_tets;
};

using Tets = std::vector<std::array<size_t, 4>>;

// the live tets' vertices and the live vertices' tets: what a refusal must leave as it was
std::pair<Tets, std::vector<std::vector<size_t>>> state(const TetMesh& m)
{
    std::pair<Tets, std::vector<std::vector<size_t>>> s;
    for (const auto& t : m.get_tets()) {
        s.first.push_back(m.oriented_tet_vids(t.tid(m)));
    }
    for (const auto& v : m.get_vertices()) {
        s.second.push_back(m.get_one_ring_tids_for_vertex(v.vid(m)));
    }
    return s;
}

// every face of the mesh in at most two tets, with opposite orientations, and each vertex's tets
// exactly those holding it
void check_valid(const TetMesh& m)
{
    std::map<std::array<size_t, 3>, std::array<int, 2>> faces;
    std::map<size_t, std::vector<size_t>> holding;
    for (const auto& t : m.get_tets()) {
        const size_t tid = t.tid(m);
        const auto vs = m.oriented_tet_vids(tid);
        static constexpr int local[4][3] = {{1, 2, 3}, {0, 3, 2}, {0, 1, 3}, {0, 2, 1}};
        for (const auto& l : local) {
            std::array<size_t, 3> f{{vs[l[0]], vs[l[1]], vs[l[2]]}};
            std::rotate(f.begin(), std::min_element(f.begin(), f.end()), f.end());
            const bool positive = f[1] < f[2];
            std::sort(f.begin(), f.end());
            ++faces[f][positive ? 0 : 1];
        }
        for (const size_t v : vs) {
            holding[v].push_back(tid);
        }
    }
    for (const auto& [f, n] : faces) {
        CHECK(n[0] <= 1);
        CHECK(n[1] <= 1);
    }
    for (const auto& v : m.get_vertices()) {
        std::vector<size_t> expected = holding[v.vid(m)];
        std::vector<size_t> got = m.get_one_ring_tids_for_vertex(v.vid(m));
        std::sort(expected.begin(), expected.end());
        std::sort(got.begin(), got.end());
        CHECK(!got.empty());
        CHECK(got == expected);
    }
}

// tet (0, 1, 2, 3) as the four tets around vertex v, each in its orientation
Tets star(const size_t v)
{
    return {{{v, 1, 2, 3}}, {{0, v, 2, 3}}, {{0, 1, v, 3}}, {{0, 1, 2, v}}};
}

} // namespace

TEST_CASE("replace_tets", "[TetMesh][replace_tets]")
{
    SECTION("a tet by the four around a new vertex")
    {
        ReplacingMesh m;
        m.init(4, {{{0, 1, 2, 3}}});
        const size_t v = m.add_vertex_slot();
        std::vector<size_t> ids;
        REQUIRE(m.replace_tets({0}, star(v), ids));
        CHECK(ids.size() == 4);
        CHECK(ids[0] == 0); // the old tet's slot first
        CHECK(m.get_tets().size() == 4);
        CHECK(m.get_vertices().size() == 5);
        check_valid(m);
    }

    SECTION("two tets by three around an edge, and back")
    {
        // (4, 1, 3, 2) meets (0, 1, 2, 3) on face (1, 2, 3), the other way round
        ReplacingMesh m;
        m.init(5, {{{0, 1, 2, 3}}, {{4, 1, 3, 2}}});
        std::vector<size_t> ids;
        REQUIRE(m.replace_tets({0, 1}, {{{0, 4, 2, 3}}, {{0, 4, 1, 2}}, {{0, 4, 3, 1}}}, ids));
        CHECK(ids.size() == 3);
        CHECK(m.get_tets().size() == 3);
        check_valid(m);
        std::vector<size_t> back;
        REQUIRE(m.replace_tets(ids, {{{0, 1, 2, 3}}, {{4, 1, 3, 2}}}, back));
        CHECK(m.get_tets().size() == 2);
        check_valid(m);
    }

    SECTION("a vertex left in no tet is removed")
    {
        ReplacingMesh m;
        m.init(4, {{{0, 1, 2, 3}}});
        const size_t v = m.add_vertex_slot();
        std::vector<size_t> ids;
        REQUIRE(m.replace_tets({0}, star(v), ids));
        std::vector<size_t> back;
        REQUIRE(m.replace_tets(ids, {{{0, 1, 2, 3}}}, back));
        CHECK(m.get_tets().size() == 1);
        CHECK(m.get_vertices().size() == 4);
        check_valid(m);
    }

    SECTION("a refusal changes nothing and releases the slots")
    {
        ReplacingMesh m;
        m.init(5, {{{0, 1, 2, 3}}, {{4, 1, 3, 2}}});
        const auto before = state(m);
        const auto refused = [&](const std::vector<size_t>& old_tids, const Tets& new_tets) {
            std::vector<size_t> ids;
            CHECK_FALSE(m.replace_tets(old_tids, new_tets, ids));
            CHECK(ids.empty());
            CHECK(state(m) == before);
        };

        Tets tets = star(m.add_vertex_slot());
        tets.pop_back();
        refused({0}, tets); // a gap: a face of the old boundary is missing

        tets = star(m.add_vertex_slot());
        tets.push_back(tets[0]);
        refused({0}, tets); // a tet twice

        tets = star(m.add_vertex_slot());
        std::swap(tets[0][2], tets[0][3]);
        refused({0}, tets); // a tet the wrong way round

        tets = star(m.add_vertex_slot());
        tets[0][1] = tets[0][0];
        refused({0}, tets); // a vertex twice in a tet

        refused({0}, {{{0, 1, 2, 99}}}); // a vertex not in the mesh

        tets = star(m.add_vertex_slot());
        m.add_vertex_slot();
        refused({0}, tets); // a slot no new tet uses

        refused({0, 0}, {{{0, 1, 2, 3}}}); // an old tet twice
        refused({7}, {{{0, 1, 2, 3}}}); // an old tet not in the mesh

        // the cone from vertex 0 over tet 1's boundary has that boundary, but faces inside it,
        // such as (0, 1, 3), are faces of tet 0, outside it
        refused({1}, {{{0, 1, 3, 2}}, {{0, 4, 2, 3}}, {{0, 4, 1, 2}}, {{0, 4, 3, 1}}});

        CHECK(m.get_vertices().size() == 5);
        check_valid(m);
    }
}
