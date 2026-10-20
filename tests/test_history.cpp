#include <wmtk/TriMesh.h>
#include <wmtk/history/Replay.hpp>
#include <wmtk/history/SurfaceHistory.hpp>

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <random>
#include <set>
#include <string>

using namespace wmtk;

namespace {

/// A rows x cols grid of vertices on the unit square, two triangles per cell.
std::vector<Eigen::Vector3d> make_grid(TriMesh& m, int rows, int cols)
{
    std::vector<Eigen::Vector3d> pos;
    std::vector<std::array<size_t, 3>> tris;
    const auto id = [cols](int i, int j) { return size_t(i * cols + j); };
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            pos.emplace_back(double(i) / (rows - 1), double(j) / (cols - 1), 0.0);
        }
    }
    for (int i = 0; i + 1 < rows; ++i) {
        for (int j = 0; j + 1 < cols; ++j) {
            tris.push_back({{id(i, j), id(i + 1, j), id(i + 1, j + 1)}});
            tris.push_back({{id(i, j), id(i + 1, j + 1), id(i, j + 1)}});
        }
    }
    m.init(pos.size(), tris);
    return pos;
}

/// Whether the replay holds the mesh's state: the same live vertex and face slots, and every
/// live face with the same corners in the same order.
bool same_state(const TriMesh& m, const history::Replay& r, std::string& why)
{
    std::vector<uint8_t> v_alive(m.vert_capacity(), 0), f_alive(m.tri_capacity(), 0);
    for (const auto& v : m.get_vertices()) v_alive[v.vid(m)] = 1;
    for (const auto& f : m.get_faces()) f_alive[f.fid(m)] = 1;

    const size_t nv = std::max(v_alive.size(), r.vertex_alive().size());
    for (size_t v = 0; v < nv; ++v) {
        const bool a = v < v_alive.size() && v_alive[v];
        const bool b = v < r.vertex_alive().size() && r.vertex_alive()[v];
        if (a != b) {
            why = "vertex " + std::to_string(v) +
                  (a ? " live in the mesh only" : " live in the replay only");
            return false;
        }
    }
    const size_t nf = std::max(f_alive.size(), r.face_alive().size());
    for (size_t f = 0; f < nf; ++f) {
        const bool a = f < f_alive.size() && f_alive[f];
        const bool b = f < r.face_alive().size() && r.face_alive()[f];
        if (a != b) {
            why = "face " + std::to_string(f) +
                  (a ? " live in the mesh only" : " live in the replay only");
            return false;
        }
        if (!a) continue;
        const auto t = m.oriented_tri_vids(f);
        const auto& u = r.faces()[f];
        if (t[0] != u[0] || t[1] != u[1] || t[2] != u[2]) {
            why = "face " + std::to_string(f) + " differs";
            return false;
        }
    }
    return true;
}

/// Runs `n_ops` random operations on `m`, replaying each recorded edit as it lands and
/// requiring the replay to match the mesh after every one.
void run_and_check(
    TriMesh& m,
    const history::Recorder& rec,
    history::Replay& r,
    int n_ops,
    unsigned seed)
{
    std::mt19937 rng(seed);
    size_t applied = 0;
    size_t committed = 0;
    for (int i = 0; i < n_ops; ++i) {
        const int dice = int(rng() % 20);
        std::vector<TriMesh::Tuple> out;
        if (dice == 0) {
            m.consolidate_mesh();
        } else if (dice < 4) {
            const auto faces = m.get_faces();
            m.split_face(faces[rng() % faces.size()], out);
        } else {
            const auto edges = m.get_edges();
            const auto& e = edges[rng() % edges.size()];
            if (dice < 9) {
                m.split_edge(e, out);
            } else if (dice < 15) {
                m.collapse_edge(e, out);
            } else {
                m.swap_edge(e, out);
            }
        }
        const auto& edits = rec.history().edits;
        for (; applied < edits.size(); ++applied) r.apply(edits[applied]);
        committed = applied;
        std::string why;
        const bool same = same_state(m, r, why);
        INFO("after operation " << i << ", edit " << committed << ": " << why);
        REQUIRE(same);
    }
    REQUIRE(committed > size_t(n_ops) / 4);
    std::set<history::EditKind> kinds;
    for (const auto& e : rec.history().edits) kinds.insert(e.kind);
    // Every kind the TriMesh operations record: the test exercises all of them.
    REQUIRE(kinds.size() == 5);
}

} // namespace

TEST_CASE("history-replay-matches-random-operations", "[history]")
{
    for (const bool link : {true, false}) {
        for (unsigned seed = 1; seed <= 5; ++seed) {
            INFO("link condition " << link << ", seed " << seed);
            TriMesh m;
            const auto pos = make_grid(m, 9, 9);
            m.set_use_link_condition(link);
            const auto rec = m.start_history([&pos](size_t v) {
                return v < pos.size() ? pos[v] : Eigen::Vector3d::Zero().eval();
            });
            history::Replay r(rec->history());
            run_and_check(m, *rec, r, 400, seed);
        }
    }
}

TEST_CASE("history-save-load-round-trip", "[history]")
{
    TriMesh m;
    const auto pos = make_grid(m, 5, 5);
    const auto rec = m.start_history([&pos](size_t v) { return pos[v]; });
    std::vector<TriMesh::Tuple> out;
    m.split_edge(m.get_edges()[3], out);
    m.collapse_edge(m.get_edges()[7], out);
    m.swap_edge(m.get_edges()[11], out);
    m.consolidate_mesh();
    rec->history().edits.push_back(
        {history::EditKind::move_vertices, {}, {}, {{2, Eigen::Vector3d(0.25, -1, 3.5)}}});
    rec->history().metadata = R"({"application": "test"})";

    const auto path =
        (std::filesystem::temp_directory_path() / "wmtk_history_round_trip.bin").string();
    rec->history().save(path);
    const history::SurfaceHistory h = history::SurfaceHistory::load(path);
    std::filesystem::remove(path);

    const history::SurfaceHistory& g = rec->history();
    CHECK(h.vertices == g.vertices);
    CHECK(h.vertex_alive == g.vertex_alive);
    CHECK(h.faces == g.faces);
    CHECK(h.face_alive == g.face_alive);
    CHECK(h.metadata == g.metadata);
    REQUIRE(h.edits.size() == g.edits.size());
    for (size_t i = 0; i < h.edits.size(); ++i) {
        CHECK(h.edits[i].kind == g.edits[i].kind);
        CHECK(h.edits[i].v == g.edits[i].v);
        CHECK(h.edits[i].faces == g.edits[i].faces);
        REQUIRE(h.edits[i].moves.size() == g.edits[i].moves.size());
        for (size_t k = 0; k < h.edits[i].moves.size(); ++k) {
            CHECK(h.edits[i].moves[k].vid == g.edits[i].moves[k].vid);
            CHECK(h.edits[i].moves[k].pos == g.edits[i].moves[k].pos);
        }
    }
}
