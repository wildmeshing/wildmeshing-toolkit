// check_surface_folds: what it refuses, on a mesh small enough to see, and what it keeps on a
// whole run.

#include <wmtk/components/tetwild/TetWildMesh.h>
#include <wmtk/components/tetwild/tetwild.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/history/History.hpp>
#include <wmtk/history/Replay.hpp>
#include <wmtk/utils/predicates.hpp>

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <map>

namespace fs = std::filesystem;
using namespace wmtk;
using namespace wmtk::components::tetwild;

namespace {

/// Whole runs of the application, too slow for the Debug CI jobs: Release builds only.
#ifdef NDEBUG
const std::string release_only = "";
#else
const std::string release_only = "[.]";
#endif

/// The vertex at (i, j, k) of a 2 x 2 x 1 block of unit cubes.
size_t block_vid(size_t i, size_t j, size_t k)
{
    return (k * 3 + j) * 3 + i;
}

/// A 2 x 2 x 1 block of unit cubes, six tets each, whose top (z = 1) is the tracked surface,
/// facing up.
void build_block(TetWildMesh& mesh)
{
    static const std::array<std::array<int, 4>, 6> KUHN = {{
        {{0, 1, 3, 7}},
        {{0, 1, 5, 7}},
        {{0, 4, 5, 7}},
        {{0, 2, 3, 7}},
        {{0, 2, 6, 7}},
        {{0, 4, 6, 7}},
    }};
    const auto pos = [](size_t v) {
        return Vector3d(double(v % 3), double((v / 3) % 3), double(v / 9));
    };
    std::vector<std::array<size_t, 4>> tets;
    for (size_t j = 0; j < 2; ++j) {
        for (size_t i = 0; i < 2; ++i) {
            for (const auto& t : KUHN) {
                std::array<size_t, 4> vs;
                for (int c = 0; c < 4; ++c) {
                    vs[c] = block_vid(i + (t[c] & 1), j + ((t[c] >> 1) & 1), (t[c] >> 2) & 1);
                }
                const Vector3d a = pos(vs[0]), b = pos(vs[1]), c = pos(vs[2]), d = pos(vs[3]);
                if ((b - a).cross(c - a).dot(d - a) < 0) std::swap(vs[1], vs[2]);
                tets.push_back(vs);
            }
        }
    }
    mesh.init(18, tets);
    std::vector<VertexAttributes> va(18);
    for (size_t v = 0; v < 18; ++v) {
        va[v].m_posf = pos(v);
        va[v].set_pos_to_posf();
        va[v].m_is_rounded = true;
        va[v].m_is_on_surface = v >= 9;
        va[v].m_sizing_scalar = 1.0;
    }
    std::vector<TetAttributes> ta(tets.size());
    mesh.create_mesh_attributes(va, ta);
    for (const auto& t : tets) {
        for (const auto& f : std::array<std::array<size_t, 3>, 4>{
                 {{{t[0], t[1], t[2]}},
                  {{t[0], t[1], t[3]}},
                  {{t[0], t[2], t[3]}},
                  {{t[1], t[2], t[3]}}}}) {
            if (f[0] < 9 || f[1] < 9 || f[2] < 9) continue;
            const auto [_, fid] = mesh.tuple_from_face(f);
            auto& fa = mesh.m_face_attribute[fid];
            fa.m_is_surface_fs = true;
            const Vector3d n = (pos(f[1]) - pos(f[0])).cross(pos(f[2]) - pos(f[0]));
            fa.set_orientation_along(f, n.z() > 0 ? 1 : -1);
        }
    }
}

/// The faces of each vertex's star not facing along the star's area vectors summed, over all
/// vertices: the folds check_surface_folds counts.
size_t star_folds(const history::Replay& r)
{
    const auto& V = r.vertices();
    std::map<uint64_t, std::vector<uint64_t>> stars;
    for (uint64_t f = 0; f < r.faces().size(); ++f) {
        if (!r.face_alive()[f]) continue;
        for (const uint64_t v : r.faces()[f]) stars[v].push_back(f);
    }
    size_t folds = 0;
    for (const auto& [v, star] : stars) {
        Eigen::Vector3d n = Eigen::Vector3d::Zero();
        for (const uint64_t f : star) {
            const auto& t = r.faces()[f];
            n += (V[t[1]] - V[t[0]]).cross(V[t[2]] - V[t[0]]);
        }
        for (const uint64_t f : star) {
            const auto& t = r.faces()[f];
            folds += !utils::predicates::faces_along(V[t[0]], V[t[1]], V[t[2]], n);
        }
    }
    return folds;
}

/// Runs tetwild on `model` and returns the folds of its tracked surface right after the
/// insertion and at the end, from its surface history.
std::pair<size_t, size_t>
folds_of_run(const std::string& model, const bool check, const int threads)
{
    const fs::path dir = fs::current_path() / "tetwild_surface_folds_test";
    fs::create_directories(dir);
    const std::string name = fs::path(model).stem().string() + (check ? "_on" : "_off");
    const fs::path hist = dir / (name + ".wmtkhist");
    wmtk::components::tetwild::tetwild({
        {"application", "tetwild"},
        {"input", {(fs::path(WMTK_DATA_DIR) / "models" / model).string()}},
        {"output", (dir / name).string()},
        {"history_output", hist.string()},
        {"check_surface_folds", check},
        {"num_threads", threads},
        {"max_iterations", 10},
        {"write_vtu", false},
    });
    const auto h = history::SurfaceHistory::load(hist.string());
    history::Replay r(h);
    size_t inserted = 0;
    for (const auto& e : h.edits) {
        r.apply(e);
        if (e.kind == history::EditKind::embed) inserted = star_folds(r);
    }
    return {inserted, star_folds(r)};
}

} // namespace

TEST_CASE("tetwild-surface-folds-refused", "[tetwild][folds]")
{
    Parameters params;
    params.init(Vector3d(-1, -1, -1), Vector3d(3, 3, 2));
    params.check_surface_folds = true;
    auto env = std::make_shared<Envelope>();
    TetWildMesh mesh(params, env, 0);
    build_block(mesh);
    REQUIRE(mesh.checks_tracked_surface());

    // The centre of the top: its star is six faces, the hexagon (0,0) (1,0) (2,1) (2,2) (1,2)
    // (0,1) around it.
    const size_t c = block_vid(1, 1, 1);
    Vector3d& p = mesh.m_vertex_attribute[c].m_posf;
    const Vector3d home = p;

    // Lifted and shifted inside the hexagon, it bends the surface and folds nothing.
    mesh.tracked_surface_before({c}, false);
    p = home + Vector3d(0.3, 0.2, 0.4);
    CHECK(mesh.tracked_surface_after({c}, false));

    // Past the hexagon's edges on the right, the two faces there turn over.
    p = home;
    mesh.tracked_surface_before({c}, false);
    p = home + Vector3d(1.5, 0, 0);
    CHECK_FALSE(mesh.tracked_surface_after({c}, false));
    CHECK(mesh.m_surface_fold_refusals == 1);

    // Already folded, a move that unfolds is fine; one that keeps the fold is too.
    mesh.tracked_surface_before({c}, false);
    p = home + Vector3d(1.4, 0, 0);
    CHECK(mesh.tracked_surface_after({c}, false));
    mesh.tracked_surface_before({c}, false);
    p = home;
    CHECK(mesh.tracked_surface_after({c}, false));

    // Away from the surface, nothing is checked.
    const size_t below = block_vid(1, 1, 0);
    mesh.tracked_surface_before({below}, false);
    mesh.m_vertex_attribute[below].m_posf += Vector3d(0, 0, 5);
    CHECK(mesh.tracked_surface_after({below}, false));
}

TEST_CASE("tetwild-surface-folds-none-made", "[tetwild][folds]" + release_only)
{
    // A whole parallel run with the check on: the tracked surface leaves the optimization with
    // no more folded faces than the insertion left it, though splits are not checked (a split
    // halves faces and moves nothing). Read off the surface history, at the insertion and at
    // the end.
    const auto [inserted, end] = folds_of_run("bunny.obj", true, 8);
    INFO("folds after the insertion " << inserted << ", at the end " << end);
    CHECK(inserted > 0);
    CHECK(end <= inserted);
}

TEST_CASE("tetwild-surface-folds-compare", "[.folds-compare]")
{
    // With the check off and on, on a few models (WMTK_FOLDS_MODELS for one).
    std::vector<std::string> models = {
        "sphere.obj",
        "bunny.obj",
        "crown.obj",
        "thingi_100036.stl",
        "37989_sf.obj",
        "Octocat.obj"};
    if (const char* m = std::getenv("WMTK_FOLDS_MODELS")) models = {m};
    for (const std::string& model : models) {
        if (!fs::exists(fs::path(WMTK_DATA_DIR) / "models" / model)) continue;
        const auto off = folds_of_run(model, false, 8);
        const auto on = folds_of_run(model, true, 8);
        fmt::print(
            "{}: check off {} -> {}, check on {} -> {}\n",
            model,
            off.first,
            off.second,
            on.first,
            on.second);
    }
}
