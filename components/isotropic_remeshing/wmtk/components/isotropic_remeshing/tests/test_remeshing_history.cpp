#include <wmtk/components/isotropic_remeshing/IsotropicRemeshing.h>
#include <wmtk/history/Replay.hpp>
#include <wmtk/history/SurfaceHistory.hpp>
#include <wmtk/io/read_triangle_mesh.hpp>

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <map>

using namespace wmtk;
using namespace components::isotropic_remeshing;

namespace {

/// Remeshes `model` with `threads` threads while recording, saves the history and loads it back,
/// then replays it from the input and requires it to reproduce the output exactly: the same
/// vertex and face ids, the same corner order in every face, bit-identical positions.
void check_replay(
    const std::string& model,
    const int threads,
    const double len_factor,
    const int its)
{
    MatrixXd V;
    MatrixXi F;
    wmtk::io::read_triangle_mesh(std::string(WMTK_DATA_DIR) + "/models/" + model, V, F, 0);
    std::vector<Eigen::Vector3d> verts;
    std::vector<std::array<size_t, 3>> tris;
    VF_to_vectors(V, F, verts, tris);

    INFO(model << ", threads " << threads);
    IsotropicRemeshing m(verts, threads);
    m.create_mesh(verts.size(), tris, {}, true, 0, 0);
    const auto rec = m.start_history([&m](size_t v) { return m.vertex_attrs.at(v).pos; });
    REQUIRE(m.uniform_remeshing(len_factor * m.average_len_valen()[0], its));
    m.consolidate_mesh();
    m.stop_history();

    const auto path = (std::filesystem::temp_directory_path() /
                       ("wmtk_remeshing_history_" + std::to_string(threads) + ".bin"))
                          .string();
    rec->history().save(path);
    const history::SurfaceHistory h = history::SurfaceHistory::load(path);
    std::filesystem::remove(path);

    std::map<history::EditKind, size_t> kinds;
    for (const auto& e : h.edits) ++kinds[e.kind];
    CHECK(kinds[history::EditKind::split_edge] > 0);
    CHECK(kinds[history::EditKind::collapse_edge] > 0);
    CHECK(kinds[history::EditKind::swap_edge] > 0);
    CHECK(kinds[history::EditKind::move_vertices] > 0);
    CHECK(kinds[history::EditKind::consolidate] == size_t(its + 1));

    history::Replay r(h);
    r.apply_all(h);

    const auto out_v = m.get_vertices();
    const auto out_f = m.get_faces();
    REQUIRE(r.vertices().size() == out_v.size());
    REQUIRE(r.faces().size() == out_f.size());
    size_t bad_v = 0, bad_f = 0;
    for (const auto& t : out_v) {
        const size_t v = t.vid(m);
        if (r.vertices()[v] != m.vertex_attrs.at(v).pos) ++bad_v;
    }
    for (const auto& t : out_f) {
        const size_t f = t.fid(m);
        const auto a = m.oriented_tri_vids(f);
        const auto& b = r.faces()[f];
        if (a[0] != b[0] || a[1] != b[1] || a[2] != b[2]) ++bad_f;
    }
    CHECK(bad_v == 0);
    CHECK(bad_f == 0);
}

} // namespace

TEST_CASE("remeshing-history-replays-to-the-output", "[test_remeshing][history]")
{
    // In parallel the passes screen every operation with a dry run first and commit
    // concurrently; the replay only matches if no dry run recorded anything and the commit
    // order is a valid serial order. The target is below the input's average edge length, so
    // every pass has work: splits, collapses, swaps and smoothing all commit.
    for (const int threads : {0, 4}) check_replay("sphere.obj", threads, 0.6, 2);
}

TEST_CASE("remeshing-history-replays-to-the-output-bunny", "[test_remeshing][history][.]")
{
    // The same on a larger model and more threads; hidden, a few seconds.
    for (const int threads : {0, 8}) check_replay("bunny.obj", threads, 0.8, 3);
}
