#include <wmtk/components/triwild/TriWildMesh.h>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <memory>
#include <random>
#include <vector>

using namespace wmtk;
using namespace wmtk::components::triwild;

namespace {

// A jittered n x n grid of squares, each cut into two triangles. Every interior vertex has room
// to move, so a smoothing pass does real work on most of them; the boundary vertices are marked
// as lying on the bounding box, which keeps them fixed.
void load_jittered_grid(TriWildMesh& mesh, const size_t n, const unsigned seed)
{
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> jitter(-0.3, 0.3);
    const auto vid = [n](size_t i, size_t j) { return j * n + i; };
    std::vector<std::array<size_t, 3>> tris;
    for (size_t j = 0; j + 1 < n; ++j) {
        for (size_t i = 0; i + 1 < n; ++i) {
            tris.push_back({{vid(i, j), vid(i + 1, j), vid(i, j + 1)}});
            tris.push_back({{vid(i + 1, j), vid(i + 1, j + 1), vid(i, j + 1)}});
        }
    }
    mesh.init(n * n, tris);
    mesh.m_vertex_attribute.resize(n * n);
    mesh.m_edge_attribute.resize(3 * tris.size());
    mesh.m_face_attribute.resize(tris.size());
    for (size_t j = 0; j < n; ++j) {
        for (size_t i = 0; i < n; ++i) {
            auto& va = mesh.m_vertex_attribute[vid(i, j)];
            if (i == 0) va.on_bbox_faces.push_back(0);
            if (i == n - 1) va.on_bbox_faces.push_back(1);
            if (j == 0) va.on_bbox_faces.push_back(2);
            if (j == n - 1) va.on_bbox_faces.push_back(3);
            va.m_posf = Vector2d(double(i), double(j));
            if (va.on_bbox_faces.empty()) va.m_posf += Vector2d(jitter(rng), jitter(rng));
            va.set_pos_to_posf();
            va.m_is_rounded = true;
            va.m_is_on_surface = false;
            va.m_sizing_scalar = 1.0;
        }
    }
    for (size_t f = 0; f < tris.size(); ++f) {
        mesh.m_face_attribute[f].m_quality = mesh.get_quality(mesh.oriented_tri_vids(f));
    }
}

} // namespace

TEST_CASE("colored smoothing does not depend on the number of threads (2D)", "[triwild][smoothing]")
{
    // The 2D twin of the tetwild test: no two vertices of a color class interact, so every
    // thread count must give the same mesh, bit for bit.
    constexpr size_t n = 48;
    std::vector<Vector2d> initial;

    struct Run
    {
        Parameters params;
        std::unique_ptr<TriWildMesh> mesh;
    };
    std::vector<Run> runs;
    for (const int threads : {1, 3, 8, 16}) {
        runs.emplace_back();
        Run& r = runs.back();
        r.params.init(Vector2d(-1, -1), Vector2d(double(n), double(n)));
        r.mesh = std::make_unique<TriWildMesh>(r.params, r.params.eps, threads);
        load_jittered_grid(*r.mesh, n, 11);
        if (initial.empty()) {
            for (size_t v = 0; v < n * n; ++v) {
                initial.push_back(r.mesh->m_vertex_attribute[v].m_posf);
            }
        }
        REQUIRE(r.mesh->use_colored_smoothing());
        r.mesh->smooth_all_vertices(2);
    }

    const TriWildMesh& ref = *runs.front().mesh;
    size_t moved = 0;
    for (size_t v = 0; v < n * n; ++v) {
        if (ref.m_vertex_attribute[v].m_posf != initial[v]) ++moved;
    }
    CHECK(moved > n * n / 4); // the passes really did something

    for (size_t r = 1; r < runs.size(); ++r) {
        const TriWildMesh& other = *runs[r].mesh;
        size_t pos_diff = 0, exact_diff = 0, quality_diff = 0;
        for (size_t v = 0; v < n * n; ++v) {
            const auto& a = ref.m_vertex_attribute[v];
            const auto& b = other.m_vertex_attribute[v];
            if (a.m_posf != b.m_posf) ++pos_diff;
            if (!(a.pos() == b.pos()) || a.m_is_rounded != b.m_is_rounded) ++exact_diff;
        }
        for (size_t f = 0; f < 2 * (n - 1) * (n - 1); ++f) {
            if (ref.m_face_attribute[f].m_quality != other.m_face_attribute[f].m_quality) {
                ++quality_diff;
            }
        }
        CHECK(pos_diff == 0);
        CHECK(exact_diff == 0);
        CHECK(quality_diff == 0);
    }
}
