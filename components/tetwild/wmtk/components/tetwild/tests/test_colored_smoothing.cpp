#include <wmtk/components/tetwild/TetWildMesh.h>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <memory>
#include <random>
#include <vector>

using namespace wmtk;
using namespace wmtk::components::tetwild;

namespace {

// A jittered n x n x n grid, each cube cut into the six tets around its main diagonal. Every
// interior vertex has room to move, so a smoothing pass does real work on most of them; the
// boundary vertices are marked as lying on the bounding box, which keeps them fixed.
struct JitteredGrid
{
    std::vector<Vector3d> points;
    std::vector<std::array<size_t, 4>> tets;
    std::vector<std::vector<int>> on_bbox;
};

JitteredGrid make_jittered_grid(const int n, const unsigned seed)
{
    JitteredGrid g;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> jitter(-0.3, 0.3);
    const auto id = [n](int i, int j, int k) { return size_t((i * n + j) * n + k); };
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            for (int k = 0; k < n; ++k) {
                std::vector<int> faces;
                if (i == 0) faces.push_back(0);
                if (i == n - 1) faces.push_back(1);
                if (j == 0) faces.push_back(2);
                if (j == n - 1) faces.push_back(3);
                if (k == 0) faces.push_back(4);
                if (k == n - 1) faces.push_back(5);
                Vector3d p(i, j, k);
                if (faces.empty()) p += Vector3d(jitter(rng), jitter(rng), jitter(rng));
                g.points.push_back(p);
                g.on_bbox.push_back(faces);
            }
        }
    }
    static constexpr std::array<std::array<int, 4>, 6> kKuhn = {
        {{{0, 1, 3, 7}},
         {{0, 1, 5, 7}},
         {{0, 2, 3, 7}},
         {{0, 2, 6, 7}},
         {{0, 4, 5, 7}},
         {{0, 4, 6, 7}}}};
    for (int i = 0; i + 1 < n; ++i) {
        for (int j = 0; j + 1 < n; ++j) {
            for (int k = 0; k + 1 < n; ++k) {
                const std::array<size_t, 8> c = {
                    {id(i, j, k),
                     id(i + 1, j, k),
                     id(i, j + 1, k),
                     id(i + 1, j + 1, k),
                     id(i, j, k + 1),
                     id(i + 1, j, k + 1),
                     id(i, j + 1, k + 1),
                     id(i + 1, j + 1, k + 1)}};
                for (const auto& t : kKuhn) {
                    std::array<size_t, 4> tet = {{c[t[0]], c[t[1]], c[t[2]], c[t[3]]}};
                    const Vector3d& p0 = g.points[tet[0]];
                    const double vol = (g.points[tet[1]] - p0)
                                           .cross(g.points[tet[2]] - p0)
                                           .dot(g.points[tet[3]] - p0);
                    if (vol < 0) std::swap(tet[2], tet[3]); // wmtk orientation: positive volume
                    g.tets.push_back(tet);
                }
            }
        }
    }
    return g;
}

void load(TetWildMesh& mesh, const JitteredGrid& g)
{
    mesh.init(g.points.size(), g.tets);
    mesh.m_vertex_attribute.resize(g.points.size());
    mesh.m_face_attribute.resize(4 * g.tets.size());
    mesh.m_tet_attribute.resize(g.tets.size());
    for (size_t v = 0; v < g.points.size(); ++v) {
        auto& a = mesh.m_vertex_attribute[v];
        a.m_posf = g.points[v];
        a.set_pos_to_posf();
        a.m_is_rounded = true;
        a.on_bbox_faces = g.on_bbox[v];
    }
    for (size_t t = 0; t < g.tets.size(); ++t) {
        mesh.m_tet_attribute[t].m_quality = mesh.get_quality(mesh.tuple_from_tet(t));
    }
}

} // namespace

TEST_CASE("colored smoothing does not depend on the number of threads", "[tetwild][smoothing]")
{
    // The point of smoothing by color class: no two vertices of a class interact, so which thread
    // smooths which vertex cannot matter, and every thread count must give the same mesh, bit for
    // bit -- positions, exact positions and the cached cell qualities.
    const JitteredGrid grid = make_jittered_grid(16, 7);

    struct Run
    {
        Parameters params;
        std::unique_ptr<TetWildMesh> mesh;
    };
    std::vector<Run> runs;
    for (const int threads : {1, 3, 8, 16}) {
        runs.emplace_back();
        Run& r = runs.back();
        r.params.init(Vector3d(-1, -1, -1), Vector3d(16, 16, 16));
        r.mesh = std::make_unique<TetWildMesh>(r.params, nullptr, threads);
        load(*r.mesh, grid);
        REQUIRE(r.mesh->use_colored_smoothing());
        r.mesh->smooth_all_vertices(2);
    }

    const TetWildMesh& ref = *runs.front().mesh;
    size_t moved = 0;
    for (size_t v = 0; v < grid.points.size(); ++v) {
        if (ref.m_vertex_attribute[v].m_posf != grid.points[v]) ++moved;
    }
    CHECK(moved > grid.points.size() / 4); // the passes really did something

    for (size_t r = 1; r < runs.size(); ++r) {
        const TetWildMesh& other = *runs[r].mesh;
        size_t pos_diff = 0, exact_diff = 0, quality_diff = 0;
        for (size_t v = 0; v < grid.points.size(); ++v) {
            const auto& a = ref.m_vertex_attribute[v];
            const auto& b = other.m_vertex_attribute[v];
            if (a.m_posf != b.m_posf) ++pos_diff;
            if (!(a.pos() == b.pos()) || a.m_is_rounded != b.m_is_rounded) ++exact_diff;
        }
        for (size_t t = 0; t < grid.tets.size(); ++t) {
            if (ref.m_tet_attribute[t].m_quality != other.m_tet_attribute[t].m_quality) {
                ++quality_diff;
            }
        }
        CHECK(pos_diff == 0);
        CHECK(exact_diff == 0);
        CHECK(quality_diff == 0);
    }
}
