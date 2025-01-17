#include "edges_to_tuples.hpp"
#include <wmtk/TriMesh.hpp>
using namespace wmtk;
std::vector<Tuple>
boundary_edges_to_tuples(const EigenMeshes& em, const std::vector<int64_t>& indices, bool offset)
{
    std::vector<Tuple> tups;
    tups.reserve(indices.size());

    for (size_t j = 0; j < indices.size() - 1; ++j) {
        int64_t a = indices[j];
        int64_t b = indices[j + 1];
        if (a == b) {
            continue;
        }

        const auto& fa = em.VF.at(a);
        const auto& fb = em.VF.at(b);
        // spdlog::info(
        //     "pair {} {} got Faces {}: {}, {}: {}",
        //     ind_a,
        //     ind_b,
        //     a,
        //     fmt::join(fa, ","),
        //     b,
        //     fmt::join(fb, ","));

        std::vector<int64_t> fs;
        std::set_intersection(fa.begin(), fa.end(), fb.begin(), fb.end(), std::back_inserter(fs));

        assert(fs.size() == 1); // must be true for a boundary edge

        int64_t fid = fs[0];
        auto f = em.F.M.row(fid);
        // spdlog::info("{} | {} => {} {}", fid, fmt::join(f, ","), a, b);
        int8_t lvid = -1;
        int8_t leid = -1;
        for (int k = 0; k < 3; ++k) {
            if (a == f(k)) {
                lvid = k;
            } else if (b != f(k)) {
                leid = k;
            }
        }
        if (offset) {
            Tuple& t = tups.emplace_back(lvid, leid, -1, fid + em.F.start());
        } else {
            Tuple& t = tups.emplace_back(lvid, leid, -1, fid);
        }
    }
    return tups;
}
