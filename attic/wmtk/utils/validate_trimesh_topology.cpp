#include <wmtk/TriMesh.h>
#include <wmtk/utils/validate_trimesh_topology.h>
#include <algorithm>
#include <iterator>
namespace wmtk::utils {

bool is_valid_trimesh_topology(const TriMesh& m)
{
    const auto [a, b] = validate_trimesh_topology(m);
    return b.empty() && a.empty();
}
std::array<std::vector<size_t>, 2> validate_trimesh_topology(const TriMesh& m)
{
    const auto& tc = m.m_tri_connectivity;
    const auto& vc = m.m_vertex_connectivity;
    const size_t vsize = vc.size();
    const size_t tsize = tc.size();
    std::array<std::vector<size_t>, 2> R;
    auto& [vbad, tbad] = R;
    for (size_t vidx = 0; vidx < vsize; ++vidx) {
        const auto& va = vc[vidx];
        if (va.m_is_removed) {
            continue;
        }
        if (!std::is_sorted(va.m_conn_tris.begin(), va.m_conn_tris.end())) {
            // fall through this check so other errors can be reported
            spdlog::warn("Vertex {} connectivity wasn't sorted {}", vidx, va.m_conn_tris);
            vbad.emplace_back(vidx);
        }
        for (const size_t tidx : va.m_conn_tris) {
            if (tidx >= tsize) {
                spdlog::warn("Vertex {} has triangle {} > #tris ({})", vidx, tidx, vsize);
                vbad.emplace_back(vidx);
                continue;
            } else if (const auto& ta = tc[tidx]; ta.m_is_removed) {
                spdlog::warn("Vertex {} has triangle {} which was removed", vidx, tidx);

                vbad.emplace_back(vidx);
                continue;
            } else if (ta.find(vidx) == -1) {
                spdlog::warn("Vertex {} has triangle {} which did not record it", vidx, tidx);

                vbad.emplace_back(vidx);
            }
        }
    }
    for (size_t tidx = 0; tidx < tsize; ++tidx) {
        const auto& ta = tc[tidx];
        if (ta.m_is_removed) {
            continue;
        }
        for (const size_t vidx : ta.m_indices) {
            if (vidx >= vsize) {
                spdlog::warn("Triangle {} has vertex {} > #verts ({})", tidx, vidx, vsize);
                tbad.emplace_back(tidx);
                continue;
            } else if (const auto& va = vc[vidx]; va.m_is_removed) {
                spdlog::warn("Triangle {} has vertex {} which was removed", tidx, vidx);

                tbad.emplace_back(tidx);
                continue;
            } else if (!std::binary_search(va.m_conn_tris.begin(), va.m_conn_tris.end(), tidx)) {
                spdlog::warn("Triangle {} has vertex {} which did not record it", tidx, vidx);

                tbad.emplace_back(tidx);
            }
        }
    }
    return R;
}
} // namespace wmtk::utils

