
#include "TopoOffsetTetMesh.h"

#include <wmtk/optimization/AMIPSEnergy.hpp>
#include <wmtk/optimization/solver.hpp>
#include <wmtk/utils/AMIPS.h>
#include <wmtk/utils/TetraQualityUtils.hpp>

namespace wmtk::components::topological_offset {

double TopoOffsetTetMesh::get_quality(const std::array<size_t, 4>& vids) const
{
    std::array<double, 12> T;
    for (int k = 0; k < 4; ++k) {
        for (int j = 0; j < 3; ++j) {
            T[k * 3 + j] = m_vertex_attribute[vids[k]].m_posf[j];
        }
    }
    return wmtk::AMIPS_energy(T);
}

double TopoOffsetTetMesh::get_quality(const Tuple& t) const
{
    return get_quality(oriented_tet_vids(t));
}

bool TopoOffsetTetMesh::smooth_before(const Tuple& t)
{
    return true;
}

bool TopoOffsetTetMesh::smooth_after(const Tuple& t)
{
    const size_t vid = t.vid(*this);

    const std::vector<Tuple> locs = get_one_ring_tets_for_vertex(t);
    assert(!locs.empty());

    double max_quality = 0.;
    for (const Tuple& tet : locs) {
        max_quality = std::max(max_quality, m_tet_attribute[tet.tid(*this)].m_quality);
    }

    // AMIPS wants each tet as 12 doubles with the moving vertex first.
    std::vector<std::array<double, 12>> assembles(locs.size());
    for (size_t i = 0; i < locs.size(); ++i) {
        std::array<size_t, 4> local_verts =
            wmtk::orient_preserve_tet_reorder(oriented_tet_vids(locs[i]), vid);
        for (int k = 0; k < 4; ++k) {
            for (int j = 0; j < 3; ++j) {
                assembles[i][k * 3 + j] = m_vertex_attribute[local_verts[k]].m_posf[j];
            }
        }
    }

    if (!m_smooth_solver) {
        m_smooth_solver = optimization::create_basic_solver();
    }

    optimization::AMIPSEnergy3D amips_energy(assembles);
    VectorXd x = m_vertex_attribute[vid].m_posf;
    try {
        m_smooth_solver->minimize(amips_energy, x);
    } catch (const std::exception&) {
        // polysolve reports a failed line search by throwing; the position it reached is
        // still the best it found, and the checks below decide whether to keep it.
    }
    m_vertex_attribute[vid].m_posf = x;

    // Inversion is caught by invariants(), called right after this by TetMesh::smooth_vertex.
    // Only the quality veto needs to be checked here.
    double max_after_quality = 0.;
    for (const Tuple& tet : locs) {
        const size_t tid = tet.tid(*this);
        const double q = get_quality(tet);
        m_tet_attribute[tid].m_quality = q;
        max_after_quality = std::max(max_after_quality, q);
    }

    return max_after_quality <= max_quality;
}

void TopoOffsetTetMesh::smooth_all_vertices(size_t n_iters)
{
    // skeleton: plain serial loop. SimWild's smooth_all_vertices uses ExecutePass for
    // multi-threading, which additionally requires a get_partition_id() hook -- left out here.
    for (size_t i = 0; i < n_iters; ++i) {
        for (const Tuple& v : get_vertices()) {
            smooth_vertex(v);
        }
    }
}

} // namespace wmtk::components::topological_offset
