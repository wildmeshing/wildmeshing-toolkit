
#include "TetWild.h"

#include <wmtk/utils/AMIPS.h>
#include <limits>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>

bool tetwild::TetWild::smooth_before(const Tuple& t)
{
    if (m_vertex_attribute[t.vid(*this)].m_is_rounded) return true;
    // try to round.
    return round(t); // Note: no need to roll back.
}

bool tetwild::TetWild::smooth_after(const Tuple& t)
{
    // Newton iterations are encapsulated here.
    // TODO: bbox/surface tags.
    // TODO: envelope check.
    wmtk::logger().trace("Newton iteration for vertex smoothing.");
    auto vid = t.vid(*this);

    auto locs = get_one_ring_tets_for_vertex(t);
    assert(locs.size() > 0);
    std::vector<std::array<double, 12>> assembles(locs.size());
    auto loc_id = 0;

    for (auto& loc : locs) {
        auto& T = assembles[loc_id];
        auto t_id = loc.tid(*this);

        assert(!is_inverted(loc));
        auto local_tuples = oriented_tet_vertices(loc);
        std::array<size_t, 4> local_verts;
        for (auto i = 0; i < 4; i++) {
            local_verts[i] = local_tuples[i].vid(*this);
        }

        local_verts = wmtk::orient_preserve_tet_reorder(local_verts, vid);

        for (auto i = 0; i < 4; i++) {
            for (auto j = 0; j < 3; j++) {
                T[i * 3 + j] = m_vertex_attribute[local_verts[i]].m_posf[j];
            }
        }
        loc_id++;
    }


    auto old_pos = m_vertex_attribute[vid].m_posf;
    m_vertex_attribute[vid].m_posf = wmtk::newton_method_from_stack(
        assembles,
        wmtk::AMIPS_energy,
        wmtk::AMIPS_jacobian,
        wmtk::AMIPS_hessian);
    wmtk::logger().trace(
        "old pos {} -> new pos {}",
        old_pos.transpose(),
        m_vertex_attribute[vid].m_posf.transpose());
    // note: duplicate code snippets.

    if (!tetrahedron_invariant(locs)) return false;

    for (auto& loc : locs) {
        auto t_id = loc.tid(*this);
        m_tet_attribute[t_id].m_qualities = get_quality(loc);
    }
    return true;
}

void tetwild::TetWild::smooth_all_vertices()
{
    auto tuples = get_vertices();
    wmtk::logger().debug("tuples");
    auto cnt_suc = 0;
    for (auto& t : tuples) { // TODO: threads
        if (smooth_vertex(t)) cnt_suc++;
    }
    wmtk::logger().debug("Smoothing Success Count {}", cnt_suc);
}
