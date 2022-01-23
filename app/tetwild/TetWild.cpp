
#include "TetWild.h"

#include <wmtk/utils/AMIPS.h>
#include <limits>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/io.hpp>

#include <igl/predicates/predicates.h>
#include <spdlog/fmt/ostr.h>

bool tetwild::TetWild::is_inverted(const Tuple& loc) const
{
    // Return a positive value if the point pd lies below the
    // plane passing through pa, pb, and pc; "below" is defined so
    // that pa, pb, and pc appear in counterclockwise order when
    // viewed from above the plane.

    auto vs = oriented_tet_vertices(loc);

    //
    if (m_vertex_attribute[vs[0].vid(*this)].m_is_rounded &&
        m_vertex_attribute[vs[1].vid(*this)].m_is_rounded &&
        m_vertex_attribute[vs[2].vid(*this)].m_is_rounded &&
        m_vertex_attribute[vs[3].vid(*this)].m_is_rounded) {
        igl::predicates::exactinit();
        auto res = igl::predicates::orient3d(
            m_vertex_attribute[vs[0].vid(*this)].m_posf,
            m_vertex_attribute[vs[1].vid(*this)].m_posf,
            m_vertex_attribute[vs[2].vid(*this)].m_posf,
            m_vertex_attribute[vs[3].vid(*this)].m_posf);
        int result;
        if (res == igl::predicates::Orientation::POSITIVE)
            result = 1;
        else if (res == igl::predicates::Orientation::NEGATIVE)
            result = -1;
        else
            result = 0;

        if (result < 0) //neg result == pos tet (tet origin from geogram delaunay)
            return false;
        return true;
    } else {
        Vector3 n = ((m_vertex_attribute[vs[1].vid(*this)].m_pos) -
                     m_vertex_attribute[vs[0].vid(*this)].m_pos)
                        .cross(
                            (m_vertex_attribute[vs[2].vid(*this)].m_pos) -
                            m_vertex_attribute[vs[0].vid(*this)].m_pos);
        Vector3 d = (m_vertex_attribute[vs[3].vid(*this)].m_pos) -
                    m_vertex_attribute[vs[0].vid(*this)].m_pos;
        auto res = n.dot(d);
        if (res > 0) // predicates returns pos value: non-inverted
            return false;
        else
            return true;
    }
}

bool tetwild::TetWild::round(const Tuple& v)
{
    size_t i = v.vid(*this);

    auto old_pos = m_vertex_attribute[i].m_pos;
    m_vertex_attribute[i].m_pos << m_vertex_attribute[i].m_posf[0], m_vertex_attribute[i].m_posf[1],
        m_vertex_attribute[i].m_posf[2];
    auto conn_tets = get_one_ring_tets_for_vertex(v);
    m_vertex_attribute[i].m_is_rounded = true;
    for (auto& tet : conn_tets) {
        if (is_inverted(tet)) {
            m_vertex_attribute[i].m_is_rounded = false;
            m_vertex_attribute[i].m_pos = old_pos;
            return false;
        }
    }

    return true;
}

double tetwild::TetWild::get_quality(const Tuple& loc) const
{
    std::array<Vector3d, 4> ps;
    auto its = oriented_tet_vertices(loc);
    for (int j = 0; j < 4; j++) {
        ps[j] = m_vertex_attribute[its[j].vid(*this)].m_posf;
    }

    std::array<double, 12> T;
    for (int j = 0; j < 3; j++) {
        T[0 * 3 + j] = ps[0][j];
        T[1 * 3 + j] = ps[1][j];
        T[2 * 3 + j] = ps[2][j];
        T[3 * 3 + j] = ps[3][j];
    }

    double energy = wmtk::AMIPS_energy(T);
    if (std::isinf(energy) || std::isnan(energy) || energy < 3 - 1e-3)
        return std::numeric_limits<double>::max();
    return energy;
}


bool tetwild::TetWild::vertex_invariant(const Tuple& t)
{
    int v_id = t.vid(*this);

    // check rounded

    // check surface
    if (m_vertex_attribute[v_id].m_is_on_surface) {
    }

    return true;
}

bool tetwild::TetWild::tetrahedron_invariant(const Tuple& t)
{
    // check inversion

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
    for (auto& loc : locs) {
        if (is_inverted(loc)) {
            m_vertex_attribute[vid].m_posf = old_pos;
            return false;
        }
    }

    for (auto& loc : locs) {
        auto t_id = loc.tid(*this);
        m_tet_attribute[t_id].m_qualities = get_quality(loc);
    }
    return true;
}


void tetwild::TetWild::output_mesh(std::string file)
{
    consolidate_mesh();

    wmtk::MshData msh;

    const auto& vtx = get_vertices();
    msh.add_tet_vertices(vtx.size(), [&](size_t k) {
        auto i = vtx[k].vid(*this);
        return m_vertex_attribute[i].m_posf;
    });

    const auto& tets = get_tets();
    msh.add_tets(tets.size(), [&](size_t k) {
        auto i = tets[k].tid(*this);
        auto vs = oriented_tet_vertices(tets[k]);
        std::array<size_t, 4> data;
        for (int j = 0; j < 4; j++) {
            data[j] = vs[j].vid(*this);
            assert(data[j] < vtx.size());
        }
        return data;
    });

    msh.add_tet_vertex_attribute<1>("tv index", [&](size_t i) { return i; });
    msh.add_tet_attribute<1>("t index", [&](size_t i) { return i; });

    msh.save(file, true);
}

std::vector<std::array<size_t, 3>> tetwild::TetWild::get_faces_by_condition(
    std::function<bool(const FaceAttributes&)> cond)
{
    auto res = std::vector<std::array<size_t, 3>>();
    for (auto f : get_faces()) {
        auto fid = f.fid(*this);
        if (cond(m_face_attribute[fid])) {
            auto tid = fid / 4, lid = fid % 4;
            auto verts = get_face_vertices(f);
            res.emplace_back(std::array<size_t, 3>{{verts[0].vid(*this), verts[1].vid(*this), verts[2].vid(*this)}});
        }
    }
    return res;
}

bool tetwild::TetWild::is_edge_on_surface(const Tuple& loc){
    //todo

    return false;
}