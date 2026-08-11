#include <wmtk/TriOptimizerMesh.h>

#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/SizingField.hpp>
#include <wmtk/utils/partition_utils.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <igl/predicates/predicates.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <queue>

namespace wmtk {

void TriOptimizerMesh::gradation_smooth_sizing(double grade, const std::vector<size_t>& seeds)
{
    utils::gradation_smooth_sizing(
        grade,
        seeds,
        [this](size_t v) -> double& { return m_vertex_attribute[v].m_sizing_scalar; },
        [this](size_t v) { return get_one_ring_vids_for_vertex_duplicate(v); });
}

void TriOptimizerMesh::partition_mesh()
{
    auto m_vertex_partition_id = partition_TriMesh(*this, NUM_THREADS);
    for (size_t i = 0; i < m_vertex_partition_id.size(); i++) {
        m_vertex_attribute[i].partition_id = m_vertex_partition_id[i];
    }
}

void TriOptimizerMesh::partition_mesh_morton()
{
    if (NUM_THREADS == 0) {
        return;
    }
    logger().info("Number of parts: {} by morton", NUM_THREADS);

    // The shared partitioner is 3D; a zero z leaves the bounding box, the scale and the
    // Morton code exactly where a 2D-specific version would put them.
    std::vector<size_t> partition_id;
    wmtk::partition_vertex_morton(
        vert_capacity(),
        [this](size_t i) {
            const Vector2d& p = m_vertex_attribute[i].m_posf;
            return Eigen::Vector3d(p[0], p[1], 0);
        },
        NUM_THREADS,
        partition_id);

    for (size_t i = 0; i < partition_id.size(); i++) {
        m_vertex_attribute[i].partition_id = partition_id[i];
    }
}

double TriOptimizerMesh::get_length2(const Tuple& l) const
{
    auto& m = *this;
    auto& v1 = l;
    auto v2 = l.switch_vertex(m);
    double length =
        (m.m_vertex_attribute[v1.vid(m)].m_posf - m.m_vertex_attribute[v2.vid(m)].m_posf)
            .squaredNorm();
    return length;
}

std::tuple<double, double> TriOptimizerMesh::get_max_avg_energy()
{
    double max_energy = -1.;
    double avg_energy = 0.;
    auto cnt = 0;

    for (int i = 0; i < tri_capacity(); i++) {
        const Tuple tup = tuple_from_tri(i);
        if (!tup.is_valid(*this)) {
            continue;
        }
        const double q = m_face_attribute[tup.fid(*this)].m_quality;
        max_energy = std::max(max_energy, q);
        avg_energy += q;
        cnt++;
    }

    avg_energy /= cnt;

    return std::make_tuple(max_energy, avg_energy);
}

bool TriOptimizerMesh::is_inverted_f(const Tuple& loc) const
{
    return is_inverted_f(loc.fid(*this));
}

bool TriOptimizerMesh::is_inverted_f(const size_t fid) const
{
    auto vs = oriented_tri_vids(fid);

    igl::predicates::exactinit();
    auto res = igl::predicates::orient2d(
        m_vertex_attribute[vs[0]].m_posf,
        m_vertex_attribute[vs[1]].m_posf,
        m_vertex_attribute[vs[2]].m_posf);
    if (res == igl::predicates::Orientation::POSITIVE) {
        return false;
    }
    return true;
}

bool TriOptimizerMesh::is_inverted(const std::array<size_t, 3>& vs) const
{
    if (m_vertex_attribute[vs[0]].m_is_rounded && m_vertex_attribute[vs[1]].m_is_rounded &&
        m_vertex_attribute[vs[2]].m_is_rounded) {
        igl::predicates::exactinit();
        auto res = igl::predicates::orient2d(
            m_vertex_attribute[vs[0]].m_posf,
            m_vertex_attribute[vs[1]].m_posf,
            m_vertex_attribute[vs[2]].m_posf);
        if (res == igl::predicates::Orientation::POSITIVE) {
            return false;
        }
        return true;
    } else {
        const Vector2r& v0 = m_vertex_attribute[vs[0]].m_pos;
        const Vector2r& v1 = m_vertex_attribute[vs[1]].m_pos;
        const Vector2r& v2 = m_vertex_attribute[vs[2]].m_pos;
        const Vector2r a = v1 - v0;
        const Vector2r b = v2 - v0;
        Rational res = a.x() * b.y() - a.y() * b.x();
        if (res > 0) {
            return false;
        } else {
            return true;
        }
    }
}

bool TriOptimizerMesh::is_inverted(const Tuple& loc) const
{
    auto vs = oriented_tri_vids(loc);
    return is_inverted(vs);
}

bool TriOptimizerMesh::is_inverted(const size_t fid) const
{
    auto vs = oriented_tri_vids(fid);
    return is_inverted(vs);
}

size_t TriOptimizerMesh::round_all_vertices()
{
    if (m_all_rounded.load(std::memory_order_relaxed)) {
        return 0;
    }

    size_t reclaimed = 0, still_unrounded = 0;
    for (const Tuple& v : get_vertices()) {
        if (m_vertex_attribute[v.vid(*this)].m_is_rounded) {
            continue;
        }
        if (round(v)) {
            ++reclaimed;
        } else {
            ++still_unrounded;
        }
    }

    if (still_unrounded == 0) {
        m_all_rounded.store(true, std::memory_order_relaxed);
    }
    if (reclaimed > 0 || still_unrounded > 0) {
        logger().info(
            "rounding sweep: reclaimed {}, still unrounded {}",
            reclaimed,
            still_unrounded);
    }
    return reclaimed;
}

bool TriOptimizerMesh::round(const Tuple& v)
{
    size_t i = v.vid(*this);
    if (m_vertex_attribute[i].m_is_rounded) {
        return true;
    }

    auto old_pos = m_vertex_attribute[i].m_pos;
    m_vertex_attribute[i].m_pos << m_vertex_attribute[i].m_posf[0], m_vertex_attribute[i].m_posf[1];
    auto conn_tets = get_one_ring_tris_for_vertex(v);
    // Set before the loop so is_inverted takes the float path: the question being asked is
    // exactly whether the ROUNDED position keeps every incident face valid.
    m_vertex_attribute[i].m_is_rounded = true;
    for (const Tuple& tet : conn_tets) {
        if (is_inverted(tet)) {
            m_vertex_attribute[i].m_is_rounded = false;
            m_vertex_attribute[i].m_pos = old_pos;
            return false;
        }
    }

    return true;
}

double TriOptimizerMesh::get_quality(const std::array<size_t, 3>& vs) const
{
    std::array<Vector2d, 3> ps;
    for (size_t k = 0; k < 3; k++) {
        ps[k] = m_vertex_attribute[vs[k]].m_posf;
    }
    double energy = -1.;
    {
        std::array<double, 6> T;
        for (size_t k = 0; k < 3; k++)
            for (size_t j = 0; j < 2; j++) {
                T[k * 2 + j] = ps[k][j];
            }
        energy = AMIPS2D_energy(T);
    }
    if (std::isinf(energy) || std::isnan(energy) || energy < 2 - 1e-3) {
        return MAX_ENERGY;
    }
    return energy;
}

double TriOptimizerMesh::get_quality(const Tuple& loc) const
{
    auto its = oriented_tri_vids(loc);
    return get_quality(its);
}

double TriOptimizerMesh::get_quality(const size_t fid) const
{
    auto its = oriented_tri_vids(fid);
    return get_quality(its);
}

std::vector<std::array<size_t, 2>> TriOptimizerMesh::get_edges_by_condition(
    std::function<bool(const EdgeAttributes&)> cond) const
{
    std::vector<std::array<size_t, 2>> res;
    for (const Tuple& e : get_edges()) {
        size_t eid = e.eid(*this);
        if (cond(m_edge_attribute[eid])) {
            res.push_back({{e.vid(*this), e.switch_vertex(*this).vid(*this)}});
        }
    }
    return res;
}

bool TriOptimizerMesh::is_edge_on_surface(const Tuple& loc) const
{
    const auto vs = get_edge_vids(loc);
    if (!m_vertex_attribute.at(vs[0]).m_is_on_surface ||
        !m_vertex_attribute.at(vs[1]).m_is_on_surface) {
        return false;
    }

    const size_t eid = loc.eid(*this);
    return m_edge_attribute[eid].m_is_surface_fs;
}
bool TriOptimizerMesh::is_edge_on_surface(const std::array<size_t, 2>& vids) const
{
    if (!m_vertex_attribute.at(vids[0]).m_is_on_surface ||
        !m_vertex_attribute.at(vids[1]).m_is_on_surface) {
        return false;
    }

    const auto [_, eid] = tuple_from_edge(vids);
    return m_edge_attribute[eid].m_is_surface_fs;
}
bool TriOptimizerMesh::is_edge_on_bbox(const Tuple& loc) const
{
    const auto vs = get_edge_vids(loc);
    if (m_vertex_attribute.at(vs[0]).on_bbox_faces.empty() ||
        m_vertex_attribute.at(vs[1]).on_bbox_faces.empty()) {
        return false;
    }

    const size_t eid = loc.eid(*this);
    return m_edge_attribute[eid].m_is_bbox_fs >= 0;
}

bool TriOptimizerMesh::is_edge_on_bbox(const std::array<size_t, 2>& vids) const
{
    if (m_vertex_attribute.at(vids[0]).on_bbox_faces.empty() ||
        m_vertex_attribute.at(vids[1]).on_bbox_faces.empty()) {
        return false;
    }
    const auto [_, eid] = tuple_from_edge(vids);
    return m_edge_attribute[eid].m_is_bbox_fs >= 0;
}

} // namespace wmtk
