#include <wmtk/TetOptimizerMesh.h>

#include <wmtk/utils/AMIPS.h>
#include <wmtk/utils/GeoUtils.h>
#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/envelope/KNN.hpp>
#include <wmtk/optimization/SmoothVertex.hpp>
#include <wmtk/threading/parallel_for.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/SizingField.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/io.hpp>
#include <wmtk/utils/partition_utils.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <igl/predicates/predicates.h>
#include <igl/write_triangle_mesh.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <queue>

namespace wmtk {

TetOptimizerMesh::VertexAttributes::VertexAttributes(const Vector3r& p)
{
    m_pos = p;
    m_posf = to_double(m_pos);
}

void TetOptimizerMesh::compute_vertex_partition()
{
    auto partition_id = partition_TetMesh(*this, NUM_THREADS);
    for (auto i = 0; i < vert_capacity(); i++) m_vertex_attribute[i].partition_id = partition_id[i];
}

void TetOptimizerMesh::compute_vertex_partition_morton()
{
    if (NUM_THREADS == 0) {
        return;
    }

    logger().info("Number of parts: {} by morton", NUM_THREADS);

    std::vector<size_t> partition_id;
    wmtk::partition_vertex_morton(
        vert_capacity(),
        [this](size_t i) { return m_vertex_attribute[i].m_posf; },
        NUM_THREADS,
        partition_id);

    for (size_t i = 0; i < partition_id.size(); i++) {
        m_vertex_attribute[i].partition_id = partition_id[i];
    }
}

double TetOptimizerMesh::swap_edge_44_energy(
    const std::vector<std::array<size_t, 4>>& tets,
    const int op_case)
{
    double max_energy = -1;
    for (const auto& vids : tets) {
        if (is_inverted(vids)) {
            return std::numeric_limits<double>::max();
        }
        const double e = get_quality(vids);
        max_energy = std::max(max_energy, e);
    }
    return max_energy;
}

double TetOptimizerMesh::swap_edge_56_energy(
    const std::vector<std::array<size_t, 4>>& tets,
    const int op_case)
{
    double max_energy = -1;
    for (const auto& vids : tets) {
        if (is_inverted(vids)) {
            return std::numeric_limits<double>::max();
        }
        const double e = get_quality(vids);
        max_energy = std::max(max_energy, e);
    }
    return max_energy;
}

bool TetOptimizerMesh::smooth_before(const Tuple& t)
{
    const bool r = round(t);

    const size_t vid = t.vid(*this);

    if (!m_vertex_attribute[vid].on_bbox_faces.empty()) return false;

    if (m_vertex_attribute[vid].m_is_rounded) return true;
    // try to round.
    // Note: no need to roll back.
    return r;
}


std::shared_ptr<SampleEnvelope> TetOptimizerMesh::smoothing_containment_envelope(const size_t) const
{
    // tetwild keeps one surface envelope, so pull target and containment test coincide --
    // unlike simwild, which has a separate working envelope.
    return m_envelope;
}

size_t TetOptimizerMesh::round_all_vertices()
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

void TetOptimizerMesh::gradation_smooth_sizing(double grade, const std::vector<size_t>& seeds)
{
    utils::gradation_smooth_sizing(
        grade,
        seeds,
        [this](size_t v) -> double& { return m_vertex_attribute[v].m_sizing_scalar; },
        [this](size_t v) { return get_one_ring_vids_for_vertex_adj(v); });
}

/////////////////////////////////////////////////////////////////////
void TetOptimizerMesh::output_faces(
    std::string file,
    std::function<bool(const FaceAttributes&)> cond)
{
    auto outface = get_faces_by_condition(cond);
    Eigen::MatrixXd matV = Eigen::MatrixXd::Zero(vert_capacity(), 3);
    for (const auto& v : get_vertices()) {
        auto vid = v.vid(*this);
        matV.row(vid) = m_vertex_attribute[vid].m_posf;
    }
    Eigen::MatrixXi matF(outface.size(), 3);
    for (auto i = 0; i < outface.size(); i++) {
        matF.row(i) << (int)outface[i][0], (int)outface[i][1], (int)outface[i][2];
    }
    logger().info("Output face size {}", outface.size());
    igl::write_triangle_mesh(file, matV, matF);
}

std::tuple<double, double> TetOptimizerMesh::get_max_avg_energy()
{
    double max_energy = -1.;
    double avg_energy = 0.;
    auto cnt = 0;
    // TetMesh::for_each_tetra([&](auto& t) {
    //     auto q = m_tet_attribute[t.tid(*this)].m_quality;
    //     max_energy = std::max(max_energy, q);
    //     avg_energy += std::cbrt(q);
    //     cnt++;
    // });
    // std::ofstream large_tet("large_energy_tet.obj");

    for (int i = 0; i < tet_capacity(); i++) {
        auto tup = tuple_from_tet(i);
        if (!tup.is_valid(*this)) continue;
        // auto vs = oriented_tet_vertices(tup);

        auto q = cell_quality(tup.tid(*this));
        max_energy = std::max(max_energy, q);
        // if (q > 1e6) {
        //     for (auto v : vs) {
        //         large_tet << "v " << m_vertex_attribute[v.vid(*this)].m_posf[0] << " "
        //                   << m_vertex_attribute[v.vid(*this)].m_posf[1] << " "
        //                   << m_vertex_attribute[v.vid(*this)].m_posf[2] << std::endl;
        //     }
        // }
        avg_energy += std::cbrt(q);
        cnt++;
    }

    avg_energy /= cnt;

    return std::make_tuple(std::cbrt(max_energy), avg_energy);
}

std::vector<size_t> TetOptimizerMesh::active_vertices() const
{
    return utils::active_vertices(
        vert_capacity(),
        tet_capacity(),
        [this](size_t tid) { return tuple_from_tet(tid).is_valid(*this); },
        [this](size_t tid) { return cell_quality(tid); },
        [this](size_t tid) { return oriented_tet_vids(tid); },
        active_quality_threshold(),
        [this](size_t vid) { return m_vertex_attribute[vid].m_is_on_surface; });
}

bool TetOptimizerMesh::is_inverted_f(const Tuple& loc) const
{
    auto vs = oriented_tet_vertices(loc);

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

    if (result < 0) // neg result == pos tet (tet origin from geogram delaunay)
        return false;
    return true;
}

bool TetOptimizerMesh::is_inverted(const std::array<size_t, 4>& vs) const
{
    // Return a positive value if the point pd lies below the
    // plane passing through pa, pb, and pc; "below" is defined so
    // that pa, pb, and pc appear in counterclockwise order when
    // viewed from above the plane.

    if (m_vertex_attribute[vs[0]].m_is_rounded && m_vertex_attribute[vs[1]].m_is_rounded &&
        m_vertex_attribute[vs[2]].m_is_rounded && m_vertex_attribute[vs[3]].m_is_rounded) {
        igl::predicates::exactinit();
        auto res = igl::predicates::orient3d(
            m_vertex_attribute[vs[0]].m_posf,
            m_vertex_attribute[vs[1]].m_posf,
            m_vertex_attribute[vs[2]].m_posf,
            m_vertex_attribute[vs[3]].m_posf);
        int result;
        if (res == igl::predicates::Orientation::POSITIVE)
            result = 1;
        else if (res == igl::predicates::Orientation::NEGATIVE)
            result = -1;
        else
            result = 0;

        if (result < 0) // neg result == pos tet (tet origin from geogram delaunay)
            return false;
        return true;
    } else {
        Vector3r n =
            ((m_vertex_attribute[vs[1]].m_pos) - m_vertex_attribute[vs[0]].m_pos)
                .cross((m_vertex_attribute[vs[2]].m_pos) - m_vertex_attribute[vs[0]].m_pos);
        Vector3r d = (m_vertex_attribute[vs[3]].m_pos) - m_vertex_attribute[vs[0]].m_pos;
        auto res = n.dot(d);
        if (res > 0) // predicates returns pos value: non-inverted
            return false;
        else
            return true;
    }
}

bool TetOptimizerMesh::is_inverted(const Tuple& loc) const
{
    auto vs = oriented_tet_vids(loc);
    return is_inverted(vs);
}

bool TetOptimizerMesh::round(const Tuple& v)
{
    size_t i = v.vid(*this);
    if (m_vertex_attribute[i].m_is_rounded) return true;

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

double TetOptimizerMesh::get_quality(const std::array<size_t, 4>& its) const
{
    std::array<Vector3d, 4> ps;
    auto use_rational = false;
    for (auto k = 0; k < 4; k++) {
        ps[k] = m_vertex_attribute[its[k]].m_posf;
        if (!m_vertex_attribute[its[k]].m_is_rounded) {
            use_rational = true;
            break;
        }
    }
    auto energy = -1.;
    if (!use_rational) {
        std::array<double, 12> T;
        for (auto k = 0; k < 4; k++)
            for (auto j = 0; j < 3; j++) T[k * 3 + j] = ps[k][j];

        energy = wmtk::AMIPS_energy_stable_p3<wmtk::Rational>(T);
    } else {
        std::array<wmtk::Rational, 12> T;
        for (auto k = 0; k < 4; k++)
            for (auto j = 0; j < 3; j++) T[k * 3 + j] = m_vertex_attribute[its[k]].m_pos[j];
        energy = wmtk::AMIPS_energy_rational_p3<wmtk::Rational>(T);
    }
    if (std::isinf(energy) || std::isnan(energy) || energy < 27 - 1e-3) return MAX_ENERGY;
    return energy;
}

double TetOptimizerMesh::get_quality(const Tuple& loc) const
{
    auto its = oriented_tet_vids(loc);
    return get_quality(its);
}

bool TetOptimizerMesh::invariants(const std::vector<Tuple>& tets)
{
    return true;
}

std::vector<std::array<size_t, 3>> TetOptimizerMesh::get_faces_by_condition(
    std::function<bool(const FaceAttributes&)> cond) const
{
    auto res = std::vector<std::array<size_t, 3>>();
    for (auto f : get_faces()) {
        auto fid = f.fid(*this);
        if (cond(m_face_attribute[fid])) {
            auto tid = fid / 4, lid = fid % 4;
            auto verts = get_face_vertices(f);
            res.emplace_back( //
                std::array<size_t, 3>{
                    {verts[0].vid(*this), verts[1].vid(*this), verts[2].vid(*this)}});
        }
    }

    return res;
}

bool TetOptimizerMesh::is_edge_on_surface(const Tuple& loc)
{
    size_t v1_id = loc.vid(*this);
    auto loc1 = loc.switch_vertex(*this);
    size_t v2_id = loc1.vid(*this);
    if (!m_vertex_attribute[v1_id].m_is_on_surface || !m_vertex_attribute[v2_id].m_is_on_surface)
        return false;

    auto tets = get_incident_tets_for_edge(loc);
    std::vector<size_t> n_vids;
    for (auto& t : tets) {
        auto vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            if (vs[j].vid(*this) != v1_id && vs[j].vid(*this) != v2_id)
                n_vids.push_back(vs[j].vid(*this));
        }
    }
    wmtk::vector_unique(n_vids);

    for (size_t vid : n_vids) {
        auto [_, fid] = tuple_from_face({{v1_id, v2_id, vid}});
        if (m_face_attribute[fid].m_is_surface_fs) return true;
    }

    return false;
}

int TetOptimizerMesh::edge_incident_surface_face_count(const Tuple& e)
{
    const size_t v1_id = e.vid(*this);
    const size_t v2_id = e.switch_vertex(*this).vid(*this);

    const auto tets = get_incident_tets_for_edge(e);
    std::vector<size_t> n_vids;
    for (const auto& t : tets) {
        const auto vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; ++j) {
            const size_t v = vs[j].vid(*this);
            if (v != v1_id && v != v2_id) n_vids.push_back(v);
        }
    }
    wmtk::vector_unique(n_vids);

    int count = 0;
    for (const size_t vid : n_vids) {
        auto [ftup, fid] = tuple_from_face({{v1_id, v2_id, vid}});
        (void)ftup;
        if (fid != static_cast<size_t>(-1) && m_face_attribute[fid].m_is_surface_fs) ++count;
    }
    return count;
}

bool TetOptimizerMesh::is_edge_on_bbox(const Tuple& loc)
{
    size_t v1_id = loc.vid(*this);
    auto loc1 = loc.switch_vertex(*this);
    size_t v2_id = loc1.vid(*this);
    if (m_vertex_attribute[v1_id].on_bbox_faces.empty() ||
        m_vertex_attribute[v2_id].on_bbox_faces.empty())
        return false;

    auto tets = get_incident_tets_for_edge(loc);
    std::vector<size_t> n_vids;
    for (auto& t : tets) {
        auto vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            if (vs[j].vid(*this) != v1_id && vs[j].vid(*this) != v2_id)
                n_vids.push_back(vs[j].vid(*this));
        }
    }
    wmtk::vector_unique(n_vids);

    for (size_t vid : n_vids) {
        auto [_, fid] = tuple_from_face({{v1_id, v2_id, vid}});
        if (m_face_attribute[fid].m_is_bbox_fs >= 0) return true;
    }

    return false;
}

bool TetOptimizerMesh::vertex_is_on_surface(const size_t vid) const
{
    return m_vertex_attribute.at(vid).m_is_on_surface;
}

bool TetOptimizerMesh::face_is_on_surface(const size_t fid) const
{
    return m_face_attribute.at(fid).m_is_surface_fs;
}

size_t TetOptimizerMesh::get_order_of_vertex(const size_t vid) const
{
    return m_vertex_attribute.at(vid).m_order;
}

} // namespace wmtk
