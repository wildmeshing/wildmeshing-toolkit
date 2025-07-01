#include "Mesh.hpp"
#include <Eigen/src/Core/functors/UnaryFunctors.h>
#include <igl/predicates/predicates.h>
#include <wmtk/utils/Morton.h>
#include <cassert>
#include <cmath>
#include "wmtk/ExecutionScheduler.hpp"
#include "wmtk/utils/AMIPS.h"
#include "wmtk/utils/ExecutorUtils.hpp"
#include "wmtk/utils/Logger.hpp"
#include "wmtk/utils/TetraQualityUtils.hpp"
#include "wmtk/utils/io.hpp"

namespace app::interior_tet_opt {

void InteriorTetOpt::initialize(
    const std::vector<Eigen::Vector3d>& verts,
    const std::vector<std::array<size_t, 4>>& tets)
{
    p_vertex_attrs = &m_vertex_attribute;
    p_tet_attrs = &m_tet_attribute;

    init(verts.size(), tets);

    for (auto i = 0; i < verts.size(); i++) {
        m_vertex_attribute[i].pos = verts[i];
    }

    // find boundary
    TetMesh::for_each_face([&](auto& f) {
        if (!switch_tetrahedron(f).has_value()) {
            auto vs = get_face_vertices(f);
            for (auto v : vs) {
                m_vertex_attribute[v.vid(*this)].freeze = true;
            }
        }
    });

    double sum_len = 0.;
    auto cnt_len = 0;
    TetMesh::for_each_edge([&](auto& e) {
        auto len2 = get_length2(e);
        sum_len += std::sqrt(len2);
        cnt_len++;
    });

    target_l = sum_len / cnt_len;
    wmtk::logger().info("Input Avg Length {}", target_l);

    m_splitting_l2 = target_l * target_l * 16 / 9;
    m_collapsing_l2 = target_l * target_l * 16 / 25;
}

bool InteriorTetOpt::split_edge_before(const Tuple& loc0)
{
    if (loc0.is_boundary_edge(*this)) split_cache.local().bnd = true;

    auto& cache = split_cache.local();
    cache.v1_id = loc0.vid(*this);
    auto loc1 = loc0.switch_vertex(*this);
    cache.v2_id = loc1.vid(*this);
    //
    cache.max_quality = 0.;
    for (auto& loc : get_one_ring_tets_for_edge(loc0)) {
        cache.max_quality = std::max(cache.max_quality, m_tet_attribute[loc.tid(*this)].quality);
    }
    return true;
}


bool InteriorTetOpt::split_edge_after(const Tuple& loc)
{ // input: locs pointing to a list of tets and v_id

    std::vector<Tuple> locs = get_one_ring_tets_for_vertex(loc);
    size_t v_id = loc.vid(*this);

    auto& cache = split_cache.local();
    size_t v1_id = split_cache.local().v1_id;
    size_t v2_id = split_cache.local().v2_id;

    /// check inversion & rounding
    m_vertex_attribute[v_id].pos =
        (m_vertex_attribute[v1_id].pos + m_vertex_attribute[v2_id].pos) / 2;

    for (auto& loc : locs) {
        if (is_inverted(loc)) {
            return false;
        }
    }

    /// update quality
    for (auto& loc : locs) {
        auto& q = m_tet_attribute[loc.tid(*this)].quality;
        q = get_quality(loc);
        if (q > cache.max_quality) return false;
    }

    if (split_cache.local().bnd) m_vertex_attribute[v_id].freeze = true;

    // surface
    // m_vertex_attribute[v_id].m_is_on_surface = split_cache.local().is_edge_on_surface;

    /// update face attribute
    // add new and erase old

    m_vertex_attribute[v_id].partition_id = m_vertex_attribute[v1_id].partition_id;
    m_vertex_attribute[v_id].m_sizing_scalar =
        (m_vertex_attribute[v1_id].m_sizing_scalar + m_vertex_attribute[v2_id].m_sizing_scalar) / 2;

    return true;
}

bool InteriorTetOpt::swap_edge_before(const wmtk::TetMesh::Tuple& t)
{
    if (t.is_boundary_face(*this)) return false;
    auto incident_tets = get_incident_tets_for_edge(t);
    auto max_energy = -1.0;
    for (auto& l : incident_tets) {
        if (is_inverted(l)) {
            return false;
        }
        max_energy = std::max(m_tet_attribute[l.tid(*this)].quality, max_energy);
    }
    swap_cache.local().max_energy = max_energy;

    return true;
}

bool InteriorTetOpt::swap_edge_after(const Tuple& t)
{
    // after swap, t points to a face with 2 neighboring tets.
    auto oppo_tet = t.switch_tetrahedron(*this);
    assert(oppo_tet.has_value() && "Should not swap boundary.");

    auto twotets = std::vector<Tuple>{{t, *oppo_tet}};
    auto max_energy = -1.0;
    for (auto& l : twotets) {
        if (is_inverted(l)) {
            return false;
        }
        auto q = get_quality(l);
        m_tet_attribute[l.tid(*this)].quality = q;
        max_energy = std::max(q, max_energy);
    }
    if (max_energy >= swap_cache.local().max_energy) {
        return false;
    }

    return true;
}


bool InteriorTetOpt::swap_face_before(const Tuple& t)
{
    if ((t.is_boundary_face(*this))) return false;
    auto fid = t.fid(*this);
    auto oppo_tet = t.switch_tetrahedron(*this);
    assert(oppo_tet.has_value() && "Should not swap boundary.");
    swap_cache.local().max_energy = std::max(
        m_tet_attribute[t.tid(*this)].quality,
        m_tet_attribute[oppo_tet->tid(*this)].quality);

    return true;
}


bool InteriorTetOpt::swap_face_after(const Tuple& t)
{
    if (!TetMesh::swap_face_after(t)) return false;

    auto incident_tets = get_incident_tets_for_edge(t);

    auto max_energy = -1.0;
    for (auto& l : incident_tets) {
        if (is_inverted(l)) {
            return false;
        }
        auto q = get_quality(l);
        m_tet_attribute[l.tid(*this)].quality = q;
        max_energy = std::max(q, max_energy);
    }
    wmtk::logger().trace("quality {} from {}", max_energy, swap_cache.local().max_energy);

    if (max_energy >= swap_cache.local().max_energy) {
        return false;
    }

    return true;
}


bool InteriorTetOpt::swap_edge_44_before(const Tuple& t)
{
    if ((t).is_boundary_edge(*this)) return false;
    auto incident_tets = get_incident_tets_for_edge(t);
    auto max_energy = -1.0;
    for (auto& l : incident_tets) {
        max_energy = std::max(m_tet_attribute[l.tid(*this)].quality, max_energy);
    }
    swap_cache.local().max_energy = max_energy;

    return true;
}

bool InteriorTetOpt::swap_edge_44_after(const Tuple& t)
{
    auto incident_tets = get_incident_tets_for_edge(t);

    auto max_energy = -1.0;
    for (auto& l : incident_tets) {
        if (is_inverted(l)) {
            return false;
        }
        auto q = get_quality(l);
        m_tet_attribute[l.tid(*this)].quality = q;
        max_energy = std::max(q, max_energy);
    }

    if (max_energy >= swap_cache.local().max_energy) {
        return false;
    }

    return true;
}

bool InteriorTetOpt::smooth_before(const Tuple& t)
{
    return (!m_vertex_attribute[t.vid(*this)].freeze);
}


bool InteriorTetOpt::smooth_after(const Tuple& t)
{
    // Newton iterations are encapsulated here.
    wmtk::logger().trace("Newton iteration for vertex smoothing.");
    auto vid = t.vid(*this);

    auto locs = get_one_ring_tets_for_vertex(t);
    auto max_quality = 0.;
    for (auto& tet : locs) {
        max_quality = std::max(max_quality, m_tet_attribute[tet.tid(*this)].quality);
    }

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
                T[i * 3 + j] = m_vertex_attribute[local_verts[i]].pos[j];
            }
        }
        loc_id++;
    }

    auto old_pos = m_vertex_attribute[vid].pos;
    auto old_asssembles = assembles;
    m_vertex_attribute[vid].pos = wmtk::newton_method_from_stack(
        assembles,
        wmtk::AMIPS_energy,
        wmtk::AMIPS_jacobian,
        wmtk::AMIPS_hessian);
    wmtk::logger().trace(
        "old pos {} -> new pos {}",
        old_pos.transpose(),
        m_vertex_attribute[vid].pos.transpose());


    for (auto& loc : locs) {
        auto t_id = loc.tid(*this);
        if (is_inverted(loc)) return false;
        m_tet_attribute[t_id].quality = get_quality(loc);
    }


    return true;
}

} // namespace app::interior_tet_opt

#include <igl/Timer.h>

namespace app::interior_tet_opt {
void InteriorTetOpt::split_all_edges()
{
    igl::Timer timer;
    double time;
    timer.start();
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_split", loc);
    time = timer.getElapsedTime();
    wmtk::logger().info("edge split prepare time: {}s", time);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = wmtk::renewal_simple;

        executor.priority = [&](auto& m, auto op, auto& t) { return m.get_length2(t); };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [&](const auto& m, const auto& ele) {
            auto [weight, op, tup] = ele;
            auto length = m.get_length2(tup);
            if (length != weight) return false;
            //
            size_t v1_id = tup.vid(*this);
            size_t v2_id = tup.switch_vertex(*this).vid(*this);
            double sizing_ratio = (m_vertex_attribute[v1_id].m_sizing_scalar +
                                   m_vertex_attribute[v2_id].m_sizing_scalar) /
                                  2;
            if (length < m_splitting_l2 * sizing_ratio * sizing_ratio) return false;
            return true;
        };
        timer.start();
        executor(*this, collect_all_ops);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge split operation time: {}s", time);
    };
    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<InteriorTetOpt, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [&](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<InteriorTetOpt, wmtk::ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }
}
void InteriorTetOpt::smooth_all_vertices()
{
    igl::Timer timer;
    double time;
    timer.start();
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_vertices()) {
        collect_all_ops.emplace_back("vertex_smooth", loc);
    }
    time = timer.getElapsedTime();
    wmtk::logger().info("vertex smoothing prepare time: {}s", time);
    wmtk::logger().debug("Num verts {}", collect_all_ops.size());
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = wmtk::ExecutePass<InteriorTetOpt, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_vertex_mutex_one_ring(e, task_id);
        };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
        time = timer.getElapsedTime();
        wmtk::logger().info("vertex smoothing operation time parallel: {}s", time);
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<InteriorTetOpt, wmtk::ExecutionPolicy::kSeq>();
        executor(*this, collect_all_ops);
        time = timer.getElapsedTime();
        wmtk::logger().info("vertex smoothing operation time serial: {}s", time);
    }
}
void InteriorTetOpt::collapse_all_edges(bool is_limit_length)
{
    igl::Timer timer;
    double time;
    timer.start();
    collapse_cache.local().is_limit_length = is_limit_length;
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) {
        collect_all_ops.emplace_back("edge_collapse", loc);
        collect_all_ops.emplace_back("edge_collapse", loc.switch_vertex(*this));
    }
    auto collect_failure_ops = tbb::concurrent_vector<std::pair<std::string, Tuple>>();
    std::atomic_int count_success = 0;
    time = timer.getElapsedTime();
    wmtk::logger().info("edge collapse prepare time: {}s", time);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = [&](const auto& m, auto op, const auto& newts) {
            count_success++;
            std::vector<std::pair<std::string, wmtk::TetMesh::Tuple>> op_tups;
            for (auto t : newts) {
                op_tups.emplace_back(op, t);
                op_tups.emplace_back(op, t.switch_vertex(m));
            }
            return op_tups;
        };
        executor.priority = [&](auto& m, auto op, auto& t) { return -m.get_length2(t); };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [&](const auto& m, const auto& ele) {
            auto [weight, op, tup] = ele;
            auto length = m.get_length2(tup);
            if (length != -weight) return false;
            //
            size_t v1_id = tup.vid(*this);
            size_t v2_id = tup.switch_vertex(*this).vid(*this);
            double sizing_ratio = (m_vertex_attribute[v1_id].m_sizing_scalar +
                                   m_vertex_attribute[v2_id].m_sizing_scalar) /
                                  2;
            if (is_limit_length && length > m_collapsing_l2 * sizing_ratio * sizing_ratio)
                return false;
            return true;
        };

        executor.on_fail = [&](auto& m, auto op, auto& t) {
            collect_failure_ops.emplace_back(op, t);
        };
        // Execute!!
        do {
            count_success.store(0, std::memory_order_release);
            wmtk::logger().info("Prepare to collapse {}", collect_all_ops.size());
            executor(*this, collect_all_ops);
            wmtk::logger().info(
                "Collapsed {}, retrying failed {}",
                (int)count_success,
                collect_failure_ops.size());
            collect_all_ops.clear();
            for (auto& item : collect_failure_ops) collect_all_ops.emplace_back(item);
            collect_failure_ops.clear();
        } while (count_success.load(std::memory_order_acquire) > 0);
    };
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = wmtk::ExecutePass<InteriorTetOpt, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge collapse operation time parallel: {}s", time);
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<InteriorTetOpt, wmtk::ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge collapse operation time serial: {}s", time);
    }
}
void InteriorTetOpt::swap_all_edges_44()
{
    igl::Timer timer;
    double time;
    timer.start();
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_swap_44", loc);
    time = timer.getElapsedTime();
    wmtk::logger().info("edge swap 44 prepare time: {}s", time);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = wmtk::renewal_edges;
        executor.priority = [&](auto& m, auto op, auto& t) { return m.get_length2(t); };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = wmtk::ExecutePass<InteriorTetOpt, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge swap 44 operation time parallel: {}s", time);
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<InteriorTetOpt, wmtk::ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge swap 44 operation time serial: {}s", time);
    }
}
void InteriorTetOpt::swap_all_edges()
{
    igl::Timer timer;
    double time;
    timer.start();
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_swap", loc);
    time = timer.getElapsedTime();
    wmtk::logger().info("edge swap prepare time: {}s", time);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = wmtk::renewal_edges;
        executor.priority = [&](auto& m, auto op, auto& t) { return m.get_length2(t); };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = wmtk::ExecutePass<InteriorTetOpt, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge swap operation time parallel: {}s", time);
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<InteriorTetOpt, wmtk::ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge swap operation time serial: {}s", time);
    }
}
void InteriorTetOpt::swap_all_faces()
{
    igl::Timer timer;
    double time;
    timer.start();
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("face_swap", loc);
    time = timer.getElapsedTime();
    wmtk::logger().info("face swap prepare time: {}s", time);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = wmtk::renewal_faces;
        executor.priority = [](auto& m, auto op, auto& t) { return m.get_length2(t); };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = wmtk::ExecutePass<InteriorTetOpt, wmtk::ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_face_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("face swap operation time parallel: {}s", time);
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<InteriorTetOpt, wmtk::ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("face swap operation time serial: {}s", time);
    }
}


double InteriorTetOpt::get_quality(const Tuple& loc) const
{
    std::array<Eigen::Vector3d, 4> ps;
    auto its = oriented_tet_vertices(loc);
    for (int j = 0; j < 4; j++) {
        ps[j] = m_vertex_attribute[its[j].vid(*this)].pos;
    }

    std::array<double, 12> T;
    for (int j = 0; j < 3; j++) {
        T[0 * 3 + j] = ps[0][j];
        T[1 * 3 + j] = ps[1][j];
        T[2 * 3 + j] = ps[2][j];
        T[3 * 3 + j] = ps[3][j];
    }

    double energy = std::pow(wmtk::AMIPS_energy(T), 3);
    if (std::isinf(energy) || std::isnan(energy)) return 1e50;
    return energy;
}


bool InteriorTetOpt::is_inverted(const Tuple& loc) const
{
    // Return a positive value if the point pd lies below the
    // plane passing through pa, pb, and pc; "below" is defined so
    // that pa, pb, and pc appear in counterclockwise order when
    // viewed from above the plane.

    auto vs = oriented_tet_vertices(loc);

    igl::predicates::exactinit();
    auto res = igl::predicates::orient3d(
        m_vertex_attribute[vs[0].vid(*this)].pos,
        m_vertex_attribute[vs[1].vid(*this)].pos,
        m_vertex_attribute[vs[2].vid(*this)].pos,
        m_vertex_attribute[vs[3].vid(*this)].pos);
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

double InteriorTetOpt::get_length2(const wmtk::TetMesh::Tuple& l) const
{
    auto& m = *this;
    auto& v1 = l;
    auto v2 = l.switch_vertex(m);
    double length =
        (m.m_vertex_attribute[v1.vid(m)].pos - m.m_vertex_attribute[v2.vid(m)].pos).squaredNorm();
    return length;
}


bool InteriorTetOpt::invariants(const std::vector<Tuple>& tets)
{
    // check inversion
    for (auto& t : tets)
        if (is_inverted(t)) return false;

    return true;
}


void InteriorTetOpt::final_output_mesh(std::string file)
{
    consolidate_mesh();

    wmtk::MshData msh;

    const auto& vtx = get_vertices();
    msh.add_tet_vertices(vtx.size(), [&](size_t k) {
        auto i = vtx[k].vid(*this);
        return m_vertex_attribute[i].pos;
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

    msh.add_tet_vertex_attribute<1>("tv index", [&](size_t i) {
        return m_vertex_attribute[i].m_sizing_scalar;
    });
    msh.add_tet_attribute<1>("t energy", [&](size_t i) { return m_tet_attribute[i].quality; });

    msh.save(file, true);
}


} // namespace app::interior_tet_opt