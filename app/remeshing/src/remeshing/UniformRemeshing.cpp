#include "UniformRemeshing.h"
#include <igl/Timer.h>
#include <igl/is_edge_manifold.h>
#include <igl/predicates/predicates.h>
#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/TupleUtils.hpp>
using namespace app::remeshing;
using namespace wmtk;

auto renew = [](auto& m, auto op, auto& tris) {
    auto edges = m.new_edges_after(tris);
    auto optup = std::vector<std::pair<std::string, TriMesh::Tuple>>();
    for (auto& e : edges) optup.emplace_back(op, e);
    return optup;
};


auto edge_locker = [](auto& m, const auto& e, int task_id) {
    // TODO: this should not be here
    return m.try_set_edge_mutex_two_ring(e, task_id);
};

UniformRemeshing::UniformRemeshing(
    std::vector<Eigen::Vector3d> _m_vertex_positions,
    int num_threads,
    bool use_exact)
{
    NUM_THREADS = num_threads;
    m_envelope.use_exact = use_exact;

    p_vertex_attrs = &vertex_attrs;
    p_edge_attrs = &edge_attrs;

    vertex_attrs.resize(_m_vertex_positions.size());

    for (auto i = 0; i < _m_vertex_positions.size(); i++)
        vertex_attrs[i] = {_m_vertex_positions[i], 0};
}

void UniformRemeshing::create_mesh(
    size_t n_vertices,
    const std::vector<std::array<size_t, 3>>& tris,
    const std::vector<size_t>& frozen_verts,
    bool m_freeze,
    double eps)
{
    wmtk::TriMesh::init(n_vertices, tris);
    std::vector<Eigen::Vector3d> V(n_vertices);
    std::vector<Eigen::Vector3i> F(tris.size());
    for (auto i = 0; i < V.size(); i++) {
        V[i] = vertex_attrs[i].pos;
    }
    for (int i = 0; i < F.size(); ++i) F[i] << tris[i][0], tris[i][1], tris[i][2];
    if (eps > 0) {
        m_envelope.init(V, F, eps);
        m_has_envelope = true;
    } else
        m_envelope.init(V, F, 0.0);

    // TODO: this should not be here
    partition_mesh_morton();

    if (m_freeze) {
        for (auto v : frozen_verts) {
            vertex_attrs[v].freeze = true;
        }
        for (auto e : get_edges()) {
            if (is_boundary_edge(e)) {
                vertex_attrs[e.vid(*this)].freeze = true;
                vertex_attrs[e.switch_vertex(*this).vid(*this)].freeze = true;
            }
        }
    }
    edge_attrs.resize(tri_capacity() * 3);
    initialize_feature_edges();
}

void UniformRemeshing::cache_edge_positions(const Tuple& t)
{
    position_cache.local().v1p = vertex_attrs[t.vid(*this)].pos;
    position_cache.local().v2p = vertex_attrs[t.switch_vertex(*this).vid(*this)].pos;
    position_cache.local().partition_id = vertex_attrs[t.vid(*this)].partition_id;

    position_cache.local().v0 = t.vid(*this);
    position_cache.local().v1 = t.switch_vertex(*this).vid(*this);
    position_cache.local().was_feature_edge = is_feature_edge(t);
}

bool UniformRemeshing::invariants(const std::vector<Tuple>& new_tris)
{
    if (m_has_envelope) {
        for (auto& t : new_tris) {
            std::array<Eigen::Vector3d, 3> tris;
            auto vs = oriented_tri_vertices(t);
            for (auto j = 0; j < 3; j++) tris[j] = vertex_attrs[vs[j].vid(*this)].pos;
            if (m_envelope.is_outside(tris)) {
                return false;
            }
        }
    }
    return true;
}

// TODO: this should not be here
void UniformRemeshing::partition_mesh()
{
    auto m_vertex_partition_id = partition_TriMesh(*this, NUM_THREADS);
    for (auto i = 0; i < m_vertex_partition_id.size(); i++)
        vertex_attrs[i].partition_id = m_vertex_partition_id[i];
}

// TODO: morton should not be here, but inside wmtk
void UniformRemeshing::partition_mesh_morton()
{
    if (NUM_THREADS == 0) return;
    wmtk::logger().info("Number of parts: {} by morton", NUM_THREADS);

    tbb::task_arena arena(NUM_THREADS);

    arena.execute([&] {
        std::vector<Eigen::Vector3d> V_v(vert_capacity());

        tbb::parallel_for(tbb::blocked_range<int>(0, V_v.size()), [&](tbb::blocked_range<int> r) {
            for (int i = r.begin(); i < r.end(); i++) {
                V_v[i] = vertex_attrs[i].pos;
            }
        });

        struct sortstruct
        {
            int order;
            Resorting::MortonCode64 morton;
        };

        std::vector<sortstruct> list_v;
        list_v.resize(V_v.size());
        const int multi = 1000;
        // since the morton code requires a correct scale of input vertices,
        //  we need to scale the vertices if their coordinates are out of range
        std::vector<Eigen::Vector3d> V = V_v; // this is for rescaling vertices
        Eigen::Vector3d vmin, vmax;
        vmin = V.front();
        vmax = V.front();

        for (size_t j = 0; j < V.size(); j++) {
            for (int i = 0; i < 3; i++) {
                vmin(i) = std::min(vmin(i), V[j](i));
                vmax(i) = std::max(vmax(i), V[j](i));
            }
        }

        Eigen::Vector3d center = (vmin + vmax) / 2;

        tbb::parallel_for(tbb::blocked_range<int>(0, V.size()), [&](tbb::blocked_range<int> r) {
            for (int i = r.begin(); i < r.end(); i++) {
                V[i] = V[i] - center;
            }
        });

        Eigen::Vector3d scale_point =
            vmax - center; // after placing box at origin, vmax and vmin are symetric.

        double xscale, yscale, zscale;
        xscale = fabs(scale_point[0]);
        yscale = fabs(scale_point[1]);
        zscale = fabs(scale_point[2]);
        double scale = std::max(std::max(xscale, yscale), zscale);
        if (scale > 300) {
            tbb::parallel_for(tbb::blocked_range<int>(0, V.size()), [&](tbb::blocked_range<int> r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    V[i] = V[i] / scale;
                }
            });
        }

        tbb::parallel_for(tbb::blocked_range<int>(0, V.size()), [&](tbb::blocked_range<int> r) {
            for (int i = r.begin(); i < r.end(); i++) {
                list_v[i].morton = Resorting::MortonCode64(
                    int(V[i][0] * multi),
                    int(V[i][1] * multi),
                    int(V[i][2] * multi));
                list_v[i].order = i;
            }
        });

        const auto morton_compare = [](const sortstruct& a, const sortstruct& b) {
            return (a.morton < b.morton);
        };

        tbb::parallel_sort(list_v.begin(), list_v.end(), morton_compare);

        int interval = list_v.size() / NUM_THREADS + 1;

        tbb::parallel_for(
            tbb::blocked_range<int>(0, list_v.size()),
            [&](tbb::blocked_range<int> r) {
                for (int i = r.begin(); i < r.end(); i++) {
                    vertex_attrs[list_v[i].order].partition_id = i / interval;
                }
            });
    });
}

std::vector<TriMesh::Tuple> UniformRemeshing::new_edges_after(
    const std::vector<TriMesh::Tuple>& tris) const
{
    std::vector<TriMesh::Tuple> new_edges;

    for (auto t : tris) {
        for (auto j = 0; j < 3; j++) {
            new_edges.push_back(tuple_from_edge(t.fid(*this), j));
        }
    }
    wmtk::unique_edge_tuples(*this, new_edges);
    return new_edges;
}

bool UniformRemeshing::swap_edge_before(const Tuple& t)
{
    // Do not swap feature edges
    if (is_feature_edge(t)) {
        return false;
    }
    if (!TriMesh::swap_edge_before(t)) return false;
    if (vertex_attrs[t.vid(*this)].freeze && vertex_attrs[t.switch_vertex(*this).vid(*this)].freeze)
        return false;
    return true;
}


bool UniformRemeshing::swap_edge_after(const TriMesh::Tuple& t)
{
    std::vector<TriMesh::Tuple> tris;
    tris.push_back(t);
    tris.push_back(t.switch_edge(*this));
    return true;
}

std::vector<TriMesh::Tuple> UniformRemeshing::replace_edges_after_split(
    const std::vector<TriMesh::Tuple>& tris,
    const size_t vid_threshold) const
{
    // For edge split, we do not immediately push back split sub-edges
    // only push back those edges which are already present, but invalidated by hash mechanism.
    std::vector<TriMesh::Tuple> new_edges;
    new_edges.reserve(tris.size());
    for (auto t : tris) {
        auto tmptup = (t.switch_vertex(*this)).switch_edge(*this);
        if (tmptup.vid(*this) < vid_threshold &&
            (tmptup.switch_vertex(*this)).vid(*this) < vid_threshold)
            new_edges.push_back(tmptup);
    }
    return new_edges;
}

std::vector<TriMesh::Tuple> UniformRemeshing::new_sub_edges_after_split(
    const std::vector<TriMesh::Tuple>& tris) const
{
    // only push back the renewed original edges
    std::vector<TriMesh::Tuple> new_edges2;
    new_edges2.reserve(tris.size() * 3);
    for (auto t : tris) {
        new_edges2.push_back(t);
        new_edges2.push_back(t.switch_edge(*this));
        new_edges2.push_back((t.switch_vertex(*this)).switch_edge(*this));
    }

    wmtk::unique_edge_tuples(*this, new_edges2);
    return new_edges2;
}


bool UniformRemeshing::collapse_edge_before(const Tuple& t)
{
    size_t v0 = t.vid(*this);
    size_t v1 = t.switch_vertex(*this).vid(*this);

    // if (is_feature_edge(t)) {
    //     return false;
    // }

    if (vertex_attrs[v0].feature || vertex_attrs[v1].feature) {
        return false;
    }

    if (vertex_attrs[v0].freeze || vertex_attrs[v1].freeze) {
        return false;
    }

    if (!TriMesh::collapse_edge_before(t)) {
        return false;
    }
    cache_edge_positions(t);
    return true;
}


bool UniformRemeshing::collapse_edge_after(const TriMesh::Tuple& t)
{
    const Eigen::Vector3d p = (position_cache.local().v1p + position_cache.local().v2p) / 2.0;
    auto vid = t.vid(*this);
    vertex_attrs[vid].pos = p;
    vertex_attrs[vid].partition_id = position_cache.local().partition_id;

    return true;
}

bool UniformRemeshing::split_edge_before(const Tuple& t)
{
    if (!TriMesh::split_edge_before(t)) return false;
    cache_edge_positions(t);
    return true;
}


// bool UniformRemeshing::split_edge_after(const TriMesh::Tuple& t)
// {
//     const Eigen::Vector3d p = (position_cache.local().v1p + position_cache.local().v2p) / 2.0;
//     auto vid = t.switch_vertex(*this).vid(*this);
//     vertex_attrs[vid].pos = p;
//     vertex_attrs[vid].partition_id = position_cache.local().partition_id;

//     size_t v0 = t.vid(*this);
//     size_t v1 = t.switch_vertex(*this).vid(*this);

//     bool f0 = is_feature_vertex(v0);
//     bool f1 = is_feature_vertex(v1);
//     vertex_attrs[vid].feature = (f0 && f1);

//     return true;
// }

bool UniformRemeshing::split_edge_after(const TriMesh::Tuple& t)
{
    const Eigen::Vector3d p = (position_cache.local().v1p + position_cache.local().v2p) / 2.0;

    const size_t vnew = t.switch_vertex(*this).vid(*this);
    vertex_attrs[vnew].pos = p;
    vertex_attrs[vnew].partition_id = position_cache.local().partition_id;

    const size_t old0 = position_cache.local().v0;
    const size_t old1 = position_cache.local().v1;

    const bool f0 = is_feature_vertex(old0);
    const bool f1 = is_feature_vertex(old1);
    vertex_attrs[vnew].feature = (f0 && f1);

    if (position_cache.local().was_feature_edge) {
        auto vt = t.switch_vertex(*this);
        for (auto e : get_one_ring_edges_for_vertex(vt)) {
            const size_t a = e.vid(*this);
            const size_t b = e.switch_vertex(*this).vid(*this);
            if ((a == vnew && (b == old0 || b == old1)) ||
                (b == vnew && (a == old0 || a == old1))) {
                edge_attrs[e.eid(*this)].feature = true;
            }
        }
    }

    return true;
}


bool UniformRemeshing::smooth_before(const Tuple& t)
{
    if (vertex_attrs[t.vid(*this)].freeze) return false;
    return true;
}

bool UniformRemeshing::smooth_after(const TriMesh::Tuple& t)
{
    auto one_ring_tris = get_one_ring_tris_for_vertex(t);
    if (one_ring_tris.size() < 2) {
        return false;
    }
    Eigen::Vector3d after_smooth = tangential_smooth(t);
    if (after_smooth.hasNaN()) return false;
    // add envelope projection and check here
    // todo
    vertex_attrs[t.vid(*this)].pos = after_smooth;
    return true;
}

double UniformRemeshing::compute_edge_cost_collapse(const TriMesh::Tuple& t, double L) const
{
    double l =
        (vertex_attrs[t.vid(*this)].pos - vertex_attrs[t.switch_vertex(*this).vid(*this)].pos)
            .norm();
    if (l < (4. / 5.) * L) return ((4. / 5.) * L - l);
    return -1;
}
double UniformRemeshing::compute_edge_cost_split(const TriMesh::Tuple& t, double L) const
{
    double l =
        (vertex_attrs[t.vid(*this)].pos - vertex_attrs[t.switch_vertex(*this).vid(*this)].pos)
            .norm();
    if (l > (4. / 3.) * L) return (l - (4. / 3.) * L);
    return -1;
}

double UniformRemeshing::compute_vertex_valence(const TriMesh::Tuple& t) const
{
    std::vector<std::pair<TriMesh::Tuple, int>> valences(3);
    valences[0] = std::make_pair(t, get_valence_for_vertex(t));
    auto t2 = t.switch_vertex(*this);
    valences[1] = std::make_pair(t2, get_valence_for_vertex(t2));
    auto t3 = (t.switch_edge(*this)).switch_vertex(*this);
    valences[2] = std::make_pair(t3, get_valence_for_vertex(t3));

    if ((t.switch_face(*this)).has_value()) {
        auto t4 = (((t.switch_face(*this)).value()).switch_edge(*this)).switch_vertex(*this);
        valences.emplace_back(t4, get_valence_for_vertex(t4));
    }
    double cost_before_swap = 0.0;
    double cost_after_swap = 0.0;

    // check if it's internal vertex or bondary vertex
    // navigating starting one edge and getting back to the start

    for (int i = 0; i < valences.size(); i++) {
        TriMesh::Tuple vert = valences[i].first;
        int val = 6;
        auto one_ring_edges = get_one_ring_edges_for_vertex(vert);
        for (auto edge : one_ring_edges) {
            if (is_boundary_edge(edge)) {
                val = 4;
                break;
            }
        }
        cost_before_swap += (double)(valences[i].second - val) * (valences[i].second - val);
        cost_after_swap +=
            (i < 2) ? (double)(valences[i].second - 1 - val) * (valences[i].second - 1 - val)
                    : (double)(valences[i].second + 1 - val) * (valences[i].second + 1 - val);
    }
    return (cost_before_swap - cost_after_swap);
}

std::vector<double> UniformRemeshing::average_len_valen()
{
    double average_len = 0.0;
    double average_valen = 0.0;
    auto edges = get_edges();
    auto verts = get_vertices();
    double maxlen = std::numeric_limits<double>::min();
    double maxval = std::numeric_limits<double>::min();
    double minlen = std::numeric_limits<double>::max();
    double minval = std::numeric_limits<double>::max();
    for (auto& loc : edges) {
        double currentlen = (vertex_attrs[loc.vid(*this)].pos -
                             vertex_attrs[loc.switch_vertex(*this).vid(*this)].pos)
                                .norm();
        average_len += currentlen;
        if (maxlen < currentlen) maxlen = currentlen;
        if (minlen > currentlen) minlen = currentlen;
    }
    average_len /= edges.size();
    for (auto& loc : verts) {
        double currentval = get_one_ring_edges_for_vertex(loc).size();
        average_valen += currentval;
        if (maxval < currentval) maxval = currentval;
        if (minval > currentval) minval = currentval;
    }
    average_valen /= verts.size();
    int cnt = 0;
    std::vector<double> rtn{average_len, maxlen, minlen, average_valen, maxval, minval};
    return rtn;
}

std::vector<TriMesh::Tuple> UniformRemeshing::new_edges_after_swap(const TriMesh::Tuple& t) const
{
    std::vector<TriMesh::Tuple> new_edges;

    new_edges.push_back(t.switch_edge(*this));
    new_edges.push_back((t.switch_face(*this).value()).switch_edge(*this));
    new_edges.push_back((t.switch_vertex(*this)).switch_edge(*this));
    new_edges.push_back(((t.switch_vertex(*this)).switch_face(*this).value()).switch_edge(*this));
    return new_edges;
}

bool UniformRemeshing::collapse_remeshing(double L)
{
    igl::Timer timer;
    timer.start();
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    auto collect_tuples = tbb::concurrent_vector<Tuple>();

    for_each_edge([&](auto& tup) { collect_tuples.emplace_back(tup); });
    collect_all_ops.reserve(collect_tuples.size());
    for (auto& t : collect_tuples) collect_all_ops.emplace_back("edge_collapse", t);
    wmtk::logger().info(
        "***** collapse get edges time *****: {} ms",
        timer.getElapsedTimeInMilliSec());

    wmtk::logger().info("size for edges to be collapse is {}", collect_all_ops.size());
    auto setup_and_execute = [&](auto executor) {
        executor.renew_neighbor_tuples = renew;
        executor.priority = [&](auto& m, auto _, auto& e) {
            return m.compute_edge_cost_collapse(e, L);
        };
        executor.num_threads = NUM_THREADS;
        executor.should_renew = [](auto val) { return (val > 0); };
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [val, op, e] = ele;
            if (val < 0) return false; // priority is negated.
            return true;
        };
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<UniformRemeshing, ExecutionPolicy::kPartition>();
        executor.lock_vertices = edge_locker;
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<UniformRemeshing, ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }

    return true;
}
bool UniformRemeshing::split_remeshing(double L)
{
    igl::Timer timer;
    timer.start();
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    size_t vid_threshold = 0;
    std::atomic_int count_success = 0;
    auto collect_tuples = tbb::concurrent_vector<Tuple>();

    for_each_edge([&](auto& tup) { collect_tuples.emplace_back(tup); });
    collect_all_ops.reserve(collect_tuples.size());
    for (auto& t : collect_tuples) collect_all_ops.emplace_back("edge_split", t);
    wmtk::logger().info(
        "***** split get edges time *****: {} ms",
        timer.getElapsedTimeInMilliSec());

    wmtk::logger().info("size for edges to be split is {}", collect_all_ops.size());
    auto edges2 = tbb::concurrent_vector<std::pair<std::string, TriMesh::Tuple>>();
    auto setup_and_execute = [&](auto& executor) {
        vid_threshold = vert_capacity();
        executor.num_threads = NUM_THREADS;
        executor.renew_neighbor_tuples = [&](auto& m, auto op, auto& tris) {
            count_success++;
            auto edges = m.replace_edges_after_split(tris, vid_threshold);
            for (auto e2 : m.new_sub_edges_after_split(tris)) edges2.emplace_back(op, e2);
            auto optup = std::vector<std::pair<std::string, TriMesh::Tuple>>();
            for (auto& e : edges) optup.emplace_back(op, e);
            return optup;
        };
        executor.priority = [&](auto& m, auto _, auto& e) {
            return m.compute_edge_cost_split(e, L);
        };
        executor.should_renew = [](auto val) { return (val > 0); };
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [val, op, e] = ele;
            if (val < 0) return false;
            return true;
        };
        // Execute!!
        do {
            count_success.store(0, std::memory_order_release);
            executor(*this, collect_all_ops);
            collect_all_ops.clear();
            for (auto& item : edges2) collect_all_ops.emplace_back(item);
            edges2.clear();
        } while (count_success.load(std::memory_order_acquire) > 0);
    };

    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<UniformRemeshing, ExecutionPolicy::kPartition>();
        executor.lock_vertices = edge_locker;
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<UniformRemeshing, ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }

    return true;
}

bool UniformRemeshing::smooth_all_vertices()
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("vertex_smooth", loc);

    auto setup_and_execute = [&](auto executor) {
        executor.num_threads = NUM_THREADS;

        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<UniformRemeshing, ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) {
            return m.try_set_vertex_mutex_one_ring(e, task_id);
        };
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<UniformRemeshing, ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }

    return true;
}

bool UniformRemeshing::swap_remeshing()
{
    igl::Timer timer;
    timer.start();
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    wmtk::logger().info("size for edges to swap is {}", collect_all_ops.size());
    auto collect_tuples = tbb::concurrent_vector<Tuple>();

    for_each_edge([&](auto& tup) { collect_tuples.emplace_back(tup); });
    collect_all_ops.reserve(collect_tuples.size());
    for (auto& t : collect_tuples) collect_all_ops.emplace_back("edge_swap", t);
    wmtk::logger().info("***** swap get edges time *****: {} ms", timer.getElapsedTimeInMilliSec());


    auto setup_and_execute = [&](auto executor) {
        executor.renew_neighbor_tuples = renew;
        executor.num_threads = NUM_THREADS;
        executor.priority = [](auto& m, auto op, const Tuple& e) {
            return m.compute_vertex_valence(e);
        };
        executor.should_renew = [](auto val) { return (val > 0); };
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [val, _, e] = ele;
            auto val_energy = (m.compute_vertex_valence(e));
            return (val_energy > 1e-5) && ((val_energy - val) * (val_energy - val) < 1e-8);
        };
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<UniformRemeshing, ExecutionPolicy::kPartition>();
        executor.lock_vertices = edge_locker;
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<UniformRemeshing, ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }

    return true;
}
double area(UniformRemeshing& m, std::array<TriMesh::Tuple, 3>& verts)
{
    return ((m.vertex_attrs[verts[0].vid(m)].pos - m.vertex_attrs[verts[2].vid(m)].pos)
                .cross(m.vertex_attrs[verts[1].vid(m)].pos - m.vertex_attrs[verts[2].vid(m)].pos))
               .norm() /
           2.0;
}

Eigen::Vector3d normal(UniformRemeshing& m, std::array<TriMesh::Tuple, 3>& verts)
{
    return ((m.vertex_attrs[verts[0].vid(m)].pos - m.vertex_attrs[verts[2].vid(m)].pos)
                .cross(m.vertex_attrs[verts[1].vid(m)].pos - m.vertex_attrs[verts[2].vid(m)].pos))
        .normalized();
}

Eigen::Vector3d UniformRemeshing::smooth(const TriMesh::Tuple& t)
{
    auto one_ring_edges = get_one_ring_edges_for_vertex(t);
    if (one_ring_edges.size() < 3) return vertex_attrs[t.vid(*this)].pos;
    Eigen::Vector3d after_smooth(0, 0, 0);
    Eigen::Vector3d after_smooth_boundary(0, 0, 0);
    int boundary = 0;
    for (auto e : one_ring_edges) {
        if (is_boundary_edge(e)) {
            after_smooth_boundary += vertex_attrs[e.vid(*this)].pos;
            boundary++;
            continue;
        }
        after_smooth += vertex_attrs[e.vid(*this)].pos;
    }

    if (boundary)
        after_smooth = after_smooth_boundary / boundary;
    else
        after_smooth /= one_ring_edges.size();
    return after_smooth;
}

Eigen::Vector3d UniformRemeshing::tangential_smooth(const Tuple& t)
{
    auto one_ring_tris = get_one_ring_tris_for_vertex(t);
    if (one_ring_tris.size() < 2) return vertex_attrs[t.vid(*this)].pos;
    Eigen::Vector3d after_smooth = smooth(t);
    // get normal and area of each face
    // auto area = [](auto& m, auto& verts) {
    //     return ((m.vertex_attrs[verts[0].vid(m)].pos - m.vertex_attrs[verts[2].vid(m)].pos)
    //                 .cross(
    //                     m.vertex_attrs[verts[1].vid(m)].pos -
    //                     m.vertex_attrs[verts[2].vid(m)].pos))
    //                .norm() /
    //            2.0;
    // };
    // auto normal = [](auto& m, auto& verts) {
    //     return ((m.vertex_attrs[verts[0].vid(m)].pos - m.vertex_attrs[verts[2].vid(m)].pos)
    //                 .cross(
    //                     m.vertex_attrs[verts[1].vid(m)].pos -
    //                     m.vertex_attrs[verts[2].vid(m)].pos))
    //         .normalized();
    // };
    // auto w0 = 0.0;
    // Eigen::Vector3d n0(0.0, 0.0, 0.0);
    // for (auto& e : one_ring_tris) {
    //     auto verts = oriented_tri_vertices(e);
    //     w0 += area(*this, verts);
    //     n0 += area(*this, verts) * normal(*this, verts);
    // }
    // n0 /= w0;
    // if (n0.norm() < 1e-10) return vertex_attrs[t.vid(*this)].pos;
    // n0 = n0.normalized();
    // after_smooth += n0 * n0.transpose() * (vertex_attrs[t.vid(*this)].pos - after_smooth);
    // assert(check_mesh_connectivity_validity());
    return after_smooth;
}


bool UniformRemeshing::uniform_remeshing(double L, int iterations)
{
    int cnt = 0;
    wmtk::logger().info("target len is: {}", L);

    igl::Timer timer;
    while (cnt < iterations) {
        cnt++;
        wmtk::logger().info("??? Pass ??? {}", cnt);
        // split
        timer.start();
        split_remeshing(L);
        wmtk::logger().info("--------split time-------: {} ms", timer.getElapsedTimeInMilliSec());
        // collpase
        timer.start();
        collapse_remeshing(L);
        wmtk::logger().info(
            "--------collapse time-------: {} ms",
            timer.getElapsedTimeInMilliSec());
        // swap edges
        timer.start();
        swap_remeshing();
        wmtk::logger().info("--------swap time-------: {} ms", timer.getElapsedTimeInMilliSec());
        // smoothing
        timer.start();
        smooth_all_vertices();
        wmtk::logger().info("--------smooth time-------: {} ms", timer.getElapsedTimeInMilliSec());

        partition_mesh_morton();
    }
    wmtk::logger().info("finished {} remeshing iterations", iterations);
    wmtk::logger().info("+++++++++finished+++++++++");
    return true;
}
bool UniformRemeshing::write_triangle_mesh(std::string path)
{
    // write the collapsed mesh into a obj and assert the mesh is manifold
    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(vert_capacity(), 3);
    for (auto& t : get_vertices()) {
        auto i = t.vid(*this);
        V.row(i) = vertex_attrs[i].pos;
    }

    Eigen::MatrixXi F = Eigen::MatrixXi::Constant(tri_capacity(), 3, -1);
    for (auto& t : get_faces()) {
        auto i = t.fid(*this);
        auto vs = oriented_tri_vertices(t);
        for (int j = 0; j < 3; j++) {
            F(i, j) = vs[j].vid(*this);
        }
    }
    igl::write_triangle_mesh(path, V, F);
    bool manifold = check_edge_manifold();
    assert(manifold);

    return manifold;
}

void UniformRemeshing::set_feature_vertices(const std::vector<size_t>& feature_vertices)
{
    for (size_t i = 0; i < vertex_attrs.size(); ++i) {
        vertex_attrs[i].feature = false;
    }

    for (size_t v : feature_vertices) {
        if (v < vertex_attrs.size()) {
            vertex_attrs[v].feature = true;
        } else {
            wmtk::logger().warn("set_feature_vertices: index {} out of range", v);
        }
    }

    wmtk::logger().info("Marked {} feature vertices", feature_vertices.size());
}

bool UniformRemeshing::is_feature_vertex(size_t vid) const
{
    return vid < vertex_attrs.size() && vertex_attrs[vid].feature;
}

bool UniformRemeshing::is_feature_edge(const Tuple& t) const
{
    return edge_attrs[t.eid(*this)].feature;
}

void UniformRemeshing::set_feature_edges(const std::vector<std::array<size_t, 2>>& feature_edges)
{
    m_input_feature_edges = feature_edges;
}

static inline uint64_t edge_key(size_t a, size_t b)
{
    if (a > b) std::swap(a, b);
    return (uint64_t(a) << 32) ^ uint64_t(b);
}

void UniformRemeshing::initialize_feature_edges()
{
    std::vector<uint64_t> feature_edge;
    feature_edge.reserve(m_input_feature_edges.size());


    for (const auto& ee : m_input_feature_edges) {
        const size_t a = ee[0];
        const size_t b = ee[1];

        feature_edge.push_back(edge_key(a, b));

        if (a < vertex_attrs.size()) vertex_attrs[a].feature = true;
        if (b < vertex_attrs.size()) vertex_attrs[b].feature = true;
    }


    std::sort(feature_edge.begin(), feature_edge.end());
    feature_edge.erase(std::unique(feature_edge.begin(), feature_edge.end()), feature_edge.end());

    for (auto e : get_edges()) {
        const size_t eid = e.eid(*this);
        if (eid >= edge_attrs.size()) edge_attrs.resize(eid + 1);
        edge_attrs[eid].feature = false;
    }


    size_t found = 0;
    for (auto e : get_edges()) {
        const size_t a = e.vid(*this);
        const size_t b = e.switch_vertex(*this).vid(*this);
        const size_t eid = e.eid(*this);

        const uint64_t k = edge_key(a, b);

        if (std::binary_search(feature_edge.begin(), feature_edge.end(), k)) {
            if (eid >= edge_attrs.size()) edge_attrs.resize(eid + 1);
            edge_attrs[eid].feature = true;
            ++found;
        }
    }

    wmtk::logger().info(
        "initialize_feature_edges: marked {} feature edges (requested {})",
        found,
        feature_edge.size());

    if (found != feature_edge.size()) {
        wmtk::logger().warn(
            "initialize_feature_edges: {} requested feature edges were not found in the mesh",
            feature_edge.size() - found);
    }
}

bool UniformRemeshing::write_feature_vertices_obj(const std::string& path) const
{
    std::ofstream out(path);
    if (!out.is_open()) return false;

    std::vector<size_t> fmap;
    fmap.reserve(vertex_attrs.size());

    for (size_t i = 0; i < vertex_attrs.size(); ++i) {
        if (!vertex_attrs[i].feature) continue;
        const auto& p = vertex_attrs[i].pos;
        out << "v " << p.x() << " " << p.y() << " " << p.z() << "\n";
        fmap.push_back(i);
    }

    for (size_t k = 0; k < fmap.size(); ++k) {
        out << "p " << (k + 1) << "\n";
    }

    return true;
}
