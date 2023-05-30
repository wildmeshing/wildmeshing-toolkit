
#include <igl/predicates/predicates.h>
#include "AdaptiveTessellation.h"
#include "wmtk/ExecutionScheduler.hpp"

#include <Eigen/src/Core/util/Constants.h>
#include <igl/Timer.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/AMIPS2D_autodiff.h>
#include <array>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TriQualityUtils.hpp>


#include <limits>
#include <optional>

namespace {

using namespace adaptive_tessellation;
using namespace wmtk;

class AdaptiveTessellationSmoothVertexOperation : public wmtk::TriMeshOperationShim<
                                                      AdaptiveTessellation,
                                                      AdaptiveTessellationSmoothVertexOperation,
                                                      wmtk::TriMeshSmoothVertexOperation>
{
public:
    ExecuteReturnData execute(AdaptiveTessellation& m, const Tuple& t)
    {
        return wmtk::TriMeshSmoothVertexOperation::execute(m, t);
    }
    bool before(AdaptiveTessellation& m, const Tuple& t)
    {
        if (wmtk::TriMeshSmoothVertexOperation::before(m, t)) {
            return m.smooth_before(t);
        }
        return false;
    }
    bool after(AdaptiveTessellation& m, ExecuteReturnData& ret_data)
    {
        if (wmtk::TriMeshSmoothVertexOperation::after(m, ret_data)) {
            ret_data.success &= m.smooth_after(ret_data.tuple);
        }
        return ret_data;
    }
    bool invariants(AdaptiveTessellation& m, ExecuteReturnData& ret_data)
    {
        if (wmtk::TriMeshSmoothVertexOperation::invariants(m, ret_data)) {
            ret_data.success &= m.invariants(ret_data.new_tris);
        }
        return ret_data;
    }
};

template <typename Executor>
void addCustomOps(Executor& e)
{
    e.add_operation(std::make_shared<AdaptiveTessellationSmoothVertexOperation>());
}
} // namespace

bool adaptive_tessellation::AdaptiveTessellation::smooth_before(const Tuple& t)
{
    if (vertex_attrs[t.vid(*this)].fixed) return false;
    if (mesh_parameters.m_bnd_freeze && is_boundary_vertex(t)) return false;
    return true;
}

bool adaptive_tessellation::AdaptiveTessellation::smooth_after(const Tuple& t)
{
    static std::atomic_int cnt = 0;
    wmtk::logger().info("smothing op # {}", cnt);
    // Newton iterations are encapsulated here.
    auto vid = t.vid(*this);
    auto locs = get_one_ring_tris_for_vertex(t);
    assert(locs.size() > 0);

    // infomation needed for newton's method
    wmtk::NewtonMethodInfo nminfo;
    nminfo.curve_id = vertex_attrs[t.vid(*this)].curve_id;
    nminfo.target_length = mesh_parameters.m_target_l;
    nminfo.neighbors.resize(locs.size(), 4);

    auto is_inverted_coordinates = [this, &vid](auto& A, auto& B) {
        auto res = igl::predicates::orient2d(A, B, this->vertex_attrs[vid].pos);
        if (res != igl::predicates::Orientation::POSITIVE)
            return true;
        else
            return false;
    };


    /// should only happen in debug
    // wmtk::logger().info("========== one ring contians  !!!{}!!!  tris ", locs.size());
    // auto last_tuples = oriented_tri_vertices(locs[0]);
    // Eigen::Vector2d last_v2, last_v3;
    // for (auto j = 0; j < 3; j++) {
    //     if (last_tuples[j].vid(*this) == vid) {
    //         last_v2 = vertex_attrs[last_tuples[(j + 1) % 3].vid(*this)].pos;
    //         last_v3 = vertex_attrs[last_tuples[(j + 2) % 3].vid(*this)].pos;
    //     }
    // }
    /// should only happen in debug

    for (auto i = 0; i < locs.size(); i++) {
        auto tri = locs[i];
        assert(!is_inverted(tri));
        auto local_tuples = oriented_tri_vertices(tri);
        for (auto j = 0; j < 3; j++) {
            if (local_tuples[j].vid(*this) == vid) {
                auto v2 = vertex_attrs[local_tuples[(j + 1) % 3].vid(*this)].pos;
                auto v3 = vertex_attrs[local_tuples[(j + 2) % 3].vid(*this)].pos;
                nminfo.neighbors.row(i) << v2(0), v2(1), v3(0), v3(1);
                assert(!is_inverted_coordinates(v2, v3));
                // if (i != 0) assert(last_v2 != v2);
                // last_v2 = v2;
                // sanity check, no inversion should be heres
            }
        }
        assert(locs.size() == nminfo.neighbors.rows());
    }
    // use a general root finding method that defaults to newton but if not changeing the
    // position, try gradient descent
    auto old_pos = vertex_attrs[vid].pos;
    auto old_t = vertex_attrs[vid].t;

    wmtk::DofVector dofx;
    if (is_boundary_vertex(t) && mesh_parameters.m_boundary_parameter) {
        dofx.resize(1);
        dofx[0] = vertex_attrs[t.vid(*this)].t; // t
    } else {
        dofx.resize(2);
        dofx = vertex_attrs[t.vid(*this)].pos; // uv;
    }

    double before_energy = get_one_ring_energy(t).first;

    // assert before energy is always less than after energy
    wmtk::State state = {};
    wmtk::newton_method_with_fallback(
        *mesh_parameters.m_energy,
        mesh_parameters.m_boundary,
        nminfo,
        dofx,
        state);

    // check boundary and project
    // this should be outdated since now every boundary vertex will be on boundary (but good
    // to have as an assert) add assert!!!!
    if (is_boundary_vertex(t) && mesh_parameters.m_boundary_parameter) {
        vertex_attrs[vid].t = dofx(0);
        vertex_attrs[vid].pos = mesh_parameters.m_boundary.t_to_uv(nminfo.curve_id, dofx(0));
    } else
        vertex_attrs[vid].pos = dofx;

    // check invariants
    if (!invariants(locs)) {
        vertex_attrs[vid].pos = old_pos;
        vertex_attrs[vid].t = old_t;
        return false;
    }
    auto energy_gradient = get_one_ring_energy(t);
    double after_energy = energy_gradient.first;
    assert(after_energy <= before_energy);
    assert(vid == t.vid(*this));
    if (!vertex_attrs[t.vid(*this)].fixed) {
        mesh_parameters.m_gradient += energy_gradient.second;
    }
    wmtk::logger().debug(
        "smoothing vertex {} before energy {} after energy {} gradient {}",
        vid,
        before_energy,
        after_energy,
        energy_gradient.second);
    assert(invariants(locs));
    cnt++;
    return true;
}

void adaptive_tessellation::AdaptiveTessellation::smooth_all_vertices()
{
    assert(mesh_parameters.m_energy != nullptr);
    wmtk::logger().info("=======smooth==========");
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
        auto executor =
            wmtk::ExecutePass<AdaptiveTessellation, wmtk::ExecutionPolicy::kPartition>();
        addCustomOps(executor);
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_vertex_mutex_one_ring(e, task_id);
        };
        executor.num_threads = NUM_THREADS;
        executor(*this, collect_all_ops);
        time = timer.getElapsedTime();
        wmtk::logger().info("vertex smoothing operation time parallel: {}s", time);
    } else {
        timer.start();
        auto executor = wmtk::ExecutePass<AdaptiveTessellation, wmtk::ExecutionPolicy::kSeq>();
        addCustomOps(executor);
        int itr = 0;
        do {
            mesh_parameters.m_gradient = Eigen::Vector2d(0., 0.);
            executor(*this, collect_all_ops);
            write_displaced_obj(
                mesh_parameters.m_output_folder + fmt::format("/smooth_{:03d}.obj", itr),
                mesh_parameters.m_project_to_3d);
            itr++;
        } while ((mesh_parameters.m_gradient / vert_capacity()).stableNorm() > 1e-4 && itr < 10);
        wmtk::logger().info("===== terminate smooth after {} itrs", itr);
        time = timer.getElapsedTime();
        wmtk::logger().info("vertex smoothing operation time serial: {}s", time);
    }
}
