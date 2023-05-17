
#include "Smooth.h"
#include <Eigen/src/Core/util/Constants.h>
#include <igl/Timer.h>
#include <lagrange/utils/timing.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/AMIPS2D_autodiff.h>
#include <wmtk/utils/Energy2dOptimizationUtils.h>
#include <array>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TriQualityUtils.hpp>
#include "AdaptiveTessellation.h"
#include "wmtk/ExecutionScheduler.hpp"

#include <limits>
#include <optional>

using namespace adaptive_tessellation;
using namespace wmtk;

TriMeshOperation::ExecuteReturnData AdaptiveTessellationSmoothVertexOperation::execute(
    AdaptiveTessellation& m,
    const Tuple& t)
{
    return wmtk::TriMeshSmoothVertexOperation::execute(m, t);
}
bool AdaptiveTessellationSmoothVertexOperation::before(AdaptiveTessellation& m, const Tuple& t)
{
    if (wmtk::TriMeshSmoothVertexOperation::before(m, t)) {
        if (m.vertex_attrs[t.vid(m)].fixed) return false;
        if (m.mesh_parameters.m_bnd_freeze && m.is_boundary_vertex(t)) return false;
        if (m.is_boundary_vertex(t)) {
            for (auto& e : m.get_one_ring_edges_for_vertex(t)) {
                if (m.is_boundary_edge(e)) {
                    m.vertex_attrs[t.vid(m)].curve_id = m.edge_attrs[e.eid(m)].curve_id.value();
                    break;
                }
            }
        }
        return true;
    }
    return false;
}
bool AdaptiveTessellationSmoothVertexOperation::after(
    AdaptiveTessellation& m,
    ExecuteReturnData& ret_data)
{
    if (wmtk::TriMeshSmoothVertexOperation::after(m, ret_data)) {
        ret_data.success &= m.smooth_after(ret_data.tuple);
    }
    return ret_data;
}

TriMeshOperation::ExecuteReturnData AdaptiveTessellationSmoothSeamVertexOperation::execute(
    AdaptiveTessellation& m,
    const Tuple& t)
{
    return wmtk::TriMeshSmoothVertexOperation::execute(m, t);
}
bool AdaptiveTessellationSmoothSeamVertexOperation::before(AdaptiveTessellation& m, const Tuple& t)
{
    static std::atomic_int cnt = 0;
    // m.write_displaced_obj(
    //     m.mesh_parameters.m_output_folder + fmt::format("/smooth_{:04d}.obj", cnt),
    //     m.mesh_parameters.m_displacement);
    // m.write_obj(m.mesh_parameters.m_output_folder + fmt::format("/smooth_{:04d}_2d.obj", cnt));

    if (wmtk::TriMeshSmoothVertexOperation::before(m, t)) {
        assert(!m.vertex_attrs[t.vid(m)].fixed);
        if (m.is_seam_vertex(t)) {
            wmtk::logger().info("///// seam vertex smooth");
        }
        cnt++;
        return m.smooth_before(t);
    }

    return false;
}
bool AdaptiveTessellationSmoothSeamVertexOperation::after(
    AdaptiveTessellation& m,
    ExecuteReturnData& ret_data)
{
    auto smooth_start_time = lagrange::get_timestamp();
    if (!wmtk::TriMeshSmoothVertexOperation::after(m, ret_data)) return false;

    if (!ret_data.success) return false;
    // check if the vertex is fixed
    if (m.vertex_attrs[ret_data.tuple.vid(m)].fixed) return false;
    static std::atomic_int cnt = 0;
    wmtk::logger().info("smothing op # {}", cnt);

    std::vector<wmtk::TriMesh::Tuple> one_ring_tris =
        m.get_one_ring_tris_for_vertex(ret_data.tuple);
    assert(one_ring_tris.size() > 0);

    // infomation needed for newton's method
    // multiple nminfo for seam vertices
    std::vector<wmtk::NewtonMethodInfo> nminfos;
    // push in current vertex's nminfo
    wmtk::NewtonMethodInfo primary_nminfo;
    m.get_nminfo_for_vertex(ret_data.tuple, primary_nminfo);
    nminfos.emplace_back(primary_nminfo);
    // check if it is seam by getting the one ring edges
    // add the nminfo for each seam edge
    // keep a list of mirror vertices to update the dofx later
    std::vector<wmtk::TriMesh::Tuple> mirror_vertices;
    // can not be start/end or t-junction of curve
    assert(m.get_all_mirror_vids(ret_data.tuple).size() <= 2);
    for (auto& e : m.get_one_ring_edges_for_vertex(ret_data.tuple)) {
        if (m.is_seam_edge(e)) {
            wmtk::NewtonMethodInfo nminfo;
            assert(e.switch_vertex(m).vid(m) == ret_data.tuple.vid(m));
            wmtk::TriMesh::Tuple mirror_edge = m.get_oriented_mirror_edge(e);
            wmtk::TriMesh::Tuple mirror_v = mirror_edge;
            assert(m.get_mirror_vertex(mirror_v).vid(m) == ret_data.tuple.vid(m));
            if (m.vertex_attrs[mirror_v.vid(m)].fixed) return false;
            if (m.mesh_parameters.m_bnd_freeze) return false;
            m.vertex_attrs[mirror_v.vid(m)].curve_id =
                m.edge_attrs[mirror_edge.eid(m)].curve_id.value();
            mirror_vertices.emplace_back(mirror_v);
            // collect the triangles for invariants check
            for (auto& mirror_v_tri : m.get_one_ring_tris_for_vertex(mirror_v)) {
                one_ring_tris.emplace_back(mirror_v_tri);
            }
            m.get_nminfo_for_vertex(mirror_v, nminfo);
            nminfos.push_back(nminfo);
            break;
        }
    }
    ret_data.new_tris = one_ring_tris;
    // use a general root finding method that defaults to newton but if not changeing
    // the position, try gradient descent
    const auto& old_pos = m.vertex_attrs[ret_data.tuple.vid(m)].pos;
    auto old_t = m.vertex_attrs[ret_data.tuple.vid(m)].t;

    wmtk::State state = {};
    if (m.is_boundary_vertex(ret_data.tuple) && m.mesh_parameters.m_boundary_parameter) {
        state.dofx.resize(1);
        state.dofx[0] = m.vertex_attrs[ret_data.tuple.vid(m)].t; // t
    } else {
        state.dofx.resize(2);
        state.dofx = m.vertex_attrs[ret_data.tuple.vid(m)].pos; // uv;
    }
    // get current state: energy, gradient, hessiane
    wmtk::optimization_state_update(
        *m.mesh_parameters.m_energy,
        nminfos,
        m.mesh_parameters.m_boundary,
        state);
    double before_energy = state.value;

    wmtk::logger().info("{} of nminfo", nminfos.size());
    wmtk::logger().info("before energy: {}", state.value);
    m.mesh_parameters.log(
        {{"smooth_op_" + std::to_string(cnt), {"before_energy", std::to_string(state.value)}}});
    m.mesh_parameters.log({{"smooth_op_" + std::to_string(cnt),
                            {"is_seam_vertex", std::to_string(m.is_seam_vertex(ret_data.tuple))}}});
    m.mesh_parameters.log(
        {{"smooth_op_" + std::to_string(cnt), {"num_neighbor", std::to_string(nminfos.size())}}});
    // this is the main optimization function
    // newton's method
    wmtk::optimization_dofx_update(
        *m.mesh_parameters.m_energy,
        m.mesh_parameters.m_boundary,
        nminfos,
        state);
    // assert after energy is always less than before energy
    assert(state.value <= before_energy);
    m.mesh_parameters.log(
        {{"smooth_op_" + std::to_string(cnt), {"after_energy", std::to_string(state.value)}}});
    // update the vertex attr for current vertex
    if (m.is_boundary_vertex(ret_data.tuple) && m.mesh_parameters.m_boundary_parameter) {
        m.vertex_attrs[ret_data.tuple.vid(m)].t = state.dofx(0);
        m.vertex_attrs[ret_data.tuple.vid(m)].pos =
            m.mesh_parameters.m_boundary.t_to_uv(primary_nminfo.curve_id, state.dofx(0));
    } else
        m.vertex_attrs[ret_data.tuple.vid(m)].pos = state.dofx;

    // check invariants after update the vertex position
    if (!m.invariants(one_ring_tris)) {
        m.vertex_attrs[ret_data.tuple.vid(m)].pos = old_pos;
        m.vertex_attrs[ret_data.tuple.vid(m)].t = old_t;
        return false;
    }

    // now update the mirror vertices
    // TODO vertify if this update is correct with Jeremie
    assert(mirror_vertices.size() == nminfos.size() - 1);
    // update the mirror vertices vertex_attrs if exists
    for (int i = 0; i < mirror_vertices.size(); i++) {
        wmtk::TriMesh::Tuple mirror_v = mirror_vertices[i];
        m.vertex_attrs[mirror_v.vid(m)].t = state.dofx(0);
        m.vertex_attrs[mirror_v.vid(m)].pos =
            m.mesh_parameters.m_boundary.t_to_uv(nminfos[i + 1].curve_id, state.dofx(0));
    }
    assert(m.invariants(one_ring_tris));
    cnt++;
    auto smooth_end_time = lagrange::get_timestamp();
    wmtk::logger().info(
        "smooth time: {}",
        lagrange::timestamp_diff_in_seconds(smooth_start_time, smooth_end_time));
    m.mesh_parameters.log(
        {{"smooth_op_" + std::to_string(cnt), {"smooth_time", std::to_string(lagrange::timestamp_diff_in_seconds(smooth_start_time, smooth_end_time))}}});
    return ret_data.success;
}


template <typename Executor>
void addCustomOps(Executor& e)
{
    e.add_operation(std::make_shared<AdaptiveTessellationSmoothVertexOperation>());
}

template <typename Executor>
void addSeamCustomOps(Executor& e)
{
    e.add_operation(std::make_shared<AdaptiveTessellationSmoothSeamVertexOperation>());
}

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

    for (auto i = 0; i < locs.size(); i++) {
        const auto& tri = locs[i];
        assert(!is_inverted(tri));
        auto local_tuples = oriented_tri_vertices(tri);
        for (size_t j = 0; j < 3; j++) {
            if (local_tuples[j].vid(*this) == vid) {
                const auto& v2 = vertex_attrs[local_tuples[(j + 1) % 3].vid(*this)].pos;
                const auto& v3 = vertex_attrs[local_tuples[(j + 2) % 3].vid(*this)].pos;
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
    const auto& old_pos = vertex_attrs[vid].pos;
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
        addSeamCustomOps(executor);
        // set_early_termination_number(mesh_parameters.m_early_stopping_number, executor);
        int itr = 0;
        do {
            mesh_parameters.m_gradient = Eigen::Vector2d(0., 0.);
            executor(*this, collect_all_ops);
            if (!mesh_parameters.m_do_not_output) {
                write_obj_with_texture_coords(
                    mesh_parameters.m_output_folder + fmt::format("/smooth_{:03d}.obj", itr));
            }
            itr++;
        } while ((mesh_parameters.m_gradient / vert_capacity()).stableNorm() > 1e-4 && itr < 10);
        wmtk::logger().info("===== terminate smooth after {} itrs", itr);
        time = timer.getElapsedTime();
        wmtk::logger().info("vertex smoothing operation time serial: {}s", time);
    }
}
