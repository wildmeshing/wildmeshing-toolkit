#include "Swap.h"
#include "wmtk/ExecutionScheduler.hpp"

#include <Eigen/src/Core/util/Constants.h>
#include <igl/Timer.h>
#include <lagrange/utils/fpe.h>
#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/ScalarUtils.h>
#include <array>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TriQualityUtils.hpp>
#include <wmtk/utils/TupleUtils.hpp>

#include <limits>
#include <optional>
using namespace adaptive_tessellation;
using namespace wmtk;

AdaptiveTessellationSwapEdgeOperation::AdaptiveTessellationSwapEdgeOperation() = default;
AdaptiveTessellationSwapEdgeOperation::~AdaptiveTessellationSwapEdgeOperation() = default;
bool AdaptiveTessellation::simulate_swap_is_degenerate(
    const TriMesh::Tuple& e,
    std::array<std::array<float, 6>, 2>& modified_tris) const
{
    auto is_degenerate_2dcoordinates = [](auto triangle) {
        Eigen::Vector2d A = Eigen::Vector2d(triangle[0], triangle[1]);
        Eigen::Vector2d B = Eigen::Vector2d(triangle[2], triangle[3]);
        Eigen::Vector2d C = Eigen::Vector2d(triangle[4], triangle[5]);
        auto res = igl::predicates::orient2d(A, B, C);
        if (res != igl::predicates::Orientation::POSITIVE)
            return true;
        else
            return false;
    };
    TriMesh::Tuple other_face = e.switch_face(*this).value();
    size_t v1 = e.vid(*this);
    size_t v2 = e.switch_vertex(*this).vid(*this);
    size_t v4 = (e.switch_edge(*this)).switch_vertex(*this).vid(*this);
    size_t v3 = other_face.switch_edge(*this).switch_vertex(*this).vid(*this);
    // get oriented vids
    std::array<size_t, 3> vids1 = oriented_tri_vids(e);
    std::array<size_t, 3> vids2 = oriented_tri_vids(other_face);

    // replace the vids with the swapped vids
    for (auto i = 0; i < 3; i++) {
        if (vids1[i] == v2) vids1[i] = v3;
        if (vids2[i] == v1) vids2[i] = v4;
    }
    for (auto i = 0; i < 3; i++) {
        modified_tris[0][i * 2] = vertex_attrs[vids1[i]].pos(0);
        modified_tris[0][i * 2 + 1] = vertex_attrs[vids1[i]].pos(1);
        modified_tris[1][i * 2] = vertex_attrs[vids2[i]].pos(0);
        modified_tris[1][i * 2 + 1] = vertex_attrs[vids2[i]].pos(1);
    }

    if (is_degenerate_2dcoordinates(modified_tris[0]) ||
        is_degenerate_2dcoordinates(modified_tris[1])) {
        return true;
    }
    return false;
}
auto AdaptiveTessellationSwapEdgeOperation::execute(AdaptiveTessellation& m, const Tuple& t)
    -> ExecuteReturnData
{
    return wmtk::TriMeshSwapEdgeOperation::execute(m, t);
}
bool AdaptiveTessellationSwapEdgeOperation::before(AdaptiveTessellation& m, const Tuple& t)
{
    static std::atomic_int cnt = 0;
    assert(t.is_valid(m));
    assert(m.check_mesh_connectivity_validity());
    if (m.is_boundary_edge(t)) {
        return false;
    }
    // clean out old data
    vid_edge_to_mirror_edge.local().clear();
    auto& vids_to_mirror_edge = vid_edge_to_mirror_edge.local();
    std::vector<Tuple> incident_tri_tuples;
    incident_tri_tuples.emplace_back(t);
    if (t.switch_face(m).has_value()) incident_tri_tuples.emplace_back(t.switch_face(m).value());
    for (const auto& tri : incident_tri_tuples) {
        for (const Tuple& edge_tuple : m.triangle_boundary_edge_tuples(tri)) {
            if (m.is_boundary_edge(edge_tuple)) {
                // for seam edge get_sibling_edge_opt return the mirror edge as std::optional
                std::optional<Tuple> opt =
                    m.is_seam_edge(edge_tuple) ? m.get_sibling_edge_opt(edge_tuple) : std::nullopt;
                size_t vid0 = edge_tuple.vid(m);
                size_t vid1 = edge_tuple.switch_vertex(m).vid(m);
                vids_to_mirror_edge[std::array<size_t, 2>{{vid0, vid1}}] = {
                    opt,
                    m.edge_attrs[edge_tuple.eid(m)].curve_id.value()};
            }
        }
    }

    if (wmtk::TriMeshSwapEdgeOperation::before(m, t)) {
        // wmtk::logger().info("swap {}", cnt);

        // if (!m.mesh_parameters.m_do_not_output) {
        //     m.write_perface_vtk(
        //         m.mesh_parameters.m_output_folder + fmt::format("/swap_{:04d}.vtk", cnt));
        // }

        cnt++;
        return m.swap_edge_before(t);
    }
    return false;
}
bool AdaptiveTessellationSwapEdgeOperation::after(
    AdaptiveTessellation& m,
    ExecuteReturnData& ret_data)
{
    if (wmtk::TriMeshSwapEdgeOperation::after(m, ret_data)) {
        const auto mod_tups = modified_triangles(m);
        const auto& tri_con = tri_connectivity(m);

        // wipe out existing face attribute data beacuse it's invalidated
        for (const Tuple& tri : mod_tups) {
            m.face_attrs[tri.fid(m)] = {};
            for (const Tuple& edge : m.triangle_boundary_edge_tuples(tri)) {
                m.edge_attrs[edge.eid(m)] = {};
            }
        }

        auto find_edge_tup =
            [&tri_con, &mod_tups, &m](const size_t vid0, const size_t vid1) -> Tuple {
            // go through the (two) triangles to find the edge
            for (const Tuple& tri : mod_tups) {
                const size_t fid = tri.fid(m);
                const auto& con = tri_con[fid].m_indices;

                // go through the vertices in the triangle to find the right local_eid (j) if it
                // exists
                for (size_t j = 0; j < con.size(); ++j) {
                    size_t jp1 = (j + 1) % con.size();
                    size_t jp2 = (j + 2) % con.size();

                    // if we find the pair of vertices use the ordering to return a tuple
                    if (con[jp1] == vid0 && con[jp2] == vid1) {
                        return Tuple(vid0, j, fid, m);
                    } else if (con[jp1] == vid1 && con[jp2] == vid0) {
                        return Tuple(vid1, j, fid, m);
                    }
                }
            }
            assert(false); // if vid pairs were in the map then they must be on the boundary
                           // and still edges
            return {};
        };

        for (const auto& [vids, mirror_edge_curveid] : vid_edge_to_mirror_edge.local()) {
            // use vids to find an edge in the current triangle
            const auto [vid0, vid1] = vids;
            const Tuple edge_tup = find_edge_tup(vid0, vid1);
#if defined(_DEBUG)
            { // make sure that the edge tuple returned is appropriate
                assert(edge_tup.is_valid(m));
                const size_t vid = edge_tup.vid(m);
                assert(vid0 == vid || vid1 == vid);
                const size_t ovid = edge_tup.switch_vertex(m).vid(m);
                assert(vid0 == ovid || vid1 == ovid);
                assert(vid != ovid);
            }
#endif
            // only set mirror edge if what's stored is a valid edge
            if (mirror_edge_curveid.first.has_value()) {
                m.set_mirror_edge_data(edge_tup, mirror_edge_curveid.first.value());
                m.set_mirror_edge_data(mirror_edge_curveid.first.value(), edge_tup);
            }

            m.edge_attrs[edge_tup.eid(m)].curve_id = mirror_edge_curveid.second;
        }

        std::vector<std::array<float, 6>> modified_tris_uv(mod_tups.size());
        if (!m.update_energy_cache(mod_tups)) {
            ret_data.success = false;
            return false;
        }
        return ret_data;
    }
}


template <typename Executor>
void addCustomOps(Executor& e)
{
    e.add_operation(std::make_shared<AdaptiveTessellationSwapEdgeOperation>());
}


auto swap_renew = [](auto& m, auto op, auto& tris) {
    auto edges = m.new_edges_after(tris);
    auto optup = std::vector<std::pair<std::string, wmtk::TriMesh::Tuple>>();
    for (auto& e : edges) optup.emplace_back(op, e);
    return optup;
};
auto swap_no_renew = [](auto& m, auto op, auto& tris) {
    auto optup = std::vector<std::pair<std::string, wmtk::TriMesh::Tuple>>();
    return optup;
};

// used for quality pass
auto swap_valence_cost = [](auto& m, const TriMesh::Tuple& t) {
    std::vector<std::pair<TriMesh::Tuple, int>> valences(3);
    size_t v1 = t.vid(m);
    valences[0] = std::make_pair(t, m.get_valence_for_vertex(t));
    auto t2 = t.switch_vertex(m);
    size_t v2 = t2.vid(m);
    valences[1] = std::make_pair(t2, m.get_valence_for_vertex(t2));
    auto t3 = (t.switch_edge(m)).switch_vertex(m);

    valences[2] = std::make_pair(t3, m.get_valence_for_vertex(t3));

    assert(t.switch_face(m).has_value());
    auto other_face = t.switch_face(m).value();
    auto t4 = (other_face.switch_edge(m)).switch_vertex(m);
    valences.emplace_back(t4, m.get_valence_for_vertex(t4));

    size_t v3 = t4.vid(m);
    size_t v4 = t3.vid(m);


    double cost_before_swap = 0.0;
    double cost_after_swap = 0.0;

    // check if it's internal vertex or bondary vertex
    // navigating starting one edge and getting back to the start

    for (int i = 0; i < valences.size(); i++) {
        TriMesh::Tuple vert = valences[i].first;
        int val = 6;
        auto one_ring_edges = m.get_one_ring_edges_for_vertex(vert);
        for (auto edge : one_ring_edges) {
            if (m.is_boundary_edge(edge)) {
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
};

// used for accuracy pass
// list the triangle verteics in order
// return cost 0.for colinear or flipped triangle after swap (to save quadrature computation)
// acsii art
//          v4                                  v4
//         /\                                   /|\ 
//        /  \                                 / | \     
//       /    \                               /  |  \ 
//      / -->  \                             /   |   \ 
//  v1 /________\ v2                    v1  /    |    \ v2
//     \        /                           \    |    /
//      \      /                             \   |   /
//       \    /                               \  |  /
//        \  /                                 \ | /
//         \/                                   \|/
//         v3                                   v3

auto swap_accuracy_cost = [](auto& m, const TriMesh::Tuple& e) {
    double e_before = 0.0;
    auto e_after = e_before;
    if (!(e.switch_face(m)).has_value()) return -std::numeric_limits<double>::infinity();
    assert(e.switch_face(m).has_value());
    lagrange::enable_fpe();
    TriMesh::Tuple other_face = e.switch_face(m).value();
    size_t v1 = e.vid(m);
    size_t v2 = e.switch_vertex(m).vid(m);
    size_t v4 = (e.switch_edge(m)).switch_vertex(m).vid(m);
    size_t v3 = other_face.switch_edge(m).switch_vertex(m).vid(m);
    size_t other_vid = other_face.switch_edge(m).switch_vertex(m).vid(m);
    if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::EDGE_ACCURACY) {
        e_before = m.mesh_parameters.m_get_length(e);
        auto t3_pos = m.vertex_attrs[v3].pos;
        auto t4_pos = m.vertex_attrs[v4].pos;
        e_after = m.mesh_parameters.m_displacement->get_error_per_edge(t3_pos, t4_pos);
    } else {
        // get oriented vids
        std::array<size_t, 3> vids1 = m.oriented_tri_vids(e);
        std::array<size_t, 3> vids2 = m.oriented_tri_vids(other_face);

        Eigen::Matrix<double, 3, 2, Eigen::RowMajor> triangle1;
        Eigen::Matrix<double, 3, 2, Eigen::RowMajor> triangle2;
        if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY) {
            // using the cached error
            e_before = m.face_attrs[e.fid(m)].accuracy_measure.cached_distance_integral;
            e_before += m.face_attrs[other_face.fid(m)].accuracy_measure.cached_distance_integral;
            // wmtk::logger().info("//// before barrier");
            // wmtk::logger().info("   tri1 {} tri2 {}", e.fid(m), other_face.fid(m));
            // wmtk::logger().info("   before accuracy energy {}", e_before);

            // e_before += m.barrier_energy_per_face(
            //     m.vertex_attrs[vids1[0]].pos_world,
            //     m.vertex_attrs[vids1[1]].pos_world,
            //     m.vertex_attrs[vids1[2]].pos_world);
            // e_before += m.barrier_energy_per_face(
            //     m.vertex_attrs[vids2[0]].pos_world,
            //     m.vertex_attrs[vids2[1]].pos_world,
            //     m.vertex_attrs[vids2[2]].pos_world);
            // wmtk::logger().info("   before total energy {}", e_before);

            std::array<std::array<float, 6>, 2> modified_tris;
            std::vector<float> compute_errors(2);
            // replace the vids with the swapped vids
            for (auto i = 0; i < 3; i++) {
                if (vids1[i] == e.switch_vertex(m).vid(m)) vids1[i] = v3;
                if (vids2[i] == e.vid(m)) vids2[i] = v4;
            }
            wmtk::logger().info("//// after barrier ");

            // for (auto i = 0; i < 3; i++) {
            //     modified_tris[0][i * 2] = m.vertex_attrs[vids1[i]].pos(0);
            //     modified_tris[0][i * 2 + 1] = m.vertex_attrs[vids1[i]].pos(1);
            //     modified_tris[1][i * 2] = m.vertex_attrs[vids2[i]].pos(0);
            //     modified_tris[1][i * 2 + 1] = m.vertex_attrs[vids2[i]].pos(1);

            //     triangle1.row(i) = m.vertex_attrs[vids1[i]].pos;
            //     triangle2.row(i) = m.vertex_attrs[vids2[i]].pos;
            // }

            if (m.simulate_swap_is_degenerate(e, modified_tris)) {
                wmtk::logger().info("   !!!! in after colinear or flipped triangle");
                return -std::numeric_limits<double>::infinity();
            }
            std::vector<std::array<float, 6>> modified_tris_vector;
            modified_tris_vector.emplace_back(modified_tris[0]);
            modified_tris_vector.emplace_back(modified_tris[1]);
            m.m_texture_integral.get_error_per_triangle(modified_tris_vector, compute_errors);

            e_after = compute_errors[0] + compute_errors[1];
            // wmtk::logger().info("   after accuracy energy {}", e_after);
            // // barrier term

            // e_after += m.barrier_energy_per_face(
            //     m.vertex_attrs[vids1[0]].pos_world,
            //     m.vertex_attrs[vids1[1]].pos_world,
            //     m.vertex_attrs[vids1[2]].pos_world);
            // e_after += m.barrier_energy_per_face(
            //     m.vertex_attrs[vids2[0]].pos_world,
            //     m.vertex_attrs[vids2[1]].pos_world,
            //     m.vertex_attrs[vids2[2]].pos_world);
            // wmtk::logger().info("   after total energy {}", e_after);
        }
        if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::TRI_QUADRICS) {
            //////// TODO
        }
    }
    // positive if error decreases
    return e_before - e_after;
};


void AdaptiveTessellation::swap_all_edges()
{
    throw std::runtime_error("swap_all_edges is archived use swap_all_edges_accuracy_pass or "
                             "swap_all_edges_quality_pass instead");
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    auto collect_tuples = tbb::concurrent_vector<Tuple>();

    for_each_edge([&](auto& tup) { collect_tuples.emplace_back(tup); });
    collect_all_ops.reserve(collect_tuples.size());
    for (auto& t : collect_tuples) collect_all_ops.emplace_back("edge_swap", t);
    wmtk::logger().info("=======swap==========");

    wmtk::logger().info("size for edges to be swap is {}", collect_all_ops.size());
    auto setup_and_execute = [&](auto executor) {
        addCustomOps(executor);
        executor.renew_neighbor_tuples = swap_renew;
        executor.priority = [&](auto& m, [[maybe_unused]] auto _, auto& e) {
            if (m.is_boundary_edge(e))
                return -std::numeric_limits<double>::infinity(); // boundary edge shouldn't swap
            double valence_cost = swap_valence_cost(m, e);
            if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::EDGE_ACCURACY ||
                m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY) {
                auto current_error = swap_accuracy_cost(m, e);
                return current_error;
            }
            return valence_cost;
        };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [weight, op, e] = ele;
            if (weight < 0) return false;
            double current_cost = 0.;
            current_cost = swap_accuracy_cost(m, e);
            if (current_cost < 0.) return false;
            if (!is_close(current_cost, weight)) return false;
            if (m.is_boundary_edge(e)) return false;
            return true;
        };
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<AdaptiveTessellation, ExecutionPolicy::kPartition>();

        executor.lock_vertices = [](auto& m, const auto& e, int task_id) {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<AdaptiveTessellation, ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }
}

// only EDGE_ACCURACY, AREA_ACCURACY, TRI_QUADRICS are supported
void AdaptiveTessellation::swap_all_edges_accuracy_pass()
{
    // cache all the 3d pos since swap doesn't change 3d pos
    // for (auto& v : get_vertices()) {
    //     auto p = vertex_attrs[v.vid(*this)].pos;
    //     vertex_attrs[v.vid(*this)].pos_world = mesh_parameters.m_displacement->get(p(0),
    //     p(1));
    // }
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    auto collect_tuples = tbb::concurrent_vector<Tuple>();

    for_each_edge([&](auto& tup) { collect_tuples.emplace_back(tup); });
    collect_all_ops.reserve(collect_tuples.size());
    for (auto& t : collect_tuples) collect_all_ops.emplace_back("edge_swap", t);
    wmtk::logger().info("=======swap==========");

    wmtk::logger().info("size for edges to be swap is {}", collect_all_ops.size());
    auto setup_and_execute = [&](auto executor) {
        addCustomOps(executor);
        executor.renew_neighbor_tuples = swap_renew;
        executor.priority = [&](auto& m, [[maybe_unused]] auto _, auto& e) {
            // boundary edge shouldn't swap
            // but it shouldn't get here. just a sfaety guard
            if (m.is_boundary_edge(e)) {
                return -std::numeric_limits<double>::infinity();
            }
            if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::EDGE_ACCURACY ||
                m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY ||
                m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::TRI_QUADRICS) {
                return swap_accuracy_cost(m, e);
            } else {
                throw std::runtime_error("unsupported edge length type in swap_accuracy_pass");
                return -std::numeric_limits<double>::infinity();
            }
        };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [weight, op, e] = ele;
            if (weight <= 0) return false;
            double accuracy_cost = swap_accuracy_cost(m, e);
            if (accuracy_cost <= 0.) return false;
            if (!is_close(accuracy_cost, weight)) return false;
            std::array<std::array<float, 6>, 2> modified_tris;
            if (m.simulate_swap_is_degenerate(e, modified_tris)) return false;
            return true;
        };
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<AdaptiveTessellation, ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<AdaptiveTessellation, ExecutionPolicy::kSeq>();
        // used for debugging for early termination
        set_early_termination_number(mesh_parameters.m_early_stopping_number, executor);
        setup_and_execute(executor);
    }
}

void AdaptiveTessellation::swap_all_edges_quality_pass()
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    auto collect_tuples = tbb::concurrent_vector<Tuple>();

    // cache all the 3d pos since swap doesn't change 3d pos
    for (auto& v : get_vertices()) {
        auto p = vertex_attrs[v.vid(*this)].pos;
        vertex_attrs[v.vid(*this)].pos_world = mesh_parameters.m_displacement->get(p(0), p(1));
    }
    for_each_edge([&](auto& tup) { collect_tuples.emplace_back(tup); });
    collect_all_ops.reserve(collect_tuples.size());
    for (auto& t : collect_tuples) collect_all_ops.emplace_back("edge_swap", t);
    wmtk::logger().info("=======swap==========");

    wmtk::logger().info("size for edges to be swap is {}", collect_all_ops.size());
    auto setup_and_execute = [&](auto executor) {
        addCustomOps(executor);
        executor.renew_neighbor_tuples = swap_renew;
        executor.priority = [&](auto& m, [[maybe_unused]] auto _, auto& e) {
            if (m.is_boundary_edge(e))
                return -std::numeric_limits<double>::infinity(); // boundary edge shouldn't swap
            return swap_valence_cost(m, e);
        };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [weight, op, e] = ele;
            if (weight <= 0) return false;
            double valence_cost = swap_valence_cost(m, e);
            if (valence_cost <= 0.) return false;
            if (!is_close(valence_cost, weight)) return false;


            if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY) {
            } else if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::EDGE_ACCURACY) {
                throw std::runtime_error("EDGE_ACCURACY is not supported for swap quality pass");
                /////// TODO
            } else if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::TRI_QUADRICS) {
                /////// TODO
                throw std::runtime_error("TRI_QUADRICS is not supported for swap quality pass");
            }
            return true;
        };
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<AdaptiveTessellation, ExecutionPolicy::kPartition>();
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<AdaptiveTessellation, ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }
}
bool AdaptiveTessellation::swap_edge_before(const Tuple& t)
{
    if (is_boundary_edge(t)) return false;
    if (is_boundary_vertex(t))
        vertex_attrs[cache.local().v1].boundary_vertex = true;
    else
        vertex_attrs[cache.local().v1].boundary_vertex = false;
    if (is_boundary_vertex(t.switch_vertex(*this)))
        vertex_attrs[cache.local().v2].boundary_vertex = true;
    else
        vertex_attrs[cache.local().v2].boundary_vertex = false;
    // // get max_energy
    // cache.local().max_energy = -1;
    // auto tris = get_one_ring_tris_for_vertex(t);
    // for (auto tri : tris) {
    //     assert(get_quality(tri) > 0);
    //     cache.local().max_energy = std::max(cache.local().max_energy, get_quality(tri));
    // }
    // m_max_energy = cache.local().max_energy;
    return true;
}
bool AdaptiveTessellation::swap_edge_after([[maybe_unused]] const Tuple& t)
{
    // check quality and degenerate
    // auto tris = get_one_ring_tris_for_vertex(t);

    // for (auto tri : tris) {
    //     // if (get_quality(tri) >= cache.local().max_energy) return false; // this is commented out for energy check. Only check valence for now
    //     if (get_quality(tri) < 0) return false; // reject operations that cause triangle inversion
    // }
    return true;
}
auto AdaptiveTessellationSwapEdgeOperation::modified_triangles(const TriMesh& m) const
    -> std::vector<Tuple>
{
    // return TriMeshSwapEdgeOperation::modified_triangles(m);
    const auto& at = static_cast<const AdaptiveTessellation&>(m);
    std::optional<Tuple> new_tuple_opt = get_return_tuple_opt();
    if (!new_tuple_opt.has_value()) {
        return {};
    }
    const Tuple& new_tuple = new_tuple_opt.value();
    assert(new_tuple.is_valid(at));
    std::optional<Tuple> new_other_face_opt =
        at.is_seam_edge(new_tuple) ? at.get_sibling_edge_opt(new_tuple) : m.switch_face(new_tuple);
    assert(new_other_face_opt);
    return {new_tuple, new_other_face_opt.value()};
}
