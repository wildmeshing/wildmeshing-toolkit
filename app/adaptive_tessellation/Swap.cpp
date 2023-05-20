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
        wmtk::logger().info("swap {}", cnt);

        if (!m.mesh_parameters.m_do_not_output) {
            m.write_obj_displaced(
                m.mesh_parameters.m_output_folder + fmt::format("/swap_{:04d}.obj", cnt));
        }

        cnt++;
        return true;
        // return  m.swap_before(t);
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

        // update the face_attrs (accuracy error)
        if (!m.mesh_parameters.m_ignore_embedding) {
            assert(bool(*this));
            // get a vector of new traingles uvs
            assert(mod_tups.size() == 2 || mod_tups.size() == 4);
            if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY) {
                std::vector<std::array<float, 6>> modified_tris_uv(mod_tups.size());
                for (int i = 0; i < mod_tups.size(); i++) {
                    auto tri = mod_tups[i];
                    auto verts = m.oriented_tri_vids(tri);
                    std::array<float, 6> tri_uv;
                    for (int i = 0; i < 3; i++) {
                        tri_uv[i * 2] = m.vertex_attrs[verts[i]].pos(0);
                        tri_uv[i * 2 + 1] = m.vertex_attrs[verts[i]].pos(1);
                    }
                    modified_tris_uv[i] = tri_uv;
                }
                std::vector<float> renewed_errors(mod_tups.size());
                m.m_texture_integral.get_error_per_triangle(modified_tris_uv, renewed_errors);
                m.set_faces_cached_distance_integral(mod_tups, renewed_errors);
            } else if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::TRI_QUADRICS) {
                std::vector<wmtk::Quadric<double>> compressed_quadrics(mod_tups.size());
                m.m_quadric_integral.get_quadric_per_triangle(
                    mod_tups.size(),
                    [&](int f) -> std::array<float, 6> {
                        // Get triangle uv positions
                        std::array<Tuple, 3> local_tuples = m.oriented_tri_vertices(mod_tups[f]);
                        const Eigen::Vector2f& p0 =
                            m.vertex_attrs[local_tuples[0].vid(m)].pos.cast<float>();
                        const Eigen::Vector2f& p1 =
                            m.vertex_attrs[local_tuples[1].vid(m)].pos.cast<float>();
                        const Eigen::Vector2f& p2 =
                            m.vertex_attrs[local_tuples[2].vid(m)].pos.cast<float>();
                        return {p0.x(), p0.y(), p1.x(), p1.y(), p2.x(), p2.y()};
                    },
                    compressed_quadrics);
                m.set_faces_quadrics(mod_tups, compressed_quadrics);
            }
        }
        return true;
    }
    return ret_data;
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

// used for quality pass
auto swap_valence_cost = [](auto& m, const TriMesh::Tuple& t) {
    std::vector<std::pair<TriMesh::Tuple, int>> valences(3);
    valences[0] = std::make_pair(t, m.get_valence_for_vertex(t));
    auto t2 = t.switch_vertex(m);
    valences[1] = std::make_pair(t2, m.get_valence_for_vertex(t2));
    auto t3 = (t.switch_edge(m)).switch_vertex(m);
    valences[2] = std::make_pair(t3, m.get_valence_for_vertex(t3));

    if ((t.switch_face(m)).has_value()) {
        auto t4 = (((t.switch_face(m)).value()).switch_edge(m)).switch_vertex(m);
        valences.emplace_back(t4, m.get_valence_for_vertex(t4));
    }
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

            std::vector<std::array<float, 6>> modified_tris(2);
            std::vector<float> compute_errors(2);
            // replace the vids with the swapped vids
            for (auto i = 0; i < 3; i++) {
                if (vids1[i] == e.switch_vertex(m).vid(m)) vids1[i] = v3;
                if (vids2[i] == e.vid(m)) vids2[i] = v4;
            }
            for (auto i = 0; i < 3; i++) {
                modified_tris[0][i * 2] = m.vertex_attrs[vids1[i]].pos(0);
                modified_tris[0][i * 2 + 1] = m.vertex_attrs[vids1[i]].pos(1);
                modified_tris[1][i * 2] = m.vertex_attrs[vids2[i]].pos(0);
                modified_tris[1][i * 2 + 1] = m.vertex_attrs[vids2[i]].pos(1);

                triangle1.row(i) = m.vertex_attrs[vids1[i]].pos;
                triangle2.row(i) = m.vertex_attrs[vids2[i]].pos;
            }

            m.m_texture_integral.get_error_per_triangle(modified_tris, compute_errors);

            if (polygon_signed_area(triangle1) <= 0 || polygon_signed_area(triangle2) <= 0) {
                return -std::numeric_limits<double>::infinity();
            } else {
                e_after = compute_errors[0] + compute_errors[1];
            }
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
            auto& [weight, op, tup] = ele;
            if (weight < 0) return false;
            double current_cost = 0.;
            current_cost = swap_accuracy_cost(m, tup);
            if (current_cost < 0.) return false;
            if (!is_close(current_cost, weight)) return false;
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
            if (m.is_boundary_edge(e)) return -std::numeric_limits<double>::infinity();
            if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::EDGE_ACCURACY ||
                m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY ||
                m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::TRI_QUADRICS) {
                auto current_error = swap_accuracy_cost(m, e);
                return current_error;
            }
            return -std::numeric_limits<double>::infinity();
        };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [weight, op, tup] = ele;
            if (weight < 0) return false;
            double current_cost = 0.;
            current_cost = swap_accuracy_cost(m, tup);
            if (current_cost < 0.) return false;
            if (!is_close(current_cost, weight)) return false;
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

            auto current_error = swap_valence_cost(m, e);
            return current_error;
        };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [weight, op, tup] = ele;
            if (weight < 0) return false;
            double current_cost = 0.;
            current_cost = swap_valence_cost(m, tup);
            if (current_cost < 0.) return false;
            if (!is_close(current_cost, weight)) return false;
            // check for the accuracy safegurads
            // simulate the swap and check if the accuracy is still within the threshold
            if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::EDGE_ACCURACY) {
                /////// TODO
            } else if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::AREA_ACCURACY) {
                /////// TODO
            } else if (m.mesh_parameters.m_edge_length_type == EDGE_LEN_TYPE::TRI_QUADRICS) {
                /////// TODO
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
auto AdaptiveTessellationSwapEdgeOperation::modified_triangles(const TriMesh& m) const -> std::vector<Tuple>
{
    //return TriMeshSwapEdgeOperation::modified_triangles(m);
    const auto& at  = static_cast<const AdaptiveTessellation&>(m);
    std::optional<Tuple> new_tuple_opt = get_return_tuple_opt();
    if (!new_tuple_opt.has_value()) {
        return {};
    }
    const Tuple& new_tuple = new_tuple_opt.value();
    assert(new_tuple.is_valid(at));
    std::optional<Tuple> new_other_face_opt = at.is_seam_edge(new_tuple) ? at.get_sibling_edge_opt(new_tuple) : m.switch_face(new_tuple);
    assert(new_other_face_opt);
    return {new_tuple, new_other_face_opt.value()};
}
