#include <interior_tet_opt/Mesh.hpp>

#include <wmtk/utils/Delaunay.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/io.hpp>
#include "wmtk/utils/Delaunay.hpp"
#include "wmtk/utils/EnergyHarmonicTet.hpp"
#include "wmtk/utils/Logger.hpp"
#include "wmtk/utils/io.hpp"
// Third-party include

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
#include <CLI/CLI.hpp>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <igl/Timer.h>
#include <igl/doublearea.h>
#include <igl/readMESH.h>
#include <igl/read_triangle_mesh.h>

#include <geogram/points/kd_tree.h>

struct
{
    std::string input;
    std::string output;
    int thread = 1;
    int max_iters = 10;
    double stop_energy = 10.;
} args;

// TODO: this should not be in the main
// this also seems unused, can we remove?
bool adjust_sizing_field(
    app::interior_tet_opt::InteriorTetOpt& mesh,
    double max_energy,
    double stop_energy)
{
    using Eigen::Vector3d;

    const double stop_filter_energy = stop_energy * 0.8;
    double filter_energy = std::max(max_energy / 100, stop_filter_energy);
    filter_energy = std::min(filter_energy, 100.);

    const auto recover_scalar = 1.5;
    const auto refine_scalar = 0.5;
    const auto min_refine_scalar = 0.25;

    // outputs scale_multipliers
    tbb::concurrent_vector<double> scale_multipliers(mesh.vert_capacity(), recover_scalar);

    std::vector<Vector3d> pts;
    std::queue<size_t> v_queue;
    mesh.TetMesh::for_each_tetra([&](auto& t) {
        auto tid = t.tid(mesh);
        if (mesh.m_tet_attribute[tid].quality < filter_energy) return;
        auto vs = mesh.oriented_tet_vids(t);
        Vector3d c(0, 0, 0);
        for (int j = 0; j < 4; j++) {
            c += (mesh.m_vertex_attribute[vs[j]].pos);
            v_queue.emplace(vs[j]);
        }
        pts.emplace_back(c / 4);
    });

    wmtk::logger().info("filter energy {} Low Quality Tets {}", filter_energy, pts.size());

    const double R = mesh.target_l * 2;

    int sum = 0;
    int adjcnt = 0;

    std::vector<bool> visited(mesh.vert_capacity(), false);

    GEO::NearestNeighborSearch_var nnsearch = GEO::NearestNeighborSearch::create(3, "BNN");
    nnsearch->set_points(pts.size(), pts[0].data());

    std::vector<size_t> cache_one_ring;
    while (!v_queue.empty()) {
        sum++;
        size_t vid = v_queue.front();
        v_queue.pop();
        if (visited[vid]) continue;
        visited[vid] = true;
        adjcnt++;

        auto& pos_v = mesh.m_vertex_attribute[vid].pos;
        auto sq_dist = 0.;
        GEO::index_t _1;
        nnsearch->get_nearest_neighbors(1, pos_v.data(), &_1, &sq_dist);
        auto dist = std::sqrt(std::max(sq_dist, 0.)); // compute dist(pts, pos_v);

        if (dist > R) { // outside R-ball, unmark.
            continue;
        }

        scale_multipliers[vid] =
            std::min(scale_multipliers[vid], (1 + dist / R) * refine_scalar); // linear interpolate

        auto vids = mesh.get_one_ring_vids_for_vertex_adj(vid, cache_one_ring);
        for (size_t n_vid : vids) {
            if (visited[n_vid]) continue;
            v_queue.push(n_vid);
        }
    }

    std::atomic_bool is_hit_min_edge_length = false;
    mesh.for_each_vertex([&](auto& v) {
        auto vid = v.vid(mesh);
        auto& v_attr = mesh.m_vertex_attribute[vid];

        auto new_scale = v_attr.m_sizing_scalar * scale_multipliers[vid];
        if (new_scale > 1)
            v_attr.m_sizing_scalar = 1;
        else if (new_scale < min_refine_scalar) {
            is_hit_min_edge_length = true;
            v_attr.m_sizing_scalar = min_refine_scalar;
        } else
            v_attr.m_sizing_scalar = new_scale;
    });

    return is_hit_min_edge_length.load();
}

#include <igl/Timer.h>
std::tuple<double, double> local_operations(
    app::interior_tet_opt::InteriorTetOpt& mesh,
    const std::array<int, 4>& ops,
    bool collapse_limit_length = true)
{
    igl::Timer timer;

    std::tuple<double, double> energy;
    energy = mesh.get_max_avg_energy();
    wmtk::logger().info("Energy: max = {} avg = {}", std::get<0>(energy), std::get<1>(energy));

    if (!mesh.invariants(mesh.get_tets())) {
        wmtk::logger().critical("Already Violating Invariants!!!");
    }

    for (int i = 0; i < ops.size(); i++) {
        timer.start();
        if (i == 0) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==splitting {}==", n);
                mesh.split_all_edges();
            }
        } else if (i == 1) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==collapsing {}==", n);
                mesh.collapse_all_edges();
            }
        } else if (i == 2) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==swapping {}==", n);
                mesh.swap_all_edges_44();
                mesh.swap_all_edges();
                mesh.swap_all_faces();
            }
        } else if (i == 3) {
            for (int n = 0; n < ops[i]; n++) {
                wmtk::logger().info("==smoothing {}==", n);
                mesh.smooth_all_vertices();
            }
        }

        energy = mesh.get_max_avg_energy();
        wmtk::logger().info("Energy: max = {} avg = {}", std::get<0>(energy), std::get<1>(energy));
    }


    energy = mesh.get_max_avg_energy();
    wmtk::logger().info("Energy: max = {} avg = {}", std::get<0>(energy), std::get<1>(energy));
    wmtk::logger().info("time = {}", timer.getElapsedTime());

    return energy;
}

void mesh_improvement(app::interior_tet_opt::InteriorTetOpt& mesh, int max_its, double stop_energy)
{
    ////preprocessing

    ////operation loops
    local_operations(mesh, {{0, 0, 0, 1}});
    const int M = 2;
    int m = 0;
    double pre_max_energy = 0., pre_avg_energy = 0.;
    for (int it = 0; it < max_its; it++) {
        ///ops
        wmtk::logger().info("\n========it {}========", it);
        auto [max_energy, avg_energy] = local_operations(mesh, {{1, 1, 1, 1}});
        mesh.consolidate_mesh();

        ///energy check
        wmtk::logger().info("max energy {} stop {}", max_energy, stop_energy);
        wmtk::logger().info("vert {} tet {}", mesh.vert_capacity(), mesh.tet_capacity());
        if (max_energy < stop_energy) break;

        ///sizing field
        if (it > 0 &&
            ((pre_max_energy - max_energy) / max_energy < 1e-1 ||
             pre_max_energy - max_energy < 1e-2) &&
            ((pre_avg_energy - avg_energy) / avg_energy < 1e-1 ||
             pre_avg_energy - avg_energy < 1e-2)) {
            m++;
            if (m == M) {
                wmtk::logger().info(">>>>adjust_sizing_field...");
                // auto is_hit_min_edge_length = adjust_sizing_field(mesh, max_energy, stop_energy);
                wmtk::logger().info(">>>>adjust_sizing_field finished...");
                m = 0;
            }
        } else
            m = 0;
        pre_max_energy = max_energy;
        pre_avg_energy = avg_energy;
    }
}

int main(int argc, char** argv)
{
    CLI::App app{argv[0]};
    auto harmonize = true;
    app.add_option("input", args.input, "Input mesh.");
    app.add_option("output", args.output, "output mesh.");
    app.add_option("-j, --thread", args.thread, "thread.");
    app.add_option("-i, --max-iters", args.max_iters, "maximum number of iterations.");
    app.add_option("-e, --stop-energy", args.stop_energy, "maximum number of iterations.");
    CLI11_PARSE(app, argc, argv);

    //

    auto vec_attrs = std::vector<Eigen::Vector3d>();
    auto tets = std::vector<std::array<size_t, 4>>();


    {
        wmtk::MshData msh;
        msh.load(args.input);
        auto setter = [&](size_t k, double x, double y, double z) { vec_attrs[k] << x, y, z; };
        auto set_tet = [&](size_t k, size_t v0, size_t v1, size_t v2, size_t v3) {
            tets[k] = {{size_t(v0), size_t(v1), size_t(v2), size_t(v3)}};
        };
        auto n = msh.get_num_tet_vertices();
        vec_attrs.resize(n);
        tets.resize(msh.get_num_tets());
        msh.extract_tet_vertices(setter);
        msh.extract_tets(set_tet);
    }

    wmtk::logger().info("Loaded v: {} t: {}", vec_attrs.size(), tets.size());
    if (tets.size() == 0) {
        wmtk::logger().critical("Empty!");
        return -1;
    }

    app::interior_tet_opt::InteriorTetOpt mesh;
    mesh.initialize(vec_attrs, tets);
    mesh_improvement(mesh, args.max_iters, args.stop_energy);
    mesh.final_output_mesh(args.output);
    return 0;
}
