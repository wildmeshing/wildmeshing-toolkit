#include <InteriorTetOpt/Mesh.hpp>

#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>
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
#include <CLI/CLI.hpp>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <igl/Timer.h>
#include <igl/doublearea.h>
#include <igl/readMESH.h>
#include <igl/read_triangle_mesh.h>

struct
{
    std::string input;
    std::string output;
    int thread = 1;
} args;


bool adjust_sizing_field(
    interior_tetopt::InteriorTetOpt& mesh,
    double max_energy,
    double stop_energy)
{
    using Scalar = double;

    const auto& vertices = mesh.get_vertices();
    const auto& tets = mesh.get_tets(); // todo: avoid copy!!!

    static const Scalar stop_filter_energy = stop_energy * 0.8;
    Scalar filter_energy =
        max_energy / 100 > stop_filter_energy ? max_energy / 100 : stop_filter_energy;
    if (filter_energy > 100) filter_energy = 100;

    Scalar recover_scalar = 1.5;
    //    std::vector<Scalar> scale_multipliers(vertices.size(), recover_scalar);
    tbb::concurrent_vector<Scalar> scale_multipliers(
        mesh.m_vertex_attribute.size(),
        recover_scalar);
    Scalar refine_scalar = 0.5;
    Scalar min_refine_scalar = 5e-2;


    tbb::task_arena arena(mesh.NUM_THREADS);

    arena.execute([&] {
        tbb::parallel_for(tbb::blocked_range<int>(0, tets.size()), [&](tbb::blocked_range<int> r) {
            // for (size_t i = 0; i < tets.size(); i++) {
            for (int i = r.begin(); i < r.end(); i++) {
                int tid = tets[i].tid(mesh);
                if (mesh.m_tet_attribute[tid].quality < filter_energy) continue;

                auto vs = mesh.oriented_tet_vertices(tets[i]);

                double sizing_ratio = 0;
                for (int j = 0; j < 4; j++) {
                    sizing_ratio += mesh.m_vertex_attribute[vs[j].vid(mesh)].m_sizing_scalar;
                }
                sizing_ratio /= 4;
                double R = mesh.target_l * 2; // * sizing_ratio;

                std::unordered_map<size_t, double> new_scalars;
                std::vector<bool> visited(mesh.m_vertex_attribute.size(), false);
                //

                // ZoneScopedN("adj_pushqueue");
                std::queue<size_t> v_queue;
                Eigen::Vector3d c(0, 0, 0);
                for (int j = 0; j < 4; j++) {
                    v_queue.push(vs[j].vid(mesh));
                    c += mesh.m_vertex_attribute[vs[j].vid(mesh)].pos;
                    new_scalars[vs[j].vid(mesh)] = 0;
                }
                c /= 4;
                //

                // ZoneScopedN("adj_whileloop");
                int sum = 0;
                int adjcnt = 0;
                while (!v_queue.empty()) {
                    sum++;
                    size_t vid = v_queue.front();
                    v_queue.pop();
                    if (visited[vid]) continue;
                    visited[vid] = true;
                    adjcnt++;

                    bool is_close = false;
                    double dist = (mesh.m_vertex_attribute[vid].pos - c).norm();
                    if (dist > R) {
                        new_scalars[vid] = 0;
                    } else {
                        new_scalars[vid] = (1 + dist / R) * refine_scalar; // linear interpolate
                        is_close = true;
                    }

                    if (!is_close) continue;

                    auto vids =
                        mesh.get_one_ring_vids_for_vertex_adj(vid, mesh.get_one_ring_cache.local());
                    for (size_t n_vid : vids) {
                        if (visited[n_vid]) continue;
                        v_queue.push(n_vid);
                    }
                }


                for (auto& info : new_scalars) {
                    if (info.second == 0) continue;

                    size_t vid = info.first;
                    double scalar = info.second;
                    if (scalar < scale_multipliers[vid]) scale_multipliers[vid] = scalar;
                }
            }
        });
    });

    bool is_hit_min_edge_length = false;
    for (size_t i = 0; i < vertices.size(); i++) {
        size_t vid = vertices[i].vid(mesh);
        auto& v_attr = mesh.m_vertex_attribute[vid];

        Scalar new_scale = v_attr.m_sizing_scalar * scale_multipliers[vid];
        if (new_scale > 1)
            v_attr.m_sizing_scalar = 1;
        else if (new_scale < min_refine_scalar) {
            is_hit_min_edge_length = true;
            v_attr.m_sizing_scalar = min_refine_scalar;
        } else
            v_attr.m_sizing_scalar = new_scale;
    }

    return is_hit_min_edge_length;
}

#include <igl/Timer.h>
std::tuple<double, double> local_operations(
    interior_tetopt::InteriorTetOpt& mesh,
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
};

void mesh_improvement(interior_tetopt::InteriorTetOpt& mesh, int max_its, double stop_energy)
{
    ////preprocessing
    // TODO: refactor to eliminate repeated partition.
    std::cout << "-----print partition size-----" << std::endl;

    ////operation loops
    local_operations(mesh, {{0, 0, 0, 1}});
    const int M = 2;
    int m = 0;
    double pre_max_energy = 0., pre_avg_energy = 0.;
    for (int it = 0; it < max_its; it++) {
        ///ops
        wmtk::logger().info("\n========it {}========", it);
        auto [max_energy, avg_energy] = local_operations(mesh, {{1, 1, 1, 1}});

        ///energy check
        wmtk::logger().info("max energy {} stop {}", max_energy, stop_energy);
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
                auto is_hit_min_edge_length = adjust_sizing_field(mesh, max_energy, stop_energy);
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

    interior_tetopt::InteriorTetOpt mesh;
    mesh.target_l = 5e-1;
    mesh.initialize(vec_attrs, tets);
    mesh_improvement(mesh, 20, 1e3);
    mesh.output_mesh(args.output);
    return 0;
}
