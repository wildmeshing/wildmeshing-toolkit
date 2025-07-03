#include "HarmonicTet.hpp"

#include <wmtk/utils/Delaunay.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/io.hpp>
#include "wmtk/utils/Delaunay.hpp"
#include "wmtk/utils/EnergyHarmonicTet.hpp"
#include "wmtk/utils/Logger.hpp"

// Third-party include

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <CLI/CLI.hpp>
#include <igl/Timer.h>
#include <igl/doublearea.h>
#include <igl/read_triangle_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on


struct
{
    std::string input;
    std::string output;
    int thread = 1;
} args;

auto stats = [](auto& har_tet) {
    auto total_e = 0.;
    auto cnt = 0;
    for (auto t : har_tet.get_tets()) {
        auto local_tuples = har_tet.oriented_tet_vertices(t);
        std::array<size_t, 4> local_verts;
        auto T = std::array<double, 12>();
        for (auto i = 0; i < 4; i++) {
            auto v = local_tuples[i].vid(har_tet);
            for (auto j = 0; j < 3; j++) {
                T[i * 3 + j] = har_tet.vertex_attrs[v].pos[j];
            }
        }
        auto e = wmtk::harmonic_tet_energy(T);
        total_e += e;
        cnt++;
    }
    total_e *= 6;
    wmtk::logger().info("Total E {}, Cnt {}, Avg {}", total_e, cnt, total_e / cnt);
    return std::pair(total_e, cnt);
};

auto process_mesh = [&args = args]() {
    auto& input = args.input;
    auto& output = args.output;
    auto& thread = args.thread;
    auto vec_attrs = std::vector<Eigen::Vector3d>();
    auto tets = std::vector<std::array<size_t, 4>>();
    wmtk::MshData msh;
    msh.load(input);
    vec_attrs.resize(msh.get_num_tet_vertices());
    tets.resize(msh.get_num_tets());
    msh.extract_tet_vertices(
        [&](size_t i, double x, double y, double z) { vec_attrs[i] << x, y, z; });
    msh.extract_tets([&](size_t i, size_t v0, size_t v1, size_t v2, size_t v3) {
        tets[i] = {{v0, v1, v2, v3}};
    });
    igl::Timer timer;
    auto time = 0.;
    timer.start();
    auto har_tet = harmonic_tet::HarmonicTet(vec_attrs, tets, thread);
    time += timer.getElapsedTimeInMilliSec();
    for (int i = 0; i <= 10; i++) {
        auto [E0, cnt0] = stats(har_tet);
        timer.start();
        auto swp = har_tet.swap_all();
        time += timer.getElapsedTimeInMilliSec();
        stats(har_tet);
        timer.start();
        har_tet.consolidate_mesh();
        har_tet.smooth_all_vertices(true);
        time += timer.getElapsedTimeInMilliSec();
        auto [E1, cnt1] = stats(har_tet);
        if (swp == 0) break;
    }
    wmtk::logger().info("Time cost {}s", time / 1e3);
    har_tet.output_mesh(output);
};

auto process_points = [&args = args]() {
    auto& input = args.input;
    auto& output = args.output;
    auto& thread = args.thread;
    auto vec_attrs = std::vector<Eigen::Vector3d>();
    auto tets = std::vector<std::array<size_t, 4>>();
    {
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
        igl::read_triangle_mesh(input, V, F);
        auto SV = V;
        Eigen::VectorXi SVI, SVJ;
        igl::remove_duplicate_vertices(SV, 1e-3, V, SVI, SVJ);

        std::vector<std::array<double, 3>> points(V.rows());
        for (auto i = 0; i < V.rows(); i++) points[i] = {{V(i, 0), V(i, 1), V(i, 2)}};
        auto [tet_V, tetT] = wmtk::delaunay::delaunay3D(points);

        vec_attrs.resize(tet_V.size());
        for (auto i = 0; i < tet_V.size(); i++) {
            for (auto j = 0; j < 3; j++) vec_attrs[i][j] = tet_V[i][j];
        }
        tets = tetT;
    }

    igl::Timer timer;
    auto time = 0.;
    timer.start();
    auto har_tet = harmonic_tet::HarmonicTet(vec_attrs, tets, thread);
    // auto [E0, cnt0] = stats(har_tet);
    har_tet.swap_all_edges(true);
    time = timer.getElapsedTimeInMilliSec();
    wmtk::logger().info("Time cost: {}", time / 1e3);
    stats(har_tet);
    har_tet.consolidate_mesh();
    // auto [E1, cnt1] = stats(har_tet);
    // wmtk::logger().info("E {} -> {} cnt {} -> {}", E0, E1, cnt0, cnt1);
    har_tet.output_mesh(output);
};

int main(int argc, char** argv)
{
    CLI::App app{argv[0]};
    auto harmonize = true;
    app.add_option("input", args.input, "Input mesh.");
    app.add_option("output", args.output, "output mesh.");
    app.add_option("-j, --thread", args.thread, "thread.");
    app.add_flag("--harmonize", harmonize, "Delaunay harmonize.");
    CLI11_PARSE(app, argc, argv);

    if (harmonize)
        process_points();
    else
        process_mesh();
    return 0;
}
