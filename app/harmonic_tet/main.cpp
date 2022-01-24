#include "HarmonicTet.hpp"

#include <wmtk/utils/Delaunay.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/io.hpp>
#include "wmtk/utils/EnergyHarmonicTet.hpp"

// Third-party include

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <CLI/CLI.hpp>
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
                T[i * 3 + j] = har_tet.vertex_attrs->m_attributes[v].pos[j];
            }
        }
        auto e = wmtk::harmonic_tet_energy(T);
        total_e += e;
        cnt++;
    }
    wmtk::logger().info("Total E {}, Cnt {}", total_e, cnt);
    return std::pair(total_e, cnt);
};

auto process_mesh = [](auto& args) {
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
    auto har_tet = harmonic_tet::HarmonicTet(vec_attrs, tets, thread);
    for (int i = 0; i <= 4; i++) {
        auto [E0, cnt0] = stats(har_tet);
        har_tet.swap_all();
        stats(har_tet);
        har_tet.consolidate_mesh();
        har_tet.smooth_all_vertices();
        auto [E1, cnt1] = stats(har_tet);
    }
    har_tet.output_mesh(output);
};

int main(int argc, char** argv)
{
    CLI::App app{argv[0]};
    app.add_option("input", args.input, "Input mesh.");
    app.add_option("output", args.output, "output mesh.");
    app.add_option("--thread", args.thread, "thread.");
    CLI11_PARSE(app, argc, argv);

    process_mesh(args);
    return 0;
}