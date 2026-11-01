// The tracking tutorial (docs/tutorials/tracking.md), in C++: coarsen the dense bunny with
// isotropic remeshing in memory, record its history, build the map from it, and map points both
// ways. python/tracking_correspondences.py does the same, then carries the map on through tetwild.

#include <wmtk/components/isotropic_remeshing/isotropic_remeshing.hpp>
#include <wmtk/components/tracking/SurfaceMap.hpp>
#include <wmtk/history/History.hpp>
#include <wmtk/io/mesh_io.hpp>

#include <cstdlib>
#include <filesystem>
#include <iostream>

int main()
{
    // The example meshes: $WMTK_DATA, or data/models from the repository root.
    const char* env = std::getenv("WMTK_DATA");
    const std::filesystem::path data = env ? env : "data/models";

    // --8<-- [start:record]
    wmtk::MeshMap meshes{{"dense", wmtk::io::read_mesh(data / "bunny.obj")}};
    auto [outputs, report] = wmtk::components::isotropic_remeshing::isotropic_remeshing(
        {{"input", "dense"},
         {"eps_rel", 2e-2},
         {"length_rel", 0.045},
         {"max_iterations", 5},
         {"num_threads", 8},
         // On the memory route any non-empty value records the history; it comes back in the
         // report as binary data.
         {"history_output", "history"}},
        meshes);
    // --8<-- [end:record]

    // --8<-- [start:map]
    using wmtk::components::tracking::SurfaceMap;
    using wmtk::components::tracking::SurfacePoint;
    const SurfaceMap map(wmtk::history::SurfaceHistory::from_bytes(report["history"].get_binary()));
    std::cout << "map warnings: " << map.build_warnings().total() << "\n";

    // A point is a face and barycentric coordinates in it: here, the centroid of input face 0,
    // mapped onto the output and back.
    const std::vector<SurfacePoint> points = {{0, Eigen::Vector3d::Constant(1.0 / 3.0)}};
    const auto on_output = map.to_output(points);
    const auto back = map.to_input(on_output);
    std::cout << "input face 0 -> output face " << on_output[0].face << " -> input face "
              << back[0].face << "\n";
    // --8<-- [end:map]
}
