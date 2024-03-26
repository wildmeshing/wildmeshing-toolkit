#include <CLI/CLI.hpp>
#include <filesystem>
#include <iostream>


// wmtk
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/MeshReader.hpp>
using namespace wmtk;

// igl
#include <igl/opengl/glfw/Viewer.h>

int main(int argc, char** argv)
{
    using path = std::filesystem::path;

    CLI::App app{"bijective_map_app"};
    path initial_mesh_file;
    // TODO: implement tracking
    // path operation_logs_dir;
    // path output_mesh_file;

    app.add_option("-i, --input", initial_mesh_file, "Initial mesh file")->required(true);
    // app.add_option("-o, --output", output_mesh_file, "Output mesh file")->required(true);
    // app.add_option("-l, --logs", operation_logs_dir, "Operation logs directory")->required(true);

    CLI11_PARSE(app, argc, argv);

    if (!std::filesystem::exists(initial_mesh_file)) {
        std::cerr << "File `" << initial_mesh_file << "` does not exist." << std::endl;
        return EXIT_FAILURE;
    }

    auto init_mesh_ptr = wmtk::read_mesh(initial_mesh_file);
    auto [F0, V0] = static_cast<TriMesh&>(*init_mesh_ptr).get_FV();

    // viewer
    {
        igl::opengl::glfw::Viewer viewer;
        Eigen::Vector4f backColor;
        backColor << 208 / 255., 237 / 255., 227 / 255., 1.;
        const Eigen::RowVector3d blue(149.0 / 255, 217.0 / 255, 244.0 / 255);
        viewer.core().background_color = backColor;
        viewer.data().set_mesh(V0, F0);
        viewer.data().set_colors(blue);

        viewer.launch();
    }
    return 0;
}