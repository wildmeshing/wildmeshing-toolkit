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
using path = std::filesystem::path;

struct query_point
{
    int64_t f_id; // face id
    Eigen::Vector3d bc; // barycentric coordinates
    Eigen::Vector3i fv_ids; // face vertex ids
};

// TODO: for testing purpose
void back_track_map(path dirPath
                    // const std::vector<query_point>& query_points
)
{
    namespace fs = std::filesystem;
    int maxIndex = -1;

    for (const auto& entry : fs::directory_iterator(dirPath)) {
        if (entry.path().filename().string().find("operation_log_") != std::string::npos) {
            ++maxIndex;
        }
    }

    for (int i = maxIndex; i >= 0; --i) {
        fs::path filePath = dirPath / ("operation_log_" + std::to_string(i) + ".txt");
        std::ifstream file(filePath);
        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filePath << std::endl;
            continue;
        }

        std::cout << "Processing file: " << filePath << std::endl;
        std::string line;
        while (std::getline(file, line)) {
            std::cout << line << std::endl;
            break;
        }
        file.close();
    }
}


int main(int argc, char** argv)
{
    CLI::App app{"bijective_map_app"};
    path initial_mesh_file;
    path operation_logs_dir;
    path output_mesh_file;

    app.add_option("-i, --input", initial_mesh_file, "Initial mesh file")->required(true);
    app.add_option("-o, --output", output_mesh_file, "Output mesh file")->required(true);
    app.add_option("-l, --logs", operation_logs_dir, "Operation logs directory")->required(true);

    CLI11_PARSE(app, argc, argv);

    if (!std::filesystem::exists(initial_mesh_file)) {
        std::cerr << "File `" << initial_mesh_file << "` does not exist." << std::endl;
        return EXIT_FAILURE;
    }
    if (!std::filesystem::exists(output_mesh_file)) {
        std::cerr << "File `" << output_mesh_file << "` does not exist." << std::endl;
        return EXIT_FAILURE;
    }

    auto init_mesh_ptr = wmtk::read_mesh(initial_mesh_file);
    auto [F_in, V_in] = static_cast<TriMesh&>(*init_mesh_ptr).get_FV();
    std::cout << "F_in size" << F_in.rows() << std::endl;
    std::cout << "V_in size" << V_in.rows() << std::endl;

    Eigen::MatrixXd V_out;
    Eigen::MatrixXi F_out;
    igl::readOBJ(output_mesh_file.string(), V_out, F_out);
    std::cout << "F_out size" << F_out.rows() << std::endl;
    std::cout << "V_out size" << V_out.rows() << std::endl;
    // viewer
    // {
    //     igl::opengl::glfw::Viewer viewer;
    //     Eigen::Vector4f backColor;
    //     backColor << 208 / 255., 237 / 255., 227 / 255., 1.;
    //     const Eigen::RowVector3d blue(149.0 / 255, 217.0 / 255, 244.0 / 255);
    //     viewer.core().background_color = backColor;
    //     viewer.data().set_mesh(V_in, F_in);
    //     viewer.data().set_colors(blue);
    //     viewer.callback_key_down =
    //         [&](igl::opengl::glfw::Viewer& viewer, unsigned char key, int mod) -> bool {
    //         switch (key) {
    //         case '0':
    //             viewer.data().clear();
    //             viewer.data().set_mesh(V_in, F_in);
    //             viewer.data().set_colors(blue);
    //             break;
    //         case '1':
    //             viewer.data().clear();
    //             viewer.data().set_mesh(V_out, F_out);
    //             viewer.data().set_colors(blue);
    //             break;
    //         default: return false;
    //         }
    //         return true;
    //     };
    //     viewer.launch();
    // }

    back_track_map(operation_logs_dir);
    return 0;
}