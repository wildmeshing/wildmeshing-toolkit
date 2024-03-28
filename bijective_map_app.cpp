#include <CLI/CLI.hpp>
#include <filesystem>
#include <iostream>
#include <sstream>

// wmtk
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/MeshReader.hpp>

using namespace wmtk;

// igl
#include <igl/opengl/glfw/Viewer.h>
#include <igl/parallel_for.h>
using path = std::filesystem::path;

#include "track_operations.hpp"

// TODO: for testing purpose
void back_track_map(path dirPath, std::vector<query_point>& query_points)
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

        std::cout << "Trace Back Operations number: " << i << std::endl;
        std::string first_line;
        std::getline(file, first_line);
        if (first_line == "Consolidate") {
            std::cout << "This Operations is Consolidate" << std::endl;
            std::vector<int64_t> face_ids_maps;
            std::vector<int64_t> vertex_ids_maps;
            parse_consolidate_file(file, face_ids_maps, vertex_ids_maps);
            handle_consolidate(face_ids_maps, vertex_ids_maps, query_points);
        } else if (first_line == "AttributesUpdate") {
            // std::cout << "This Operations is AttributeUpdate" << std::endl;
        } else if (first_line == "EdgeSplit") {
            std::cout << "This Operations is EdgeSplit" << std::endl;
            Eigen::MatrixXi F_after, F_before;
            Eigen::MatrixXd V_after, V_before;
            std::vector<int64_t> id_map_after, id_map_before;
            std::vector<int64_t> v_id_map_after, v_id_map_before;
            parse_edge_split_file(
                file,
                V_before,
                F_before,
                id_map_before,
                v_id_map_before,
                V_after,
                F_after,
                id_map_after,
                v_id_map_after);
            handle_split_edge(
                V_before,
                F_before,
                id_map_before,
                v_id_map_before,
                V_after,
                F_after,
                id_map_after,
                v_id_map_after,
                query_points);

        } else if (first_line == "EdgeCollapse") {
            // std::cout << "This Operations is EdgeCollapse" << std::endl;
        } else {
            // std::cout << "This Operations is not implemented" << std::endl;
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

    // TODO: get test example for query_points
    std::vector<query_point> query_points;
    for (int i = 0; i < F_out.rows(); i++) {
        query_point qp;
        qp.f_id = i;
        qp.bc = Eigen::Vector3d(1.0 / 3, 1.0 / 3, 1.0 / 3);
        qp.fv_ids = F_out.row(i);
        query_points.push_back(qp);
    }

    std::vector<query_point> query_points_origin = query_points;
    Eigen::MatrixXd pts_on_surface_after(query_points.size(), 3);
    for (int ii = 0; ii < query_points_origin.size(); ii++) {
        const query_point& qp = query_points_origin[ii];
        Eigen::Vector3d p(0, 0, 0);
        for (int i = 0; i < 3; i++) {
            p += V_out.row(qp.fv_ids[i]) * qp.bc(i);
        }
        pts_on_surface_after.row(ii) = p;
    }

    // do back track
    back_track_map(operation_logs_dir, query_points);

    Eigen::MatrixXd pts_on_surface_before(query_points.size(), 3);
    for (int ii = 0; ii < query_points.size(); ii++) {
        const query_point& qp = query_points[ii];
        Eigen::Vector3d p(0, 0, 0);
        for (int i = 0; i < 3; i++) {
            p += V_in.row(qp.fv_ids[i]) * qp.bc(i);
        }
        pts_on_surface_before.row(ii) = p;
    }

    // viewer
    {
        igl::opengl::glfw::Viewer viewer;
        Eigen::Vector4f backColor;
        backColor << 208 / 255., 237 / 255., 227 / 255., 1.;
        const Eigen::RowVector3d blue(149.0 / 255, 217.0 / 255, 244.0 / 255);
        viewer.core().background_color = backColor;
        viewer.data().set_mesh(V_in, F_in);
        viewer.data().set_colors(blue);
        viewer.data().add_points(pts_on_surface_before, Eigen::RowVector3d(0, 0, 0));
        viewer.data().point_size = 10;

        viewer.callback_key_down =
            [&](igl::opengl::glfw::Viewer& viewer, unsigned char key, int mod) -> bool {
            switch (key) {
            case '0':
                viewer.data().clear();
                viewer.data().set_mesh(V_in, F_in);
                viewer.data().set_colors(blue);
                viewer.data().add_points(pts_on_surface_before, Eigen::RowVector3d(0, 0, 0));
                viewer.data().point_size = 10;
                break;
            case '1':
                viewer.data().clear();
                viewer.data().set_mesh(V_out, F_out);
                viewer.data().set_colors(blue);
                viewer.data().add_points(pts_on_surface_after, Eigen::RowVector3d(0, 0, 0));
                viewer.data().point_size = 10;
                break;
            default: return false;
            }
            return true;
        };
        viewer.launch();
    }

    return 0;
}