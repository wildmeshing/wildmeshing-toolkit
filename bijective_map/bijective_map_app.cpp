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
#include <igl/Timer.h>
#include <igl/boundary_loop.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/parallel_for.h>
#include <igl/stb/write_image.h>
using path = std::filesystem::path;

#include <nlohmann/json.hpp>
using json = nlohmann::json;
#include <igl/stb/write_image.h>
#include "render_utils.hpp"
#include "track_operations.hpp"

// TODO: for testing purpose
void back_track_map(path dirPath, std::vector<query_point>& query_points, bool do_forward = false)
{
    namespace fs = std::filesystem;
    int maxIndex = -1;

    for (const auto& entry : fs::directory_iterator(dirPath)) {
        if (entry.path().filename().string().find("operation_log_") != std::string::npos) {
            ++maxIndex;
        }
    }

    for (int i = maxIndex; i >= 0; --i) {
        int file_id = i;
        if (do_forward) {
            file_id = maxIndex - i;
        }
        fs::path filePath = dirPath / ("operation_log_" + std::to_string(file_id) + ".json");
        std::ifstream file(filePath);
        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filePath << std::endl;
            continue;
        }
        json operation_log;
        file >> operation_log;

        std::cout << "Trace Operations number: " << file_id << std::endl;
        std::string operation_name;
        operation_name = operation_log["operation_name"];

        if (operation_name == "MeshConsolidate") {
            std::cout << "This Operations is Consolidate" << std::endl;
            std::vector<int64_t> face_ids_maps;
            std::vector<int64_t> vertex_ids_maps;
            parse_consolidate_file(operation_log, face_ids_maps, vertex_ids_maps);
            if (do_forward) {
                handle_consolidate_forward(face_ids_maps, vertex_ids_maps, query_points);
            } else {
                handle_consolidate(face_ids_maps, vertex_ids_maps, query_points);
            }
        } else if (operation_name == "AttributesUpdate") {
            // std::cout << "This Operations is AttributeUpdate" << std::endl;
        } else if (operation_name == "EdgeSplit") {
            std::cout << "This Operations is EdgeSplit" << std::endl;
            Eigen::MatrixXi F_after, F_before;
            Eigen::MatrixXd V_after, V_before;
            std::vector<int64_t> id_map_after, id_map_before;
            std::vector<int64_t> v_id_map_after, v_id_map_before;
            parse_edge_split_file(
                operation_log,
                V_before,
                F_before,
                id_map_before,
                v_id_map_before,
                V_after,
                F_after,
                id_map_after,
                v_id_map_after);

            if (do_forward) {
                handle_split_edge(
                    V_after,
                    F_after,
                    id_map_after,
                    v_id_map_after,
                    V_before,
                    F_before,
                    id_map_before,
                    v_id_map_before,
                    query_points);
            } else {
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
            }

        } else if (operation_name == "EdgeCollapse") {
            std::cout << "This Operations is EdgeCollapse" << std::endl;
            Eigen::MatrixXi F_after, F_before;
            Eigen::MatrixXd UV_joint;
            std::vector<int64_t> v_id_map_joint;
            std::vector<int64_t> id_map_after, id_map_before;
            parse_edge_collapse_file(
                operation_log,
                UV_joint,
                F_before,
                F_after,
                v_id_map_joint,
                id_map_before,
                id_map_after);
            if (do_forward) {
                handle_collapse_edge(
                    UV_joint,
                    F_after,
                    F_before,
                    v_id_map_joint,
                    id_map_after,
                    id_map_before,
                    query_points);
            } else {
                handle_collapse_edge(
                    UV_joint,
                    F_before,
                    F_after,
                    v_id_map_joint,
                    id_map_before,
                    id_map_after,
                    query_points);
            }


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
    std::string application_name = "back";
    app.add_option("-i, --input", initial_mesh_file, "Initial mesh file")->required(true);
    app.add_option("-o, --output", output_mesh_file, "Output mesh file")->required(true);
    app.add_option("-l, --logs", operation_logs_dir, "Operation logs directory")->required(true);
    app.add_option("a, --app", application_name, "Application name");
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

    std::cout << "F_in size " << F_in.rows() << ", " << F_in.cols() << std::endl;
    std::cout << "V_in size " << V_in.rows() << ", " << V_in.cols() << std::endl;

    Eigen::MatrixXd V_out;
    Eigen::MatrixXi F_out;
    igl::readOBJ(output_mesh_file.string(), V_out, F_out);
    std::cout << "F_out size" << F_out.rows() << ", " << F_out.cols() << std::endl;
    std::cout << "V_out size" << V_out.rows() << ", " << V_in.cols() << std::endl;

    std::vector<std::vector<int>> bd_loops;
    igl::boundary_loop(F_out, bd_loops);
    std::cout << "bd_loops size" << bd_loops.size() << std::endl;

    // test forward track
    if (application_name == "forward") {
        // TODO: get test example for query_points
        std::vector<query_point> query_points;
        for (int i = 0; i < F_in.rows(); i++) {
            query_point qp;
            qp.f_id = i;
            qp.bc = Eigen::Vector3d(1.0 / 3, 1.0 / 3, 1.0 / 3);
            qp.fv_ids = F_in.row(i);
            query_points.push_back(qp);
        }

        std::vector<query_point> query_points_origin = query_points;

        Eigen::MatrixXd pts_on_surface_before(query_points.size(), 3);
        for (int ii = 0; ii < query_points.size(); ii++) {
            const query_point& qp = query_points[ii];
            Eigen::Vector3d p(0, 0, 0);
            for (int i = 0; i < 3; i++) {
                p += V_in.row(qp.fv_ids[i]) * qp.bc(i);
            }
            pts_on_surface_before.row(ii) = p;
        }


        // do back track
        back_track_map(operation_logs_dir, query_points, true);

        Eigen::MatrixXd pts_on_surface_after(query_points.size(), 3);
        for (int ii = 0; ii < query_points.size(); ii++) {
            const query_point& qp = query_points[ii];
            Eigen::Vector3d p(0, 0, 0);
            for (int i = 0; i < 3; i++) {
                p += V_out.row(qp.fv_ids[i]) * qp.bc(i);
            }
            pts_on_surface_after.row(ii) = p;
        }
        // viewer

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
    } else if (application_name == "back") {
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
    } else if (application_name == "render") {
        // funciton to generate the picture
        auto writePNG = [&](const std::string& name, int W, int H, camera_info cam) {
            std::cout << "try get png" << std::endl;
            Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R, G, B, A;
            R.resize(W, H);
            G.resize(W, H);
            B.resize(W, H);
            A.resize(W, H);
            R.setConstant(255);
            G.setConstant(255);
            B.setConstant(255);
            A.setConstant(255);

            auto [fids, bcs] = get_pt_mat(cam, V_out, F_out, W, H);

            std::vector<query_point> query_points;
            for (int ii = 0; ii < fids.size(); ii++) {
                query_point qp;
                qp.f_id = fids[ii];

                if (fids[ii] == -1) {
                    qp.fv_ids = F_out.row(0);
                } else {
                    qp.fv_ids = F_out.row(fids[ii]);
                }
                qp.bc = bcs[ii];

                query_points.push_back(qp);
            }
            igl::Timer timer;
            timer.start();
            back_track_map(operation_logs_dir, query_points);
            timer.stop();
            std::cout << "query time: " << timer.getElapsedTime() << " s" << std::endl;

            igl::parallel_for(W * H, [&](int id) {
                if (fids[id] != -1) {
                    R(id % W, H - 1 - id / W) = color_map[query_points[id].f_id % 20][0];
                    G(id % W, H - 1 - id / W) = color_map[query_points[id].f_id % 20][1];
                    B(id % W, H - 1 - id / W) = color_map[query_points[id].f_id % 20][2];
                } else {
                    A(id % W, H - 1 - id / W) = 0;
                }
            });
            igl::stb::write_image(name + ".png", R, G, B, A);
            addShading(R, G, B, V_out, F_out, fids, bcs, std::get<0>(cam), false);
            igl::stb::write_image(name + "_shading.png", R, G, B, A);
        };

        igl::opengl::glfw::Viewer viewer;
        viewer.data().set_mesh(V_out, F_out);
        viewer.launch();
        camera_info camera =
            std::make_tuple(viewer.core().view, viewer.core().proj, viewer.core().viewport);
        writePNG("iso_out", 1280, 800, camera);
    }
    return 0;
}