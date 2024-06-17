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
#include <igl/stb/read_image.h>
#include <igl/stb/write_image.h>
using path = std::filesystem::path;

#include <nlohmann/json.hpp>
using json = nlohmann::json;
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
        } else if (operation_name == "TriEdgeSwap") {
            std::cout << "This Operations is TriEdgeSwap" << std::endl;
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
                handle_swap_edge(
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

void back_track_map_rational(path dirPath, std::vector<query_point_r>& query_points)
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
            handle_consolidate(face_ids_maps, vertex_ids_maps, query_points);
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

            handle_collapse_edge_r(
                UV_joint,
                F_before,
                F_after,
                v_id_map_joint,
                id_map_before,
                id_map_after,
                query_points);

        } else {
            std::cout << "This Operations is not implemented" << std::endl;
        }

        file.close();
    }
}

void back_track_lines(path dirPath, query_curve& curve, bool do_forward = false)
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

            handle_consolidate(face_ids_maps, vertex_ids_maps, curve);
        } else if (operation_name == "TriEdgeSwap") {
            std::cout << "This Operations is TriEdgeSwap" << std::endl;
            // TODO:
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
            // TODO:
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

            handle_collapse_edge(
                UV_joint,
                F_before,
                F_after,
                v_id_map_joint,
                id_map_before,
                id_map_after,
                curve);
        } else {
            // std::cout << "This Operations is not implemented" << std::endl;
        }

        file.close();
    }
}

void back_track_lines_rational(path dirPath, query_curve_r& curve)
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
            handle_consolidate(face_ids_maps, vertex_ids_maps, curve);
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
            handle_collapse_edge_r(
                UV_joint,
                F_before,
                F_after,
                v_id_map_joint,
                id_map_before,
                id_map_after,
                curve);
        } else {
            std::cout << "This Operations is not implemented" << std::endl;
        }

        file.close();
    }
}

void forward_track_app(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Eigen::MatrixXd& V_out,
    const Eigen::MatrixXi& F_out,
    const path& operation_logs_dir)
{
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
}

void back_track_app(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Eigen::MatrixXd& V_out,
    const Eigen::MatrixXi& F_out,
    const path& operation_logs_dir)
{
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
}

void back_track_app_rational(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Eigen::MatrixXd& V_out,
    const Eigen::MatrixXi& F_out,
    const path& operation_logs_dir)
{
    std::vector<query_point_r> query_points;
    for (int i = 0; i < F_out.rows(); i++) {
        query_point_r qp;
        qp.f_id = i;
        qp.bc = Eigen::Vector3<wmtk::Rational>(
            wmtk::Rational(1.0 / 3),
            wmtk::Rational(1.0 / 3),
            wmtk::Rational(1.0 / 3));
        qp.fv_ids = F_out.row(i);
        query_points.push_back(qp);
    }

    std::vector<query_point_r> query_points_origin = query_points;
    Eigen::MatrixXd pts_on_surface_after(query_points.size(), 3);
    for (int ii = 0; ii < query_points_origin.size(); ii++) {
        const query_point_r& qp = query_points_origin[ii];
        Eigen::Vector3d p(0, 0, 0);
        for (int i = 0; i < 3; i++) {
            p += V_out.row(qp.fv_ids[i]) * qp.bc(i).to_double();
        }
        pts_on_surface_after.row(ii) = p;
    }
    // do back track
    back_track_map_rational(operation_logs_dir, query_points);

    Eigen::MatrixXd pts_on_surface_before(query_points.size(), 3);
    for (int ii = 0; ii < query_points.size(); ii++) {
        const query_point_r& qp = query_points[ii];
        Eigen::Vector3d p(0, 0, 0);
        for (int i = 0; i < 3; i++) {
            p += V_in.row(qp.fv_ids[i]) * qp.bc(i).to_double();
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
}

void render_app(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Eigen::MatrixXd& V_out,
    const Eigen::MatrixXi& F_out,
    const path& operation_logs_dir)
{
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

#include <igl/triangle_triangle_adjacency.h>
query_curve generate_curve(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const int curve_length,
    const int start_face_id)
{
    Eigen::MatrixXi TT, TTi;
    igl::triangle_triangle_adjacency(F, TT, TTi);
    std::vector<int> visited_faces(F.rows(), 0);

    int current_face_id = start_face_id;
    int current_edge_id = 0;

    query_curve curve;

    Eigen::Matrix3d bary_coord_ref;
    bary_coord_ref << 0.5, 0.5, 0, 0, 0.5, 0.5, 0.5, 0, 0.5;

    while (curve.segments.size() < curve_length) {
        visited_faces[current_face_id] = 1;
        int next_edge_id = (current_edge_id + 1) % 3;
        if (TT(current_face_id, next_edge_id) == -1 ||
            visited_faces[TT(current_face_id, next_edge_id)] == 1) {
            next_edge_id = (current_edge_id + 2) % 3;
        }

        if (TT(current_face_id, next_edge_id) == -1 ||
            visited_faces[TT(current_face_id, next_edge_id)] == 1) {
            break;
        }

        std::cout << "current_face_id: " << current_face_id << std::endl;
        std::cout << "current_edge_id: " << current_edge_id << std::endl;
        std::cout << "next_edge_id: " << next_edge_id << std::endl;
        std::cout << "TT.row(current_face_id): " << TT.row(current_face_id) << std::endl;
        std::cout << "TTi.row(current_face_id): " << TTi.row(current_face_id) << std::endl;


        query_segment seg;
        seg.f_id = current_face_id;
        seg.bcs[0] = bary_coord_ref.row(current_edge_id);
        seg.bcs[1] = bary_coord_ref.row(next_edge_id);
        seg.fv_ids = F.row(current_face_id);

        curve.segments.push_back(seg);

        current_edge_id = TTi(current_face_id, next_edge_id);
        current_face_id = TT(current_face_id, next_edge_id);
    }

    curve.next_segment_ids.resize(curve.segments.size());
    for (int i = 0; i < curve.segments.size() - 1; i++) {
        curve.next_segment_ids[i] = i + 1;
    }
    curve.next_segment_ids[curve.segments.size() - 1] = -1;

    return curve;
}

void back_track_line_app(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Eigen::MatrixXd& V_out,
    const Eigen::MatrixXi& F_out,
    const path& operation_logs_dir,
    bool use_rational = false)
{
    query_curve curve_in = generate_curve(V_out, F_out, 10, 0);
    for (const auto& seg : curve_in.segments) {
        std::cout << "f_id: " << seg.f_id << std::endl;
        std::cout << "bcs: " << seg.bcs[0] << ", " << seg.bcs[1] << std::endl;
    }
    if (!use_rational) {
        query_curve curve = curve_in;
        query_curve curve_origin = curve;
        back_track_lines(operation_logs_dir, curve);
        {
            igl::opengl::glfw::Viewer viewer;
            viewer.data().set_mesh(V_out, F_out);
            viewer.data().point_size /= 3;
            for (const auto& seg : curve_origin.segments) {
                Eigen::MatrixXd pts(2, 3);
                for (int i = 0; i < 2; i++) {
                    Eigen::Vector3d p(0, 0, 0);
                    for (int j = 0; j < 3; j++) {
                        p += V_out.row(seg.fv_ids[j]) * seg.bcs[i](j);
                    }
                    pts.row(i) = p;
                }
                std::cout << "pts: \n" << pts << std::endl;
                viewer.data().add_points(pts.row(0), Eigen::RowVector3d(1, 0, 0));
                viewer.data().add_points(pts.row(1), Eigen::RowVector3d(1, 0, 0));
                viewer.data().add_edges(pts.row(0), pts.row(1), Eigen::RowVector3d(1, 0, 0));
            }

            viewer.launch();
        }
        {
            igl::opengl::glfw::Viewer viewer;
            viewer.data().set_mesh(V_in, F_in);
            viewer.data().point_size /= 3;
            for (const auto& seg : curve.segments) {
                Eigen::MatrixXd pts(2, 3);
                for (int i = 0; i < 2; i++) {
                    Eigen::Vector3d p(0, 0, 0);
                    for (int j = 0; j < 3; j++) {
                        p += V_in.row(seg.fv_ids[j]) * seg.bcs[i](j);
                    }
                    pts.row(i) = p;
                }
                viewer.data().add_points(pts.row(0), Eigen::RowVector3d(1, 0, 0));
                viewer.data().add_points(pts.row(1), Eigen::RowVector3d(1, 0, 0));
                viewer.data().add_edges(pts.row(0), pts.row(1), Eigen::RowVector3d(1, 0, 0));
            }


            viewer.launch();
        }
    } else {
        query_curve_r curve;
        // convert to rational
        for (const auto& seg : curve_in.segments) {
            query_segment_r seg_r;
            seg_r.f_id = seg.f_id;
            seg_r.bcs[0] = Eigen::Vector3<wmtk::Rational>(
                wmtk::Rational(seg.bcs[0](0)),
                wmtk::Rational(seg.bcs[0](1)),
                wmtk::Rational(seg.bcs[0](2)));
            seg_r.bcs[1] = Eigen::Vector3<wmtk::Rational>(
                wmtk::Rational(seg.bcs[1](0)),
                wmtk::Rational(seg.bcs[1](1)),
                wmtk::Rational(seg.bcs[1](2)));
            seg_r.fv_ids = seg.fv_ids;
            curve.segments.push_back(seg_r);
        }

        query_curve_r curve_origin = curve;
        back_track_lines_rational(operation_logs_dir, curve);
        {
            igl::opengl::glfw::Viewer viewer;
            viewer.data().set_mesh(V_out, F_out);
            viewer.data().point_size /= 3;
            for (const auto& seg : curve_origin.segments) {
                Eigen::MatrixXd pts(2, 3);
                for (int i = 0; i < 2; i++) {
                    Eigen::Vector3d p(0, 0, 0);
                    for (int j = 0; j < 3; j++) {
                        p += V_out.row(seg.fv_ids[j]) * seg.bcs[i](j).to_double();
                    }
                    pts.row(i) = p;
                }
                std::cout << "pts: \n" << pts << std::endl;
                viewer.data().add_points(pts.row(0), Eigen::RowVector3d(1, 0, 0));
                viewer.data().add_points(pts.row(1), Eigen::RowVector3d(1, 0, 0));
                viewer.data().add_edges(pts.row(0), pts.row(1), Eigen::RowVector3d(1, 0, 0));
            }

            viewer.launch();
        }
        {
            igl::opengl::glfw::Viewer viewer;
            viewer.data().set_mesh(V_in, F_in);
            viewer.data().point_size /= 3;
            for (const auto& seg : curve.segments) {
                Eigen::MatrixXd pts(2, 3);
                for (int i = 0; i < 2; i++) {
                    Eigen::Vector3d p(0, 0, 0);
                    for (int j = 0; j < 3; j++) {
                        p += V_in.row(seg.fv_ids[j]) * seg.bcs[i](j).to_double();
                    }
                    pts.row(i) = p;
                }
                viewer.data().add_points(pts.row(0), Eigen::RowVector3d(1, 0, 0));
                viewer.data().add_points(pts.row(1), Eigen::RowVector3d(1, 0, 0));
                viewer.data().add_edges(pts.row(0), pts.row(1), Eigen::RowVector3d(1, 0, 0));
            }


            viewer.launch();
        }
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
    app.add_option("-a, --app", application_name, "Application name");

    path input_obj_file;
    app.add_option("--input_obj", input_obj_file, "Input obj file");
    path input_texture_file;
    app.add_option("--input_texture", input_texture_file, "Input texture file");

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


    Eigen::MatrixXd V_out, Vt_out, Vn_out;
    Eigen::MatrixXi F_out, Ft_out, Fn_out;
    std::cout << "\nloading output obj file..." << std::endl;
    igl::readOBJ(output_mesh_file.string(), V_out, Vt_out, Vn_out, F_out, Ft_out, Fn_out);
    std::cout << "F_out size" << F_out.rows() << ", " << F_out.cols() << std::endl;
    std::cout << "V_out size" << V_out.rows() << ", " << V_in.cols() << std::endl;

    if (application_name == "texture") {
        Eigen::MatrixXd V_in_obj, Vt_in_obj, Vn_in_obj;
        Eigen::MatrixXi F_in_obj, Ft_in_obj, Fn_in_obj;

        std::cout << "\nloading input obj file..." << std::endl;
        igl::readOBJ(
            input_obj_file.string(),
            V_in_obj,
            Vt_in_obj,
            Vn_in_obj,
            F_in_obj,
            Ft_in_obj,
            Fn_in_obj);

        std::cout << "F_in_obj size " << F_in_obj.rows() << ", " << F_in_obj.cols() << std::endl;
        std::cout << "V_in_obj size " << V_in_obj.rows() << ", " << V_in_obj.cols() << std::endl;

        Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R, G, B, A;
        std::cout << "\nloading texture file..." << std::endl;
        igl::stb::read_image(input_texture_file.string(), R, G, B, A);
        int height = R.rows();
        int width = R.cols();
        std::cout << "height: " << height << ", width: " << width << std::endl;

        Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R_out, G_out, B_out, A_out;
        R_out.resize(height, width);
        G_out.resize(height, width);
        B_out.resize(height, width);
        A_out.resize(height, width);

        std::cout << "hello0" << std::endl;
        // TODO: sampling query points on Ft_out, Vt_out
        auto isPointInTriangle = [](double px,
                                    double py,
                                    double ax,
                                    double ay,
                                    double bx,
                                    double by,
                                    double cx,
                                    double cy) {
            double denominator = ((by - cy) * (ax - cx) + (cx - bx) * (ay - cy));
            double a = ((by - cy) * (px - cx) + (cx - bx) * (py - cy)) / denominator;
            double b = ((cy - ay) * (px - cx) + (ax - cx) * (py - cy)) / denominator;
            double c = 1.0 - a - b;
            return std::make_tuple(
                0 <= a && a <= 1 && 0 <= b && b <= 1 && 0 <= c && c <= 1,
                a,
                b,
                c);
        };
        std::cout << "hello1" << std::endl;
        // for each pixel in the texture image
        std::vector<query_point> query_points;
        for (int y = 0; y < height; ++y) {
            std::cout << "processing row " << y << std::endl;
            for (int x = 0; x < width; ++x) {
                double u = double(x) / (width - 1);
                double v = double(y) / (height - 1);

                bool found_intersect = false;
                for (int t_id = 0; t_id < Ft_out.rows(); ++t_id) {
                    auto [inside, a, b, c] = isPointInTriangle(
                        u,
                        v,
                        Vt_out(Ft_out(t_id, 0), 0),
                        Vt_out(Ft_out(t_id, 0), 1),
                        Vt_out(Ft_out(t_id, 1), 0),
                        Vt_out(Ft_out(t_id, 1), 1),
                        Vt_out(Ft_out(t_id, 2), 0),
                        Vt_out(Ft_out(t_id, 2), 1));

                    if (inside) {
                        query_point qp;
                        qp.f_id = t_id;
                        qp.fv_ids = F_out.row(t_id);
                        qp.bc = Eigen::Vector3d(a, b, c);
                        query_points.push_back(qp);
                        found_intersect = true;
                        break;
                    }
                }

                if (!found_intersect) {
                    // add an empty query point
                    query_point qp;
                    qp.f_id = -1;
                    qp.fv_ids = F_out.row(0);
                    qp.bc = Eigen::Vector3d(1.0 / 3, 1.0 / 3, 1.0 / 3);
                    query_points.push_back(qp);
                }
            }
        }

        std::cout << "done finding all query points" << std::endl;
        // print out the query points
        for (int i = 0; i < query_points.size(); i++) {
            std::cout << "query point " << i << "'s fid: " << query_points[i].f_id << std::endl;
        }
        back_track_map(operation_logs_dir, query_points);

        igl::parallel_for(height * width, [&](int id) {
            if (query_points[id].f_id != -1) {
                int f_id = query_points[id].f_id;
                Eigen::Vector3d p(0, 0, 0);
                for (int i = 0; i < 3; i++) {
                    p += Vt_in_obj.row(Ft_in_obj(f_id, i)) * query_points[id].bc(i);
                }
                int x = int(p(0) * (width - 1));
                int y = int(p(1) * (height - 1));
                R_out(id % height, width - 1 - id / height) = R(y, x);
                G_out(id % height, width - 1 - id / height) = G(y, x);
                B_out(id % height, width - 1 - id / height) = B(y, x);
            } else {
                A_out(id % height, width - 1 - id / height) = 0;
            }
        });
        // write the output image
        igl::stb::write_image("output_texture.png", R_out, G_out, B_out, A_out);
    }


    // test forward track
    if (application_name == "forward") {
        forward_track_app(V_in, F_in, V_out, F_out, operation_logs_dir);
    } else if (application_name == "back") {
        back_track_app(V_in, F_in, V_out, F_out, operation_logs_dir);
    } else if (application_name == "render") {
        render_app(V_in, F_in, V_out, F_out, operation_logs_dir);
    } else if (application_name == "back_lines") {
        back_track_line_app(V_in, F_in, V_out, F_out, operation_logs_dir);
    } else if (application_name == "back_r") {
        back_track_app_rational(V_in, F_in, V_out, F_out, operation_logs_dir);
    } else if (application_name == "back_lines_r") {
        back_track_line_app(V_in, F_in, V_out, F_out, operation_logs_dir, true);
    }
    return 0;
}