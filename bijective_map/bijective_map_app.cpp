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


void back_track_map(path dirPath, std::vector<query_point>& query_points, bool do_forward = false, bool use_rational = false)
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
                    query_points,
                    use_rational);
            } else {
                handle_collapse_edge(
                    UV_joint,
                    F_before,
                    F_after,
                    v_id_map_joint,
                    id_map_before,
                    id_map_after,
                    query_points,
                    use_rational);
            }


        } else {
            // std::cout << "This Operations is not implemented" << std::endl;
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

            handle_collapse_edge_curve(
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

void back_track_lines_rational(path dirPath, query_curve& curve)
{
    // TODO: implement this
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

// TODO: swap operation
void back_track_app(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Eigen::MatrixXd& V_out,
    const Eigen::MatrixXi& F_out,
    const path& operation_logs_dir,
    bool use_rational = false)
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
    back_track_map(operation_logs_dir, query_points, false, use_rational);

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
        /*
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
        }*/
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
        back_track_app(V_in, F_in, V_out, F_out, operation_logs_dir, true);
    } else if (application_name == "back_lines_r") {
        back_track_line_app(V_in, F_in, V_out, F_out, operation_logs_dir, true);
    }
    return 0;
}