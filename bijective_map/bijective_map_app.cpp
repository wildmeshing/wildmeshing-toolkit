#include <CLI/CLI.hpp>
#include <filesystem>
#include <iostream>
#include <sstream>

// wmtk
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/orient.hpp>
using namespace wmtk;

// igl
#include <igl/Timer.h>
#include <igl/boundary_loop.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/parallel_for.h>
#include <igl/stb/read_image.h>
#include <igl/stb/write_image.h>

#include <igl/in_element.h>
using path = std::filesystem::path;

#include <nlohmann/json.hpp>
using json = nlohmann::json;
#include "generate_iso_line.hpp"
#include "render_utils.hpp"
#include "track_operations.hpp"

void back_track_map(
    path dirPath,
    std::vector<query_point>& query_points,
    bool do_forward = false,
    bool use_rational = false)
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
        } else if (operation_name == "TriEdgeSwap" || operation_name == "AttributesUpdate") {
            std::cout << "This Operations is" << operation_name << std::endl;
            Eigen::MatrixXi F_after, F_before;
            Eigen::MatrixXd V_after, V_before;
            std::vector<int64_t> id_map_after, id_map_before;
            std::vector<int64_t> v_id_map_after, v_id_map_before;
            bool is_skipped;
            parse_non_collapse_file(
                operation_log,
                is_skipped,
                V_before,
                F_before,
                id_map_before,
                v_id_map_before,
                V_after,
                F_after,
                id_map_after,
                v_id_map_after);

            if (is_skipped) {
                continue;
            }

            if (do_forward) {
                handle_swap_edge(
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
            bool is_skipped;
            parse_non_collapse_file(
                operation_log,
                is_skipped,
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
            if (do_forward) {
                handle_consolidate_forward(face_ids_maps, vertex_ids_maps, curve);
            } else {
                handle_consolidate(face_ids_maps, vertex_ids_maps, curve);
            }
        } else if (operation_name == "TriEdgeSwap" || operation_name == "AttributesUpdate") {
            std::cout << "This Operations is" << operation_name << std::endl;
            Eigen::MatrixXi F_after, F_before;
            Eigen::MatrixXd V_after, V_before;
            std::vector<int64_t> id_map_after, id_map_before;
            std::vector<int64_t> v_id_map_after, v_id_map_before;
            bool is_skipped;

            parse_non_collapse_file(
                operation_log,
                is_skipped,
                V_before,
                F_before,
                id_map_before,
                v_id_map_before,
                V_after,
                F_after,
                id_map_after,
                v_id_map_after);

            if (is_skipped) {
                continue;
            }

            if (do_forward) {
                handle_swap_edge_curve(
                    V_after,
                    F_after,
                    id_map_after,
                    v_id_map_after,
                    V_before,
                    F_before,
                    id_map_before,
                    v_id_map_before,
                    curve);
            } else {
                handle_swap_edge_curve(
                    V_before,
                    F_before,
                    id_map_before,
                    v_id_map_before,
                    V_after,
                    F_after,
                    id_map_after,
                    v_id_map_after,
                    curve);
            }
        } else if (operation_name == "EdgeSplit") {
            std::cout << "This Operations is EdgeSplit" << std::endl;
            Eigen::MatrixXi F_after, F_before;
            Eigen::MatrixXd V_after, V_before;
            std::vector<int64_t> id_map_after, id_map_before;
            std::vector<int64_t> v_id_map_after, v_id_map_before;
            bool is_skipped;
            parse_non_collapse_file(
                operation_log,
                is_skipped,
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
            if (do_forward) {
                handle_collapse_edge_curve(
                    UV_joint,
                    F_after,
                    F_before,
                    v_id_map_joint,
                    id_map_after,
                    id_map_before,
                    curve);
            } else {
                handle_collapse_edge_curve(
                    UV_joint,
                    F_before,
                    F_after,
                    v_id_map_joint,
                    id_map_before,
                    id_map_after,
                    curve);
            }
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

void forward_track_line_app(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Eigen::MatrixXd& uv_in,
    const Eigen::MatrixXi& Fuv_in,
    const Eigen::MatrixXd& V_out,
    const Eigen::MatrixXi& F_out,
    const path& operation_logs_dir)
{
    // get all curves
    std::vector<query_curve> curves;
    {
        int N = 5;

        auto curve_from_intersections = [&](const std::vector<Intersection>& input_intersections) {
            query_curve curve;
            for (int i = 0; i < input_intersections.size(); i += 2) {
                query_segment seg;
                if (input_intersections[i].fid != input_intersections[i + 1].fid) {
                    std::cout << "something wrong with input_intersections" << std::endl;
                }
                seg.f_id = input_intersections[i].fid;
                seg.bcs[0] = input_intersections[i].barycentric;
                seg.bcs[1] = input_intersections[i + 1].barycentric;
                seg.fv_ids = F_in.row(input_intersections[i].fid);
                curve.segments.push_back(seg);
            }

            curve.next_segment_ids.resize(curve.segments.size());
            for (int i = 0; i < curve.segments.size() - 1; i++) {
                curve.next_segment_ids[i] = i + 1;
            }
            curve.next_segment_ids[curve.segments.size() - 1] = -1;

            return curve;
        };

        for (int k = 0; k < N - 1; k++) {
            double value = 1.0 / N * (k + 1);
            auto intersectionsX = computeIsoLineIntersectionsX(uv_in, Fuv_in, value);
            auto curveX = curve_from_intersections(intersectionsX);
            curves.push_back(curveX);
            auto intersectionsY = computeIsoLineIntersectionsY(uv_in, Fuv_in, value);
            auto curveY = curve_from_intersections(intersectionsY);
            curves.push_back(curveY);
        }
    }

    auto curves_origin = curves;

    {
        igl::opengl::glfw::Viewer viewer;
        viewer.data().set_mesh(V_in, F_in);
        viewer.data().point_size /= 3;
        for (const auto curve_origin : curves_origin) {
            for (const auto& seg : curve_origin.segments) {
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
        }
        viewer.launch();
    }

    save_query_curves(curves, "curves.in");

    // forward track lines
    for (auto& curve : curves) {
        back_track_lines(operation_logs_dir, curve, true);
    }

    save_query_curves(curves, "curves.out");

    {
        igl::opengl::glfw::Viewer viewer;
        viewer.data().set_mesh(V_out, F_out);
        viewer.data().point_size /= 3;
        for (const auto& curve : curves) {
            for (const auto& seg : curve.segments) {
                Eigen::MatrixXd pts(2, 3);
                for (int i = 0; i < 2; i++) {
                    Eigen::Vector3d p(0, 0, 0);
                    for (int j = 0; j < 3; j++) {
                        p += V_out.row(seg.fv_ids[j]) * seg.bcs[i](j);
                    }
                    pts.row(i) = p;
                }
                viewer.data().add_points(pts.row(0), Eigen::RowVector3d(1, 0, 0));
                viewer.data().add_points(pts.row(1), Eigen::RowVector3d(1, 0, 0));
                viewer.data().add_edges(pts.row(0), pts.row(1), Eigen::RowVector3d(1, 0, 0));
            }
        }
        viewer.launch();
    }
}

void check_iso_lines(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Eigen::MatrixXd& V_out,
    const Eigen::MatrixXi& F_out,
    const std::vector<query_curve>& curves_in,
    const std::vector<query_curve>& curves_out)
{
    std::cout << "curves_in sizes:\n";
    for (const auto& c : curves_in) {
        std::cout << c.segments.size() << std::endl;
    }
    std::cout << "curves_out sizes:\n";
    for (const auto& c : curves_out) {
        std::cout << c.segments.size() << std::endl;
    }

    if (true) {
        igl::opengl::glfw::Viewer viewer;
        viewer.data().set_mesh(V_in, F_in);
        viewer.data().point_size /= 3;
        for (const auto curve_origin : curves_in) {
            for (const auto& seg : curve_origin.segments) {
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
        }
        viewer.launch();
    }
    if (false) {
        igl::opengl::glfw::Viewer viewer;
        viewer.data().set_mesh(V_out, F_out);
        viewer.data().point_size /= 3;
        for (const auto& curve : curves_out) {
            for (const auto& seg : curve.segments) {
                Eigen::MatrixXd pts(2, 3);
                for (int i = 0; i < 2; i++) {
                    Eigen::Vector3d p(0, 0, 0);
                    for (int j = 0; j < 3; j++) {
                        p += V_out.row(seg.fv_ids[j]) * seg.bcs[i](j);
                    }
                    pts.row(i) = p;
                }
                viewer.data().add_points(pts.row(0), Eigen::RowVector3d(1, 0, 0));
                viewer.data().add_points(pts.row(1), Eigen::RowVector3d(1, 0, 0));
                viewer.data().add_edges(pts.row(0), pts.row(1), Eigen::RowVector3d(1, 0, 0));
            }
        }
        viewer.launch();
    }


    auto doIntersect = [](const Eigen::RowVector2d& p1,
                          const Eigen::RowVector2d& q1,
                          const Eigen::RowVector2d& p2,
                          const Eigen::RowVector2d& q2) -> bool {
        auto onSegment = [](const Eigen::RowVector2d& p,
                            const Eigen::RowVector2d& q,
                            const Eigen::RowVector2d& r) -> bool {
            return q[0] <= std::max(p[0], r[0]) && q[0] >= std::min(p[0], r[0]) &&
                   q[1] <= std::max(p[1], r[1]) && q[1] >= std::min(p[1], r[1]);
        };

        // auto orientation = [](const Eigen::RowVector2d& p,
        //                       const Eigen::RowVector2d& q,
        //                       const Eigen::RowVector2d& r) -> int {
        //     double val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1]);
        //     if (val == 0) return 0; // collinear
        //     return (val > 0) ? 1 : 2; // clock or counterclock wise
        // };

        int o1 = wmtk::utils::wmtk_orient2d(p1, q1, p2);
        int o2 = wmtk::utils::wmtk_orient2d(p1, q1, q2);
        int o3 = wmtk::utils::wmtk_orient2d(p2, q2, p1);
        int o4 = wmtk::utils::wmtk_orient2d(p2, q2, q1);


        // General case
        if (o1 != o2 && o3 != o4) return true;

        // Special Cases
        if (o1 == 0 && onSegment(p1, p2, q1)) return true;
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;

        return false; // No intersection
    };
    auto count_curve_intersection = [&](const std::vector<query_curve>& curve) {
        for (int i = 0; i < curve.size(); i++) {
            // for (int j = i; j < curve.size(); j++) {
            for (int j = i; j < i + 1; j++) {
                int intersect_count = 0;
                for (int seg_i = 0; seg_i < curve[i].segments.size(); seg_i++) {
                    for (int seg_j = 0; seg_j < curve[j].segments.size(); seg_j++) {
                        if (i == j &&
                            (seg_i == seg_j || curve[i].next_segment_ids[seg_i] == seg_j ||
                             curve[j].next_segment_ids[seg_j] == seg_i)) {
                            continue;
                        }
                        if (curve[i].segments[seg_i].f_id != curve[j].segments[seg_j].f_id) {
                            continue;
                        }

                        Eigen::RowVector2d p1, q1, p2, q2;
                        p1 = curve[i].segments[seg_i].bcs[0].head(2);
                        q1 = curve[i].segments[seg_i].bcs[1].head(2);
                        p2 = curve[j].segments[seg_j].bcs[0].head(2);
                        q2 = curve[j].segments[seg_j].bcs[1].head(2);

                        // if ((p1 - p2).norm() < 1e-8 || (p1 - q2).norm() < 1e-8 ||
                        //     (q1 - p2).norm() < 1e-8 || (q1 - q2).norm() < 1e-8) {
                        //     continue;
                        // }

                        if (doIntersect(p1, q1, p2, q2)) {
                            intersect_count++;
                            std::cout << "i = " << i << ", seg_i = " << seg_i
                                      << ", next[seg_i] = " << curves_out[i].next_segment_ids[seg_i]
                                      << std::endl;
                            std::cout << "p1: " << p1 << std::endl;
                            std::cout << "q1: " << q1 << std::endl;
                            std::cout << "j = " << j << ", seg_j = " << seg_j
                                      << ", next[seg_j] = " << curves_out[j].next_segment_ids[seg_j]
                                      << std::endl;
                            std::cout << "p2: " << p2 << std::endl;
                            std::cout << "q2: " << q2 << std::endl;

                            std::cout << "next[seg_i]: "
                                      << curves_out[i]
                                             .segments[curves_out[i].next_segment_ids[seg_i]]
                                             .bcs[0]
                                             .head(2)
                                             .transpose()
                                      << ", "
                                      << curves_out[i]
                                             .segments[curves_out[i].next_segment_ids[seg_i]]
                                             .bcs[1]
                                             .head(2)
                                             .transpose()
                                      << std::endl;
                            std::cout << "next[seg_j]: "
                                      << curves_out[j]
                                             .segments[curves_out[j].next_segment_ids[seg_j]]
                                             .bcs[0]
                                             .head(2)
                                             .transpose()
                                      << ", "
                                      << curves_out[j]
                                             .segments[curves_out[j].next_segment_ids[seg_j]]
                                             .bcs[1]
                                             .head(2)
                                             .transpose()

                                      << std::endl;
                            std::cout << std::endl;
                        }
                    }
                }
                std::cout << "curve " << i << " and curve " << j << " intersect " << intersect_count
                          << " times" << std::endl;
            }
            std::cout << std::endl;
        }
    };
    std::cout << "count curve_in intersection" << std::endl;
    count_curve_intersection(curves_in);
    std::cout << "count curve_out intersection" << std::endl;
    count_curve_intersection(curves_out);
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

    // options for texture transfer application
    path input_obj_file;
    path input_texture_file;
    app.add_option("--input_obj", input_obj_file, "Input obj file");
    app.add_option("--input_texture", input_texture_file, "Input texture file");
    int height_out = 200;
    int width_out = 200;
    app.add_option("--height_out", height_out, "Height of the output image");
    app.add_option("--width_out", width_out, "Width of the output image");


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
        int width_in = R.rows();
        int height_in = R.cols();
        std::cout << "height: " << height_in << ", width: " << width_in << std::endl;

        int height = height_out;
        int width = width_out;

        Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R_out, G_out, B_out, A_out;
        R_out.resize(height, width);
        G_out.resize(height, width);
        B_out.resize(height, width);
        A_out.resize(height, width);

        igl::Timer timer;
        timer.start();
        // use igl::AABB to make this part faster
        auto t0 = timer.getElapsedTime();
        std::vector<query_point> query_points;
        {
            igl::AABB<Eigen::MatrixXd, 2> tree;
            tree.init(Vt_out, Ft_out);
            Eigen::MatrixXd Q;
            Q.resize(height * width, 2);
            for (int y = 0; y < height; ++y) {
                for (int x = 0; x < width; ++x) {
                    double u = double(x) / (width - 1);
                    double v = double(y) / (height - 1);
                    Q.row(y * width + x) << u, v;
                }
            }
            Eigen::VectorXi I;
            igl::in_element(Vt_out, Ft_out, Q, tree, I);

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
            // for each pixel in the texture image
            for (int y = 0; y < height; ++y) {
                if (y % 100 == 0) std::cout << "processing row " << y << std::endl;
                for (int x = 0; x < width; ++x) {
                    double u = double(x) / (width - 1);
                    double v = double(y) / (height - 1);

                    bool found_intersect = (I(y * width + x) != -1);

                    if (found_intersect) {
                        int fid = I(y * width + x);
                        query_point qp;
                        qp.f_id = fid;
                        qp.fv_ids = F_out.row(fid);
                        auto [inside, a, b, c] = isPointInTriangle(
                            u,
                            v,
                            Vt_out(Ft_out(fid, 0), 0),
                            Vt_out(Ft_out(fid, 0), 1),
                            Vt_out(Ft_out(fid, 1), 0),
                            Vt_out(Ft_out(fid, 1), 1),
                            Vt_out(Ft_out(fid, 2), 0),
                            Vt_out(Ft_out(fid, 2), 1));
                        if (!inside) {
                            std::cout << "not inside, computation problem" << std::endl;
                        }
                        qp.bc = Eigen::Vector3d(a, b, c);
                        query_points.push_back(qp);
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
        }
        auto t1 = timer.getElapsedTime();
        std::cout << "find query points time: " << t1 - t0 << " s" << std::endl;
        std::cout << "done finding all query points" << std::endl;

        back_track_map(operation_logs_dir, query_points);

        auto t2 = timer.getElapsedTime();
        std::cout << "back track time: " << t2 - t1 << " s" << std::endl;

        igl::parallel_for(height * width, [&](int id) {
            if (query_points[id].f_id != -1) {
                int f_id = query_points[id].f_id;
                Eigen::Vector2d p(0, 0);

                int offset = 0;
                for (int i = 0; i < 3; i++) {
                    if (F_in_obj(f_id, i) == query_points[id].fv_ids[0]) {
                        offset = i;
                        break;
                    }
                }

                for (int i = 0; i < 3; i++) {
                    p += Vt_in_obj.row(Ft_in_obj(f_id, (i + offset) % 3)) * query_points[id].bc(i);
                }
                int x = int(p(0) * (width_in - 1));
                int y = int(p(1) * (height_in - 1));
                R_out(id / width, id % width) = R(x, y);
                G_out(id / width, id % width) = G(x, y);
                B_out(id / width, id % width) = B(x, y);
                A_out(id / width, id % width) = A(x, y);
            } else {
                A_out(id / width, id % width) = 0;
            }
        });
        // write the output image
        igl::stb::write_image(
            "output_texture.png",
            R_out.transpose(),
            G_out.transpose(),
            B_out.transpose(),
            A_out.transpose());
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
        back_track_app(V_in, F_in, V_out, F_out, operation_logs_dir, true);
    } else if (application_name == "back_lines_r") {
        back_track_line_app(V_in, F_in, V_out, F_out, operation_logs_dir, true);
    } else if (application_name == "iso_lines") {
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
        forward_track_line_app(V_in, F_in, Vt_in_obj, Ft_in_obj, V_out, F_out, operation_logs_dir);
    } else if (application_name == "check_iso_lines") {
        // Eigen::MatrixXd V_in_obj, Vt_in_obj, Vn_in_obj;
        // Eigen::MatrixXi F_in_obj, Ft_in_obj, Fn_in_obj;
        // std::cout << "\nloading input obj file..." << std::endl;
        // igl::readOBJ(
        //     input_obj_file.string(),
        //     V_in_obj,
        //     Vt_in_obj,
        //     Vn_in_obj,
        //     F_in_obj,
        //     Ft_in_obj,
        //     Fn_in_obj);
        std::vector<query_curve> curves_in = load_query_curves("curves.in");
        std::vector<query_curve> curves_out = load_query_curves("curves.out");
        check_iso_lines(V_in, F_in, V_out, F_out, curves_in, curves_out);
    }
    return 0;
}