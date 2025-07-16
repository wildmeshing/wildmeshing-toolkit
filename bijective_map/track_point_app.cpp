#include "track_point_app.hpp"
#include <igl/Timer.h>
#include <igl/in_element.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/parallel_for.h>
#include <igl/stb/read_image.h>
#include <igl/stb/write_image.h>
#include "render_utils.hpp"

void track_point_one_operation(
    const json& operation_log,
    std::vector<query_point>& query_points,
    bool do_forward,
    bool use_rational)
{
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
            return;
        }

        if (do_forward) {
            handle_non_collapse_operation(
                V_after,
                F_after,
                id_map_after,
                v_id_map_after,
                V_before,
                F_before,
                id_map_before,
                v_id_map_before,
                query_points,
                operation_name,
                100000);
        } else {
            handle_non_collapse_operation(
                V_before,
                F_before,
                id_map_before,
                v_id_map_before,
                V_after,
                F_after,
                id_map_after,
                v_id_map_after,
                query_points,
                operation_name,
                100000);
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

        if (do_forward) {
            handle_non_collapse_operation(
                V_after,
                F_after,
                id_map_after,
                v_id_map_after,
                V_before,
                F_before,
                id_map_before,
                v_id_map_before,
                query_points,
                "EdgeSplit",
                1e-3);
        } else {
            handle_non_collapse_operation(
                V_before,
                F_before,
                id_map_before,
                v_id_map_before,
                V_after,
                F_after,
                id_map_after,
                v_id_map_after,
                query_points,
                "EdgeSplit",
                1e-3);
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
        std::cout << "This Operations is not implemented" << std::endl;
    }
}

void track_point(
    path dirPath,
    std::vector<query_point>& query_points,
    bool do_forward,
    bool use_rational)
{
    // get all the operation logs
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

        track_point_one_operation(operation_log, query_points, do_forward, use_rational);

        file.close();
    }
}

void back_track_point_app(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Eigen::MatrixXd& V_out,
    const Eigen::MatrixXi& F_out,
    const path& operation_logs_dir,
    bool use_rational)
{
    // init query points
    std::vector<query_point> query_points;
    for (int i = 0; i < F_out.rows(); i++) {
        query_point qp;
        qp.f_id = i;
        qp.bc = Eigen::Vector3d(1.0 / 3, 1.0 / 3, 1.0 / 3);
        qp.fv_ids = F_out.row(i);
        query_points.push_back(qp);
    }

    // generate points on surface
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
    track_point(operation_logs_dir, query_points, false, use_rational);

    // generate points on surface after back track
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

void forward_track_point_app(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Eigen::MatrixXd& V_out,
    const Eigen::MatrixXi& F_out,
    const path& operation_logs_dir,
    bool use_rational)
{
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

    // do forward track
    track_point(operation_logs_dir, query_points, true, use_rational);

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

void render_index_app(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Eigen::MatrixXd& V_out,
    const Eigen::MatrixXi& F_out,
    const path& operation_logs_dir,
    int W,
    int H)
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

        // auto [fids, bcs] = get_pt_mat(cam, V_out, F_out, W, H);
        auto result = get_pt_mat(cam, V_out, F_out, W, H);
        auto& fids = std::get<0>(result);
        auto& bcs = std::get<1>(result);

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
        track_point(operation_logs_dir, query_points);
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
    writePNG("iso_out", W, H, camera);
}

void transfer_texture_app(
    const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& R,
    const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& G,
    const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& B,
    const Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& A,
    const Eigen::MatrixXi& F_in_obj,
    const Eigen::MatrixXd& Vt_in_obj,
    const Eigen::MatrixXi& Ft_in_obj,
    const Eigen::MatrixXd& Vt_out,
    const Eigen::MatrixXi& Ft_out,
    const Eigen::MatrixXi& F_out,
    const path& operation_logs_dir,
    int width_out,
    int height_out)
{
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

    track_point(operation_logs_dir, query_points);

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