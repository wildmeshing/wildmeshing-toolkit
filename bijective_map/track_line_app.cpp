#include "track_line_app.hpp"
#include <igl/Timer.h>
#ifdef USE_IGL_VIEWER
#include <igl/opengl/glfw/Viewer.h>
#endif
#include <igl/parallel_for.h>
#include <wmtk/utils/orient.hpp>
#include "generate_iso_line.hpp"
#include "generate_plane_curves.hpp"
#include "vtu_utils.hpp"

bool validate_and_convert_curves_to_edge_mesh(
    const std::vector<query_curve>& curves,
    const Eigen::MatrixXd& V,
    Eigen::MatrixXd& edge_V,
    Eigen::MatrixXi& edge_E,
    double tolerance = 1e-8)
{
    if (curves.empty()) {
        std::cerr << "No curves provided" << std::endl;
        return false;
    }

    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector2i> edges;

    for (size_t curve_id = 0; curve_id < curves.size(); ++curve_id) {
        const query_curve& curve = curves[curve_id];

        if (curve.segments.empty()) {
            std::cout << "Warning: Curve " << curve_id << " has no segments" << std::endl;
            continue;
        }

        std::cout << "Processing curve " << curve_id << " with " << curve.segments.size()
                  << " segments" << std::endl;

        // Track which segments we've visited
        std::vector<bool> visited(curve.segments.size(), false);
        std::vector<int> curve_vertex_indices;

        // Start from segment 0
        int current_seg_id = 0;
        int segments_processed = 0;

        {
            // Find the id that is not existed in the next_segment_ids
            std::vector<bool> next_segment_ids_exist(curve.segments.size(), false);
            int cnt = 0;
            for (int i = 0; i < curve.next_segment_ids.size(); i++) {
                if (curve.next_segment_ids[i] != -1) {
                    next_segment_ids_exist[curve.next_segment_ids[i]] = true;
                    cnt++;
                }
            }

            if (cnt != curve.next_segment_ids.size()) {
                for (int i = 0; i < curve.next_segment_ids.size(); i++) {
                    if (!next_segment_ids_exist[i]) {
                        current_seg_id = i;
                        break;
                    }
                }
            }
        }

        std::cout << "start seg id: " << current_seg_id << std::endl;

        while (current_seg_id != -1 && current_seg_id < (int)curve.segments.size()) {
            if (visited[current_seg_id]) {
                break;
            }

            visited[current_seg_id] = true;
            segments_processed++;

            const query_segment& seg = curve.segments[current_seg_id];

            // Compute world coordinates for segment endpoints using barycentric coordinates
            Eigen::Vector3d p0 = seg.bcs[0](0) * V.row(seg.fv_ids[0]) +
                                 seg.bcs[0](1) * V.row(seg.fv_ids[1]) +
                                 seg.bcs[0](2) * V.row(seg.fv_ids[2]);

            Eigen::Vector3d p1 = seg.bcs[1](0) * V.row(seg.fv_ids[0]) +
                                 seg.bcs[1](1) * V.row(seg.fv_ids[1]) +
                                 seg.bcs[1](2) * V.row(seg.fv_ids[2]);

            // Add vertices and check connectivity
            int start_vertex_idx, end_vertex_idx;

            if (segments_processed == 1) {
                // First segment: add both vertices
                start_vertex_idx = vertices.size();
                vertices.push_back(p0);
                curve_vertex_indices.push_back(start_vertex_idx);

                end_vertex_idx = vertices.size();
                vertices.push_back(p1);
                curve_vertex_indices.push_back(end_vertex_idx);
            } else {
                // Check if p0 matches the end of previous segment
                Eigen::Vector3d prev_end = vertices[curve_vertex_indices.back()];
                double dist = (p0 - prev_end).norm();

                if (dist > tolerance) {
                    std::cout << "Warning: Curve " << curve_id << ", segment " << current_seg_id
                              << " start point does not connect to previous segment end."
                              << std::endl;
                    std::cout << "Distance: " << dist << " (tolerance: " << tolerance << ")"
                              << std::endl;
                    std::cout << "Previous end: " << prev_end.transpose() << std::endl;
                    std::cout << "Current start: " << p0.transpose() << std::endl;

                    // Add the new start point anyway
                    start_vertex_idx = vertices.size();
                    vertices.push_back(p0);
                    curve_vertex_indices.push_back(start_vertex_idx);
                } else {
                    // Use previous end point (connected)
                    start_vertex_idx = curve_vertex_indices.back();
                }

                // Add end vertex
                end_vertex_idx = vertices.size();
                vertices.push_back(p1);
                curve_vertex_indices.push_back(end_vertex_idx);
            }

            // Add edge
            edges.push_back(Eigen::Vector2i(start_vertex_idx, end_vertex_idx));

            // std::cout << "  Segment " << current_seg_id << ": vertices [" << start_vertex_idx
            //           << ", " << end_vertex_idx << "] at (" << p0.transpose() << ") -> ("
            //           << p1.transpose() << ")" << std::endl;

            // Get next segment ID
            if (current_seg_id < (int)curve.next_segment_ids.size()) {
                current_seg_id = curve.next_segment_ids[current_seg_id];
                // std::cout << "  Next segment ID: " << current_seg_id << std::endl;
            } else {
                std::cout << "  No next_segment_ids entry for segment " << current_seg_id
                          << std::endl;
                break;
            }
        }

        // Check if we processed all segments
        if (segments_processed == (int)curve.segments.size()) {
            std::cout << "  Successfully processed all " << segments_processed << " segments"
                      << std::endl;
        } else {
            std::cout << "  Warning: Only processed " << segments_processed << " out of "
                      << curve.segments.size() << " segments" << std::endl;

            // Check which segments were not visited
            for (size_t i = 0; i < visited.size(); ++i) {
                if (!visited[i]) {
                    std::cout << "    Segment " << i << " was not visited" << std::endl;
                }
            }
        }

        // Check if curve is closed (last segment connects to first)
        if (curve_vertex_indices.size() >= 4) { // At least 2 segments
            int first_vertex = curve_vertex_indices[0];
            int last_vertex = curve_vertex_indices.back();
            double closure_dist = (vertices[first_vertex] - vertices[last_vertex]).norm();

            if (closure_dist <= tolerance) {
                std::cout << "  Curve " << curve_id << " is closed (distance: " << closure_dist
                          << ")" << std::endl;
            } else {
                std::cout << "  Curve " << curve_id
                          << " is open (end-to-start distance: " << closure_dist << ")"
                          << std::endl;
            }
        }

        std::cout << "  Total vertices for curve " << curve_id << ": "
                  << curve_vertex_indices.size() << std::endl;
    }

    // Convert to Eigen matrices
    edge_V.resize(vertices.size(), 3);
    for (size_t i = 0; i < vertices.size(); ++i) {
        edge_V.row(i) = vertices[i].transpose();
    }

    edge_E.resize(edges.size(), 2);
    for (size_t i = 0; i < edges.size(); ++i) {
        edge_E.row(i) = edges[i].transpose();
    }

    std::cout << "Total edge mesh: " << edge_V.rows() << " vertices, " << edge_E.rows() << " edges"
              << std::endl;
    return true;
}

void write_curves_to_vtu(
    const std::vector<query_curve>& curves,
    const Eigen::MatrixXd& V,
    const std::string& filename,
    double tolerance = 1e-8)
{
    Eigen::MatrixXd edge_V;
    Eigen::MatrixXi edge_E;

    if (validate_and_convert_curves_to_edge_mesh(curves, V, edge_V, edge_E, tolerance)) {
        vtu_utils::write_edge_mesh_to_vtu(edge_V, edge_E, filename);
        std::cout << "Successfully wrote curves to " << filename << std::endl;
    } else {
        std::cerr << "Failed to convert curves to edge mesh" << std::endl;
    }
}

template <typename CoordType>
void track_line_one_operation(
    const json& operation_log,
    query_curve_t<CoordType>& curve,
    bool do_forward)
{
    bool verbose = false;
    std::string operation_name;
    operation_name = operation_log["operation_name"];
    igl::Timer op_total_timer;
    op_total_timer.start();
    double t_parse_ms = 0.0;
    double t_handle_ms = 0.0;

    if (operation_name == "MeshConsolidate") {
        // std::cout << "This Operations is Consolidate" << std::endl;
        std::vector<int64_t> face_ids_maps;
        std::vector<int64_t> vertex_ids_maps;
        {
            igl::Timer parse_timer;
            parse_timer.start();
            parse_consolidate_file(operation_log, face_ids_maps, vertex_ids_maps);
            t_parse_ms += parse_timer.getElapsedTime() * 1000.0;
        }
        if (do_forward) {
            igl::Timer handle_timer;
            handle_timer.start();
            handle_consolidate_forward(face_ids_maps, vertex_ids_maps, curve);
            t_handle_ms += handle_timer.getElapsedTime() * 1000.0;
        } else {
            igl::Timer handle_timer;
            handle_timer.start();
            handle_consolidate(face_ids_maps, vertex_ids_maps, curve);
            t_handle_ms += handle_timer.getElapsedTime() * 1000.0;
        }
    } else if (
        operation_name == "TriEdgeSwap" || operation_name == "AttributesUpdate" ||
        operation_name == "EdgeSplit") {
        // std::cout << "This Operations is" << operation_name << std::endl;
        Eigen::MatrixXi F_after, F_before;
        Eigen::MatrixXd V_after, V_before;
        std::vector<int64_t> id_map_after, id_map_before;
        std::vector<int64_t> v_id_map_after, v_id_map_before;
        bool is_skipped;

        {
            igl::Timer parse_timer;
            parse_timer.start();
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
            t_parse_ms += parse_timer.getElapsedTime() * 1000.0;
        }

        if (is_skipped) {
            return;
        }

        if (do_forward) {
            igl::Timer handle_timer;
            handle_timer.start();
            handle_non_collapse_operation_curve_t(
                V_after,
                F_after,
                id_map_after,
                v_id_map_after,
                V_before,
                F_before,
                id_map_before,
                v_id_map_before,
                curve,
                operation_name,
                verbose);
            t_handle_ms += handle_timer.getElapsedTime() * 1000.0;
        } else {
            igl::Timer handle_timer;
            handle_timer.start();
            handle_non_collapse_operation_curve_t(
                V_before,
                F_before,
                id_map_before,
                v_id_map_before,
                V_after,
                F_after,
                id_map_after,
                v_id_map_after,
                curve,
                operation_name,
                verbose);
            t_handle_ms += handle_timer.getElapsedTime() * 1000.0;
        }
    } else if (operation_name == "EdgeCollapse") {
        Eigen::MatrixXi F_after, F_before;
        Eigen::MatrixXd UV_joint;
        std::vector<int64_t> v_id_map_joint;
        std::vector<int64_t> id_map_after, id_map_before;
        {
            igl::Timer parse_timer;
            parse_timer.start();
            parse_edge_collapse_file(
                operation_log,
                UV_joint,
                F_before,
                F_after,
                v_id_map_joint,
                id_map_before,
                id_map_after);
            t_parse_ms += parse_timer.getElapsedTime() * 1000.0;
        }

        if (do_forward) {
            igl::Timer handle_timer;
            handle_timer.start();
            handle_collapse_edge_curve_t(
                UV_joint,
                F_after,
                F_before,
                v_id_map_joint,
                id_map_after,
                id_map_before,
                curve,
                true,
                verbose);
            t_handle_ms += handle_timer.getElapsedTime() * 1000.0;
        } else {
            igl::Timer handle_timer;
            handle_timer.start();
            handle_collapse_edge_curve_t(
                UV_joint,
                F_before,
                F_after,
                v_id_map_joint,
                id_map_before,
                id_map_after,
                curve,
                true,
                verbose);
            t_handle_ms += handle_timer.getElapsedTime() * 1000.0;
        }
    } else {
        // std::cout << "This Operations is not implemented" << std::endl;
    }
#ifdef DEBUG_CURVES
    // save the curve
    std::string curve_file = "./curve_debug/curve_" + std::to_string(file_id) + ".json";
    std::vector<query_curve> curves{curve};
    save_query_curves(curves, curve_file);
#endif
    // Only print overall time per operation for this curve
    // double t_total_ms = op_total_timer.getElapsedTime() * 1000.0;
    // std::cout << "operation total time (" << operation_name << ", curve): " << t_total_ms << "
    // ms"
    //           << std::endl;
}

template <typename CoordType>
void track_line(path dirPath, query_curve_t<CoordType>& curve, bool do_forward)
{
    namespace fs = std::filesystem;
    int maxIndex = -1;

    for (const auto& entry : fs::directory_iterator(dirPath)) {
        if (entry.path().filename().string().find("operation_log_") != std::string::npos) {
            ++maxIndex;
        }
    }
    int curve_old_size = curve.segments.size();
    int clean_up_threshold = 2;
    for (int i = maxIndex; i >= 0; --i) {
        int file_id = i;
        if (do_forward) {
            file_id = maxIndex - i;
        }

        //
        fs::path filePath = dirPath / ("operation_log_" + std::to_string(file_id) + ".json");
        std::ifstream file(filePath);
        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filePath << std::endl;
            continue;
        }
        json operation_log;
        file >> operation_log;

        int curve_size_before = curve.segments.size();
        // quiet per-operation tracing at track_line level
        igl::Timer timer;
        timer.start();
        track_line_one_operation<CoordType>(operation_log, curve, do_forward);
        timer.stop();

        if (curve.segments.size() > clean_up_threshold * curve_old_size) {
            clean_up_curve_t(curve);
            curve_old_size = curve.segments.size();
            {
                int self_intersections = compute_curve_self_intersections_t(curve, true);
                if (self_intersections != 0) {
                    std::cout << "Error: self intersections are not correct" << std::endl;
                    throw std::runtime_error("Error: self intersections are not correct");
                }
            }
        }

        file.close();
    }

    clean_up_curve_t(curve);
    std::cout << "finished" << std::endl;
}

template <typename CoordType>
void track_lines(
    path dirPath,
    std::vector<query_curve_t<CoordType>>& curves,
    bool do_forward,
    bool do_parallel)
{
    // use igl parallel_for
    if (do_parallel) {
        std::cout << "parallel mode" << std::endl;
        igl::parallel_for(curves.size(), [&](int i) {
            track_line<CoordType>(dirPath, curves[i], do_forward);
        });
    } else {
        std::cout << "sequential mode" << std::endl;
        for (int i = 0; i < curves.size(); i++) {
            track_line<CoordType>(dirPath, curves[i], do_forward);
        }
    }
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
        seg.origin_segment_id = curve.segments.size(); // Set origin_segment_id
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

void back_track_one_curve_app(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Eigen::MatrixXd& V_out,
    const Eigen::MatrixXi& F_out,
    const path& operation_logs_dir)
{
    query_curve curve_in = generate_curve(V_out, F_out, 10, 0);
    for (const auto& seg : curve_in.segments) {
        std::cout << "f_id: " << seg.f_id << std::endl;
        std::cout << "bcs: " << seg.bcs[0] << ", " << seg.bcs[1] << std::endl;
    }

    query_curve curve = curve_in;
    query_curve curve_origin = curve;
    track_line(operation_logs_dir, curve);

    // render before
#ifdef USE_IGL_VIEWER
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

    // render after
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
#endif
}

void forward_track_iso_lines_app(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Eigen::MatrixXd& uv_in,
    const Eigen::MatrixXi& Fuv_in,
    const Eigen::MatrixXd& V_out,
    const Eigen::MatrixXi& F_out,
    const path& operation_logs_dir,
    int N,
    bool do_parallel,
    const std::string& model_name)
{
    auto to_three_cols = [](const Eigen::MatrixXd& V) {
        if (V.cols() == 2) {
            Eigen::MatrixXd V_temp(V.rows(), 3);
            V_temp << V, Eigen::VectorXd::Zero(V.rows());
            return V_temp;
        } else {
            return V;
        }
    };
    Eigen::MatrixXd V_in_3d = to_three_cols(V_in);
    Eigen::MatrixXd V_out_3d = to_three_cols(V_out);


    // get all curves
    std::vector<query_curve> curves;
    {
        auto curve_from_intersections = [&](const std::vector<Intersection>& input_intersections) {
            query_curve curve;
            for (int i = 0; i < input_intersections.size(); i += 2) {
                query_segment seg;
                if (input_intersections[i].fid != input_intersections[i + 1].fid) {
                    std::cout << "something wrong with input_intersections" << std::endl;
                }
                seg.f_id = input_intersections[i].fid;
                seg.origin_segment_id = curve.segments.size(); // Set origin_segment_id
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

        // Compute the min and max for x and y in uv_in
        double min_x = uv_in.col(0).minCoeff();
        double max_x = uv_in.col(0).maxCoeff();
        double min_y = uv_in.col(1).minCoeff();
        double max_y = uv_in.col(1).maxCoeff();
        double scale_x = max_x - min_x;
        double scale_y = max_y - min_y;
        for (int k = 0; k < N - 1; k++) {
            double value_x = min_x + scale_x / N * (k + 1);
            double value_y = min_y + scale_y / N * (k + 1);
            auto intersectionsX = computeIsoLineIntersectionsX(uv_in, Fuv_in, value_x);
            auto curveX = curve_from_intersections(intersectionsX);
            curves.push_back(curveX);
            auto intersectionsY = computeIsoLineIntersectionsY(uv_in, Fuv_in, value_y);
            auto curveY = curve_from_intersections(intersectionsY);
            curves.push_back(curveY);
        }
    }

    auto curves_origin = curves;

    if (false) {
#ifdef USE_IGL_VIEWER
        igl::opengl::glfw::Viewer viewer;
        viewer.data().set_mesh(V_in, F_in);
        viewer.data().point_size /= 3;
        for (const auto curve_origin : curves_origin) {
            for (const auto& seg : curve_origin.segments) {
                Eigen::MatrixXd pts(2, 3);
                for (int i = 0; i < 2; i++) {
                    Eigen::Vector3d p(0, 0, 0);
                    for (int j = 0; j < 3; j++) {
                        p += V_in_3d.row(seg.fv_ids[j]) * seg.bcs[i](j);
                    }
                    pts.row(i) = p;
                }
                viewer.data().add_points(pts.row(0), Eigen::RowVector3d(1, 0, 0));
                viewer.data().add_points(pts.row(1), Eigen::RowVector3d(1, 0, 0));
                viewer.data().add_edges(pts.row(0), pts.row(1), Eigen::RowVector3d(1, 0, 0));
            }
        }
        viewer.launch();
#endif
    }

    save_query_curves(curves, model_name + "_curves.in");
    // write curves to vtu
    {
        std::cout << "\n=== Writing initial curves to VTU ===" << std::endl;
        write_curves_to_vtu(curves, V_in, model_name + "_curves_in.vtu");
    }

    // TODO: get inference
    std::vector<std::vector<int>> intersection_reference;
    intersection_reference.resize(curves.size());
    for (int i = 0; i < curves.size(); i++) {
        intersection_reference[i].resize(curves.size());
        intersection_reference[i][i] = compute_curve_self_intersections(curves[i]);
        for (int j = i + 1; j < curves.size(); j++) {
            intersection_reference[i][j] =
                compute_intersections_between_two_curves(curves[i], curves[j]);
        }
    }

    track_lines(operation_logs_dir, curves, true, do_parallel);


    save_query_curves(curves, model_name + "_curves.out");

    // write final curves to vtu
    {
        std::cout << "\n=== Writing final curves to VTU ===" << std::endl;
        write_curves_to_vtu(curves, V_out, model_name + "_curves_out.vtu");
    }

    check_curves_topology(curves, intersection_reference);

#ifdef USE_IGL_VIEWER
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
                        p += V_out_3d.row(seg.fv_ids[j]) * seg.bcs[i](j);
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
#endif
}


void forward_track_plane_curves_app(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Eigen::MatrixXd& V_out,
    const Eigen::MatrixXi& F_out,
    const path& operation_logs_dir,
    int N,
    bool do_parallel,
    const std::string& model_name)
{
    if (V_in.cols() == 2) {
        throw std::runtime_error("V_in is 2D, not supported");
    }

    auto to_three_cols = [](const Eigen::MatrixXd& V) {
        if (V.cols() == 2) {
            Eigen::MatrixXd V_temp(V.rows(), 3);
            V_temp << V, Eigen::VectorXd::Zero(V.rows());
            return V_temp;
        } else {
            return V;
        }
    };
    Eigen::MatrixXd V_in_3d = to_three_cols(V_in);
    Eigen::MatrixXd V_out_3d = to_three_cols(V_out);

    // Get all curves using the plane curve generation
    std::vector<query_curve> curves_double = generatePlaneCurves(V_in, F_in, N);
    auto curves_origin = curves_double;

    // Convert curves to rational for exact arithmetic
    std::vector<query_curve_t<wmtk::Rational>> curves;
    curves.reserve(curves_double.size());
    for (const auto& curve_double : curves_double) {
        curves.push_back(convert_query_curve<double, wmtk::Rational>(curve_double));
    }

    if (false) {
#ifdef USE_IGL_VIEWER
        igl::opengl::glfw::Viewer viewer;
        viewer.data().set_mesh(V_in, F_in);
        viewer.data().point_size /= 3;
        for (const auto curve_origin : curves_origin) {
            for (const auto& seg : curve_origin.segments) {
                Eigen::MatrixXd pts(2, 3);
                for (int i = 0; i < 2; i++) {
                    Eigen::Vector3d p(0, 0, 0);
                    for (int j = 0; j < 3; j++) {
                        p += V_in_3d.row(seg.fv_ids[j]) * seg.bcs[i](j);
                    }
                    pts.row(i) = p;
                }
                viewer.data().add_points(pts.row(0), Eigen::RowVector3d(1, 0, 0));
                viewer.data().add_points(pts.row(1), Eigen::RowVector3d(1, 0, 0));
                viewer.data().add_edges(pts.row(0), pts.row(1), Eigen::RowVector3d(1, 0, 0));
            }
        }
        viewer.launch();
#endif
    }

    // Save original double curves for backward compatibility
    save_query_curves(curves_double, model_name + "_plane_curves.in");
    // write curves to vtu
    {
        std::cout << "\n=== Writing initial plane curves to VTU ===" << std::endl;
        write_curves_to_vtu(curves_double, V_in, model_name + "_plane_curves_in.vtu");
    }

    std::vector<std::vector<int>> intersection_reference;
    intersection_reference.resize(curves.size());
    for (int i = 0; i < curves.size(); i++) {
        intersection_reference[i].resize(curves.size());
        intersection_reference[i][i] = compute_curve_self_intersections_t(curves[i], true);
        for (int j = i + 1; j < curves.size(); j++) {
            intersection_reference[i][j] =
                compute_intersections_between_two_curves_t(curves[i], curves[j], true);
        }
    }

    track_lines<wmtk::Rational>(operation_logs_dir, curves, true, do_parallel);

    std::cout << "finished track lines" << std::endl;

    // Convert back to double for saving
    std::vector<query_curve> curves_result_double;
    curves_result_double.reserve(curves.size());
    for (const auto& curve_rational : curves) {
        curves_result_double.push_back(convert_query_curve<wmtk::Rational, double>(curve_rational));
    }
    save_query_curves(curves_result_double, model_name + "_plane_curves.out");
    std::cout << "finished save query curves" << std::endl;

    // write final curves to vtu
    {
        std::cout << "\n=== Writing final plane curves to VTU ===" << std::endl;
        write_curves_to_vtu(curves_result_double, V_out, model_name + "_plane_curves_out.vtu");
    }
    std::cout << "finished write curves to vtu" << std::endl;

    // Use rational curves for topology checking for better precision
    bool is_correct = check_curves_topology<wmtk::Rational>(curves, intersection_reference);
    if (!is_correct) {
        std::cout << "Error: curves topology is not correct" << std::endl;
    } else {
        std::cout << "curves topology is correct" << std::endl;
    }
    std::cout << "finished check curves topology" << std::endl;

    if (false) {
#ifdef USE_IGL_VIEWER
        igl::opengl::glfw::Viewer viewer;
        viewer.data().set_mesh(V_out, F_out);
        viewer.data().point_size /= 3;
        for (const auto& curve : curves_result_double) {
            for (const auto& seg : curve.segments) {
                Eigen::MatrixXd pts(2, 3);
                for (int i = 0; i < 2; i++) {
                    Eigen::Vector3d p(0, 0, 0);
                    for (int j = 0; j < 3; j++) {
                        p += V_out_3d.row(seg.fv_ids[j]) * seg.bcs[i](j);
                    }
                    pts.row(i) = p;
                }
                viewer.data().add_points(pts.row(0), Eigen::RowVector3d(1, 0, 0));
                viewer.data().add_points(pts.row(1), Eigen::RowVector3d(1, 0, 0));
                viewer.data().add_edges(pts.row(0), pts.row(1), Eigen::RowVector3d(1, 0, 0));
            }
        }
        viewer.launch();
#endif
    }
}


template <typename CoordType>
bool check_curves_topology(
    const std::vector<query_curve_t<CoordType>>& curves,
    const std::vector<std::vector<int>>& intersection_reference)
{
    bool is_correct = true;
    for (int i = 0; i < curves.size(); i++) {
        int self_intersections = compute_curve_self_intersections_t(curves[i], true);
        std::cout << "curve " << i << " has " << self_intersections << " self intersections"
                  << std::endl;
        if (self_intersections != intersection_reference[i][i]) {
            std::cout << "Error: self intersections are not correct" << std::endl;
            std::cout << "expected: " << intersection_reference[i][i] << std::endl;
            is_correct = false;
        }
        for (int j = i + 1; j < curves.size(); j++) {
            int intersections =
                compute_intersections_between_two_curves_t(curves[i], curves[j], true);
            std::cout << "curve " << i << " and curve " << j << " intersect " << intersections
                      << " times" << std::endl;
            if (intersections != intersection_reference[i][j]) {
                std::cout << "Error: intersections between curve " << i << " and curve " << j
                          << " are not correct" << std::endl;
                std::cout << "expected: " << intersection_reference[i][j] << std::endl;
                is_correct = false;
            } else {
                std::cout << "intersections between curve " << i << " and curve " << j
                          << " are correct" << std::endl;
            }
        }
    }
    return is_correct;
}

// Backward compatibility version
bool check_curves_topology(
    const std::vector<query_curve>& curves,
    const std::vector<std::vector<int>>& intersection_reference)
{
    return check_curves_topology<double>(curves, intersection_reference);
}

// Explicit template instantiation for compilation
template void track_line_one_operation<double>(
    const json& operation_log,
    query_curve_t<double>& curve,
    bool do_forward);
template void track_line_one_operation<wmtk::Rational>(
    const json& operation_log,
    query_curve_t<wmtk::Rational>& curve,
    bool do_forward);

template void track_line<double>(path dirPath, query_curve_t<double>& curve, bool do_forward);
template void
track_line<wmtk::Rational>(path dirPath, query_curve_t<wmtk::Rational>& curve, bool do_forward);

template void track_lines<double>(
    path dirPath,
    std::vector<query_curve_t<double>>& curves,
    bool do_forward,
    bool do_parallel);
template void track_lines<wmtk::Rational>(
    path dirPath,
    std::vector<query_curve_t<wmtk::Rational>>& curves,
    bool do_forward,
    bool do_parallel);

template bool check_curves_topology<double>(
    const std::vector<query_curve_t<double>>& curves,
    const std::vector<std::vector<int>>& intersection_reference);
template bool check_curves_topology<wmtk::Rational>(
    const std::vector<query_curve_t<wmtk::Rational>>& curves,
    const std::vector<std::vector<int>>& intersection_reference);

// Additional backward compatibility versions for query_curve_t<double>
void track_line(path dirPath, query_curve_t<double>& curve, bool do_forward)
{
    return track_line<double>(dirPath, curve, do_forward);
}

void track_lines(
    path dirPath,
    std::vector<query_curve_t<double>>& curves,
    bool do_forward,
    bool do_parallel)
{
    return track_lines<double>(dirPath, curves, do_forward, do_parallel);
}


void check_iso_lines(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Eigen::MatrixXd& V_out,
    const Eigen::MatrixXi& F_out,
    const std::vector<query_curve>& curves_in,
    const std::vector<query_curve>& curves_out,
    bool render_before,
    bool render_after)
{
    std::cout << "curves_in sizes:\n";
    for (const auto& c : curves_in) {
        std::cout << c.segments.size() << std::endl;
    }
    std::cout << "curves_out sizes:\n";
    for (const auto& c : curves_out) {
        std::cout << c.segments.size() << std::endl;
    }

    if (render_before) {
#ifdef USE_IGL_VIEWER
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
#endif
    }
    if (render_after) {
#ifdef USE_IGL_VIEWER
        igl::opengl::glfw::Viewer viewer;
        viewer.data().set_mesh(V_out, F_out);
        viewer.data().point_size /= 3;
        for (int i = 0; i < curves_out.size(); i++) {
            const auto& curve = curves_out[i];
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
#endif
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
            for (int j = i; j < curve.size(); j++) {
                // for (int j = i; j < i + 1; j++) {
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
                            // std::cout << "i = " << i << ", seg_i = " << seg_i
                            //           << ", next[seg_i] = " <<
                            //           curves_out[i].next_segment_ids[seg_i]
                            //           << std::endl;
                            // std::cout << "p1: " << p1 << std::endl;
                            // std::cout << "q1: " << q1 << std::endl;
                            // std::cout << "j = " << j << ", seg_j = " << seg_j
                            //           << ", next[seg_j] = " <<
                            //           curves_out[j].next_segment_ids[seg_j]
                            //           << std::endl;
                            // std::cout << "p2: " << p2 << std::endl;
                            // std::cout << "q2: " << q2 << std::endl;

                            // std::cout << "next[seg_i]: "
                            //           << curves_out[i]
                            //                  .segments[curves_out[i].next_segment_ids[seg_i]]
                            //                  .bcs[0]
                            //                  .head(2)
                            //                  .transpose()
                            //           << ", "
                            //           << curves_out[i]
                            //                  .segments[curves_out[i].next_segment_ids[seg_i]]
                            //                  .bcs[1]
                            //                  .head(2)
                            //                  .transpose()
                            //           << std::endl;
                            // std::cout << "next[seg_j]: "
                            //           << curves_out[j]
                            //                  .segments[curves_out[j].next_segment_ids[seg_j]]
                            //                  .bcs[0]
                            //                  .head(2)
                            //                  .transpose()
                            //           << ", "
                            //           << curves_out[j]
                            //                  .segments[curves_out[j].next_segment_ids[seg_j]]
                            //                  .bcs[1]
                            //                  .head(2)
                            //                  .transpose()

                            //           << std::endl;
                            // std::cout << std::endl;
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

#ifdef DEBUG_CURVES
void check_iso_lines_step_by_step(
    const Eigen::MatrixXd& V_in,
    const Eigen::MatrixXi& F_in,
    const Eigen::MatrixXd& V_out,
    const Eigen::MatrixXi& F_out,
    const path& operation_logs_dir)
{
    std::vector<query_curve> curves_in = load_query_curves("curves.in");
    namespace fs = std::filesystem;
    int file_id = -1;
    int step_size = 1;
    int view_mode = 0;
    auto key_down = [&](igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier) {
        if (key == '1') {
            file_id += step_size;
            std::cout << "file_id: " << file_id << std::endl;
        }

        if (key == '0') {
            file_id -= step_size;
            std::cout << "file_id: " << file_id << std::endl;
        }

        if (key == '=') {
            step_size *= 10;
            std::cout << "step_size: " << step_size << std::endl;
        }

        if (key == '-') {
            if (step_size > 1) step_size /= 10;
            std::cout << "step_size: " << step_size << std::endl;
        }

        if (key == ' ') {
            view_mode = 0;
        }

        if (key == '8') {
            view_mode = 1;
        }

        if (key == '9') {
            view_mode = 2;
        }

        if (key == '6') {
            view_mode = 3;
        }

        if (key == '7') {
            view_mode = 4;
        }

        if (file_id >= 0) {
            if (view_mode == 0) {
                // read obj from file
                Eigen::MatrixXd V_cur, Vt_cur, Vn_cur;
                Eigen::MatrixXi F_cur, Ft_cur, Fn_cur;
                fs::path MeshfilePath = operation_logs_dir / ("VF_all_after_operation_" +
                                                              std::to_string(file_id) + ".obj");
                igl::readOBJ(MeshfilePath.string(), V_cur, Vt_cur, Vn_cur, F_cur, Ft_cur, Fn_cur);

                // read F_flag_after_operation_*.txt
                std::vector<int> F_flag;
                fs::path F_flag_path = operation_logs_dir / ("F_flag_after_operation_" +
                                                             std::to_string(file_id) + ".txt");
                std::ifstream file(F_flag_path.string());
                if (file.is_open()) {
                    int flag;
                    while (file >> flag) {
                        F_flag.push_back(flag);
                    }
                    file.close();
                }

                // read curve from file
                std::vector<query_curve> curves_cur =
                    load_query_curves("curve_debug/curve_" + std::to_string(file_id) + ".json");

                // build new F, clean up the ones in F_cur that F_flag is 0
                Eigen::MatrixXi F_new;
                int count = 0;
                for (int i = 0; i < F_cur.rows(); i++) {
                    if (F_flag[i] == 1) {
                        F_new.conservativeResize(count + 1, 3);
                        F_new.row(count) = F_cur.row(i);
                        count++;
                    }
                }

                viewer.data().clear();
                viewer.data().set_mesh(V_cur, F_new);
                viewer.core().align_camera_center(V_in, F_in);
                // viewer.data().point_size /= 3;

                for (const auto& curve : curves_cur) {
                    for (const auto& seg : curve.segments) {
                        Eigen::MatrixXd pts(2, 3);
                        for (int i = 0; i < 2; i++) {
                            Eigen::Vector3d p(0, 0, 0);
                            for (int j = 0; j < 3; j++) {
                                p += V_cur.row(seg.fv_ids[j]) * seg.bcs[i](j);
                            }
                            pts.row(i) = p;
                        }
                        viewer.data().add_points(pts.row(0), Eigen::RowVector3d(1, 0, 0));
                        viewer.data().add_points(pts.row(1), Eigen::RowVector3d(1, 0, 0));
                        viewer.data().add_edges(
                            pts.row(0),
                            pts.row(1),
                            Eigen::RowVector3d(1, 0, 0));
                    }
                }
            } else {
                fs::path filePath =
                    operation_logs_dir / ("operation_log_" + std::to_string(file_id) + ".json");
                std::ifstream file(filePath);
                json operation_log;
                file >> operation_log;

                std::cout << "Trace Operations number: " << file_id << std::endl;
                std::string operation_name;
                operation_name = operation_log["operation_name"];

                Eigen::MatrixXi F_after, F_before;
                Eigen::MatrixXd V_after, V_before;
                std::vector<int64_t> id_map_after, id_map_before;
                std::vector<int64_t> v_id_map_after, v_id_map_before;

                if (operation_name == "TriEdgeSwap" || operation_name == "AttributesUpdate") {
                    std::cout << "This Operations is" << operation_name << std::endl;
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
                } else if (operation_name == "EdgeCollapse") {
                    std::cout << "This Operations is EdgeCollapse" << std::endl;
                    Eigen::MatrixXd UV_joint;
                    std::vector<int64_t> v_id_map_joint;

                    parse_edge_collapse_file(
                        operation_log,
                        UV_joint,
                        F_before,
                        F_after,
                        v_id_map_joint,
                        id_map_before,
                        id_map_after);

                    V_before = UV_joint;
                    V_after = UV_joint;
                    v_id_map_before = v_id_map_joint;
                    v_id_map_after = v_id_map_joint;
                }

                viewer.data().clear();
                if (view_mode == 1) {
                    viewer.data().set_mesh(V_before, F_before);
                    viewer.core().align_camera_center(V_before, F_before);
                    // read curve and draw
                    if (file_id > 0) {
                        std::vector<query_curve> curves_cur = load_query_curves(
                            "curve_debug/curve_" + std::to_string(file_id - 1) + ".json");
                        for (const auto& curve : curves_cur) {
                            for (const auto& seg : curve.segments) {
                                if (std::find(
                                        id_map_before.begin(),
                                        id_map_before.end(),
                                        seg.f_id) == id_map_before.end()) {
                                    continue;
                                }
                                Eigen::MatrixXd pts(2, 2);
                                for (int i = 0; i < 2; i++) {
                                    Eigen::Vector2d p(0, 0);
                                    for (int j = 0; j < 3; j++) {
                                        if (std::find(
                                                v_id_map_before.begin(),
                                                v_id_map_before.end(),
                                                seg.fv_ids[j]) == v_id_map_before.end()) {
                                            std::cout << "not found" << std::endl;
                                        }
                                        int id = std::distance(
                                            v_id_map_before.begin(),
                                            std::find(
                                                v_id_map_before.begin(),
                                                v_id_map_before.end(),
                                                seg.fv_ids[j]));

                                        if (seg.bcs[i](j) < 0 || seg.bcs[i](j) > 1) {
                                            std::cout << "Error: bcs out of range" << std::endl;
                                            std::cout << "seg.bcs[i](j): " << seg.bcs[i](j)
                                                      << std::endl;
                                        }
                                        p += V_before.row(id) * seg.bcs[i](j);
                                    }
                                    pts.row(i) = p;
                                }
                                viewer.data().add_points(pts.row(0), Eigen::RowVector3d(1, 0, 0));
                                viewer.data().add_points(pts.row(1), Eigen::RowVector3d(1, 0, 0));
                                viewer.data().add_edges(
                                    pts.row(0),
                                    pts.row(1),
                                    Eigen::RowVector3d(1, 0, 0));
                            }
                        }
                    }
                } else if (view_mode == 2) {
                    viewer.data().set_mesh(V_after, F_after);
                    viewer.core().align_camera_center(V_before, F_before);
                    // read curve and draw
                    std::vector<query_curve> curves_cur =
                        load_query_curves("curve_debug/curve_" + std::to_string(file_id) + ".json");
                    for (const auto& curve : curves_cur) {
                        for (const auto& seg : curve.segments) {
                            if (std::find(id_map_after.begin(), id_map_after.end(), seg.f_id) ==
                                id_map_after.end()) {
                                continue;
                            }
                            Eigen::MatrixXd pts(2, 2);
                            for (int i = 0; i < 2; i++) {
                                Eigen::Vector2d p(0, 0);
                                for (int j = 0; j < 3; j++) {
                                    if (std::find(
                                            v_id_map_after.begin(),
                                            v_id_map_after.end(),
                                            seg.fv_ids[j]) == v_id_map_after.end()) {
                                        std::cout << "not found" << std::endl;
                                    }
                                    int offset_for_collapse = 0;
                                    if (operation_name == "EdgeCollapse") {
                                        offset_for_collapse = 1;
                                    }
                                    int id = std::distance(
                                        v_id_map_after.begin(),
                                        std::find(
                                            v_id_map_after.begin() + offset_for_collapse,
                                            v_id_map_after.end(),
                                            seg.fv_ids[j]));
                                    if (seg.bcs[i](j) < 0 || seg.bcs[i](j) > 1) {
                                        std::cout << "Error: bcs out of range" << std::endl;
                                        std::cout << "seg.bcs[i](j): " << seg.bcs[i](j)
                                                  << std::endl;
                                    }

                                    p += V_after.row(id) * seg.bcs[i](j);
                                }
                                pts.row(i) = p;
                            }
                            viewer.data().add_points(pts.row(0), Eigen::RowVector3d(1, 0, 0));
                            viewer.data().add_points(pts.row(1), Eigen::RowVector3d(1, 0, 0));
                            viewer.data().add_edges(
                                pts.row(0),
                                pts.row(1),
                                Eigen::RowVector3d(1, 0, 0));
                        }
                    }
                } else if (view_mode == 3) {
                    Eigen::MatrixXd V_cur, Vt_cur, Vn_cur;
                    Eigen::MatrixXi F_cur, Ft_cur, Fn_cur;
                    fs::path MeshfilePath =
                        operation_logs_dir /
                        ("VF_all_after_operation_" + std::to_string(file_id - 1) + ".obj");
                    igl::readOBJ(
                        MeshfilePath.string(),
                        V_cur,
                        Vt_cur,
                        Vn_cur,
                        F_cur,
                        Ft_cur,
                        Fn_cur);
                    Eigen::MatrixXi F_slice(id_map_before.size(), 3);
                    for (int i = 0; i < id_map_before.size(); i++) {
                        F_slice.row(i) = F_cur.row(id_map_before[i]);
                    }

                    viewer.data().set_mesh(V_cur, F_slice);
                    viewer.core().align_camera_center(V_cur, F_slice);
                    // read curve and draw
                    if (file_id > 0) {
                        std::vector<query_curve> curves_cur = load_query_curves(
                            "curve_debug/curve_" + std::to_string(file_id - 1) + ".json");
                        for (const auto& curve : curves_cur) {
                            for (const auto& seg : curve.segments) {
                                if (std::find(
                                        id_map_before.begin(),
                                        id_map_before.end(),
                                        seg.f_id) == id_map_before.end()) {
                                    continue;
                                }
                                Eigen::MatrixXd pts(2, 3);
                                for (int i = 0; i < 2; i++) {
                                    Eigen::Vector3d p(0, 0, 0);
                                    for (int j = 0; j < 3; j++) {
                                        p += V_cur.row(seg.fv_ids[j]) * seg.bcs[i](j);
                                    }
                                    pts.row(i) = p;
                                }
                                viewer.data().add_points(pts.row(0), Eigen::RowVector3d(1, 0, 0));
                                viewer.data().add_points(pts.row(1), Eigen::RowVector3d(1, 0, 0));
                                viewer.data().add_edges(
                                    pts.row(0),
                                    pts.row(1),
                                    Eigen::RowVector3d(1, 0, 0));
                            }
                        }
                    }
                } else if (view_mode == 4) {
                    Eigen::MatrixXd V_cur, Vt_cur, Vn_cur;
                    Eigen::MatrixXi F_cur, Ft_cur, Fn_cur;
                    fs::path MeshfilePath = operation_logs_dir / ("VF_all_after_operation_" +
                                                                  std::to_string(file_id) + ".obj");
                    igl::readOBJ(
                        MeshfilePath.string(),
                        V_cur,
                        Vt_cur,
                        Vn_cur,
                        F_cur,
                        Ft_cur,
                        Fn_cur);
                    Eigen::MatrixXi F_slice(id_map_after.size(), 3);
                    for (int i = 0; i < id_map_after.size(); i++) {
                        F_slice.row(i) = F_cur.row(id_map_after[i]);
                    }

                    viewer.data().set_mesh(V_cur, F_slice);
                    viewer.core().align_camera_center(V_cur, F_slice);
                    // read curve and draw
                    if (file_id > 0) {
                        std::vector<query_curve> curves_cur = load_query_curves(
                            "curve_debug/curve_" + std::to_string(file_id) + ".json");
                        for (const auto& curve : curves_cur) {
                            for (const auto& seg : curve.segments) {
                                if (std::find(id_map_after.begin(), id_map_after.end(), seg.f_id) ==
                                    id_map_after.end()) {
                                    continue;
                                }
                                Eigen::MatrixXd pts(2, 3);
                                for (int i = 0; i < 2; i++) {
                                    Eigen::Vector3d p(0, 0, 0);
                                    for (int j = 0; j < 3; j++) {
                                        p += V_cur.row(seg.fv_ids[j]) * seg.bcs[i](j);
                                    }
                                    pts.row(i) = p;
                                }
                                viewer.data().add_points(pts.row(0), Eigen::RowVector3d(1, 0, 0));
                                viewer.data().add_points(pts.row(1), Eigen::RowVector3d(1, 0, 0));
                                viewer.data().add_edges(
                                    pts.row(0),
                                    pts.row(1),
                                    Eigen::RowVector3d(1, 0, 0));
                            }
                        }
                    }
                }
            }
        } else {
            viewer.data().clear();
            viewer.data().set_mesh(V_in, F_in);
            // viewer.data().point_size /= 3;
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
        }

        return false;
    };

    igl::opengl::glfw::Viewer viewer;

    // read obj from file
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
    viewer.callback_key_down = key_down;
    viewer.launch();
}
#endif