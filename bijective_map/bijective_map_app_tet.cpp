#include <CLI/CLI.hpp>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <unordered_set>
// wmtk
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/orient.hpp>

#include <igl/tet_tet_adjacency.h>
#include "track_operations_tet.hpp"
using namespace wmtk;
using path = std::filesystem::path;

// igl, nothing for now

// applications
Eigen::MatrixXd readVertices(const std::string& filename)
{
    std::ifstream file(filename);
    std::string line;
    std::vector<std::vector<double>> data;
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::istringstream s(line);
        std::string field;
        std::vector<double> row;
        while (std::getline(s, field, ',')) {
            row.push_back(std::stod(field));
        }
        data.push_back(row);
    }

    Eigen::MatrixXd V(data.size(), data[0].size());
    for (size_t i = 0; i < data.size(); ++i) {
        for (size_t j = 0; j < data[0].size(); ++j) {
            V(i, j) = data[i][j];
        }
    }
    return V;
}

Eigen::MatrixXi readTetrahedrons(const std::string& filename)
{
    std::ifstream file(filename);
    std::string line;
    std::vector<std::vector<int>> data;

    std::getline(file, line);

    while (std::getline(file, line)) {
        std::istringstream s(line);
        std::string field;
        std::vector<int> row;

        while (std::getline(s, field, ',')) {
            row.push_back(std::stoi(field));
        }
        data.push_back(row);
    }

    Eigen::MatrixXi T(data.size(), data[0].size());
    for (size_t i = 0; i < data.size(); ++i) {
        for (size_t j = 0; j < data[0].size(); ++j) {
            T(i, j) = data[i][j];
        }
    }
    return T;
}

void write_points_to_file(
    const std::vector<query_point_tet>& query_points,
    const Eigen::MatrixXd& V,
    const std::string& filename)
{
    std::ofstream file(filename);
    for (int i = 0; i < query_points.size(); i++) {
        auto& qp = query_points[i];
        Eigen::Vector3d p(0, 0, 0);
        for (int j = 0; j < 4; j++) {
            p += qp.bc(j) * V.row(qp.tv_ids[j]);
        }
        file << p[0] << "," << p[1] << ", " << p[2] << "\n";
    }
    file.close();
}

void track_point_one_operation_tet(
    const json& operation_log,
    std::vector<query_point_tet>& query_points,
    bool do_forward = false,
    bool use_rational = false)
{
    std::string operation_name;
    operation_name = operation_log["operation_name"];

    if (operation_name == "MeshConsolidate") {
        std::cout << "This Operations is Consolidate" << std::endl;
        std::vector<int64_t> tet_ids_maps;
        std::vector<int64_t> vertex_ids_maps;
        parse_consolidate_file_tet(operation_log, tet_ids_maps, vertex_ids_maps);

        handle_consolidate_tet(tet_ids_maps, vertex_ids_maps, query_points, do_forward);
    } else {
        std::cout << "This Operations is " << operation_name << std::endl;
        Eigen::MatrixXi T_after, T_before;
        Eigen::MatrixXd V_after, V_before;
        std::vector<int64_t> id_map_after, id_map_before;
        std::vector<int64_t> v_id_map_after, v_id_map_before;
        parse_non_collapse_file_tet(
            operation_log,
            V_before,
            T_before,
            id_map_before,
            v_id_map_before,
            V_after,
            T_after,
            id_map_after,
            v_id_map_after);

        if (do_forward) {
            handle_local_mapping_tet(
                V_after,
                T_after,
                id_map_after,
                v_id_map_after,
                V_before,
                T_before,
                id_map_before,
                v_id_map_before,
                query_points);
        } else {
            handle_local_mapping_tet(
                V_before,
                T_before,
                id_map_before,
                v_id_map_before,
                V_after,
                T_after,
                id_map_after,
                v_id_map_after,
                query_points);
        }
    }
}

void track_curve_one_operation_tet(
    const json& operation_log,
    query_curve_tet& curve,
    bool do_forward = false,
    bool use_rational = false)
{
    std::string operation_name;
    operation_name = operation_log["operation_name"];

    if (operation_name == "MeshConsolidate") {
        std::cout << "This Operations is Consolidate" << std::endl;
        std::vector<int64_t> tet_ids_maps;
        std::vector<int64_t> vertex_ids_maps;
        parse_consolidate_file_tet(operation_log, tet_ids_maps, vertex_ids_maps);

        handle_consolidate_tet_curve(tet_ids_maps, vertex_ids_maps, curve, do_forward);
    } else {
        std::cout << "This Operations is " << operation_name << std::endl;
        Eigen::MatrixXi T_after, T_before;
        Eigen::MatrixXd V_after, V_before;
        std::vector<int64_t> id_map_after, id_map_before;
        std::vector<int64_t> v_id_map_after, v_id_map_before;
        parse_non_collapse_file_tet(
            operation_log,
            V_before,
            T_before,
            id_map_before,
            v_id_map_before,
            V_after,
            T_after,
            id_map_after,
            v_id_map_after);

        if (do_forward) {
            handle_local_mapping_tet_curve(
                V_after,
                T_after,
                id_map_after,
                v_id_map_after,
                V_before,
                T_before,
                id_map_before,
                v_id_map_before,
                curve);
        } else {
            handle_local_mapping_tet_curve(
                V_before,
                T_before,
                id_map_before,
                v_id_map_before,
                V_after,
                T_after,
                id_map_after,
                v_id_map_after,
                curve);
        }
    }
}
void track_point_tet(
    path dirPath,
    std::vector<query_point_tet>& query_points,
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
        track_point_one_operation_tet(operation_log, query_points, do_forward, use_rational);

        file.close();
    }
}

void track_curve_tet(
    path dirPath,
    query_curve_tet& curve,
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
        track_curve_one_operation_tet(operation_log, curve, do_forward, use_rational);

        file.close();
    }
}


int main(int argc, char** argv)
{
    CLI::App app{"bijective_map_app_tet"};
    std::filesystem::path initial_mesh_file;
    std::filesystem::path operation_logs_dir;
    std::string application_name = "back";
    app.add_option("-a, --app", application_name, "Application name");
    app.add_option("-i, --input", initial_mesh_file, "Initial mesh file")->required(true);
    app.add_option("-l, --logs", operation_logs_dir, "Operation logs directory")->required(true);
    CLI11_PARSE(app, argc, argv);

    std::cout << "Application name: " << application_name << std::endl;
    auto init_mesh_ptr = wmtk::read_mesh(initial_mesh_file);

    // write initial mesh to vtu
    // std::cout << "Writing initial mesh to vtu" << std::endl;
    // wmtk::io::ParaviewWriter
    //     writer("initial_mesh", "vertices", *init_mesh_ptr, true, true, true, true);
    // init_mesh_ptr->serialize(writer);

    // TODO:
    // 2. figure out how to read the outputmesh in vtu format out is after remesh
    auto T_out = readTetrahedrons("../build/T_matrix_out.csv");
    auto V_out = readVertices("../build/V_matrix_out.csv");
    // TODO: for now, first we convert it with TV matrix
    auto T_in = readTetrahedrons("../build/T_matrix_in.csv");
    auto V_in = readVertices("../build/V_matrix_in.csv");


    if (application_name == "back") {
        std::cout << "Back tracking" << std::endl;
        // get points in T,V out
        // sample points, id every 100 points
        std::vector<query_point_tet> query_points;
        // for (int i = 0; i < T_out.rows() / 100; i++) {
        //     query_point_tet qp;
        //     qp.t_id = i * 100;
        //     qp.bc = Eigen::Vector4d(0.25, 0.25, 0.25, 0.25);
        //     qp.tv_ids = T_out.row(i * 100);
        //     query_points.push_back(qp);
        // }

        // Sample some points on boundary tetrahedrons
        std::vector<query_point_tet> boundary_query_points;
        std::unordered_map<std::string, int> face_count;

        // Count the occurrence of each face
        for (int i = 0; i < T_out.rows(); i++) {
            auto tet = T_out.row(i);
            for (int j = 0; j < 4; j++) {
                Eigen::Vector3i face;
                face << tet[j], tet[(j + 1) % 4], tet[(j + 2) % 4];
                std::sort(face.data(), face.data() + 3);
                std::string face_key = std::to_string(face[0]) + "_" + std::to_string(face[1]) +
                                       "_" + std::to_string(face[2]);
                face_count[face_key]++;
            }
        }

        // Find boundary tetrahedrons and sample
        for (int i = 0; i < T_out.rows(); i++) {
            if (i % 3 != 0) continue;
            auto tet = T_out.row(i);
            int boundary_face_count = 0;
            for (int j = 0; j < 4; j++) {
                Eigen::Vector3i face;
                face << tet[j], tet[(j + 1) % 4], tet[(j + 2) % 4];
                std::sort(face.data(), face.data() + 3);
                std::string face_key = std::to_string(face[0]) + "_" + std::to_string(face[1]) +
                                       "_" + std::to_string(face[2]);
                if (face_count[face_key] == 1) {
                    boundary_face_count++;
                }
            }
            if (boundary_face_count > 0) {
                query_point_tet qp;
                qp.t_id = i;
                qp.bc = Eigen::Vector4d::Random().cwiseAbs(); // Randomize barycentric coordinates
                qp.bc /= qp.bc.sum(); // Normalize to ensure they sum to 1
                qp.tv_ids = T_out.row(i);
                boundary_query_points.push_back(qp);
            }
        }

        // Add sampled points from boundary tetrahedrons to the total query points
        query_points.insert(
            query_points.end(),
            boundary_query_points.begin(),
            boundary_query_points.end());


        // compute postion and save to file
        std::cout << "Writing points to file after remesh" << std::endl;
        write_points_to_file(query_points, V_out, "points_after_remesh.csv");
        track_point_tet(operation_logs_dir, query_points, false, false);

        std::cout << "Writing points to file after back tracking" << std::endl;
        write_points_to_file(query_points, V_in, "points_after_back_tracking.csv");
    } else if (application_name == "back_curve") {
        // TODO: orgnize the application code to a separate function
        // 1. sample a curve in the tet mesh (output mesh)
        Eigen::MatrixXi TT, TTi;
        igl::tet_tet_adjacency(T_out, TT, TTi);
        query_curve_tet curve;
        int curve_length = 10;
        {
            auto mid_point_on_face = [](int i) {
                if (i == 0) {
                    // face [0,1,2]
                    return Eigen::Vector4d(1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0, 0.0);
                } else if (i == 1) {
                    // face [0,1,3]
                    return Eigen::Vector4d(1.0 / 3.0, 1.0 / 3.0, 0.0, 1.0 / 3.0);
                } else if (i == 2) {
                    // face [1,2,3]
                    return Eigen::Vector4d(0.0, 1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0);
                } else if (i == 3) {
                    // face [2,0,3]
                    return Eigen::Vector4d(1.0 / 3.0, 0.0, 1.0 / 3.0, 1.0 / 3.0);
                } else {
                    throw std::runtime_error("Invalid face index");
                }
            };
            std::cout << "Start sampling the curve" << std::endl;
            // Set random seed for reproducibility
            srand(42);
            // sample the start point
            int start_tet_id = rand() % T_out.rows();
            std::unordered_set<int> visited_tets;

            int current_tet_id = start_tet_id;
            int prev_face = -1;

            for (int seg_id = 0; seg_id < curve_length; seg_id++) {
                if (current_tet_id == -1 || visited_tets.count(current_tet_id) > 0) {
                    break;
                }
                visited_tets.insert(current_tet_id);
                int first_face = prev_face == -1 ? rand() % 4 : prev_face;
                int second_face = (first_face + 1 + rand() % 3) % 4;

                // std::cout << "current_tet_id: " << current_tet_id << std::endl;
                // std::cout << "first_face: " << first_face << std::endl;
                // std::cout << "second_face: " << second_face << std::endl;

                query_segment_tet seg;
                seg.t_id = current_tet_id;
                seg.bcs[0] = mid_point_on_face(first_face);
                seg.bcs[1] = mid_point_on_face(second_face);
                seg.tv_ids = T_out.row(current_tet_id);
                curve.segments.push_back(seg);

                std::cout << "TTi.row(current_tet_id): " << TTi.row(current_tet_id) << std::endl;
                std::cout << "TT.row(current_tet_id): " << TT.row(current_tet_id) << std::endl;
                prev_face = TTi(current_tet_id, second_face);
                current_tet_id = TT(current_tet_id, second_face);
            }
            curve.next_segment_ids.resize(curve.segments.size());
            for (int i = 0; i < curve.segments.size() - 1; i++) {
                curve.next_segment_ids[i] = i + 1;
            }
            curve.next_segment_ids[curve.segments.size() - 1] = -1;

            std::cout << "curve.segments.size(): " << curve.segments.size() << std::endl;
        }

        auto write_curve_points_to_file = [](const Eigen::MatrixXd& V,
                                             const query_curve_tet& curve,
                                             const std::string& filename1,
                                             const std::string& filename2) {
            // Convert barycentric coordinates to real positions and write to CSV files
            std::ofstream start_points_file(filename1);
            std::ofstream end_points_file(filename2);


            for (const auto& seg : curve.segments) {
                // Get vertices of the tetrahedron
                Eigen::Vector4i tet_verts = seg.tv_ids;

                // Calculate real position for start point (bcs[0])
                Eigen::Vector3d start_pos = Eigen::Vector3d::Zero();
                for (int i = 0; i < 4; i++) {
                    start_pos += seg.bcs[0](i) * V.row(tet_verts(i)).transpose();
                }

                // Calculate real position for end point (bcs[1])
                Eigen::Vector3d end_pos = Eigen::Vector3d::Zero();
                for (int i = 0; i < 4; i++) {
                    end_pos += seg.bcs[1](i) * V.row(tet_verts(i)).transpose();
                }

                // Write to CSV files
                start_points_file << start_pos(0) << "," << start_pos(1) << "," << start_pos(2)
                                  << "\n";
                end_points_file << end_pos(0) << "," << end_pos(1) << "," << end_pos(2) << "\n";
            }

            start_points_file.close();
            end_points_file.close();

            std::cout << "Written curve points to curve_start_points.csv and curve_end_points.csv"
                      << std::endl;
        };
        write_curve_points_to_file(V_out, curve, "curve_start_points.csv", "curve_end_points.csv");
        track_curve_tet(operation_logs_dir, curve, false, false);
        // 2. back track the curve
        // 3. write the curve to a file, it should be a edge mesh in vtu format
    }

    return 0;
}