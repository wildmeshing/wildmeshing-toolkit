#include <CLI/CLI.hpp>
#include <filesystem>
#include <iostream>
#include <sstream>

// wmtk
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/utils/orient.hpp>

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
            std::string face_key = std::to_string(face[0]) + "_" + std::to_string(face[1]) + "_" +
                                   std::to_string(face[2]);
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
            std::string face_key = std::to_string(face[0]) + "_" + std::to_string(face[1]) + "_" +
                                   std::to_string(face[2]);
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

    if (application_name == "back") {
        std::cout << "Back tracking" << std::endl;
        track_point_tet(operation_logs_dir, query_points, false, false);

        std::cout << "Writing points to file after back tracking" << std::endl;
        // TODO: this should be V_in
        write_points_to_file(query_points, V_in, "points_after_back_tracking.csv");
    }

    return 0;
}