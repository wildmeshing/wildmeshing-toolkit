#pragma once

#include <fstream>
#include <iostream>
#include <sstream>

// igl
#include <igl/parallel_for.h>


struct query_point
{
    int64_t f_id; // face id
    Eigen::Vector3d bc; // barycentric coordinates
    Eigen::Vector3i fv_ids; // face vertex ids
};

void handle_consolidate(
    const std::vector<int64_t>& face_ids_maps,
    const std::vector<int64_t>& vertex_ids_maps,
    std::vector<query_point>& query_points)
{
    std::cout << "Handling Consolidate" << std::endl;
    igl::parallel_for(query_points.size(), [&](int id) {
        query_point& qp = query_points[id];
        if (face_ids_maps[qp.f_id] != -1) {
            const auto old_f_id = qp.f_id;
            qp.f_id = face_ids_maps[qp.f_id];
        }
        for (int i = 0; i < 3; i++) {
            if (vertex_ids_maps[qp.fv_ids[i]] != -1) {
                qp.fv_ids[i] = vertex_ids_maps[qp.fv_ids[i]];
            }
        }
    });
}

void skipLine(std::ifstream& file)
{
    std::string dummy;
    std::getline(file, dummy);
}

void parse_consolidate_file(
    std::ifstream& file,
    std::vector<int64_t>& face_ids_maps,
    std::vector<int64_t>& vertex_ids_maps)
{
    int map_dimension = -1;
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string word;
        ss >> word;

        if (word == "dimension") {
            ss >> map_dimension;
        } else if (word == "size:") {
            int size;
            ss >> size;
            if (map_dimension == 0) {
                vertex_ids_maps = std::vector<int64_t>(size, -1);
            } else if (map_dimension == 2) {
                face_ids_maps = std::vector<int64_t>(size, -1);
            }
        } else if (!word.empty()) {
            int64_t key = std::stoi(word);
            int64_t value;
            ss >> value;

            if (map_dimension == 0) {
                vertex_ids_maps[key] = value;
                // std::cout << "vertex_ids_maps[" << key << "] = " << value << std::endl;
            } else if (map_dimension == 2) {
                face_ids_maps[key] = value;
                // std::cout << "face_ids_maps[" << key << "] = " << value << std::endl;
            }
        }
    }
}

template <typename Matrix>
void readMatrix(std::ifstream& file, Matrix& matrix, int rows, int cols)
{
    matrix.resize(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            file >> matrix(i, j);
        }
    }
}

template <typename Vector>
void readVector(std::ifstream& file, Vector& vec)
{
    int64_t value;
    std::string line;
    std::getline(file, line); // Read the entire line for id_map
    std::istringstream iss(line);
    while (iss >> value) {
        vec.push_back(value);
    }
}

void parse_edge_split_file(
    std::ifstream& file,
    Eigen::MatrixXd& V_before,
    Eigen::MatrixXi& F_before,
    std::vector<int64_t>& id_map_before,
    Eigen::MatrixXd& V_after,
    Eigen::MatrixXi& F_after,
    std::vector<int64_t>& id_map_after)
{
    std::cout << "Parsing SplitEdge file\n";
    std::string line;
    int rows, cols = 3; // Assuming matrices have 3 columns

    // Reading F_after
    file >> line >> rows; // Consumes "F_after: <rows>"
    readMatrix(file, F_after, rows, cols);

    // Reading V_after
    file >> line >> rows; // Consumes "V_after: <rows>"
    readMatrix(file, V_after, rows, cols);

    // Reading id_map_after
    file >> line; // Consumes "F_id_map_after:"
    skipLine(file); // Moves to the actual data line
    readVector(file, id_map_after);

    // Reading F_before
    file >> line >> rows; // Consumes "F_before: <rows>"
    readMatrix(file, F_before, rows, cols);

    // Reading V_before
    file >> line >> rows; // Consumes "V_before: <rows>"
    readMatrix(file, V_before, rows, cols);

    // Reading id_map_before
    file >> line; // Consumes "F_id_map_before:"
    skipLine(file); // Moves to the actual data line
    readVector(file, id_map_before);

    // Outputs (for verification)
    std::cout << "F_after:\n" << F_after << "\n\n";
    std::cout << "V_after:\n" << V_after << "\n\n";
    std::cout << "id_map_after: ";
    for (auto id : id_map_after) std::cout << id << " ";
    std::cout << "\n\n";

    std::cout << "F_before:\n" << F_before << "\n\n";
    std::cout << "V_before:\n" << V_before << "\n\n";
    std::cout << "id_map_before: ";
    for (auto id : id_map_before) std::cout << id << " ";
    std::cout << "\n";
}