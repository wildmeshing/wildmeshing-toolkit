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

void wriet_points_to_file(
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


int main(int argc, char** argv)
{
    CLI::App app{"bijective_map_app_tet"};
    std::filesystem::path initial_mesh_file;
    std::string application_name = "back";
    app.add_option("-a, --app", application_name, "Application name");
    app.add_option("-i, --input", initial_mesh_file, "Initial mesh file")->required(true);
    CLI11_PARSE(app, argc, argv);

    std::cout << "Application name: " << application_name << std::endl;
    auto init_mesh_ptr = wmtk::read_mesh(initial_mesh_file);

    // write initial mesh to vtu
    std::cout << "Writing initial mesh to vtu" << std::endl;
    wmtk::io::ParaviewWriter
        writer("initial_mesh", "vertices", *init_mesh_ptr, true, true, true, true);
    init_mesh_ptr->serialize(writer);

    // TODO:
    // 2. figure out how to read the outputmesh in vtu format
    auto T_out = readTetrahedrons("T_matrix.csv");
    auto V_out = readVertices("V_matrix.csv");
    // TODO: for now, first we convert it with TV matrix

    // get points in T,V out
    // sample points, id every 100 points
    std::vector<query_point_tet> query_points;
    for (int i = 0; i < T_out.rows() / 100; i++) {
        query_point_tet qp;
        qp.t_id = i * 100;
        qp.bc = Eigen::Vector4d(0.25, 0.25, 0.25, 0.25);
        qp.tv_ids = T_out.row(i * 100);
        query_points.push_back(qp);
    }

    // compute postion and save to file
    wriet_points_to_file(query_points, V_out, "points_after_remesh.csv");

    // TODO: do back tracking

    return 0;
}