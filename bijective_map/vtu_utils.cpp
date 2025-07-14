#include "vtu_utils.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <cctype>

namespace vtu_utils {

void write_triangle_mesh_to_vtu(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const std::string& filename)
{
    std::ofstream outfile(filename);
    outfile << "<?xml version=\"1.0\"?>\n";
    outfile << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
    outfile << "  <UnstructuredGrid>\n";
    outfile << "    <Piece NumberOfPoints=\"" << V.rows() << "\" NumberOfCells=\"" << F.rows()
            << "\">\n";

    // Write points
    outfile << "      <Points>\n";
    outfile << "        <DataArray type=\"Float64\" NumberOfComponents=\"3\" format=\"ascii\">\n";
    for (int i = 0; i < V.rows(); i++) {
        outfile << "          " << V(i, 0) << " " << V(i, 1) << " " << V(i, 2) << "\n";
    }
    outfile << "        </DataArray>\n";
    outfile << "      </Points>\n";

    // Write cells
    outfile << "      <Cells>\n";
    outfile << "        <DataArray type=\"Int32\" Name=\"connectivity\" format=\"ascii\">\n";
    for (int i = 0; i < F.rows(); i++) {
        outfile << "          " << F(i, 0) << " " << F(i, 1) << " " << F(i, 2) << "\n";
    }
    outfile << "        </DataArray>\n";
    outfile << "        <DataArray type=\"Int32\" Name=\"offsets\" format=\"ascii\">\n";
    for (int i = 0; i < F.rows(); i++) {
        outfile << "          " << (i + 1) * 3 << "\n";
    }
    outfile << "        </DataArray>\n";
    outfile << "        <DataArray type=\"UInt8\" Name=\"types\" format=\"ascii\">\n";
    for (int i = 0; i < F.rows(); i++) {
        outfile << "          5\n"; // VTK_TRIANGLE = 5
    }
    outfile << "        </DataArray>\n";
    outfile << "      </Cells>\n";
    outfile << "    </Piece>\n";
    outfile << "  </UnstructuredGrid>\n";
    outfile << "</VTKFile>\n";
    outfile.close();
}

void write_point_mesh_to_vtu(const Eigen::MatrixXd& V, const std::string& filename)
{
    std::ofstream outfile(filename);
    outfile << "<?xml version=\"1.0\"?>\n";
    outfile << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
    outfile << "  <UnstructuredGrid>\n";
    outfile << "    <Piece NumberOfPoints=\"" << V.rows() << "\" NumberOfCells=\"" << V.rows()
            << "\">\n";

    // Write points
    outfile << "      <Points>\n";
    outfile << "        <DataArray type=\"Float64\" NumberOfComponents=\"3\" format=\"ascii\">\n";
    for (int i = 0; i < V.rows(); i++) {
        outfile << "          " << V(i, 0) << " " << V(i, 1) << " " << V(i, 2) << "\n";
    }
    outfile << "        </DataArray>\n";
    outfile << "      </Points>\n";

    // Write cells (each point is a vertex cell)
    outfile << "      <Cells>\n";
    outfile << "        <DataArray type=\"Int32\" Name=\"connectivity\" format=\"ascii\">\n";
    for (int i = 0; i < V.rows(); i++) {
        outfile << "          " << i << "\n";
    }
    outfile << "        </DataArray>\n";
    outfile << "        <DataArray type=\"Int32\" Name=\"offsets\" format=\"ascii\">\n";
    for (int i = 0; i < V.rows(); i++) {
        outfile << "          " << (i + 1) << "\n";
    }
    outfile << "        </DataArray>\n";
    outfile << "        <DataArray type=\"UInt8\" Name=\"types\" format=\"ascii\">\n";
    for (int i = 0; i < V.rows(); i++) {
        outfile << "          1\n"; // VTK_VERTEX = 1
    }
    outfile << "        </DataArray>\n";
    outfile << "      </Cells>\n";
    outfile << "    </Piece>\n";
    outfile << "  </UnstructuredGrid>\n";
    outfile << "</VTKFile>\n";
    outfile.close();
}
void write_tet_mesh_to_vtu(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& T,
    const std::string& filename)
{
    std::ofstream outfile(filename);
    outfile << "<?xml version=\"1.0\"?>\n";
    outfile << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
    outfile << "  <UnstructuredGrid>\n";
    outfile << "    <Piece NumberOfPoints=\"" << V.rows() << "\" NumberOfCells=\"" << T.rows()
            << "\">\n";

    // Write points
    outfile << "      <Points>\n";
    outfile << "        <DataArray type=\"Float64\" NumberOfComponents=\"3\" format=\"ascii\">\n";
    for (int i = 0; i < V.rows(); i++) {
        outfile << "          " << V(i, 0) << " " << V(i, 1) << " " << V(i, 2) << "\n";
    }
    outfile << "        </DataArray>\n";
    outfile << "      </Points>\n";

    // Write cells (tetrahedra)
    outfile << "      <Cells>\n";
    outfile << "        <DataArray type=\"Int32\" Name=\"connectivity\" format=\"ascii\">\n";
    for (int i = 0; i < T.rows(); i++) {
        outfile << "          " << T(i, 0) << " " << T(i, 1) << " " << T(i, 2) << " " << T(i, 3)
                << "\n";
    }
    outfile << "        </DataArray>\n";
    outfile << "        <DataArray type=\"Int32\" Name=\"offsets\" format=\"ascii\">\n";
    for (int i = 0; i < T.rows(); i++) {
        outfile << "          " << ((i + 1) * 4) << "\n";
    }
    outfile << "        </DataArray>\n";
    outfile << "        <DataArray type=\"UInt8\" Name=\"types\" format=\"ascii\">\n";
    for (int i = 0; i < T.rows(); i++) {
        outfile << "          10\n"; // VTK_TETRA = 10
    }
    outfile << "        </DataArray>\n";
    outfile << "      </Cells>\n";
    outfile << "    </Piece>\n";
    outfile << "  </UnstructuredGrid>\n";
    outfile << "</VTKFile>\n";
    outfile.close();
}

bool read_triangle_mesh_from_vtu(
    const std::string& filename,
    Eigen::MatrixXd& V,
    Eigen::MatrixXi& F)
{
    std::ifstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "Error: Cannot open VTU file " << filename << std::endl;
        return false;
    }

    std::string line;
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> faces;
    
    bool in_points = false;
    bool in_connectivity = false;
    bool in_types = false;
    
    while (std::getline(infile, line)) {
        // Remove leading/trailing whitespace
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);
        
        if (line.find("<DataArray type=\"Float64\" NumberOfComponents=\"3\"") != std::string::npos) {
            in_points = true;
            continue;
        }
        
        if (line.find("<DataArray type=\"Int32\" Name=\"connectivity\"") != std::string::npos) {
            in_connectivity = true;
            continue;
        }
        
        if (line.find("<DataArray type=\"UInt8\" Name=\"types\"") != std::string::npos) {
            in_types = true;
            continue;
        }
        
        if (line.find("</DataArray>") != std::string::npos) {
            in_points = false;
            in_connectivity = false;
            in_types = false;
            continue;
        }
        
        if (in_points) {
            std::istringstream iss(line);
            double x, y, z;
            if (iss >> x >> y >> z) {
                vertices.push_back(Eigen::Vector3d(x, y, z));
            }
        }
        
        if (in_connectivity) {
            std::istringstream iss(line);
            std::vector<int> indices;
            int idx;
            while (iss >> idx) {
                indices.push_back(idx);
            }
            // Group indices into triangles (sets of 3)
            for (size_t i = 0; i + 2 < indices.size(); i += 3) {
                faces.push_back(Eigen::Vector3i(indices[i], indices[i+1], indices[i+2]));
            }
        }
    }
    
    infile.close();
    
    // Convert to Eigen matrices
    V.resize(vertices.size(), 3);
    for (size_t i = 0; i < vertices.size(); ++i) {
        V.row(i) = vertices[i].transpose();
    }
    
    F.resize(faces.size(), 3);
    for (size_t i = 0; i < faces.size(); ++i) {
        F.row(i) = faces[i].transpose();
    }
    
    return true;
}

} // namespace vtu_utils