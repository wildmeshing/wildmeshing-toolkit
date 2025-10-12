#include "vtu_utils.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <string>
#include <algorithm>
#include <cctype>
#include <cstring>
#include <cstdint>

namespace vtu_utils {

// Base64 decoding helper function
std::vector<uint8_t> base64_decode(const std::string& encoded) {
    const std::string chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::vector<uint8_t> decoded;
    
    int val = 0, valb = -8;
    for (unsigned char c : encoded) {
        if (chars.find(c) == std::string::npos) continue;
        val = (val << 6) + chars.find(c);
        valb += 6;
        if (valb >= 0) {
            decoded.push_back(char((val >> valb) & 0xFF));
            valb -= 8;
        }
    }
    return decoded;
}

// Extract data from binary VTU format
template<typename T>
std::vector<T> extract_binary_data(const std::string& base64_data) {
    std::vector<uint8_t> binary_data = base64_decode(base64_data);
    
    if (binary_data.size() < sizeof(uint64_t)) {
        std::cerr << "Binary data too small for header" << std::endl;
        return std::vector<T>();
    }
    
    // VTU binary format: first 8 bytes are the size (uint64_t), then the data
    uint64_t data_size;
    std::memcpy(&data_size, binary_data.data(), sizeof(uint64_t));
    
    size_t expected_elements = data_size / sizeof(T);
    std::vector<T> result(expected_elements);
    
    if (binary_data.size() < sizeof(uint64_t) + data_size) {
        std::cerr << "Binary data size mismatch" << std::endl;
        return std::vector<T>();
    }
    
    std::memcpy(result.data(), binary_data.data() + sizeof(uint64_t), data_size);
    return result;
}

void write_triangle_mesh_to_vtu(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const std::string& filename)
{
    Eigen::MatrixXd V3;
    if (V.cols() == 3) {
        V3 = V;
    } else if (V.cols() == 2) {
        V3.resize(V.rows(), 3);
        V3.leftCols(2) = V;
        V3.col(2).setZero();
    } else {
        std::cerr << "write_triangle_mesh_to_vtu expects V with 2 or 3 columns, got " << V.cols()
                  << std::endl;
        return;
    }

    std::ofstream outfile(filename);
    outfile << "<?xml version=\"1.0\"?>\n";
    outfile << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
    outfile << "  <UnstructuredGrid>\n";
    outfile << "    <Piece NumberOfPoints=\"" << V3.rows() << "\" NumberOfCells=\"" << F.rows()
            << "\">\n";

    // Write points
    outfile << "      <Points>\n";
    outfile << "        <DataArray type=\"Float64\" NumberOfComponents=\"3\" format=\"ascii\">\n";
    for (int i = 0; i < V3.rows(); i++) {
        outfile << "          " << V3(i, 0) << " " << V3(i, 1) << " " << V3(i, 2) << "\n";
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
    Eigen::MatrixXd V3;
    if (V.cols() == 3) {
        V3 = V;
    } else if (V.cols() == 2) {
        V3.resize(V.rows(), 3);
        V3.leftCols(2) = V;
        V3.col(2).setZero();
    } else {
        std::cerr << "write_point_mesh_to_vtu expects V with 2 or 3 columns, got " << V.cols()
                  << std::endl;
        return;
    }

    std::ofstream outfile(filename);
    outfile << "<?xml version=\"1.0\"?>\n";
    outfile << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
    outfile << "  <UnstructuredGrid>\n";
    outfile << "    <Piece NumberOfPoints=\"" << V3.rows() << "\" NumberOfCells=\"" << V3.rows()
            << "\">\n";

    // Write points
    outfile << "      <Points>\n";
    outfile << "        <DataArray type=\"Float64\" NumberOfComponents=\"3\" format=\"ascii\">\n";
    for (int i = 0; i < V3.rows(); i++) {
        outfile << "          " << V3(i, 0) << " " << V3(i, 1) << " " << V3(i, 2) << "\n";
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

void write_edge_mesh_to_vtu(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& E,
    const std::string& filename,
    const Eigen::VectorXi* cell_scalar,
    const std::string& cell_scalar_name)
{
    std::ofstream outfile(filename);
    outfile << "<?xml version=\"1.0\"?>\n";
    outfile << "<VTKFile type=\"UnstructuredGrid\" version=\"0.1\" byte_order=\"LittleEndian\">\n";
    outfile << "  <UnstructuredGrid>\n";
    outfile << "    <Piece NumberOfPoints=\"" << V.rows() << "\" NumberOfCells=\"" << E.rows()
            << "\">\n";

    if (cell_scalar != nullptr) {
        if (cell_scalar->rows() != E.rows()) {
            std::cerr << "write_edge_mesh_to_vtu: cell scalar size (" << cell_scalar->rows()
                      << ") does not match number of edges (" << E.rows() << ")" << std::endl;
        } else {
            outfile << "      <CellData Scalars=\"" << cell_scalar_name << "\">\n";
            outfile << "        <DataArray type=\"Int32\" Name=\"" << cell_scalar_name
                    << "\" format=\"ascii\">\n";
            for (int i = 0; i < cell_scalar->rows(); i++) {
                outfile << "          " << (*cell_scalar)(i) << "\n";
            }
            outfile << "        </DataArray>\n";
            outfile << "      </CellData>\n";
        }
    }

    // Write points
    outfile << "      <Points>\n";
    outfile << "        <DataArray type=\"Float64\" NumberOfComponents=\"3\" format=\"ascii\">\n";
    for (int i = 0; i < V.rows(); i++) {
        if (V.cols() == 2) {
            // 2D case: add z=0
            outfile << "          " << V(i, 0) << " " << V(i, 1) << " " << 0.0 << "\n";
        } else {
            // 3D case
            outfile << "          " << V(i, 0) << " " << V(i, 1) << " " << V(i, 2) << "\n";
        }
    }
    outfile << "        </DataArray>\n";
    outfile << "      </Points>\n";

    // Write cells (edges)
    outfile << "      <Cells>\n";
    outfile << "        <DataArray type=\"Int32\" Name=\"connectivity\" format=\"ascii\">\n";
    for (int i = 0; i < E.rows(); i++) {
        outfile << "          " << E(i, 0) << " " << E(i, 1) << "\n";
    }
    outfile << "        </DataArray>\n";
    outfile << "        <DataArray type=\"Int32\" Name=\"offsets\" format=\"ascii\">\n";
    for (int i = 0; i < E.rows(); i++) {
        outfile << "          " << (i + 1) * 2 << "\n";
    }
    outfile << "        </DataArray>\n";
    outfile << "        <DataArray type=\"UInt8\" Name=\"types\" format=\"ascii\">\n";
    for (int i = 0; i < E.rows(); i++) {
        outfile << "          3\n"; // VTK_LINE = 3
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

    std::string content;
    std::string line;
    while (std::getline(infile, line)) {
        content += line + "\n";
    }
    infile.close();

    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> faces;
    
    // Parse points (vertices)
    size_t points_start = content.find("<DataArray type=\"Float64\" NumberOfComponents=\"3\"");
    if (points_start == std::string::npos) {
        std::cerr << "Could not find points data array" << std::endl;
        return false;
    }
    
    size_t points_format_start = content.find("format=\"", points_start);
    if (points_format_start == std::string::npos) {
        std::cerr << "Could not find format specification for points" << std::endl;
        return false;
    }
    points_format_start += 8; // Skip 'format="'
    
    bool points_binary = (content.substr(points_format_start, 6) == "binary");
    
    size_t points_data_start = content.find(">", points_start) + 1;
    size_t points_data_end = content.find("</DataArray>", points_data_start);
    
    if (points_data_start == std::string::npos || points_data_end == std::string::npos) {
        std::cerr << "Could not find points data" << std::endl;
        return false;
    }
    
    std::string points_data = content.substr(points_data_start, points_data_end - points_data_start);
    
    if (points_binary) {
        // Handle binary format
        std::vector<double> point_values = extract_binary_data<double>(points_data);
        for (size_t i = 0; i + 2 < point_values.size(); i += 3) {
            vertices.push_back(Eigen::Vector3d(point_values[i], point_values[i+1], point_values[i+2]));
        }
    } else {
        // Handle ASCII format
        std::istringstream points_stream(points_data);
        double x, y, z;
        while (points_stream >> x >> y >> z) {
            vertices.push_back(Eigen::Vector3d(x, y, z));
        }
    }
    
    // Parse connectivity (faces) - try both Int32 and Int64
    size_t conn_start = content.find("<DataArray type=\"Int32\" Name=\"connectivity\"");
    bool is_int64 = false;
    if (conn_start == std::string::npos) {
        conn_start = content.find("<DataArray type=\"Int64\" Name=\"connectivity\"");
        is_int64 = true;
    }
    if (conn_start == std::string::npos) {
        std::cerr << "Could not find connectivity data array (tried Int32 and Int64)" << std::endl;
        return false;
    }
    
    size_t conn_format_start = content.find("format=\"", conn_start);
    if (conn_format_start == std::string::npos) {
        std::cerr << "Could not find format specification for connectivity" << std::endl;
        return false;
    }
    conn_format_start += 8; // Skip 'format="'
    
    bool conn_binary = (content.substr(conn_format_start, 6) == "binary");
    
    size_t conn_data_start = content.find(">", conn_start) + 1;
    size_t conn_data_end = content.find("</DataArray>", conn_data_start);
    
    if (conn_data_start == std::string::npos || conn_data_end == std::string::npos) {
        std::cerr << "Could not find connectivity data" << std::endl;
        return false;
    }
    
    std::string conn_data = content.substr(conn_data_start, conn_data_end - conn_data_start);
    
    std::vector<int> connectivity;
    if (conn_binary) {
        // Handle binary format
        if (is_int64) {
            std::vector<int64_t> conn_values = extract_binary_data<int64_t>(conn_data);
            connectivity.assign(conn_values.begin(), conn_values.end());
        } else {
            std::vector<int32_t> conn_values = extract_binary_data<int32_t>(conn_data);
            connectivity.assign(conn_values.begin(), conn_values.end());
        }
    } else {
        // Handle ASCII format
        std::istringstream conn_stream(conn_data);
        int idx;
        while (conn_stream >> idx) {
            connectivity.push_back(idx);
        }
    }
    
    // Group connectivity into triangles (sets of 3)
    for (size_t i = 0; i + 2 < connectivity.size(); i += 3) {
        faces.push_back(Eigen::Vector3i(connectivity[i], connectivity[i+1], connectivity[i+2]));
    }
    
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
