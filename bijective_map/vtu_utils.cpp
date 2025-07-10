#include "vtu_utils.hpp"
#include <fstream>

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

} // namespace vtu_utils