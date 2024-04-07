#include "BackupHelper.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include "wmtk/utils/EigenMatrixWriter.hpp"

namespace wmtk {
void backup(TetMesh& mesh, std::string tag_name, std::string filename)
{
    // if (!std::filesystem::exists(folder)) {
    //     // create folder
    //     std::filesystem::create_directory(folder);
    // }

    mesh.consolidate();
    wmtk::utils::EigenMatrixWriter writer;
    mesh.serialize(writer);
    Eigen::MatrixXd V;
    Eigen::MatrixX<int64_t> T;
    Eigen::MatrixX<int64_t> Tag;
    writer.get_position_matrix(V);
    writer.get_TV_matrix(T);
    writer.get_int64_t_matrix("tag_name", PrimitiveType::Vertex, Tag);

    // std::string filename = folder + "/" + std::to_string(x_idx) + "_" + std::to_string(y_idx) +
    //                        "_" + std::to_string(z_idx) + ".save";

    std::ofstream file(filename, std::ios::binary);
    if (file.is_open()) {
        int V_rows = V.rows();
        file.write(reinterpret_cast<const char*>(&V_rows), sizeof(int));
        file.write(reinterpret_cast<const char*>(&V(0, 0)), V.rows() * V.cols() * sizeof(double));

        int T_rows = T.rows();
        file.write(reinterpret_cast<const char*>(&T_rows), sizeof(int));
        file.write(reinterpret_cast<const char*>(&T(0, 0)), T.rows() * T.cols() * sizeof(int64_t));

        int Tag_rows = Tag.rows();
        file.write(reinterpret_cast<const char*>(&Tag_rows), sizeof(int));
        file.write(
            reinterpret_cast<const char*>(&Tag(0, 0)),
            Tag.rows() * Tag.cols() * sizeof(int64_t));

        file.close();
    } else {
        std::cerr << "Error: Unable to open file " << filename << " for writing." << std::endl;
    }
}

void getbackup(
    Eigen::MatrixXd& V,
    Eigen::MatrixX<int64_t>& T,
    Eigen::MatrixX<int64_t>& Tag,
    std::string filename)
{
    // std::string filename = folder + "/" + std::to_string(x_idx) + "_" + std::to_string(y_idx) +
    //                        "_" + std::to_string(z_idx) + ".save";

    std::ifstream file(filename, std::ios::binary);
    if (file.is_open()) {
        int V_rows;
        file.read(reinterpret_cast<char*>(&V_rows), sizeof(int));
        V.resize(V_rows, 3);
        file.read(reinterpret_cast<char*>(V.data()), V.rows() * V.cols() * sizeof(double));

        int T_rows;
        file.read(reinterpret_cast<char*>(&T_rows), sizeof(int));
        T.resize(T_rows, 4);
        file.read(reinterpret_cast<char*>(T.data()), T.rows() * T.cols() * sizeof(int64_t));

        int Tag_rows;
        file.read(reinterpret_cast<char*>(&Tag_rows), sizeof(int));
        Tag.resize(T_rows, 1);
        file.read(reinterpret_cast<char*>(Tag.data()), Tag.rows() * Tag.cols() * sizeof(int64_t));

        file.close();
    } else {
        std::cerr << "Error: Unable to open file " << filename << " for reading." << std::endl;
    }
}

void getsection(
    Eigen::MatrixXd& V,
    Eigen::MatrixX<int64_t>& T,
    Eigen::MatrixX<int64_t>& Tag,
    double x_half,
    double y_half,
    double z_half,
    unsigned int section_idx)
{
    std::vector<Eigen::Vector4<int64_t>> Tets;
    std::vector<int64_t> Tags;

    // pick the tets in the section
    for (unsigned int i = 0; i < T.rows(); i++) {
        bool in_section = false;
        for (unsigned int j = 0; j < 4; j++) {
            unsigned int idx = T(i, j);
            switch (section_idx) {
            case 0: {
                if (V(idx, 0) < x_half && V(idx, 1) < y_half && V(idx, 2) < z_half) {
                    in_section = true;
                    break;
                }
            } break;
            case 1: {
                if (V(idx, 0) >= x_half && V(idx, 1) < y_half && V(idx, 2) < z_half) {
                    in_section = true;
                    break;
                }
            } break;
            case 2: {
                if (V(idx, 0) < x_half && V(idx, 1) >= y_half && V(idx, 2) < z_half) {
                    in_section = true;
                    break;
                }
            } break;
            case 3: {
                if (V(idx, 0) >= x_half && V(idx, 1) >= y_half && V(idx, 2) < z_half) {
                    in_section = true;
                    break;
                }
            } break;
            case 4: {
                if (V(idx, 0) < x_half && V(idx, 1) < y_half && V(idx, 2) >= z_half) {
                    in_section = true;
                    break;
                }
            } break;
            case 5: {
                if (V(idx, 0) >= x_half && V(idx, 1) < y_half && V(idx, 2) >= z_half) {
                    in_section = true;
                    break;
                }
            } break;
            case 6: {
                if (V(idx, 0) < x_half && V(idx, 1) >= y_half && V(idx, 2) >= z_half) {
                    in_section = true;
                    break;
                }
            } break;
            case 7: {
                if (V(idx, 0) >= x_half && V(idx, 1) >= y_half && V(idx, 2) >= z_half) {
                    in_section = true;
                    break;
                }
            } break;
            default: std::runtime_error("section idx is invalid");
            }
        }
        if (in_section) {
            Tets.push_back(Eigen::Vector4<int64_t>(T(i, 0), T(i, 1), T(i, 2), T(i, 3)));
            Tags.push_back(Tag(i, 0));
        }
    }

    Tag.resize(Tets.size(), 1);
    Eigen::MatrixX<int64_t> tempT(Tets.size(), 4);
    for (unsigned int i = 0; i < Tets.size(); i++) {
        tempT.row(i) << Tets[0].x(), Tets[0].y(), Tets[0].z(), Tets[0].w();
        Tag.row(i) << Tags[i];
    }

    TetMesh mesh;
    mesh.initialize(tempT);
    wmtk::attribute::MeshAttributeHandle v_handle =
        mesh.register_attribute<double>("vertices", PrimitiveType::Vertex, 3);
    wmtk::attribute::Accessor v_acc = mesh.create_accessor<double>(v_handle);

    std::vector<Tuple> vtuples = mesh.get_all(PrimitiveType::Vertex);
    for (unsigned int i = 0; i < tempT.rows(); i++) {
        for (unsigned int j = 0; j < 4; j++) {
            unsigned int idx = tempT(i, j);
            v_acc.vector_attribute(vtuples[idx]) = V.row(idx);
        }
    }

    mesh.consolidate();

    wmtk::utils::EigenMatrixWriter writer;
    mesh.serialize(writer);
    writer.get_position_matrix(V);
    writer.get_TV_matrix(T);
}

void merge(
    Eigen::MatrixXd& V1,
    Eigen::MatrixX<int64_t>& T1,
    Eigen::MatrixX<int64_t>& Tag1,
    Eigen::MatrixXd& V2,
    Eigen::MatrixX<int64_t>& T2,
    Eigen::MatrixX<int64_t>& Tag2)
{
    std::map<Eigen::RowVector3d, unsigned int, VectorComparer> mapping_for_V1;
    // todo
    // ...

    merge(V1, T1, Tag1, V2, T2, Tag2, mapping_for_V1);
}


void merge(
    Eigen::MatrixXd& V1,
    Eigen::MatrixX<int64_t>& T1,
    Eigen::MatrixX<int64_t>& Tag1,
    Eigen::MatrixXd& V2,
    Eigen::MatrixX<int64_t>& T2,
    Eigen::MatrixX<int64_t>& Tag2,
    std::map<Eigen::RowVector3d, unsigned int, VectorComparer>& mapping_for_V1)
{
    std::vector<Eigen::RowVector3d> V2prime;

    unsigned int next_p_idx = V1.rows();
    for (unsigned int i = 0; i < V2.rows(); i++) {
        unsigned int cnt = mapping_for_V1.count(V2.row(i));
        if (cnt > 1) {
            std::runtime_error("mapping has repeated boundary vertices");
        }
        if (cnt == 0) {
            mapping_for_V1[V2.row(i)] = next_p_idx;
            V2prime.push_back(V2.row(i));
            next_p_idx++;
        }
    }

    // adjust T
    for (unsigned int i = 0; i < T2.rows(); i++) {
        unsigned int idx0, idx1, idx2, idx3;
        idx0 = mapping_for_V1[V2.row(T2(i, 0))];
        idx1 = mapping_for_V1[V2.row(T2(i, 1))];
        idx2 = mapping_for_V1[V2.row(T2(i, 2))];
        idx3 = mapping_for_V1[V2.row(T2(i, 3))];
        T2.row(i) << idx0, idx1, idx2, idx3;
    }

    // adjust V
    V2.resize(V2prime.size(), 3);
    for (unsigned int i = 0; i < V2prime.size(); i++) {
        V2.row(i) = V2prime[i];
    }

    Eigen::MatrixXd newV(next_p_idx, 3);
    Eigen::MatrixX<int64_t> newT(T1.rows() + T2.rows(), 4);
    Eigen::MatrixX<int64_t> newTag(Tag1.rows() + Tag2.rows(), 1);

    newV << V1, V2;
    newT << T1, T2;
    newTag << Tag1, Tag2;
    T1 = newT;
    Tag1 = newTag;
    V1 = newV;
}

void loadFromRawData(
    unsigned int xb,
    unsigned int yb,
    unsigned int zb,
    unsigned int xe,
    unsigned int ye,
    unsigned int ze,
    const std::string& folder,
    VolumetricData& data)
{
    for (unsigned int z = zb; z <= ze; ++z) {
        std::string filename = folder + "/z" + std::to_string(z) + ".dat";

        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open()) {
            std::cerr << "Error: Failed to open file " << filename << std::endl;
            continue;
        }

        unsigned int rows, cols;
        file.read(reinterpret_cast<char*>(&rows), sizeof(unsigned int));
        file.read(reinterpret_cast<char*>(&cols), sizeof(unsigned int));

        std::vector<std::vector<double>> slice(rows, std::vector<double>(cols));

        for (unsigned int y = 0; y < rows; ++y) {
            for (unsigned int x = 0; x < cols; ++x) {
                double value;
                file.read(reinterpret_cast<char*>(&value), sizeof(double));
                slice[y][x] = value;
            }
        }

        data.push_back(slice);

        file.close();
    }
}

void simplify()
{
    // todo
}
} // namespace wmtk