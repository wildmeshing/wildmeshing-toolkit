#include "Extract.hpp"

#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <filesystem>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/mesh_utils.hpp>

namespace wmtk::components::internal {
void extract_triangle_soup_from_image(std::string output_path, std::string filename)
{
    std::vector<std::vector<std::vector<unsigned int>>> data;
    read_array_data(data, filename);
    unsigned int tri_num = 0;
    for (unsigned int i = 0; i < data.size() - 1; i++) {
        for (unsigned int j = 0; j < data[0].size() - 1; j++) {
            for (unsigned int k = 0; k < data[0][0].size() - 1; k++) {
                unsigned int cur_data = data[i][j][k];
                unsigned int right_data = data[i][j][k + 1];
                unsigned int ahead_data = data[i][j + 1][k];
                unsigned int top_data = data[i + 1][j][k];
                if (cur_data != right_data) {
                    tri_num += 2;
                }
                if (cur_data != ahead_data) {
                    tri_num += 2;
                }
                if (cur_data != top_data) {
                    tri_num += 2;
                }
            }
        }
    }

    int target_value;
    Eigen::MatrixXi F(tri_num, 3);
    Eigen::MatrixXd V(3 * tri_num, 3);
    // unsigned int itr = 0;
    // for (unsigned int i = 0; i < data.size(); i++) {
    //     for (unsigned int j = 0; j < data[0].size(); j++) {
    //         for (unsigned int k = 0; k < data[0][0].size(); k++) {
    //             V.row(itr++) << i * delta_x, j * delta_x, k * delta_x;
    //         }
    //     }
    // }

    unsigned int cur_v_id = 0;
    unsigned int cur_f_id = 0;
    for (unsigned int i = 0; i < data.size() - 1; i++) {
        for (unsigned int j = 0; j < data[0].size() - 1; j++) {
            for (unsigned int k = 0; k < data[0][0].size() - 1; k++) {
                unsigned int cur_data = data[i][j][k];
                unsigned int right_data = data[i][j][k + 1];
                unsigned int ahead_data = data[i][j + 1][k];
                unsigned int top_data = data[i + 1][j][k];
                Eigen::RowVector3d v0(i, j + 1, k);
                Eigen::RowVector3d v1(i, j + 1, k + 1);
                Eigen::RowVector3d v2(i, j, k + 1);
                Eigen::RowVector3d v3(i + 1, j + 1, k);
                Eigen::RowVector3d v4(i + 1, j + 1, k + 1);
                Eigen::RowVector3d v5(i + 1, j, k + 1);
                Eigen::RowVector3d v6(i + 1, j, k);

                if (cur_data > right_data) {
                    unsigned int vid0, vid1, vid2, vid3, vid4, vid5, fid0, fid1;
                    vid0 = cur_v_id;
                    vid1 = vid0 + 1;
                    vid2 = vid1 + 1;
                    vid3 = vid2 + 1;
                    vid4 = vid3 + 1;
                    vid5 = vid4 + 1;
                    fid0 = cur_f_id;
                    fid1 = fid0 + 1;
                    cur_v_id += 6;
                    cur_f_id += 2;
                    F.row(fid0) << vid0, vid1, vid2;
                    F.row(fid1) << vid3, vid4, vid5;
                    V.row(vid0) = v2;
                    V.row(vid1) = v5;
                    V.row(vid2) = v4;
                    V.row(vid3) = v2;
                    V.row(vid4) = v4;
                    V.row(vid5) = v1;

                } else if (cur_data < right_data) {
                    unsigned int vid0, vid1, vid2, vid3, vid4, vid5, fid0, fid1;
                    vid0 = cur_v_id;
                    vid1 = vid0 + 1;
                    vid2 = vid1 + 1;
                    vid3 = vid2 + 1;
                    vid4 = vid3 + 1;
                    vid5 = vid4 + 1;
                    fid0 = cur_f_id;
                    fid1 = fid0 + 1;
                    cur_v_id += 6;
                    cur_f_id += 2;
                    F.row(fid0) << vid0, vid1, vid2;
                    F.row(fid1) << vid3, vid4, vid5;
                    V.row(vid0) = v1;
                    V.row(vid1) = v4;
                    V.row(vid2) = v2;
                    V.row(vid3) = v4;
                    V.row(vid4) = v5;
                    V.row(vid5) = v2;
                }

                if (cur_data > ahead_data) {
                    unsigned int vid0, vid1, vid2, vid3, vid4, vid5, fid0, fid1;
                    vid0 = cur_v_id;
                    vid1 = vid0 + 1;
                    vid2 = vid1 + 1;
                    vid3 = vid2 + 1;
                    vid4 = vid3 + 1;
                    vid5 = vid4 + 1;
                    fid0 = cur_f_id;
                    fid1 = fid0 + 1;
                    cur_v_id += 6;
                    cur_f_id += 2;
                    F.row(fid0) << vid0, vid1, vid2;
                    F.row(fid1) << vid3, vid4, vid5;
                    V.row(vid0) = v0;
                    V.row(vid1) = v1;
                    V.row(vid2) = v3;
                    V.row(vid3) = v1;
                    V.row(vid4) = v4;
                    V.row(vid5) = v3;
                } else if (cur_data < ahead_data) {
                    unsigned int vid0, vid1, vid2, vid3, vid4, vid5, fid0, fid1;
                    vid0 = cur_v_id;
                    vid1 = vid0 + 1;
                    vid2 = vid1 + 1;
                    vid3 = vid2 + 1;
                    vid4 = vid3 + 1;
                    vid5 = vid4 + 1;
                    fid0 = cur_f_id;
                    fid1 = fid0 + 1;
                    cur_v_id += 6;
                    cur_f_id += 2;
                    F.row(fid0) << vid0, vid1, vid2;
                    F.row(fid1) << vid3, vid4, vid5;
                    V.row(vid0) = v0;
                    V.row(vid1) = v3;
                    V.row(vid2) = v1;
                    V.row(vid3) = v3;
                    V.row(vid4) = v4;
                    V.row(vid5) = v1;
                }

                if (cur_data > top_data) {
                    unsigned int vid0, vid1, vid2, vid3, vid4, vid5, fid0, fid1;
                    vid0 = cur_v_id;
                    vid1 = vid0 + 1;
                    vid2 = vid1 + 1;
                    vid3 = vid2 + 1;
                    vid4 = vid3 + 1;
                    vid5 = vid4 + 1;
                    fid0 = cur_f_id;
                    fid1 = fid0 + 1;
                    cur_v_id += 6;
                    cur_f_id += 2;
                    F.row(fid0) << vid0, vid1, vid2;
                    F.row(fid1) << vid3, vid4, vid5;
                    V.row(vid0) = v6;
                    V.row(vid1) = v3;
                    V.row(vid2) = v5;
                    V.row(vid3) = v3;
                    V.row(vid4) = v4;
                    V.row(vid5) = v5;
                } else if (cur_data < top_data) {
                    unsigned int vid0, vid1, vid2, vid3, vid4, vid5, fid0, fid1;
                    vid0 = cur_v_id;
                    vid1 = vid0 + 1;
                    vid2 = vid1 + 1;
                    vid3 = vid2 + 1;
                    vid4 = vid3 + 1;
                    vid5 = vid4 + 1;
                    fid0 = cur_f_id;
                    fid1 = fid0 + 1;
                    cur_v_id += 6;
                    cur_f_id += 2;
                    F.row(fid0) << vid0, vid1, vid2;
                    F.row(fid1) << vid3, vid4, vid5;
                    V.row(vid0) = v3;
                    V.row(vid1) = v5;
                    V.row(vid2) = v4;
                    V.row(vid3) = v3;
                    V.row(vid4) = v6;
                    V.row(vid5) = v5;
                }
            }
        }
    }

    igl::writeOFF(output_path, V, F);
    // igl::writeOBJ(output_path, V, F);
}


void read_array_data(
    std::vector<std::vector<std::vector<unsigned int>>>& data,
    const std::string& filename)
{
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << " for reading." << std::endl;
        return;
    }

    // 读取三个维度的大小
    int dim1, dim2, dim3;
    file.read(reinterpret_cast<char*>(&dim1), sizeof(int));
    file.read(reinterpret_cast<char*>(&dim2), sizeof(int));
    file.read(reinterpret_cast<char*>(&dim3), sizeof(int));

    // 调整data的大小
    data.resize(
        dim1,
        std::vector<std::vector<unsigned int>>(dim2, std::vector<unsigned int>(dim3)));

    // 读取数组数据
    for (int i = 0; i < dim1; ++i) {
        for (int j = 0; j < dim2; ++j) {
            for (int k = 0; k < dim3; ++k) {
                unsigned int value;
                file.read(reinterpret_cast<char*>(&value), sizeof(unsigned int));
                data[i][j][k] = value;
            }
        }
    }
}

} // namespace wmtk::components::internal
