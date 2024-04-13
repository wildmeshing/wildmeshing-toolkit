#include "Extract.hpp"

#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <filesystem>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/mesh_utils.hpp>

namespace wmtk::components::internal {
void extract_triangle_soup_from_image(std::string output_path, std::string filename, double delta_x)
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
                Eigen::RowVector3d v0(i * delta_x, (j + 1) * delta_x, k * delta_x);
                Eigen::RowVector3d v1(i * delta_x, (j + 1) * delta_x, (k + 1) * delta_x);
                Eigen::RowVector3d v2(i * delta_x, j * delta_x, (k + 1) * delta_x);
                Eigen::RowVector3d v3((i + 1) * delta_x, (j + 1) * delta_x, k * delta_x);
                Eigen::RowVector3d v4((i + 1) * delta_x, (j + 1) * delta_x, (k + 1) * delta_x);
                Eigen::RowVector3d v5((i + 1) * delta_x, j * delta_x, (k + 1) * delta_x);
                Eigen::RowVector3d v6((i + 1) * delta_x, j * delta_x, k * delta_x);

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

    int dim1, dim2, dim3;
    file.read(reinterpret_cast<char*>(&dim1), sizeof(int));
    file.read(reinterpret_cast<char*>(&dim2), sizeof(int));
    file.read(reinterpret_cast<char*>(&dim3), sizeof(int));

    data.resize(
        dim1,
        std::vector<std::vector<unsigned int>>(dim2, std::vector<unsigned int>(dim3)));

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

void octree_add_points(const Eigen::MatrixXd& V, unsigned int max_level)
{
    Eigen::VectorXd min_vals = V.colwise().minCoeff();
    Eigen::VectorXd max_vals = V.colwise().maxCoeff();
    std::map<Eigen::Vector3d, bool, VectorComparer> record;
    std::vector<Eigen::Vector3d> sub_points;
    octree_add_points(
        record,
        sub_points,
        20,
        min_vals.x() - 0.5,
        min_vals.y() - 0.5,
        min_vals.z() - 0.5,
        max_vals.x() + 0.5,
        max_vals.y() + 0.5,
        max_vals.z() + 0.5);
}

void octree_add_points(
    std::map<Eigen::Vector3d, bool, VectorComparer>& record,
    const std::vector<Eigen::Vector3d>& sub_points,
    unsigned int itr,
    double min_x,
    double min_y,
    double min_z,
    double max_x,
    double max_y,
    double max_z)
{
    if (itr == 0) {
        return;
    }
    if (sub_points.empty()) {
        return;
    }

    double mid_x = 0.5 * (min_x + max_x);
    double mid_y = 0.5 * (min_y + max_y);
    double mid_z = 0.5 * (min_z + max_z);

    // add 7 points
    Eigen::Vector3d p0(mid_x, mid_y, mid_z);
    Eigen::Vector3d p1(mid_x, mid_y, min_z);
    Eigen::Vector3d p2(min_x, mid_y, mid_z);
    Eigen::Vector3d p3(max_x, mid_y, mid_z);
    Eigen::Vector3d p4(mid_x, mid_y, max_z);
    Eigen::Vector3d p5(mid_x, max_y, mid_z);
    Eigen::Vector3d p6(mid_x, min_y, mid_z);
    if (!is_int(mid_x) && !is_int(mid_y) && !is_int(mid_z)) {
        record[p0] = true;
    }
    if (!is_int(mid_x) && !is_int(mid_y) && !is_int(min_z)) {
        record[p1] = true;
    }
    if (!is_int(min_x) && !is_int(mid_y) && !is_int(mid_z)) {
        record[p2] = true;
    }
    if (!is_int(max_x) && !is_int(mid_y) && !is_int(mid_z)) {
        record[p3] = true;
    }
    if (!is_int(mid_x) && !is_int(mid_y) && !is_int(max_z)) {
        record[p4] = true;
    }
    if (!is_int(mid_x) && !is_int(max_y) && !is_int(mid_z)) {
        record[p5] = true;
    }
    if (!is_int(mid_x) && !is_int(min_y) && !is_int(mid_z)) {
        record[p6] = true;
    }


    // divide more sub-domains
    std::vector<Eigen::Vector3d> sub_points0;
    std::vector<Eigen::Vector3d> sub_points1;
    std::vector<Eigen::Vector3d> sub_points2;
    std::vector<Eigen::Vector3d> sub_points3;
    std::vector<Eigen::Vector3d> sub_points4;
    std::vector<Eigen::Vector3d> sub_points5;
    std::vector<Eigen::Vector3d> sub_points6;
    std::vector<Eigen::Vector3d> sub_points7;
    for (const Eigen::Vector3d p : sub_points) {
        if (p.x() < mid_x && p.y() < mid_y && p.z() < mid_z) {
            sub_points0.push_back(p);
        } else if (p.x() > mid_x && p.y() < mid_y && p.z() < mid_z) {
            sub_points1.push_back(p);
        } else if (p.x() < mid_x && p.y() < mid_y && p.z() > mid_z) {
            sub_points2.push_back(p);
        } else if (p.x() > mid_x && p.y() < mid_y && p.z() > mid_z) {
            sub_points3.push_back(p);
        } else if (p.x() < mid_x && p.y() > mid_y && p.z() < mid_z) {
            sub_points4.push_back(p);
        } else if (p.x() > mid_x && p.y() > mid_y && p.z() < mid_z) {
            sub_points5.push_back(p);
        } else if (p.x() < mid_x && p.y() > mid_y && p.z() > mid_z) {
            sub_points6.push_back(p);
        } else if (p.x() > mid_x && p.y() > mid_y && p.z() > mid_z) {
            sub_points7.push_back(p);
        }
    }

    // recursively adding points
    octree_add_points(record, sub_points0, itr - 1, min_x, min_y, min_z, mid_x, mid_y, mid_z);


    octree_add_points(record, sub_points1, itr - 1, mid_x, min_y, min_z, max_x, mid_y, mid_z);


    octree_add_points(record, sub_points2, itr - 1, min_x, mid_y, min_z, mid_x, max_y, mid_z);


    octree_add_points(record, sub_points3, itr - 1, mid_x, mid_y, min_z, max_x, max_y, mid_z);


    octree_add_points(record, sub_points4, itr - 1, min_x, min_y, mid_z, mid_x, mid_y, max_z);


    octree_add_points(record, sub_points5, itr - 1, mid_x, min_y, mid_z, max_x, mid_y, max_z);


    octree_add_points(record, sub_points6, itr - 1, min_x, mid_y, mid_z, mid_x, max_y, max_z);


    octree_add_points(record, sub_points7, itr - 1, mid_x, mid_y, mid_z, max_x, max_y, max_z);
}


bool is_int(double v)
{
    return floor(v) == ceil(v);
}

} // namespace wmtk::components::internal
