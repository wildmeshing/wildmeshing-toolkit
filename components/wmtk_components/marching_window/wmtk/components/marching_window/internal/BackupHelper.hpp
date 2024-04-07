#pragma once

#include <map>
#include <string>
#include <vector>
#include "wmtk/TetMesh.hpp"

namespace wmtk {

using VolumetricData = std::vector<std::vector<std::vector<double>>>;

struct VectorComparer
{
    bool operator()(const Eigen::RowVector3d& a, const Eigen::RowVector3d& b) const
    {
        return std::lexicographical_compare(
            a.data(),
            a.data() + a.size(),
            b.data(),
            b.data() + b.size());
    }
};

void backup(TetMesh& mesh, std::string tag_name, std::string filename);

void getbackup(
    Eigen::MatrixXd& V,
    Eigen::MatrixX<int64_t>& T,
    Eigen::MatrixX<int64_t>& Tag,
    std::string filename);

/**
 *    _ _ _
 *  /     /| <-- level 2
 * /_ _ _/ |
 * |_ _ _|/| <-- level 1
 * |_ _ _|/
 *
 * level 1
 *  2 3
 * 0 1
 * level 2
 *  6 7
 * 4 5
 */
void getsection(
    Eigen::MatrixXd& V,
    Eigen::MatrixX<int64_t>& T,
    Eigen::MatrixX<int64_t>& Tag,
    double x_half,
    double y_half,
    double z_half,
    unsigned int section_idx);

void merge(
    Eigen::MatrixXd& V1,
    Eigen::MatrixX<int64_t>& T1,
    Eigen::MatrixX<int64_t>& Tag1,
    Eigen::MatrixXd& V2,
    Eigen::MatrixX<int64_t>& T2,
    Eigen::MatrixX<int64_t>& Tag2);

void merge(
    Eigen::MatrixXd& V1,
    Eigen::MatrixX<int64_t>& T1,
    Eigen::MatrixX<int64_t>& Tag1,
    Eigen::MatrixXd& V2,
    Eigen::MatrixX<int64_t>& T2,
    Eigen::MatrixX<int64_t>& Tag2,
    std::map<Eigen::RowVector3d, unsigned int, VectorComparer>& mapping_for_V1);

void loadFromRawData(
    unsigned int xb,
    unsigned int yb,
    unsigned int zb,
    unsigned int xe,
    unsigned int ye,
    unsigned int ze,
    const std::string& folder,
    VolumetricData& data);

void simplify();

} // namespace wmtk