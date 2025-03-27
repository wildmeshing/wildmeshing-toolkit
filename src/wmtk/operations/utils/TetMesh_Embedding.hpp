#pragma once

#include <Eigen/Core>
#include <unordered_set>
#include <vector>

namespace wmtk::operations::utils {

/**
 * @brief 分析四面体网格和边界面的关系，找出内部顶点
 * @param T 四面体网格的连接信息
 * @param F_bd 边界面的连接信息
 * @return 内部顶点的索引列表
 */
std::vector<int> embed_mesh(const Eigen::MatrixXi& T, const Eigen::MatrixXi& F_bd);

/**
 * @brief 找出由内部顶点构成的所有三角面
 * @param non_bd_vertices 内部顶点的集合
 * @param T 四面体网格的连接信息
 * @return 由内部顶点构成的三角面矩阵
 */
Eigen::MatrixXi find_F_top(
    const std::unordered_set<int>& non_bd_vertices,
    const Eigen::MatrixXi& T);

} // namespace wmtk::operations::utils