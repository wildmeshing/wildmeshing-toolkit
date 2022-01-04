//
// Created by Yixin Hu on 1/4/22.
//

#pragma once

#include <Eigen/Core>

namespace wmtk {
template<typename T>
bool segment_triangle_intersection(const std::array<Eigen::Matrix<T, 3, 1>, 2>& seg,
                                   const std::array<Eigen::Matrix<T, 3, 1>, 3>& tri,
                                   Eigen::Matrix<T, 3, 1>& p){//open segment

    return true;
}

template<typename T>
void squeeze_points_to_2d(const std::vector<Eigen::Matrix<T, 3, 1>>& points3,
                          const std::vector<Eigen::Matrix<T, 2, 1>>& points2){

}

template<typename T>
bool is_point_inside_triangle(const Eigen::Matrix<T, 2, 1>& p, const std::array<Eigen::Matrix<T, 2, 1>, 3>& tri){
    return true;
}

}
