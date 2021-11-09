//
// Created by Yixin Hu on 10/12/21.
//

#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <array>
#include <Eigen/Core>

namespace wmtk {
    using std::cout;
    using std::cin;
    using std::endl;

#define MAX_ENERGY 1e50

    typedef Eigen::Vector3d Vector3f; // float?
    typedef Eigen::Vector3d Vector3;

    typedef double Scalar;
    typedef Eigen::Matrix<Scalar, 3, 3> Matrix3;

    std::vector<size_t> set_intersection(const std::vector<size_t>& v1, const std::vector<size_t>& v2);

    template<class T>
    void vector_unique(std::vector<T>& v){
        //todo
    }

    template<class T>
    void vector_erase(std::vector<T>& v, const T& t){
        //todo
    }
}

