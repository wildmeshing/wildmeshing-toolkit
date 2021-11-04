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
    typedef Eigen::Vector3d Vector3f; // float?
    typedef Eigen::Vector3d Vector3;

    typedef double Scalar;

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

