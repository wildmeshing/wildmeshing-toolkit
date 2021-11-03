//
// Created by Yixin Hu on 10/12/21.
//

#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <array>

namespace wmtk {
    typedef std::array<double, 3> Vector3f; // float?
    typedef std::array<double, 3> Vector3;

    typedef double Scalar;

    std::vector<size_t> set_intersection(const std::vector<size_t>& v1, const std::vector<size_t>& v2);

    template<class T>
    void vector_unique(std::vector<T>& v){
        //todo
    }
}

