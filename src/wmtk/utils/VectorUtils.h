#pragma once

#include <algorithm>
#include <array>
#include <iostream>
#include <map>
#include <set>
#include <vector>
#include "wmtk/utils/Logger.hpp"

namespace wmtk {

template <class T>
inline std::vector<T> set_intersection(const std::vector<T>& v1, const std::vector<T>& v2)
{
    if (v1.size() > 1) {
        assert(std::is_sorted(v1.begin(), v1.end()));
    }
    if (v2.size() > 1) {
        assert(std::is_sorted(v2.begin(), v2.end()));
    }

    std::vector<T> v;
    v.reserve(std::min(v1.size(), v2.size()));
    std::set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), std::back_inserter(v));
    return v;
}

template <class T>
inline void vector_unique(std::vector<T>& v)
{
    if (v.size() > 1) {
        std::sort(v.begin(), v.end());
        v.erase(std::unique(v.begin(), v.end()), v.end());
    }
}

template <class T, typename Comp, typename Equal>
inline void vector_unique(std::vector<T>& v, Comp comp, Equal equal)
{
    if (v.size() > 1) {
        std::sort(v.begin(), v.end(), comp);
        v.erase(std::unique(v.begin(), v.end(), equal), v.end());
    }
}


template <class T>
inline void vector_print(std::vector<T>& v)
{
    wmtk::logger().info("vector {}", v);
}

template <class T>
inline void vector_sort(std::vector<T>& v)
{
    if (v.size() > 1) {
        std::sort(v.begin(), v.end());
    }
}

template <class T>
inline bool vector_erase(std::vector<T>& v, const T& t)
{
    auto it = std::find(v.begin(), v.end(), t);
    if (it == v.end()) return false;
    v.erase(it);
    return true;
}

template <class T>
inline bool vector_contains(const std::vector<T>& v, const T& t)
{
    auto it = std::find(v.begin(), v.end(), t);
    if (it == v.end()) return false;
    return true;
}

template <typename T>
inline bool set_erase(std::vector<T>& v, const T& t)
{
    assert(std::is_sorted(v.begin(), v.end()));
    auto it = std::lower_bound(v.begin(), v.end(), t);
    if (it == v.end() || *it != t) return false; // not found
    v.erase(it);
    return true;
}

template <typename T>
inline bool set_insert(std::vector<T>& vec, const T& val)
{
    assert(std::is_sorted(vec.begin(), vec.end()));
    auto it = std::lower_bound(vec.begin(), vec.end(), val);
    vec.insert(it, val);
    return true;
}
template <typename T>
std::vector<T> set_union(const std::vector<T>& vec, const std::vector<T>& vec2)
{
    assert(std::is_sorted(vec.begin(), vec.end()));
    assert(std::is_sorted(vec2.begin(), vec2.end()));
    std::vector<T> merged;
    merged.reserve(vec2.size() + vec.size());
    std::merge(vec.begin(), vec.end(), vec2.begin(), vec2.end(), std::back_inserter(merged));
    vector_unique(merged);
    return merged;
}

template <typename T>
void set_union_inplace(std::vector<T>& vec, const std::vector<T>& vec2)
{
    assert(std::is_sorted(vec.begin(), vec.end()));
    assert(std::is_sorted(vec2.begin(), vec2.end()));
    // boundary_it specifies the boundary between vec and vec2 data
    const auto boundary_it = vec.insert(vec.end(), vec2.begin(), vec2.end());
    std::inplace_merge(vec.begin(), boundary_it, vec.end());
    vector_unique(vec);
}

template <typename T, size_t N>
inline void array_replace_inline(std::array<T, N>& arr, const T& v0, const T& v1)
{
    for (auto j = 0; j < N; j++) {
        if (arr[j] == v0) {
            arr[j] = v1;
            break;
        }
    }
}

template <typename T, size_t N>
inline std::array<T, N> array_replace(const std::array<T, N>& arr, const T& v0, const T& v1)
{
    std::array<size_t, 4> out = arr;
    array_replace_inline(out, v0, v1);
    return out;
}

} // namespace wmtk
