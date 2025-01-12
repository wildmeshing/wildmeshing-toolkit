#pragma once
#include <numeric>
#include <vector>

namespace wmtk::utils {

class DisjointSet
{
public:
    DisjointSet(size_t size);

    void merge(size_t a, size_t b);

    size_t get_root(size_t a) const;
    bool is_root(size_t a) const;

    std::vector<size_t> roots() const;


private:
    void merge_sorted(size_t lower, size_t higher);
    // overwrites all roots to simplify the code
    size_t get_root_recursive(size_t a);
    std::vector<size_t> _parents;
};
inline DisjointSet::DisjointSet(size_t size)
    : _parents(size)
{
    std::iota(_parents.begin(), _parents.end(), 0);
}


inline bool DisjointSet::is_root(size_t a) const
{
    size_t r = _parents[a];
    return r == a;
}
inline size_t DisjointSet::get_root(size_t a) const
{
    size_t r = _parents[a];
    while (r != a) {
        a = r;
        r = _parents[a];
    }
    return r;
}
inline size_t DisjointSet::get_root_recursive(size_t a)
{
    size_t& r = _parents[a];
    if (r == a) {
        return a;
    } else {
        return r = get_root_recursive(r);
    }
}
inline void DisjointSet::merge(size_t a, size_t b)
{
    a = get_root_recursive(a);
    b = get_root_recursive(b);
    if (a > b) {
        _parents[a] = b;
    } else {
        _parents[b] = a;
    }
}
inline std::vector<size_t> DisjointSet::roots() const
{
    std::vector<size_t> r;
    r.reserve(_parents.size());
    for (size_t j = 0; j < _parents.size(); ++j) {
        if (is_root(j)) {
            r.emplace_back(j);
        }
    }
    return r;
}
} // namespace wmtk::utils
