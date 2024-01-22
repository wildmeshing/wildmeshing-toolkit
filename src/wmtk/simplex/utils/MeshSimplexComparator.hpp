#pragma once
#include "SimplexComparisons.hpp"

namespace wmtk::simplex::utils {

class MeshSimplexComparator
{
public:
    using KeyType = std::tuple<const Mesh*, simplex::Simplex>;

    class Less
    {
    public:
        bool operator()(const KeyType& a, const KeyType& b) const
        {
            const auto& [a_mesh_ptr, a_simplex] = a;
            const auto& [b_mesh_ptr, b_simplex] = b;

            if (a_mesh_ptr == b_mesh_ptr) {
                return SimplexComparisons::less(*a_mesh_ptr, a_simplex, b_simplex);
            } else {
                return a_mesh_ptr < b_mesh_ptr;
            }
        }
    };

    class Equal
    {
    public:
        bool operator()(const KeyType& a, const KeyType& b) const
        {
            const auto& [a_mesh_ptr, a_simplex] = a;
            const auto& [b_mesh_ptr, b_simplex] = b;

            if (a_mesh_ptr == b_mesh_ptr) {
                return SimplexComparisons::equal(*a_mesh_ptr, a_simplex, b_simplex);
            } else {
                return false;
            }
        }
    };
};
} // namespace wmtk::simplex::utils
