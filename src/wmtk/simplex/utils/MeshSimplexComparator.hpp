#pragma once
#include "SimplexComparisons.hpp"

namespace wmtk::simplex::utils {

class MeshSimplexComparator
{
public:
    using KeyType = std::tuple<const Mesh*, simplex::Simplex>;
    using KeyType2 = std::tuple<const Mesh*, simplex::Simplex, int64_t>;
    using KeyType3 = std::tuple<const Mesh*, simplex::NavigatableSimplex>;

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
        bool operator()(const KeyType2& a, const KeyType2& b) const
        {
            const auto& [a_mesh_ptr, a_simplex, a_id] = a;
            const auto& [b_mesh_ptr, b_simplex, b_id] = b;

            if (a_mesh_ptr == b_mesh_ptr) {
                return a_id == b_id;
            } else {
                return a_mesh_ptr < b_mesh_ptr;
            }
        }
        bool operator()(const KeyType3& a, const KeyType3& b) const
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
        bool operator()(const KeyType2& a, const KeyType2& b) const
        {
            const auto& [a_mesh_ptr, a_simplex, a_id] = a;
            const auto& [b_mesh_ptr, b_simplex, b_id] = b;

            if (a_mesh_ptr == b_mesh_ptr) {
                return a_id == b_id;
            } else {
                return false;
            }
        }
        bool operator()(const KeyType3& a, const KeyType3& b) const
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
