#pragma once
#include <tuple>
#include "SimplexComparisons.hpp"

namespace wmtk::simplex::utils {

class MeshSimplexComparator
{
public:
    using KeyType = std::tuple<const Mesh*, simplex::Simplex>;
    using KeyType2 = std::tuple<const Mesh*, int64_t>;

    class Less
    {
    public:
        bool operator()(const KeyType& a, const KeyType& b) const
        {
            const auto& [a_mesh_ptr, a_simplex] = a;
            const auto& [b_mesh_ptr, b_simplex] = b;

            if (a_mesh_ptr == b_mesh_ptr) {
#if defined(WMTK_ENABLE_SIMPLEX_ID_CACHING)
                return SimplexComparisons::less(*a_mesh_ptr, a_simplex, b_simplex);
#else
                return SimplexComparisons::less_subdart(*a_mesh_ptr, a_simplex, b_simplex);
                // if (a_simplex.primitive_type() == b_simplex.primitive_type()) {
                //     return a_simplex.tuple() < b_simplex.tuple();
                // } else {
                //     return a_simplex.primitive_type() < b_simplex.primitive_type();
                // }
#endif
            } else {
                return a_mesh_ptr < b_mesh_ptr;
            }
        }
        bool operator()(const KeyType2& a, const KeyType2& b) const
        {
            const auto& [a_mesh_ptr, a_simplex] = a;
            const auto& [b_mesh_ptr, b_simplex] = b;

            if (a_mesh_ptr == b_mesh_ptr) {
                return a_simplex < b_simplex;
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
#if defined(WMTK_ENABLE_SIMPLEX_ID_CACHING)
                return SimplexComparisons::equal(*a_mesh_ptr, a_simplex, b_simplex);
#else
                return SimplexComparisons::equal(*a_mesh_ptr, a_simplex, b_simplex);

                // return a_simplex.tuple() == b_simplex.tuple() &&
                //        a_simplex.primitive_type() == b_simplex.primitive_type();
#endif
            } else {
                return false;
            }
        }
        bool operator()(const KeyType2& a, const KeyType2& b) const
        {
            const auto& [a_mesh_ptr, a_simplex] = a;
            const auto& [b_mesh_ptr, b_simplex] = b;

            if (a_mesh_ptr == b_mesh_ptr) {
                return a_simplex == b_simplex;
            } else {
                return false;
            }
        }
    };
};
} // namespace wmtk::simplex::utils
