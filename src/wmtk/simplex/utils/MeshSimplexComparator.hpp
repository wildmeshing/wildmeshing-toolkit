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
#if defined(WMTK_ENABLE_SIMPLEX_ID_CACHING)
                return SimplexComparisons::less(*a_mesh_ptr, a_simplex, b_simplex);
#else
                if (a_simplex.primitive_type() == b_simplex.primitive_type()) {
                    return a_simplex.tuple() < b_simplex.tuple();
                } else {
                    return a_simplex.primitive_type() < b_simplex.primitive_type();
                }
#endif
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

                return a_simplex.tuple() == b_simplex.tuple() &&
                       a_simplex.primitive_type() == b_simplex.primitive_type();
#endif
            } else {
                return false;
            }
        }
    };
};
} // namespace wmtk::simplex::utils
