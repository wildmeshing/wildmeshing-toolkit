#pragma once
#include <wmtk/autogen/point_mesh/SimplexDart.hpp>
#include <wmtk/autogen/edge_mesh/SimplexDart.hpp>
#include <wmtk/autogen/tri_mesh/SimplexDart.hpp>
#include <wmtk/autogen/tet_mesh/SimplexDart.hpp>
#include <wmtk/autogen/SimplexDart.hpp>

namespace wmtk::autogen {
    template <int Dim>
    constexpr auto get_simplex_dart() {
        static_assert(Dim > 0);
        static_assert(Dim <= 4);
        switch(Dim) {
            case 1:
                return point_mesh::SimplexDart{};
            case 2:
                return edge_mesh::SimplexDart{};
            case 3:
                return tri_mesh::SimplexDart{};
            case 4:
                return tet_mesh::SimplexDart{};
            default:
                return nullptr;
        }
    }
}
