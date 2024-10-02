#include "SplitAlternateFacetOptionData.hpp"
#include <wmtk/autogen/utils/largest_shared_subdart_size.hpp>
#include <wmtk/operations/internal/ear_actions.hpp>


namespace wmtk::operations::internal {
namespace {
// hi!

using Pair = std::array<int8_t, 2>;
using PairPair = std::array<Pair, 2>;
template <int8_t Dim>
auto make_inds()
{
    auto run = [](auto size_const) {
        using T = decltype(size_const);
        constexpr static size_t C = T::value;
        std::array<PairPair, C> ret = {};
        const auto& sd = wmtk::autogen::SimplexDart(PrimitiveType(Dim));
        const auto boundary_type = PrimitiveType(Dim - 1);
        for (size_t j = 0; j < ret.size(); ++j) {
            auto& r = ret[j];
            auto& main = std::get<0>(r);
            auto& dual = std::get<1>(r);
            const int8_t edge_index = j;

            for (int8_t k = 0; k < sd.size(); ++k) {
                if (sd.simplex_index(k, wmtk::PrimitiveType::Edge) == edge_index) {
                    const int8_t left = sd.product(left_ear_action(sd.simplex_type()), k);
                    const int8_t right = sd.product(right_ear_action(sd.simplex_type()), k);
                    main[0] = sd.simplex_index(left, boundary_type);
                    main[1] = sd.simplex_index(right, boundary_type);
                    break;
                }
            }


            dual[1] = main[0];
            dual[0] = main[1];
        }
        return ret;
    };
    if constexpr (Dim == 1) {
        return run(std::integral_constant<size_t, 1>{});
    } else if constexpr (Dim == 2) {
        return run(std::integral_constant<size_t, 3>{});
    } else if constexpr (Dim == 3) {
        return run(std::integral_constant<size_t, 6>{});
    }
}
template <size_t ArrSize>
const std::array<int8_t, 2>& boundary_indices_(
    const wmtk::autogen::SimplexDart& sd,
    int8_t o,
    const std::array<PairPair, ArrSize>& arr)
{
    const int8_t edge_index = sd.simplex_index(o, wmtk::PrimitiveType::Edge);
    const int8_t boundary_index =
        sd.simplex_index(sd.product(sd.opposite(), o), sd.simplex_type() - 1);
    const PairPair& pairpair = arr.at(edge_index);
    const Pair& pair = std::get<0>(pairpair);
    static_assert(std::tuple_size_v<Pair> == 2);
    if (boundary_index == std::get<1>(pair)) {
        return pairpair[0];
    } else {
        return pairpair[1];
    }
}

const std::array<int8_t, 2>& boundary_indices_(
    const wmtk::autogen::SimplexDart& sd,
    int8_t orientation)
{
    const static std::array<PairPair, 1> edge_inds = make_inds<1>();
    const static std::array<PairPair, 3> tri_inds = make_inds<2>();
    const static std::array<PairPair, 6> tet_inds = make_inds<3>();


    switch (sd.simplex_type()) {
    case PrimitiveType::Tetrahedron: return boundary_indices_(sd, orientation, tet_inds);
    case PrimitiveType::Triangle: return boundary_indices_(sd, orientation, tri_inds);
    case PrimitiveType::Edge: return boundary_indices_(sd, orientation, edge_inds);
    case PrimitiveType::Vertex:
    default: break;
    }
    constexpr static Pair empty = {};
    assert(false);
    return empty;
}
} // namespace
auto SplitAlternateFacetOptionData::boundary_indices(PrimitiveType mesh_type) const
    -> const std::array<int8_t, 2>&
{
    const auto& sd = wmtk::autogen::SimplexDart::get_singleton(mesh_type);
    const auto& boundary_inds = boundary_indices_(sd, input.local_orientation());
    return boundary_inds;
}

SplitAlternateFacetOptionData::SplitAlternateFacetOptionData(
    const Dart& i,
    const std::array<int64_t, 2>& n)
    : input(i)
    , new_facet_indices(n)
{}


auto SplitAlternateFacetOptionData::new_gid(PrimitiveType mesh_type, int8_t orientation) const
    -> int64_t
{
    const auto& sd = wmtk::autogen::SimplexDart::get_singleton(mesh_type);
    const auto& boundary_simplex_indices = boundary_indices(mesh_type);


    const PrimitiveType boundary_type = mesh_type - 1;
    // the local indices of the ears of faces

    auto get_size = [&](size_t index) {
        return wmtk::autogen::utils::largest_shared_subdart_size(
            mesh_type,
            orientation,
            boundary_type,
            boundary_simplex_indices[index]);
    };
    const std::array<int8_t, 2> sizes{{get_size(0), get_size(1)}};
    if (sizes[0] >= sizes[1]) {
        return new_facet_indices[0];
    } else {
        return new_facet_indices[1];
    }
}


} // namespace wmtk::operations::internal
