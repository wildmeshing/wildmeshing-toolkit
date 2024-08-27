#include "SplitAlternateFacetOptionData.hpp"
#include <wmtk/autogen/utils/largest_shared_subdart_size.hpp>


namespace wmtk::operations::internal {
namespace {
// hi!

using Pair = std::array<int8_t, 2>;
using PairPair = std::array<Pair, 2>;
template <int8_t Dim>
auto make_inds()
{
    const auto& sd = wmtk::autogen::SimplexDart(PrimitiveType(Dim));
    auto run = [](auto& ret) {
        for (size_t j = 0; j < ret.size(); ++j) {
        }
    };
    if constexpr (Dim == 1) {
        std::array<PairPair, 1> ret;
        run(ret);
        return ret;
    } else if constexpr (Dim == 2) {
        std::array<PairPair, 3> ret;
        run(ret);
        return ret;
    } else if constexpr (Dim == 3) {
        std::array<PairPair, 6> ret;
        run(ret);
        return ret;
    }
}

const std::array<int8_t, 2>& boundary_indices_(
    const wmtk::autogen::SimplexDart& sd,
    int8_t orientation)
{
    const static std::array<PairPair, 1> edge_inds = make_inds<1>();
    const static std::array<PairPair, 3> tri_inds = make_inds<2>();
    const static std::array<PairPair, 6> tet_inds = make_inds<3>();

    auto run = [](const wmtk::autogen::SimplexDart& sd_,
                  int8_t o,
                  const auto& arr) -> const std::array<int8_t, 2>& {
        const int8_t edge_index = sd_.simplex_index(o, wmtk::PrimitiveType::Edge);
        const int8_t boundary_index =
            sd_.simplex_index(sd_.product(sd_.opposite(), o), sd_.simplex_type() - 1);
        const auto& pair = arr[edge_index];
        if (boundary_index == pair[0][1]) {
            return pair[0];
        } else {
            return pair[1];
        }
    };

    switch (sd.simplex_type()) {
    case PrimitiveType::Tetrahedron: return run(sd, orientation, tet_inds);
    case PrimitiveType::Triangle: return run(sd, orientation, tri_inds);
    case PrimitiveType::Edge: return run(sd, orientation, edge_inds);
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
