#include "SplitAlternateFacetOptionData.hpp"


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

const std::array<int8_t, 2>& boundary_indices(
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
}

SplitAlternateFacetOptionData::SplitAlternateFacetOptionData(
    const autogen::SimplexDart& sd,
    const Dart& input)
    : input(input)
{}


auto SplitAlternateFacetOptionData::new_gid(PrimitiveType mesh_type, int8_t orientation) const
    -> int64_t
{
    const auto& sd = wmtk::autogen::SimplexDart::get_singleton(mesh_type);
    const auto& boundaries = boundary_indices(sd, input.local_orientation());

    // sd.simplex_index(orientation, wmtk::PrimitiveType::Edge);
    wmtk::autogen::Dart dart = sd.dart_from_tuple(t);

    const PrimitiveType boundary_type = mesh_pt - 1;
    auto get_ear_max_subdart_size = [&](int8_t action) {
        const int8_t orientation = sd.product(action, dart.local_orientation());
        const int8_t simplex = sd.simplex_index(orientation, boundary_type);
    };

    const int8_t left_ear_max_subdart_size = get_ear_max_subdart_size(left_ear_action(mesh_pt));
    const int8_t right_ear_max_subdart_size = get_ear_max_subdart_size(right_ear_action(mesh_pt));


    else if (int8_t preserved_subdarts_to_face = 0; preserved_subdarts_to_face > 0)
    { // TODO: make this do something

        new_global_cid = alts[0];
    }
    else
    {
        new_global_cid = alts[1];
    }

} // namespace wmtk::operations::internal
