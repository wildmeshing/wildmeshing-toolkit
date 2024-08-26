#pragma once
#include <array>
#include <wmtk/autogen/Dart.hpp>
#include <wmtk/autogen/SimplexDart.hpp>
namespace wmtk {
class Mesh;
namespace autogen {
class SimplexDart;
}
} // namespace wmtk
namespace wmtk::operations::internal {
class SplitAlternateFacetOptionData
{
public:
    using Dart = autogen::Dart;
    SplitAlternateFacetOptionData(const Mesh& m, const Tuple& input_tuple);
    SplitAlternateFacetOptionData(
        const Mesh& m,
        const autogen::SimplexDart& sd,
        const Tuple& input_tuple);
    autogen::Dart input;

    // Stores {ear_global_id, M}
    // where M is defined by:
    // Let {G, O} be the input dart
    // Let {Ge, Oe} be the left/0 or right/1 ear opposite
    // Let R be such that Oe = R O
    // We select an arbitrary dart that includes the right face
    // {G,Oa} a dart that includes the ear face, and
    // {Ge,Ob} a dart denoting a art on Ge whose D-1 subdart is equivalent
    // Let M be  Ob = M Oa.
    // Note that for D-1 subdart encoded on G, M will return the equivalent D-1 subdart on Ge
    //
    std::array<int8_t, 2> alts;
    std::array<int8_t, 2> local_boundary_indices;

    // Let d be a dart where every D-simplex for D <the input mesh dimension
    // lies in left/index=0 (equivalently right/index=1) ear then
    // returns a dart such that those simplices are preserved using the other face
    //
    // This is given by the definition of alts and applying
    // Let {G, Od} be d
    // We compute {G, M Od}
    //
    Dart convert(const Dart& d, size_t index) const;


private:
    Dart left_switches(const Mesh& m, const Tuple& t) const;
    Dart right_switches(const Mesh& m, const Tuple& t) const;
    // given an ear tuple reports the relative orientation across the edge
    Dart get_neighbor_action(const Mesh& m, const Tuple& t, int8_t local_action) const;
};
} // namespace wmtk::operations::internal
