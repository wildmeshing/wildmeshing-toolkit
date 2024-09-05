
#pragma once
#include <array>
#include <vector>
#include <wmtk/Tuple.hpp>
#include "SplitAlternateFacetOptionData.hpp"

namespace wmtk {
class Mesh;
}
namespace wmtk::operations::internal {

/// Given a global id returns global ids
class SplitAlternateFacetData
{
public:
    using Data = SplitAlternateFacetOptionData;
    using AltData = std::vector<Data>;
    AltData m_facet_maps;


    const Data& add_facet(
        const wmtk::Mesh& mesh,
        const wmtk::Tuple& edge_tuple,
        const std::array<int64_t, 2>& new_facet_indices);
    const Data& add_facet(wmtk::Mesh& mesh, const wmtk::Tuple& edge_tuple);

    void sort();

    // assumes the split facet map has been sorted
    const std::array<int64_t, 2>& get_alternative_facets(const int64_t& input_facet) const;

    AltData::const_iterator get_alternative_facets_it(const int64_t& input_facet) const;

    Tuple get_alternative(
        const PrimitiveType mesh_pt,
        const Tuple& t,
        const PrimitiveType simplex_dimension) const;
};

} // namespace wmtk::operations::internal
