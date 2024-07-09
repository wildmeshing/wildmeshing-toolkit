
#pragma once
#include <array>
#include <vector>
#include <wmtk/Tuple.hpp>

namespace wmtk {
class Mesh;
}
namespace wmtk::operations::internal {

/// Given a global id returns global ids
class SplitAlternateFacetData
{
public:
    using AltData = std::vector<std::tuple<int64_t, std::array<int64_t, 2>>>;
    AltData m_facet_maps;

    void sort();

    // assumes the split facet map has been sorted
    const std::array<int64_t, 2>& get_alternative_facets(const int64_t& input_facet) const;

    AltData::const_iterator get_alternative_facets_it(const int64_t& input_facet) const;
};

} // namespace wmtk::operations::internal
