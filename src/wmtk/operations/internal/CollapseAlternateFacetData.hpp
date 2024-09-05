
#pragma once
#include <array>
#include <vector>
#include <wmtk/Tuple.hpp>
#include "wmtk/operations/internal/CollapseAlternateFacetOptionData.hpp"

namespace wmtk {
class Mesh;
}
namespace wmtk::operations::internal {


class CollapseAlternateFacetData
{
public:
    using Data = CollapseAlternateFacetOptionData;
    CollapseAlternateFacetData();
    ~CollapseAlternateFacetData();

    void add(const Mesh& m, const Tuple& input_tuple);

    std::array<Tuple, 2> get_alternatives(
        const PrimitiveType mesh_pt,
        const Tuple& t,
        const PrimitiveType simplex_dimension) const;
    Tuple get_alternative(
        const PrimitiveType mesh_pt,
        const Tuple& t,
        const PrimitiveType simplex_dimension) const;

    using AltData = std::vector<Data>;
    AltData m_data;

    void sort();

    const Data& get_alternatives_data(const Tuple& t) const;

    AltData::const_iterator get_alternative_data_it(const int64_t& input_facet) const;
};
} // namespace wmtk::operations::internal
