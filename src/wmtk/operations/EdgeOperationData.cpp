
#include <array>
#include <vector>
#include <wmtk/Tuple.hpp>

#include <wmtk/Mesh.hpp>
#include "EdgeOperationData.hpp"
#include "internal/CollapseAlternateFacetData.hpp"
#include "internal/SplitAlternateFacetData.hpp"
namespace wmtk::operations {

EdgeOperationData::EdgeOperationData() = default;
EdgeOperationData::~EdgeOperationData() = default;
EdgeOperationData::EdgeOperationData(EdgeOperationData&&) = default;
EdgeOperationData& EdgeOperationData::operator=(EdgeOperationData&&) = default;
auto EdgeOperationData::tuple_from_id(const Mesh& m, const PrimitiveType type, const int64_t gid)
    -> Tuple
{
    return m.tuple_from_id(type, gid);
}
simplex::Simplex EdgeOperationData::simplex_from_id(const Mesh& m, const PrimitiveType type, const int64_t gid)
{
    return m.simplex_from_id(type, gid);
}

const internal::SplitAlternateFacetData& EdgeOperationData::split_facet_data() const
{
    const auto& ptr = std::get<std::unique_ptr<internal::SplitAlternateFacetData>>(m_op_data);
    if (!bool(ptr)) {
        throw std::runtime_error(
            "Split alternate facet data does not exist, ptr in variant was null");
    }
    return *ptr;
}
const internal::CollapseAlternateFacetData& EdgeOperationData::collapse_facet_data() const
{
    const auto& ptr = std::get<std::unique_ptr<internal::CollapseAlternateFacetData>>(m_op_data);
    if (!bool(ptr)) {
        throw std::runtime_error(
            "Collapse alternate facet data does not exist, ptr in variant was null");
    }
    return *ptr;
}

} // namespace wmtk::operations
