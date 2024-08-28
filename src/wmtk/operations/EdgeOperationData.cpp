
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
simplex::Simplex
EdgeOperationData::simplex_from_id(const Mesh& m, const PrimitiveType type, const int64_t gid)
{
    return m.simplex_from_id(type, gid);
}
void EdgeOperationData::set_split()
{
    m_op_data = std::make_unique<internal::SplitAlternateFacetData>();
}
void EdgeOperationData::set_collapse()
{
    m_op_data = std::make_unique<internal::CollapseAlternateFacetData>();
}

const internal::SplitAlternateFacetData& EdgeOperationData::const_split_facet_data() const
{
    const auto& ptr = std::get<std::unique_ptr<internal::SplitAlternateFacetData>>(m_op_data);
    if (!bool(ptr)) {
        throw std::runtime_error(
            "Split alternate facet data does not exist, ptr in variant was null");
    }
    return *ptr;
}
const internal::CollapseAlternateFacetData& EdgeOperationData::const_collapse_facet_data() const
{
    const auto& ptr = std::get<std::unique_ptr<internal::CollapseAlternateFacetData>>(m_op_data);
    if (!bool(ptr)) {
        throw std::runtime_error(
            "Collapse alternate facet data does not exist, ptr in variant was null");
    }
    return *ptr;
}
internal::SplitAlternateFacetData& EdgeOperationData::split_facet_data()
{
    auto& ptr = std::get<std::unique_ptr<internal::SplitAlternateFacetData>>(m_op_data);
    if (!bool(ptr)) {
        throw std::runtime_error(
            "Split alternate facet data does not exist, ptr in variant was null");
    }
    return *ptr;
}
internal::CollapseAlternateFacetData& EdgeOperationData::collapse_facet_data()
{
    auto& ptr = std::get<std::unique_ptr<internal::CollapseAlternateFacetData>>(m_op_data);
    if (!bool(ptr)) {
        throw std::runtime_error(
            "Collapse alternate facet data does not exist, ptr in variant was null");
    }
    return *ptr;
}
std::vector<int64_t>
EdgeOperationData::request_simplex_indices(Mesh& mesh, const PrimitiveType type, int64_t count)
{
    mesh.guarantee_more_attributes(type, count);

    return mesh.request_simplex_indices(type, count);
}

} // namespace wmtk::operations
