#include "MinOperationSequence.hpp"


namespace wmtk::operations {


MinOperationSequence::MinOperationSequence(
    Mesh& mesh,
    const std::vector<std::shared_ptr<Operation>>& operations)
    : OperationSequence(mesh, operations)
{}

MinOperationSequence::~MinOperationSequence() = default;


std::vector<simplex::Simplex> MinOperationSequence::execute_operations(
    const simplex::Simplex& simplex)
{
    assert(m_value != nullptr);
    assert(!m_operations.empty());

    std::vector<std::pair<int64_t, double>> values;
    values.reserve(m_operations.size());

    for (int64_t i = 0; i < m_operations.size(); ++i) {
        values.emplace_back(i, m_value(i, simplex));
    }
    std::sort(values.begin(), values.end(), [](const auto& a, const auto& b) {
        return a.second < b.second;
    });

    for (int64_t i = 0; i < m_operations.size(); ++i) {
        const auto& o = m_operations[values[i].first];
        const auto out = (*o)(simplex);
        if (!out.empty()) return out;
    }

    return {};
}


} // namespace wmtk::operations
