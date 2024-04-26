#include "MinOperationSequence.hpp"


namespace wmtk::operations {


MinOperationSequence::MinOperationSequence(
    Mesh& mesh,
    const std::vector<std::shared_ptr<Operation>>& operations)
    : Operation(mesh)
    , m_operations(operations)
{}

MinOperationSequence::~MinOperationSequence() = default;


std::vector<simplex::Simplex> MinOperationSequence::operator()(const simplex::Simplex& simplex)
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

void MinOperationSequence::reserve_enough_simplices()
{
    assert(!m_operations.empty());
    assert(m_value != nullptr);

    for (auto& o : m_operations) {
        o->reserve_enough_simplices();
    }
}

PrimitiveType MinOperationSequence::primitive_type() const
{
    assert(!m_operations.empty());
    assert(m_value != nullptr);

    const PrimitiveType res = m_operations.front()->primitive_type();
#ifndef NDEBUG
    for (const auto& o : m_operations) {
        assert(o->primitive_type() == res);
    }
#endif

    return res;
}


} // namespace wmtk::operations
