#include "OrOperationSequence.hpp"


namespace wmtk::operations {


OrOperationSequence::OrOperationSequence(
    Mesh& mesh,
    const std::vector<std::shared_ptr<Operation>>& operations)
    : Operation(mesh)
    , m_operations(operations)
{}

OrOperationSequence::~OrOperationSequence() = default;


std::vector<simplex::Simplex> OrOperationSequence::operator()(const simplex::Simplex& simplex)
{
    assert(!m_operations.empty());
    for (auto& o : m_operations) {
        const auto out = (*o)(simplex);
        if (!out.empty()) return out;
    }

    return {};
}


void OrOperationSequence::reserve_enough_simplices()
{
    assert(!m_operations.empty());

    for (auto& o : m_operations) {
        o->reserve_enough_simplices();
    }
}

PrimitiveType OrOperationSequence::primitive_type() const
{
    assert(!m_operations.empty());
    const PrimitiveType res = m_operations.front()->primitive_type();
#ifndef NDEBUG
    for (const auto& o : m_operations) {
        assert(o->primitive_type() == res);
    }
#endif

    return res;
}


} // namespace wmtk::operations
