#include "OperationSequence.hpp"


namespace wmtk::operations {


OperationSequence::OperationSequence(
    Mesh& mesh,
    const std::vector<std::shared_ptr<Operation>>& operations)
    : Operation(mesh)
    , m_operations(operations)
{}

OperationSequence::~OperationSequence() = default;


std::vector<simplex::Simplex> OperationSequence::operator()(const simplex::Simplex& simplex)
{
    assert(!m_operations.empty());

    if (!before(simplex)) {
        return {};
    }

    const auto simplex_resurrect = simplex;

    auto mods = execute_operations(simplex_resurrect);

    // TODO after?
    return mods;
}


void OperationSequence::reserve_enough_simplices()
{
    assert(!m_operations.empty());

    for (auto& o : m_operations) {
        o->reserve_enough_simplices();
    }
}

PrimitiveType OperationSequence::primitive_type() const
{
    assert(!m_operations.empty());
    const PrimitiveType res = m_operations.front()->primitive_type();
    // #ifndef NDEBUG
    //     for (const auto& o : m_operations) {
    //         assert(o->primitive_type() == res);
    //     }
    // #endif
    return res;
}


} // namespace wmtk::operations
