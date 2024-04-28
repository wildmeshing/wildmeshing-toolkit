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

    std::vector<simplex::Simplex> queue;
    queue.push_back(simplex);
    for (int64_t i = 0; i < m_operations.size(); ++i) {
        auto& o = m_operations[i];
        assert(queue.size() == 1);

        const auto new_queue = (*o)(simplex::Simplex(o->primitive_type(), queue.front().tuple()));
        if (new_queue.empty()) return i == 0 ? std::vector<simplex::Simplex>() : queue;

        queue = new_queue;
    }

    return queue;
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

    return res;
}


} // namespace wmtk::operations
