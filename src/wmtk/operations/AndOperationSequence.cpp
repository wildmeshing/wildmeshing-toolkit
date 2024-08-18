#include "AndOperationSequence.hpp"


namespace wmtk::operations {


AndOperationSequence::AndOperationSequence(
    Mesh& mesh,
    const std::vector<std::shared_ptr<Operation>>& operations)
    : OperationSequence(mesh, operations)
{}

AndOperationSequence::~AndOperationSequence() = default;

std::vector<simplex::Simplex> AndOperationSequence::execute_operations(
    const simplex::Simplex& simplex)
{
    assert(!m_operations.empty());

    std::vector<simplex::Simplex> queue;
    queue.push_back(simplex);
    for (int64_t i = 0; i < m_operations.size(); ++i) {
        auto& o = m_operations[i];
        // assert(queue.size() == 1);

        auto tuple_resurrected = queue.front().tuple();

        const auto new_queue =
            (*o)(simplex::Simplex(mesh(), o->primitive_type(), tuple_resurrected));
        if (new_queue.empty()) return i == 0 ? std::vector<simplex::Simplex>() : queue;

        queue = new_queue;
    }

    return queue;
}


} // namespace wmtk::operations
