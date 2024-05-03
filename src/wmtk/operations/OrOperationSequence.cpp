#include "OrOperationSequence.hpp"


namespace wmtk::operations {


OrOperationSequence::OrOperationSequence(
    Mesh& mesh,
    const std::vector<std::shared_ptr<Operation>>& operations)
    : OperationSequence(mesh, operations)
{}

OrOperationSequence::~OrOperationSequence() = default;


std::vector<simplex::Simplex> OrOperationSequence::execute_operations(
    const simplex::Simplex& simplex)
{
    assert(!m_operations.empty());
    for (auto& o : m_operations) {
        const auto out = (*o)(simplex);
        if (!out.empty()) return out;
    }

    return {};
}


} // namespace wmtk::operations
