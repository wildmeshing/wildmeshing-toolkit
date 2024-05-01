#pragma once

#include "OperationSequence.hpp"

#include <wmtk/Tuple.hpp>


namespace wmtk {
class Mesh;

namespace operations {


class OrOperationSequence : public OperationSequence
{
public:
    // friend class utils::MultiMeshEdgeSplitFunctor;
    // friend class utils::MultiMeshEdgeCollapseFunctor;

    OrOperationSequence(Mesh& mesh, const std::vector<std::shared_ptr<Operation>>& operations = {});
    virtual ~OrOperationSequence();


protected:
    std::vector<simplex::Simplex> execute_operations(const simplex::Simplex& simplex) override;
};

} // namespace operations
} // namespace wmtk
