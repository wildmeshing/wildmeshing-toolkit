#pragma once

#include "OperationSequence.hpp"

#include <wmtk/Tuple.hpp>


namespace wmtk {
class Mesh;

namespace operations {


class AndOperationSequence : public OperationSequence
{
public:
    AndOperationSequence(
        Mesh& mesh,
        const std::vector<std::shared_ptr<Operation>>& operations = {});

    virtual ~AndOperationSequence();

protected:
    std::vector<simplex::Simplex> execute_operations(const simplex::Simplex& simplex) override;
};

} // namespace operations
} // namespace wmtk
