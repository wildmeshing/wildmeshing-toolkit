#pragma once

#include "Operation.hpp"

#include <wmtk/Tuple.hpp>


namespace wmtk {
class Mesh;

namespace operations {


class OperationSequence : public Operation
{
public:
    // friend class utils::MultiMeshEdgeSplitFunctor;
    // friend class utils::MultiMeshEdgeCollapseFunctor;

    OperationSequence(Mesh& mesh, const std::vector<std::shared_ptr<Operation>>& operations = {});
    virtual ~OperationSequence();

    // main entry point of the operator by the scheduler
    std::vector<simplex::Simplex> operator()(const simplex::Simplex& simplex) override;

    PrimitiveType primitive_type() const override;

    void reserve_enough_simplices() override;

    void add_operation(const std::shared_ptr<Operation>& op) { m_operations.push_back(op); }


protected:
    /**
     * @brief returns an empty vector in case of failure
     */
    std::vector<simplex::Simplex> execute(const simplex::Simplex& simplex) override
    {
        throw std::runtime_error("This shoud never be called");
    }

    /**
     * Returns all simplices that will be potentially affected by the OperationSequence
     */
    std::vector<simplex::Simplex> unmodified_primitives(
        const simplex::Simplex& simplex) const override
    {
        throw std::runtime_error("This shoud never be called");
    }

private:
    std::vector<std::shared_ptr<Operation>> m_operations;
};

} // namespace operations
} // namespace wmtk
