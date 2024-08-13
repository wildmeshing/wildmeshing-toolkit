#pragma once

#include "Operation.hpp"

#include <wmtk/Tuple.hpp>


namespace wmtk {
class Mesh;

namespace operations {


class OperationSequence : public Operation
{
public:
    OperationSequence(Mesh& mesh, const std::vector<std::shared_ptr<Operation>>& operations = {});
    virtual ~OperationSequence();

    // main entry point of the operator by the scheduler
    std::vector<simplex::Simplex> operator()(const simplex::Simplex& simplex) override;

    double priority(const simplex::Simplex& simplex) const override
    {
        assert(!m_operations.empty());
        return m_priority == nullptr ? m_operations.front()->priority(simplex)
                                     : m_priority(simplex);
    }

    bool use_random_priority() const override
    {
        assert(!m_operations.empty());
        if (m_internal_random_priority) return Operation::use_random_priority();

        return m_operations.front()->use_random_priority();
    }

    bool& use_random_priority() override
    {
        assert(!m_operations.empty());
        m_internal_random_priority = true;
        return Operation::use_random_priority();
    }

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

    virtual std::vector<simplex::Simplex> execute_operations(const simplex::Simplex& simplex) = 0;

    std::vector<std::shared_ptr<Operation>> m_operations;

private:
    bool m_internal_random_priority = false;
};

} // namespace operations
} // namespace wmtk
