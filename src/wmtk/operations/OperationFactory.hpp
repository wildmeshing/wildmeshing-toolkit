#pragma once
#include <wmtk/Primitive.hpp>
#include "Operation.hpp"

namespace wmtk{
class OperationFactoryBase
{
    virtual std::unique_ptr<Operation> create(Mesh& m, const Tuple& t) const;
    PrimitiveType primitive() const { return m_primitive; }
    private:
    PrimitiveType m_primitive;
};

template <typename OperationType>
class OperationFactory : public OperationFactoryBase
{
    public:
    std::unique_ptr<Operation> create(Mesh& m, const Tuple& t) const
    {
        return std::make_unique<OperationType>(m, t);
    }
};

/*
class OperationQueue
{
    void run()
    {
        while (!empty()) {
            auto [name, tup] = pop_top();
            auto new_op = get_factory(name)->create(m, tup);
            if (new_op->run()) {
                new_op->renew();
            }
        }
    }

    std::queue<std::pair<std::string, Tuple>> queue;
};
*/

}
