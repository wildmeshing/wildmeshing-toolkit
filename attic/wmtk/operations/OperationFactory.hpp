#pragma once
class OperationFactoryBase
{
    std::unique_ptr<Operation> create(const MeshType& m, const TupleType& t) const;
};

template <typename OperationType>
class OperationFactory : public OperationFactoryBase
{
    std::unique_ptr<Operation> create(const MeshType& m, const TupleType& t) const
    {
        return std::make_unique<OperationType>(m, t);
    }
};

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

