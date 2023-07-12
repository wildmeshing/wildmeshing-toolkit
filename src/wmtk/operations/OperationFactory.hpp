#pragma once
#include "Primitive.h"
#include "Operation.h"

namespace wmtk::operations {
class OperationFactoryBase
{
    //OperationFactoryBase(PrimitiveType p): m_primitive(p) {}
    virtual std::unique_ptr<Operation> create(const TupleType& t) const = 0;
    //PrimitiveType primitive() const { return m_primitive; }
    //private:
    //PrimitiveType m_primitive;
    Mesh& m_mesh;
};

template <typename OperationType>
class OperationFactory : public OperationFactoryBase
{
    std::unique_ptr<Operation> create(const TupleType& t) const
    {
        return std::make_unique<OperationType>(m_mesh, t);
    }
};







OperationFactory<TriMeshSplitEdgeOperation> factory;



std::map<std::string, std::unique_ptr<OperationFactoryBase>> ops;


if(use_basic_split) {
ops.emplace("split_EdgE", std::make_unique<OperationFactory<TriMeshSplitEdgeOperation>>());
} else if(use_seamed_mesh_split) {
ops.emplace("split_EdgE", std::make_unique<OperationFactory<SeamedTriMeshSplitEdgeOperation>>());
}


// per thread objects
class OperationQueue
{

    void enqueue(const std::string, const Tuple& t) {
        auto new_op = get_factory(name)->create(m, tup);
        priority_queue.emplace(new_op);
    }
    void run()
    {
        while (!empty()) {
            auto new_op = pop_top();
            new_op->run();
        }
    }

    //std::queue<std::pair<std::string, Tuple>> queue;

    std::priority_queue<std::unique_ptr<Operation>> queue;
    // uses a->priority() < b->priority()
};

}
