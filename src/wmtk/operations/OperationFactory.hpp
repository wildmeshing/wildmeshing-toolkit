#pragma once
#include <memory>
#include <wmtk/Primitive.hpp>
#include "Operation.hpp"

namespace wmtk {
class OperationFactoryBase
{
public:
    OperationFactoryBase(PrimitiveType pt);
    virtual ~OperationFactoryBase();
    virtual std::unique_ptr<Operation> create(Mesh& m, const Tuple& t) const = 0;
    PrimitiveType primitive() const { return m_primitive; }

private:
    PrimitiveType m_primitive;
};

template <typename OperationType, typename OperationSettingsType = std::nullptr_t>
class OperationFactory : public OperationFactoryBase
{
public:
    OperationFactory()
        : OperationFactoryBase(OperationType::primitive_type())
        , m_handles(nullptr)
    {}

    OperationFactory(const OperationSettingsType& handles)
        : OperationFactoryBase(OperationType::primitive_type())
        , m_handles(handles)
    {}

    std::unique_ptr<Operation> create(Mesh& m, const Tuple& t) const override
    {
        if constexpr (std::is_same<OperationSettingsType, std::nullptr_t>::value) {
            return std::make_unique<OperationType>(m, t);
        } else {
            return std::make_unique<OperationType>(m, t, m_handles);
        }
    }

protected:
    const OperationSettingsType m_handles;
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

} // namespace wmtk
