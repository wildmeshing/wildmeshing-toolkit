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

template <typename OperationType, typename SettingsType = std::nullptr_t>
class OperationFactory : public OperationFactoryBase
{
public:
    OperationFactory()
        : OperationFactoryBase(OperationType::primitive_type())
        , m_settings(nullptr)
    {}

    OperationFactory(const SettingsType& settings)
        : OperationFactoryBase(OperationType::primitive_type())
        , m_settings(settings)
    {}

    std::unique_ptr<Operation> create(Mesh& m, const Tuple& t) const override
    {
        if constexpr (std::is_same<SettingsType, std::nullptr_t>::value) {
            return std::make_unique<OperationType>(m, t);
        } else {
            return std::make_unique<OperationType>(m, t, m_settings);
        }
    }

protected:
    const SettingsType m_settings;
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
