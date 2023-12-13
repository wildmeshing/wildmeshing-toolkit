#pragma once
#include <spdlog/spdlog.h>
#include <memory>
#include <type_traits>
#include <wmtk/Primitive.hpp>
#include <wmtk/Simplex.hpp>
#include "Operation.hpp"

namespace wmtk::operations {
class OperationFactoryBase
{
public:
    OperationFactoryBase(PrimitiveType pt);
    virtual ~OperationFactoryBase();
    virtual std::unique_ptr<Operation> create(wmtk::Mesh& m, const Tuple& t) const = 0;
    virtual void initialize_invariants() = 0;
    PrimitiveType primitive() const { return m_primitive; }

private:
    PrimitiveType m_primitive;
};

template <typename OperationType>
class OperationFactory : public OperationFactoryBase
{
public:
    OperationFactory(const OperationSettings<OperationType>& settings)
        : OperationFactoryBase(OperationType::primitive_type())
        , m_settings(settings)
    {
        static_assert(std::is_base_of<OperationSettingsBase, OperationSettings<OperationType>>());
    }
    OperationFactory(OperationSettings<OperationType>&& settings)
        : OperationFactoryBase(OperationType::primitive_type())
        , m_settings(std::move(settings))
    {
        static_assert(std::is_base_of<OperationSettingsBase, OperationSettings<OperationType>>());
    }

    std::unique_ptr<Operation> create(wmtk::Mesh& m, const Tuple& t) const override
    {
        spdlog::debug("Using default create");
        return std::make_unique<OperationType>(
            m,
            Simplex(OperationType::primitive_type(), t),
            m_settings);
    }

    void initialize_invariants()
    {
        static_assert(std::is_base_of<OperationSettingsBase, OperationSettings<OperationType>>());
        m_settings.create_invariants();
    }

    const OperationSettings<OperationType>& settings() const { return m_settings; }

protected:
    OperationSettings<OperationType> m_settings;
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

} // namespace wmtk::operations
