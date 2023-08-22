#include "Scheduler.hpp"
#include "operations/OperationQueue.hpp"

namespace wmtk {
Scheduler::Scheduler(Mesh& m)
    : m_mesh(m)
    , m_per_thread_queues(1)
{}

Scheduler::~Scheduler() = default;
void Scheduler::run_operation_on_all(PrimitiveType type, const std::string& name)
{
    auto ops = create_operations(type, name);
    spdlog::info("Created {} operations", ops.size());
    std::sort(ops.begin(), ops.end(), [](auto&& p_a, auto&& p_b) { return *p_a < *p_b; });
    enqueue_operations(std::move(ops));
    // run();
    for (operations::OperationQueue& q : m_per_thread_queues) {
        q.run();
    }
    // enqueue_operations(ops);
    // TODO: pick some strategy for running these operations
    // tbb::parallel_for(ops, [&](const auto& ops) { (*op)(); });
}

void Scheduler::enqueue_operations(std::vector<std::unique_ptr<operations::Operation>>&& ops)
{
    for (size_t index = 0; index < ops.size();) {
        for (operations::OperationQueue& queue : m_per_thread_queues) {
            if (index < ops.size()) {
                queue.enqueue(std::move(ops[index]));
                index++;
            } else {
                return;
            }
        }
    }
}
operations::OperationFactoryBase const* Scheduler::get_factory(const std::string_view& name) const
{
    if (auto it = m_factories.find(std::string(name)); it != m_factories.end()) {
        return it->second.get();
    } else {
        return nullptr;
    }
}

std::vector<std::unique_ptr<operations::Operation>> Scheduler::create_operations(
    PrimitiveType type,
    const std::string& name)
{
    const auto tups = m_mesh.get_all(type);

    std::vector<std::unique_ptr<operations::Operation>> ops;
    auto factory_ptr = get_factory(name);
    assert(factory_ptr != nullptr);

    ops.reserve(tups.size());
    for (const Tuple& tup : tups) {
        ops.emplace_back(factory_ptr->create(m_mesh, tup));
    }
    return ops;
}
} // namespace wmtk
