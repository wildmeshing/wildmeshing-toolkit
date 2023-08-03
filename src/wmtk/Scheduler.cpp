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
    std::sort(ops.begin(), ops.end(), [](auto&& p_a, auto&& p_b) { return *p_a < *p_b; });
    enqueue_operations(std::move(ops));
    run();
    for (const auto& q : m_per_thread_queues) {
        q.run();
    }
    // enqueue_operations(ops);
    // TODO: pick some strategy for running these operations
    // tbb::parallel_for(ops, [&](const auto& ops) { (*op)(); });
}

void Scheduler::enqueue_operations(std::vector<std::unique_ptr<Operation>>&& ops)
{
    size_t index = 0;
    for (size_t index = 0; index < ops.size(); ++index) {
        for (auto& queue : m_per_thread_queues) {
            if (index < ops.size()) {
                queue.enqueue(std::(ops[index]));
                index++;
            } else {
                return;
            }
        }
    }
}

std::vector<std::unique_ptr<Operation>> Scheduler::create_operations(
    PrimitiveType type,
    const std::string& name)
{
    auto tups = m.get_all(type);

    std::vector<std::unique_ptr<Operation>> ops;
    auto& factory = get_factory(name);
    for (const auto& tup : tups) {
        ops.emplace_back(factory->create(m, tup));
    }
}
} // namespace wmtk
