#include <spdlog/spdlog.h>
#include <mutex>
#include <queue>
#include <vector>
#include "OperationFactory.hpp"
namespace wmtk::operations {
// per thread objects
// TODO:: this should be done with a proper tbb per-thread queue thing
// just for demonstration of the desired logic
class OperationQueue
{
public:
    void enqueue(std::unique_ptr<Operation>&& op) { queue.emplace_back(std::move(op)); }
    // void enqueue(const std::string, const Tuple& t)
    //{
    //     std::scoped_lock sl(mut);
    //     auto new_op = get_factory(name)->create(m, tup);
    //     priority_queue.emplace(new_op);
    // }

    //// sorts all of the operations according to the prioirty queue
    // void sort_operations_in_place(std::vector<std::unique_ptr<Operation>>& ops)
    //{
    //     std::sort(ops.begin(), ops.end(), [](const auto& a_ptr, const auto& b_ptr) {
    //         return a_ptr->priority() < b_ptr->priority();
    //     });
    // }
    void run()
    {
        // reset counters
        m_num_op_success = 0;
        m_num_op_fail = 0;

        spdlog::debug("Running with queue starting at {} of {}", current_index, queue.size());
        while (!empty()) {
            execute_next();
        }
    }

    void execute_next()
    {
        auto op = pop_top();
        if ((*op)()) {
            ++m_num_op_success; // needs to be done thread safe
            spdlog::debug("Op succeeded");
        } else {
            ++m_num_op_fail; // needs to be done thread safe
            spdlog::debug("Op failed");
        }
    }

    bool empty() const
    {
        std::scoped_lock lock(mut);
        return current_index >= queue.size();
    }

    std::unique_ptr<Operation> pop_top()
    {
        std::scoped_lock lock(mut);
        std::unique_ptr<Operation> op = std::move(queue[current_index++]);
        return op;
    }

    // std::queue<std::pair<std::string, Tuple>> queue;

    mutable std::mutex mut;
    std::vector<std::unique_ptr<Operation>> queue;
    size_t current_index = 0;
    // uses a->priority() < b->priority()

    /**
     * @brief Returns the number of successful operations performed by the operation queue.
     */
    long number_of_successful_operations() const { return m_num_op_success; }
    /**
     * @brief Returns the number of failed operations performed by the operation queue.
     */
    long number_of_failed_operations() const { return m_num_op_fail; }

private:
    long m_num_op_success = 0;
    long m_num_op_fail = 0;
};
} // namespace wmtk::operations
