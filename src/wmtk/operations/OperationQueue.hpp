#include <mutex>
#include <queue>
#include <vector>
#include "OperationFactory.hpp"
namespace wmtk {
// per thread objects
// TODO:: this should be done with a proper tbb per-thread queue thing
// just for demonstration of the desired logic
class OperationQueue
{
public:
    void enqueue(std::unique_ptr<Operation>&& op) { queue.emplace(std::move(op)); }
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
        while (true) {
            while (!empty()) {
                auto op = pop_top();
                (*op)();
            }
        }
    }

    bool empty() const
    {
        std::scoped_lock lock(mut);
        return queue.empty();
    }

    std::unique_ptr<Operation> pop_top()
    {
        std::scoped_lock lock(mut);
        std::unique_ptr<Operation> op = std::move(queue.top());
        queue.pop();
        return op;
    }

    // std::queue<std::pair<std::string, Tuple>> queue;

    std::mutex mut;
    std::priority_queue<std::unique_ptr<Operation>> queue;
    // uses a->priority() < b->priority()
};
} // namespace wmtk
