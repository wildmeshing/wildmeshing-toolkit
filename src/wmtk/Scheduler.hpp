#pragma once


namespace wmtk {

//  Scheduler scheduler;
// if (use_basic_split) {
//     scheduler.add_operation_type<SplitEdgeOperation>("split_edge");
// } else if (use_seamed_mesh_split) {
//     scheduler.add_operation_type<SeamedTriMeshSplitEdgeOperation>("split_edge");
// }

class Scheduler
{
    // user specifies a sort of operation they want
    template <typename OperationType, typename... Args>
    void add_operation_type(const std::string& name, PrimitiveType primitive_type, Args... args)
    {
        m_factories[name] = std::make_unique<OperationFactory<OperationType>>(
            primitive_type,
            std::forward<Args>(args)...);
    }

    void enqueue_operations(const std::vector<std::unique_ptr<Operation>>& ops)
    {
        size_t index = 0;
        for (size_t index = 0; index < ops.size(); ++index) {
            for (auto& queue : m_per_thread_queues) {
                if (index < ops.size()) {
                    actor.enqueue(std::(ops[index]));
                    index++;
                } else {
                    return;
                }
            }
        }
    }


    // sorts all of the operations according to the prioirty queue
    void sort_operations_in_place(std::vector<std::unique_ptr<Operation>>& ops)
    {
        std::sort(ops.begin(), ops.end(), [](const auto& a_ptr, const auto& b_ptr) {
            return a_ptr->priority() < b_ptr->priority();
        });
    }


    // creates all of the operations of a paritcular type to be run
    std::vector<std::unique_ptr<Operation>> create_operations(
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


    std::unique_ptr<Operation> get_operation(const Op& pr)
    {
        const auto& [name, tup] = pr;
        return get_factory(name)->create(m, tup);
    }


    void run_operation(PrimitiveType type, const std::string& name)
    {
        auto ops = create_ops(type, name);
        sort_operations_in_place(ops);
        // enqueue_operations(ops);
        // TODO: pick some strategy for running these operations
        tbb::parallel_for(ops, [&](const auto& ops) { (*op)(); });
    }

private:
    Mesh& m;
    std::unordered_map<std::string, std::unique_ptr<OperationFactory>> m_factories;
    //    tbb::enumerable_per_thread<OperationQueue> m_per_thread_queues;
};

} // namespace wmtk
