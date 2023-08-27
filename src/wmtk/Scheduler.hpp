#pragma once
#include <string_view>
#include <unordered_map>
#include "Mesh.hpp"
#include "operations/Operation.hpp"
#include "operations/OperationFactory.hpp"


namespace wmtk {

//  Scheduler scheduler;
// if (use_basic_split) {
//     scheduler.add_operation_type<SplitEdgeOperation>("split_edge");
// } else if (use_seamed_mesh_split) {
//     scheduler.add_operation_type<SeamedTriMeshSplitEdgeOperation>("split_edge");
// }

namespace operations {
class OperationQueue;
}
class Scheduler
{
public:
    Scheduler(Mesh& m);
    ~Scheduler();
    // user specifies a sort of operation they want
    // template <typename OperationType, typename... Args>
    // void add_operation_type(const std::string& name, PrimitiveType primitive_type, Args... args)
    //{
    //    m_factories[name] = std::make_unique<OperationFactory<OperationType>>(
    //        primitive_type,
    //        std::forward<Args>(args)...);
    //}
    template <typename OperationType>
    void add_operation_type(const std::string& name)
    {
        m_factories[name] = std::make_unique<operations::OperationFactory<OperationType>>();
    }

    template <typename OperationType>
    void add_operation_type(
        const std::string& name,
        const operations::OperationSettings<OperationType>& settings)
    {
        m_factories[name] = std::make_unique<operations::OperationFactory<OperationType>>(settings);
    }

    void enqueue_operations(std::vector<std::unique_ptr<operations::Operation>>&& ops);


    // creates all of the operations of a paritcular type to be run
    std::vector<std::unique_ptr<operations::Operation>> create_operations(
        PrimitiveType type,
        const std::string& name);


    void run_operation_on_all(PrimitiveType type, const std::string& name);

    operations::OperationFactoryBase const* get_factory(const std::string_view& name) const;

private:
    wmtk::Mesh& m_mesh;
    std::unordered_map<std::string, std::unique_ptr<operations::OperationFactoryBase>> m_factories;
    //    tbb::enumerable_per_thread<OperationQueue> m_per_thread_queues;
    std::vector<operations::OperationQueue> m_per_thread_queues;
};

} // namespace wmtk
