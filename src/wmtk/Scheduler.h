#pragma once


namespace wmtk {


class Executor
{
    using Op = std::pair<std::string, Tuple>;
    template <typename OperationType>
    void add_operation_type(const std::string& name)
    {
        m_factories[name] = std::make_unique<OperationFactory<OperationType>>();
    }

    void run_queue(const std::vector<Op>& tuples)
    {
        while (!empty()) {
            auto new_op = get_operation(pop_top());
            auto new_op = get_factory(name)->create(m, tup);
            new_op->run();
        }
    }
    std::unique_ptr<Operation> get_operation(const Op& pr)
    {
        const auto& [name, tup] = pr;
        return get_factory(name)->create(m, tup);
    }


private:
    Mesh& m;
    std::unordered_map<std::string, std::unique_ptr<OperationFactory>> m_factories;
};

} // namespace wmtk
