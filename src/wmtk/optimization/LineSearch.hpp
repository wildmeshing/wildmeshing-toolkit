#pragma once

#include <wmtk/function/utils/DifferentiableFunctionEvaluator.hpp>
#include <wmtk/invariants/InvariantCollection.hpp>


namespace wmtk::optimization {


class LineSearch
{
public:
    using InvariantCollection = wmtk::InvariantCollection;
    LineSearch(
        function::utils::DifferentiableFunctionEvaluator& interface,
        const InvariantCollection& invariants);

    using Vector = Eigen::VectorXd;

    double run(const Eigen::VectorXd& direction, double step_size);
    double _run(const Eigen::VectorXd& direction, double step_size);

    void set_create_scope(bool enable) { m_create_scope = enable; }
    void set_max_steps(long max_steps) { m_max_steps = max_steps; }
    void set_min_step_size_ratio(long min_step_size_ratio)
    {
        m_min_step_size_ratio = min_step_size_ratio;
    }

protected:
    function::utils::DifferentiableFunctionEvaluator& m_interface;
    const InvariantCollection& m_invariants;

    bool m_create_scope = true;
    long m_max_steps = 10;
    double m_min_step_size_ratio = 1e-6;

    std::vector<Tuple> modified_simplices(PrimitiveType pt) const;

    // TODO: formally define what checking the state means
    // we currently make sure that we pass before on the input tuple and after on all top level
    // simplices, but should we be passing all of that every time?
    bool check_state() const;

public:
protected:
};

} // namespace wmtk::optimization
