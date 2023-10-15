#pragma once

#include <wmtk/invariants/InvariantCollection.hpp>
#include "FunctionInterface.hpp"


namespace wmtk::optimization {


template <int Dim>
class LineSearch
{
public:
    using InvariantCollection = wmtk::InvariantCollection;
    LineSearch(FunctionInterface<Dim>& interface, const InvariantCollection& invariants)
        : m_interface(interface)
        , m_invariants(invariants)
    {}

    void set_create_scope(bool enable) { m_create_scope = enable; }
    void set_max_steps(long max_steps) { m_max_steps = max_steps; }
    void set_min_step_size_ratio(long min_step_size_ratio)
    {
        m_min_step_size_ratio = min_step_size_ratio;
    }


    template <typename Derived>
    double run(const Eigen::MatrixBase<Derived>& direction, double step_size);
    template <typename Derived>
    double _run(
        const Eigen::MatrixBase<Derived>& direction,
        double step_size,
        const std::vector<Tuple>& modified_top_simplices);

protected:
    std::vector<Tuple> modified_top_simplices() const;

    // TODO: formally define what checking the state means
    // we currently make sure that we pass before on the input tuple and after on all top level
    // simplices, but should we be passing all of that every time?
    bool check_state() const;

    FunctionInterface<Dim>& m_interface;
    const InvariantCollection& m_invariants;

    bool m_create_scope = true;
    long m_max_steps = 10;
    double m_min_step_size_ratio = 1e-6;
};

template <int Dim>
template <typename Derived>
double LineSearch<Dim>::run(const Eigen::MatrixBase<Derived>& direction, double step_size)
{
    PrimitiveType top_type = m_interface.mesh().top_simplex_type();

    std::vector<Tuple> modified_top_simplices = this->modified_top_simplices();
    if (!check_state()) {
        return 0;
    }
    if (m_create_scope) {
        {
            auto scope = m_interface.mesh().create_scope();
            double retval = _run(direction, step_size, modified_top_simplices);
            if (retval == 0) {
                scope.mark_failed();
            }
            return retval;
        }

    } else {
        return _run(direction, step_size, modified_top_simplices);
    }
}

template <int Dim>
template <typename Derived>
double LineSearch<Dim>::_run(
    const Eigen::MatrixBase<Derived>& direction,
    double init_step_size,
    const std::vector<Tuple>& modified_top_simplices)
{
    return 0;
    /*
    PrimitiveType top_type = interface.mesh().top_simplex_type();
    int steps = 0;
    // just to make sure we try the initial stepsize
    double step_size = init_step_size;
    double next_step_size = step_size;
    double min_step_size = m_min_step_size_ratio * step_size;
    Vector<double, Dim> current_pos = interface.get_const_coordinate();
    Vector<double, Dim> new_pos;
    do {
        new_pos = current_pos + direction * step_size;
        m_function_interface.store(new_pos);

        step_size = next_step_size;
        next_step_size /= 2;
    } while (steps++ < m_max_steps && step_size > min_step_size && !check_state());
    if (steps == m_max_steps || step_size < min_step_size) {
        return 0;
    } else {
        return step_size;
    }
    */
}
} // namespace wmtk::optimization

