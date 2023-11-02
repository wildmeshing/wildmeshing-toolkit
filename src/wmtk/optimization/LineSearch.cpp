#include "LineSearch.hpp"
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/utils/primitive_range.hpp>

namespace wmtk::optimization {

LineSearch::LineSearch(
    function::utils::DifferentiableFunctionEvaluator& interface,
    const InvariantCollection& invariants)
    : m_interface(interface)
    , m_invariants(invariants)
{}

std::vector<Tuple> LineSearch::modified_simplices(PrimitiveType pt) const
{
    return wmtk::simplex::cofaces_single_dimension_tuples(
        m_interface.mesh(),
        m_interface.simplex(),
        pt);
    // return m_interface.upper_level_cofaces();
}

bool LineSearch::check_state() const
{
    PrimitiveType top_type = m_interface.mesh().top_simplex_type();
    bool before_pass = m_invariants.before(m_interface.tuple());
    bool after_pass = true;
    for (const PrimitiveType pt : wmtk::utils::primitive_below(top_type)) {
        after_pass |= m_invariants.after(pt, modified_simplices(pt));
    }
    return before_pass && after_pass;
}
double LineSearch::run(const Eigen::VectorXd& direction, double step_size)
{
    if (!check_state()) {
        return 0;
    }
    if (m_create_scope) {
        {
            auto scope = m_interface.mesh().create_scope();
            double retval = _run(direction, step_size);
            if (retval == 0) {
                scope.mark_failed();
            }
            return retval;
        }

    } else {
        return _run(direction, step_size);
    }
}

double LineSearch::_run(const Eigen::VectorXd& direction, double init_step_size)
{
    int steps = 0;
    // just to make sure we try the initial stepsize
    double step_size = init_step_size;
    double next_step_size = step_size;
    double min_step_size = m_min_step_size_ratio * step_size;
    Vector current_pos = m_interface.get_const_coordinate();
    Vector new_pos;
    do {
        new_pos = current_pos + direction * step_size;
        m_interface.store(new_pos);

        step_size = next_step_size;
        next_step_size /= 2;
    } while (steps++ < m_max_steps && step_size > min_step_size && !check_state());
    if (steps == m_max_steps || step_size < min_step_size) {
        return 0;
    } else {
        return step_size;
    }
}
} // namespace wmtk::optimization
