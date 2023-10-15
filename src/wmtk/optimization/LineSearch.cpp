#include <LineSearch.hpp>
#include <wmtk/simplex/top_level_cofaces.hpp>
namespace wmtk::optimization {
std::vector<Tuple> LineSearchBase::modified_top_simplices() const
{
    return simplex::top_level_cofaces_tuples(interface.mesh());
}
bool LineSearchBase::check_state() const
{
    bool before_pass = m_invariants.before(m_interface.tuple());
    bool after_pass = m_invariants.after(top_type, modified_top_simplices);
    return before_pass && after_pass;
}
} // namespace wmtk::optimization
