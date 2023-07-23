#include "FlagInspector.hpp"
#include <wmtk/attribute/AccessorBase.hpp>
#include "SimplexFlag.hpp"


namespace wmtk{

// number of simplices for which we have flags
long FlagInspector::size() const
{
    return m_accessor.size();
}

// the total number of simplices that are registered as active
long FlagInspector::active_simplex_count() const
{
    long count = 0;
    for (long index = 0; index < m_accessor.size(); ++index) {
        if(is_active(index)) {
            count++;
        }
    }
}

// the index of the largest active simplex, could theoretically shrink size() to
// `longest_active_simplex_index() + 1`.
long FlagInspector::largest_active_simplex_index() const {
    long index;
    for (index = m_accessor.size() - 1; index >= 0; --index) {
        if(is_active(index)) {
            break;
        }
    }
    return index + 1;
}


bool FlagInspector::is_active(long index) const {
    return is_flag_active<SimplexFlag::Active>(m_accessor.scalar_attribute(index));
}
} // namespace wmtk::utils
