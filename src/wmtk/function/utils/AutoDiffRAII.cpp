#include "AutoDiffRAII.hpp"
#include "autodiff.h"

namespace wmtk::function::utils {
AutoDiffRAII::AutoDiffRAII(size_t size)
    : m_previous_variable_count(DiffScalarBase::getVariableCount())
{
    DiffScalarBase::setVariableCount(size);
}
AutoDiffRAII::~AutoDiffRAII()
{
    DiffScalarBase::setVariableCount(m_previous_variable_count);
}
} // namespace wmtk::function::utils
