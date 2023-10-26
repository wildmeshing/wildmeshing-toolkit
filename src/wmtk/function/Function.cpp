#include "Function.hpp"
#include <wmtk/simplex/top_level_cofaces.hpp>
namespace wmtk {
namespace function {

Function::Function(std::shared_ptr<PerSimplexFunction>&& function)
    : m_function(function)
{}

Function::~Function() = default;

std::shared_ptr<PerSimplexFunction> Function::get_function() const
{
    return m_function;
}
} // namespace function
} // namespace wmtk