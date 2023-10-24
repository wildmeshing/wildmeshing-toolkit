#include "Function.hpp"
#include <wmtk/simplex/top_level_cofaces.hpp>
namespace wmtk {
namespace function {

Function::Function(std::unique_ptr<PerSimplexFunction>&& function)
    : m_function(std::move(function))
{}

Function::~Function() = default;


} // namespace function
} // namespace wmtk