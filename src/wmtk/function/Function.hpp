#pragma once
#include <memory>
#include <wmtk/Primitive.hpp>
#include "PerSimplexFunction.hpp"
namespace wmtk {
namespace function {
class Function
{
public:
    Function(std::shared_ptr<PerSimplexFunction>&& function);
    virtual ~Function();

public:
    // evaluate the function on the top level simplex of the tuple
    virtual double get_value(const Simplex& simplex) const = 0;
    std::shared_ptr<PerSimplexFunction> get_function() const;

private:
    std::shared_ptr<PerSimplexFunction> m_function;
};
} // namespace function
} // namespace wmtk
