#pragma once

#include <wmtk/Accessor.hpp>
#include <wmtk/simplex/Simplex.hpp>

namespace wmtk::function {

class Function
{
public:
    Function();
    virtual ~Function() {}

    /**
     * @brief Given a function f(x), get_value evaluate the function at the input simplex.
     *
     * @param variable_simplex The input simplex. f(x) is defined wrt this argument.
     * @return double The value of the function at the input simplex.
     */
    virtual double get_value(const simplex::Simplex& variable_simplex) const = 0;

    virtual PrimitiveType primitive_type() const = 0;

    Mesh& mesh();
    virtual const Mesh& mesh() const = 0;
};
} // namespace wmtk::function
