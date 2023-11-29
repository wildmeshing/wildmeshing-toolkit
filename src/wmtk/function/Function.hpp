#pragma once
#include <memory>
#include <wmtk/simplex/Simplex.hpp>
namespace wmtk {
class Mesh;
namespace simplex {
class Simplex;
}

} // namespace wmtk
namespace wmtk::function {
/**
 * @brief The base class for functions defined wrt attributes on a mesh
 *
 */
class Function
{
public:
    virtual ~Function();
    /**
     * @brief Given a function f(x), get_value evaluate the function at the input simplex.
     *
     * @param variable_simplex The input simplex. f(x) is defined wrt this argument.
     * @return double The value of the function at the input simplex.
     */
    virtual double get_value(const simplex::Simplex& variable_simplex) const = 0;
    virtual const Mesh& mesh() const = 0;
};
} // namespace wmtk::function
