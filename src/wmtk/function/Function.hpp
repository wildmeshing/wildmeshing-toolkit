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
class Function
{
public:
    virtual ~Function();
    // Defined as f(x) where the variable x is a simplex
    virtual double get_value(const simplex::Simplex& variable_simplex) const = 0;
    virtual const Mesh& mesh() const = 0;
};
} // namespace wmtk::function
