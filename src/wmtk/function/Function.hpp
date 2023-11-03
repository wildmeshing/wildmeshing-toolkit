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
    // evaluate the function on the top level simplex of the tuple
    virtual double get_value(const simplex::Simplex& simplex) const = 0;
    virtual const Mesh& mesh() const = 0;
};
} // namespace wmtk::function
