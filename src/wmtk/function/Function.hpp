#pragma once
#include <wmtk/Mesh.hpp>
#include <wmtk/Tuple.hpp>
namespace wmtk {
namespace function {
class Function
{
protected:
    const Mesh& m_mesh;


public:
    Function(const Mesh& mesh);
    virtual ~Function();

public:
    // evaluate the function on the top level simplex of the tuple
    virtual double get_value(const Tuple& tuple) const = 0;
};
} // namespace function
} // namespace wmtk