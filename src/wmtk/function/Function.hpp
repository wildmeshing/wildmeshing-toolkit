#pragma once
#include <wmtk/Mesh.hpp>
#include <wmtk/Tuple.hpp>
namespace wmtk {
namespace function {
class Function
{
private:
    const Mesh& m_mesh;


public:
    Function(const Mesh& mesh);
    virtual ~Function();

    const Mesh& mesh() const;
    double get_one_ring_value(const Tuple& vertex) const;
    double get_value_sum(const std::vector<Tuple>& top_level_simplices) const;

public:
    // evaluate the function on the top level simplex of the tuple
    virtual double get_value(const Tuple& top_level_simplex) const = 0;
};
} // namespace function
} // namespace wmtk
