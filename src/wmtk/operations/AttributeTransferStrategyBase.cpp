
#include "AttributeTransferStrategyBase.hpp"
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>

#include <wmtk/simplex/utils/unique_homogeneous_simplices.hpp>

namespace wmtk::operations {
    AttributeTransferStrategyBase::~AttributeTransferStrategyBase() = default;


std::vector<Tuple>
AttributeTransferStrategyBase::get_parent_simplices(const Mesh& m, const Mesh& parent, const Simplex& s, PrimitiveType parent_primitive_type)
{
    const PrimitiveType my_primitive_type = s.primitive_type();


    // the set of simplices that can be traversed without crossing a mesh-mesh edge more than once
    std::vector<Tuple> parent_tuples = m.lub_map_tuples(parent, s);

    // lambda for running either of the cases
    auto run = [&](auto&& f) {
        std::vector<Tuple> r;
        for (const auto& parent_tup : parent_tuples) {
            std::vector<Tuple> c = f(parent_tup);
            std::copy(c.begin(), c.end(), std::back_inserter(r));
        }
        if (parent_tuples.size() > 1) {
            simplex::utils::unique_homogeneous_simplices_inline(m, r);
        }

        return r;
    };

    if (my_primitive_type < pt) { // simplex is a face of the parent


        return run[&](const Simplex& a)
        {
            return simplex::cofaces_single_dimension_tuples(mesh, a, pt);
        }
    } else if (my_primitive_type > pt) {
        return run[&](const Simplex& a)
        {
            return simplex::faces_single_dimension_tuples(mesh, a, pt);
        }
    } else {
        return parent_tuples;
    }
}
} // namespace wmtk::operations
