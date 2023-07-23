#pragma once
#include <wmtk/Primitive.hpp>

namespace wmtk {
class Mesh;
template <typename T>
class AccessorBase;
} // namespace wmtk
namespace wmtk{
class FlagInspector: private AccessorBase<char>
{
public:
    FlagInspector(const AccessorBase<char>& flag_accessor);
    // number of simplices for which we have flags
    //long size() const;

    // the total number of simplices that are registered as active
    long active_simplex_count() const;

    // the index of the largest active simplex, could theoretically shrink size() to
    // `longest_active_simplex_index() + 1`.
    long largest_active_simplex_index() const;


protected:
    bool is_active(long index) const;
    using AccessorBase::scalar_attribute;
    using AccessorBase::const_scalar_attribute;


};
} // namespace wmtk::utils
