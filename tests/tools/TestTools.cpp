#include <wmtk/Mesh.hpp>

#include "TestTools.hpp"


namespace wmtk::tests::tools {
int64_t TestTools::global_id(const Mesh& m, const Tuple& a, const PrimitiveType& pt)
{
    return m.id(a, pt);
}
} // namespace wmtk::tests::tools
