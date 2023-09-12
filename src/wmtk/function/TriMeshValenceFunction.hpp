#include <wmtk/TriMesh.hpp>
#include "Function.hpp"
namespace wmtk {
namespace function {

class TriMeshValenceFunction : public Function
{
public:
    TriMeshValenceFunction(const TriMesh& mesh);
    double get_value(const Tuple& tuple) const override;

protected:
    TriMesh& mesh() const;
};
} // namespace function
} // namespace wmtk