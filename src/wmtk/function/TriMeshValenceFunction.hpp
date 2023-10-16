#include "Function.hpp"
namespace wmtk {
class TriMesh;
namespace function {

class TriMeshValenceFunction : public Function
{
public:
    TriMeshValenceFunction(const TriMesh& mesh);
    double get_value(const Tuple& tuple) const override;

protected:
    const TriMesh& mesh() const;
};
} // namespace function
} // namespace wmtk
