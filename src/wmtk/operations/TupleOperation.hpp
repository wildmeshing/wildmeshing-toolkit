#include "Operation.hpp"
namespace wmtk {
class TupleOperation : public Operation
{
    TpleOperation(Mesh& m, const Tuple& t);

protected:
    Tuple m_input_tuple;
};
} // namespace wmtk
