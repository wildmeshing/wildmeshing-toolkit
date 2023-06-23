#include <wmtk/operations/SingleTupleTriMeshOperation.h>
namespace wmtk {
void SingleTupleTriMeshOperation::reset()
{
    m_return_tuple_opt.local().reset();
}
SingleTupleTriMeshOperation::~SingleTupleTriMeshOperation() = default;
bool SingleTupleTriMeshOperation::operator()(TriMesh& m, const Tuple& t)
{
    // reset
    mark_failed();
    auto attr_raa = start_protected_attributes_raii(m);
    auto con_raa = start_protected_connectivity_raii(m);

    if (before(m, t)) {
        if (execute(m, t)) { // success should be marked here
            assert(bool(*this));
            if (after(m)) {
                if (invariants(m)) {
                    return true;
                }
            }
        }
    }
    rollback_protected(m);
    mark_failed();
    return false;
}

void SingleTupleTriMeshOperation::set_return_tuple(const Tuple& t)
{
    m_return_tuple_opt = t;
}
void SingleTupleTriMeshOperation::mark_failed()
{
    reset();
}
} // namespace wmtk
