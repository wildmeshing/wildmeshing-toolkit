#include <wmtk/TriMeshOperation.h>
#include <wmtk/utils/VectorUtils.h>
using namespace wmtk;

Operation::Operation() = default;
Operation::~Operation() = default;


bool Operation::operator()()
{
    auto attr_raa = start_protected_attributes_raii(m);
    auto con_raa = start_protected_connectivity_raii(m);

    if (before(m, t)) {
        if (execute(m, t)) { // success should be marked here
            if (after(m)) {
                if (invariants(m)) {
                    return true;
                }
            }
        }
    }
    m.rollback_protected();
    return false;
}



