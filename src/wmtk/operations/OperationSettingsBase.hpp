#include <memory>
namespace wmtk {
class Mesh;
class InvariantCollection;

namespace operations {
/**
 * Base class for operation settings. All operation settings must inherit from this class.
 * The function `create_inveriants` must initialize the invariants pointer.
 */
class OperationSettingsBase
{
public:
    OperationSettingsBase();

    std::shared_ptr<InvariantCollection> invariants;
    virtual void create_invariants() = 0;
    virtual ~OperationSettingsBase();
};

}
}
