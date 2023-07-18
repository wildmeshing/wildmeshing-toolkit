#pragma once
#include <tbb/enumerable_thread_specific.h>
namespace wmtk {
class Mesh;
class AttributeScope;


// Maintains per-thread stacks of attribute scopes
// Should only be constructed and manipulated by the mesh class
class AttributeScopeManager
{
    friend class Mesh;
    AttributeScopeManager();
    // returns nullptr if the scope does not exist
    AttributeScope* get_scope();
    AttributeScopeHandle create_scope(Mesh& m);
};
} // namespace wmtk
