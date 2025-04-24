#pragma once

namespace wmtk::attribute {
class AttributeManager;

/**
 * This handle is a wrapper for the MeshManager scope funtions.
 * It's main task is to enable RAII for attribute scopes.
 *
 */
class AttributeScopeHandle
{
public:
    /**
     * @brief Constructs an `AttributeScopeHandle` for the given `AttributeManager`.
     *
     * The handle creates a new scope for all attributes of the manager.
     */
    AttributeScopeHandle(AttributeManager& manager);
    AttributeScopeHandle(AttributeScopeHandle&&);

    /**
     * @brief Destructor of `AttributeScopeHandle`.
     *
     * Automatically pops the scope for all attributes of the manager. If `mark_failed()` was
     * called, all changes are discarded, otherwise they are applied to the parent scope.
     */
    ~AttributeScopeHandle();

    /**
     * @brief Mark scope as failed.
     *
     * All changes in the scope are discarded upon deletion, i.e. when the scope is popped from the
     * stack.
     */
    void mark_failed();

private:
    AttributeManager& m_manager;
    bool m_failed = false;
    bool m_was_moved = false;
};
} // namespace wmtk::attribute
