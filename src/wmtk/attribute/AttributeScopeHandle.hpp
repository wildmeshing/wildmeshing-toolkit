#pragma once

namespace wmtk {
    class AttributeScopeManager;
    class AttributeScopeHandle {
        public:
            AttributeScopeHandle(AttributeScopeManager& scope_manager);
            ~AttributeScopeHandle();
        private:
            AttributeScopeManager& m_scope_manager;

    };
}
