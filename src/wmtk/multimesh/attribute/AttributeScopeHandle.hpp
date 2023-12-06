#pragma once
#include <vector>

namespace wmtk::attribute{
    class AttributeScopeHandle;
}
namespace wmtk::multimesh::attribute {
    class AttributeScopeHandle {
        public:
            using single_handle_type = wmtk::attribute::AttributeScopeHandle;
            AttributeScopeHandle(Mesh& m);
            ~AttributeScopeHandle();

            void mark_failed();
        private:
            std::vector<single_scope_handle> m_scopes;
    };
}
