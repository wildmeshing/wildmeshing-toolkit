#pragma once
#include "Invariant.hpp"


namespace wmtk {
    class Mesh;

    class MeshInvariant: public Invariant {
        public:
            MeshInvariant(const Mesh& m);

        const Mesh& mesh() const;
        private:
        const Mesh& m_mesh;
    };
}
