#pragma once
namespace wmtk {
    class Mesh;
}

namespace wmtk::multimesh {

    class Mappable {
        public:
            virtual const Mesh& mesh() const = 0;
    };
}
