#pragma once
#include "AttributeTransferStrategyBase.hpp"

namespace wmtk::operations {
    // someday this will store and process a DAG of attribute update objects / linearize the attribute update stuff
    class AttributeTransferStrategyBase;
    class AttributeUpdateStrategyCollection//: public AttributeTransferStrategyBase {
        public:


    bool matches_handle(const wmtk::attribute::MeshAttributeHandleVariant& attr, const simplex::Simplex& s) const override;
    bool run(const simplex::Simplex& s) override;

        private:
            std::vector<std::shared_ptr<AttributeTransferStrategyBase>> m_strategies;
            Mesh& m_mesh;

    };
}
