#pragma once
#include "UpdateStrategy.hpp"

namespace wmtk::operations::attribute {
class CollapseUpdateStrategy : public UpdateStrategy
{
public:
    void update(const Tuple& a, const Tuple& b) override;

    virtual void assign(const Tuple& a, const Tuple& b, const Tuple& c);

private:
};
} // namespace wmtk::operations::attribute
