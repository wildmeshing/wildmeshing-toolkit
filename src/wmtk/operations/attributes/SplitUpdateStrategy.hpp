#pragma once
#include "UpdateStrategy.hpp"


namespace wmtk::operations::attribute {

class SplitUpdateStrategy : public UpdateStrategy
{
public:
    void update(const Tuple& a, const Tuple& b) override;

private:
};
} // namespace wmtk::operations::attribute
