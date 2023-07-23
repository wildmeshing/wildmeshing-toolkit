#pragma once
#include "FlagInspector.hpp"

namespace wmtk::utils {
class FlagModifier : public FlagInspector
{
    FlagModifier(AccessorBase<char>& flag_accessor);

    bool activate(long index) const;
    bool deactivate(long index) const;


private:
    AccessorBase<char>& mutable_accessor();
};
} // namespace wmtk::utils
