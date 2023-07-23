#include "FlagModifier.hpp"
#include <wmtk/attribute/AccessorBase.hpp>
#include "SimplexFlag.hpp"


namespace wmtk{

FlagModifier::FlagModifier(const AccessorBase<char>& flag_accessor)
    : m_accessor(flag_accessor)
{}
void FlagModifier::activate(long index) 
{
    activate_flag<SimplexFlag::Active>(mutable_accessor().scalar_attribute(index));
}
void FlagModifier::deactivate(long index) 
{
    deactivate_flag<SimplexFlag::Active>(mutable_accessor().scalar_attribute(index));
}

} // namespace wmtk::utils
