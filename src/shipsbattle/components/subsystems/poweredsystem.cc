#include <shipsbattle/components/subsystems/poweredsystem.h>
#include <shipsbattle/components/powersystem.h>
#include <ugdk/action/3D/element.h>



namespace shipsbattle {
namespace components {
namespace subsystems {

PoweredSystem::PoweredSystem(const std::string& name) 
    : DamageableSystem(name), activated_(true), efficiency_(1.0), energy_consumption_(1.0)
{

}

void PoweredSystem::RegisterToPowerSystem(PowerSystem* power) {
    power->RegisterPoweredSystem(this);
}

void PoweredSystem::OnRegister() {
    parent_->owner()->component<PowerSystem>()->RegisterPoweredSystem(this);
}

} // namespace subsystems
} // namespace components
} // namespace shipsbattle
