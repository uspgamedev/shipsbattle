#include <shipsbattle/components/subsystems/impulseengine.h>

namespace shipsbattle {
namespace components {
namespace subsystems {

ImpulseEngine::ImpulseEngine(const std::string& name)
: PoweredSystem(name)
{
}

double ImpulseEngine::NeedsRecharge() {
    return 1;
}
void ImpulseEngine::OnRecharge(double energy) {
    
}

void ImpulseEngine::Update(double dt) {
    
}

} // namespace subsystems
} // namespace components
} // namespace shipsbattle
