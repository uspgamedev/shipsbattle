#include <shipsbattle/components/subsystems/thruster.h>

namespace shipsbattle {
namespace components {
namespace subsystems {

Thruster::Thruster(const std::string& name)
: PoweredSystem(name)
{
}

double Thruster::NeedsRecharge() {
    return 1;
}
void Thruster::OnRecharge(double energy) {

}

void Thruster::Update(double dt) {
    
}

} // namespace subsystems
} // namespace components
} // namespace shipsbattle
