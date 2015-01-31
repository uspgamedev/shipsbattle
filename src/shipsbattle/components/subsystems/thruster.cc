#include <shipsbattle/components/subsystems/thruster.h>

namespace shipsbattle {
namespace components {
namespace subsystems {

Thruster::Thruster(const std::string& name)
: PoweredSystem(name)
{
}

double Thruster::NeedsRecharge() {
    return spent_energy_;
}
void Thruster::OnRecharge(double energy) {
    spent_energy_ -= energy;
}

double Thruster::current_thrust_power() const {
    return thrust_power_ * efficiency_;
}

void Thruster::GenerateThrust(double power, double dt) {
    
    spent_energy_ += energy_consumption_ * dt * power / thrust_power_;
}

} // namespace subsystems
} // namespace components
} // namespace shipsbattle
