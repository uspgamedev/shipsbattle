#include <shipsbattle/components/subsystems/thruster.h>

#include <ugdk/action/3D/component/body.h>
#include <ugdk/action/3D/element.h>

#include <BtOgreExtras.h>

using ugdk::action::mode3d::component::Body;

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

Ogre::Vector3 Thruster::rotational_axis() {
    auto axis = BtOgre::Convert::toOgre(position()).crossProduct(thrust_direction_);
    axis.normalise();
    return axis;
}

void Thruster::GenerateThrust(double power, double dt) {
    if (power > current_thrust_power()) power = current_thrust_power();
    if (power < 0.0) return;

    spent_energy_ += energy_consumption_ * dt * power / thrust_power_;

    auto body = parent()->owner()->component<Body>();
    auto torque = rotational_axis() * power * dt;
    body->Rotate(torque);
}

} // namespace subsystems
} // namespace components
} // namespace shipsbattle
