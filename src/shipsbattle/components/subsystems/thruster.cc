#include <shipsbattle/components/subsystems/thruster.h>

#include <ugdk/action/3D/component/body.h>
#include <ugdk/action/3D/element.h>

#include <BtOgreExtras.h>

using ugdk::action::mode3d::component::Body;

namespace shipsbattle {
namespace components {
namespace subsystems {

Thruster::Thruster(const std::string& name)
: PoweredSystem(name), thrust_direction_(Ogre::Vector3::ZERO), thrust_power_(10.0), spent_energy_(0.0)
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
    auto axis = thrust_direction_.crossProduct(BtOgre::Convert::toOgre(position()));
    axis.normalise();
    return axis;
}

void Thruster::GenerateThrust(double power, double dt) {
    if (power > current_thrust_power()) power = current_thrust_power();
    if (power < 0.0) return;

    spent_energy_ += energy_consumption_ * dt * power / thrust_power_;

    auto body = parent()->owner()->component<Body>();
    // rotational_axis is in local (ship) coordinates
    // to apply torque to body, the vector must be in global (world) coordinates.
    auto torque = body->orientation() * rotational_axis() * power * dt;
    body->Rotate(torque);
}

} // namespace subsystems
} // namespace components
} // namespace shipsbattle
