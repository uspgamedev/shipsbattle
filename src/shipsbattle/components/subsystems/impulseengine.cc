#include <shipsbattle/components/subsystems/impulseengine.h>

#include <ugdk/action/3D/component/body.h>
#include <ugdk/action/3D/element.h>

#include <BtOgreExtras.h>

using ugdk::action::mode3d::component::Body;

namespace shipsbattle {
namespace components {
namespace subsystems {

ImpulseEngine::ImpulseEngine(const std::string& name)
    : PoweredSystem(name), spent_energy_(0.0)
{
}

double ImpulseEngine::NeedsRecharge() {
    return spent_energy_;
}
void ImpulseEngine::OnRecharge(double energy) {
    spent_energy_ -= energy;
}

double ImpulseEngine::current_exhaust_power() const {
    //CHECK: perhaps also take into account stored energy?
    return exhaust_power_ * efficiency_;
}

Ogre::Vector3 ImpulseEngine::GenerateThrust(double power, double dt) {
    if (power > current_exhaust_power()) power = current_exhaust_power();
    if (power < 0.0) return Ogre::Vector3::ZERO;

    spent_energy_ += energy_consumption_ * dt * power / exhaust_power_;

    auto body = parent()->owner()->component<Body>();
    auto impulse = current_direction_ * power * dt;
    auto pos = BtOgre::Convert::toOgre(position());
    body->ApplyImpulse(impulse, pos);
    return pos.crossProduct(impulse); //generated torque (check Bullet's apply impulse to check it out)
}

} // namespace subsystems
} // namespace components
} // namespace shipsbattle
