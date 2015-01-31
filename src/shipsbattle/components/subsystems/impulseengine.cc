#include <shipsbattle/components/subsystems/impulseengine.h>

#include <ugdk/action/3D/component/body.h>
#include <ugdk/action/3D/element.h>

#include <BtOgreExtras.h>

using ugdk::action::mode3d::component::Body;

namespace shipsbattle {
namespace components {
namespace subsystems {

ImpulseEngine::ImpulseEngine(const std::string& name)
: PoweredSystem(name), exhaust_direction_(Ogre::Vector3::NEGATIVE_UNIT_Z), exhaust_angle_(45.0), 
exhaust_power_(100.0), current_direction_(Ogre::Vector3::NEGATIVE_UNIT_Z), spent_energy_(0.0)
{
}

double ImpulseEngine::NeedsRecharge() {
    return spent_energy_;
}
void ImpulseEngine::OnRecharge(double energy) {
    spent_energy_ -= energy;
}

Ogre::Vector3 ImpulseEngine::GetVectorClosestTo(const Ogre::Vector3& dir) {
    //CHECK: when (not if) we refactor this subsystem to have a generic thrust direction region,
    // this method will need to be generalized into this 'generic region' object.
    if (exhaust_direction_.angleBetween(dir).valueRadians() <= exhaust_angle_)
        return dir;

    auto exhaust_dir = BtOgre::Convert::toBullet(exhaust_direction_);
    auto axis = exhaust_dir.cross(BtOgre::Convert::toBullet(dir));
    axis.normalize();
    auto path = BtOgre::Convert::toOgre(exhaust_dir.rotate(axis, static_cast<btScalar>(exhaust_angle_)));
    path.normalise();
    return path;
}

void ImpulseEngine::set_exhaust_direction(const Ogre::Vector3& dir) {
    exhaust_direction_ = dir.normalisedCopy();
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
    auto impulse = -current_direction_ * power * dt;
    auto pos = BtOgre::Convert::toOgre(position());
    body->ApplyImpulse(impulse, pos);
    return pos.crossProduct(impulse); //generated torque (check Bullet's apply impulse to check it out)
}

} // namespace subsystems
} // namespace components
} // namespace shipsbattle
