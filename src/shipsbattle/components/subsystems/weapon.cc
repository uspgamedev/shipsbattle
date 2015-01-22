#include <shipsbattle/components/subsystems/weapon.h>
#include <shipsbattle/objects/targets.h>

#include <ugdk/action/3D/component/body.h>
#include <ugdk/action/3D/element.h>

#include <OgreVector3.h>
#include <BtOgreExtras.h>

using ugdk::action::mode3d::component::Body;

namespace shipsbattle {
namespace components {
namespace subsystems {

Weapon::Weapon(const std::string& name)
: PoweredSystem(name), elapsed_(0.0), cooldown_(0.5), energy_(100.0), max_energy_(100.0), firing_angle_(60.0),
    direction_(Ogre::Vector3::UNIT_Z)
{
}

double Weapon::NeedsRecharge() {
    return max_energy_ - energy_;
}
void Weapon::OnRecharge(double energy) {
    set_energy(this->energy() + energy);
}

bool Weapon::CanFire() const {
    return !disabled() && activated() && (elapsed_ <= 0.0);
}

bool Weapon::CanFireAt(const objects::Target& target) {
    if (disabled() || !activated()) return false;

    auto target_pos = BtOgre::Convert::toOgre(target->world_position());
    auto pos = BtOgre::Convert::toOgre(world_position());
    auto dir = parent_->owner()->component<Body>()->orientation() * direction_;
    auto toTarget = target_pos - pos;
    dir.normalise();
    toTarget.normalise();
    return dir.directionEquals(toTarget, Ogre::Degree(firing_angle_));
}
bool Weapon::TryFire(const objects::Target& target) {
    if (CanFireAt(target) && elapsed_ <= 0.0) {
        if (Fire(target)) {
            elapsed_ = cooldown_;
            return true;
        }
    }
    return false;
}

void Weapon::set_energy(double enrgy) {
    energy_ = enrgy;
    if (energy_ > max_energy_) energy_ = max_energy_;
    if (energy_ < 0.0) energy_ = 0.0;
}
void Weapon::set_max_energy(double max_enrgy) {
    max_energy_ = max_enrgy;
    if (max_energy_ < 0.0) max_energy_ = 0.0;
    if (energy_ > max_energy_) energy_ = max_energy_;
}

void Weapon::Update(double dt) {
    if (elapsed_ > 0.0)
        elapsed_ -= dt;
}

} // namespace subsystems
} // namespace components
} // namespace shipsbattle
