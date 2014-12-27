#include <shipsbattle/components/subsystems/damageablesystem.h>
#include <shipsbattle/components/hull.h>

namespace shipsbattle {
namespace components {
namespace subsystems {

DamageableSystem::DamageableSystem(const std::string& name) : name_(name),
    max_hitpoints_(1000), hitpoints_(1000.0), armor_rating_(0.0), 
    required_(false), position_(Ogre::Vector3::ZERO), radius_(0.1) 
{

}

void DamageableSystem::set_position(const Ogre::Vector3& pos) {
    //FIXME: we should check if pos is inside the mesh
    position_ = pos; 
}

void DamageableSystem::TakeDamage(double dmg, double piercing) {
    bool is_dmg = (dmg >= 0.0);
    dmg = (is_dmg) ? dmg : -dmg;
    double armor = armor_rating_ * (1.0 - piercing); //TODO: check
    if (armor > dmg) armor = dmg;
    double actual_dmg = dmg - armor;
    double armor_dmg = max_armor_rating_*(dmg / max_hitpoints_); //TODO: check

    if (!is_dmg) {
        actual_dmg = -actual_dmg;
        armor_dmg = -armor_dmg;
    }
    armor_rating_ -= armor_dmg;
    if (armor_rating_ > max_armor_rating_) armor_rating_ = max_armor_rating_;
    if (armor_rating_ < 0.0) armor_rating_ = 0.0;

    hitpoints_ -= actual_dmg;
    if (hitpoints_ > max_hitpoints_) hitpoints_ = max_hitpoints_;
    if (hitpoints_ < 0.0) {
        //TODO: KABUM
    }
}

void DamageableSystem::RegisterToHull(Hull* hull) {
    hull->RegisterDamageableSystem(this);
}

} // namespace subsystems
} // namespace components
} // namespace shipsbattle
