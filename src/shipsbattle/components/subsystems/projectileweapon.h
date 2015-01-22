#ifndef SHIPSBATTLE_SUBSYSTEMS_PROJECTILEWEAPON_H
#define SHIPSBATTLE_SUBSYSTEMS_PROJECTILEWEAPON_H

#include <shipsbattle/components/subsystems/weapon.h>
#include <shipsbattle/objects/projectilemodel.h>

namespace shipsbattle {
namespace components {
namespace subsystems {

class ProjectileWeapon : public Weapon {
public:
    ProjectileWeapon(const std::string& name);

    bool Fire(const objects::Target& target) override;

    /** Gets the projectile model this weapon uses to fire. */
    objects::ProjectileModel projectile() const { return projectile_; }
    void set_projectile(const objects::ProjectileModel& model) { projectile_ = model; }
    /** Gets the speed at which the projectile is launched from this weapon. */
    double launching_speed() const { return launching_speed_; }
    void set_launching_speed(double speed) { launching_speed_ = speed; }

protected:
    objects::ProjectileModel projectile_;
    double launching_speed_;
};


} // namespace subsystems
} // namespace components
} // namespace shipsbattle

#endif // SHIPSBATTLE_SUBSYSTEMS_PROJECTILEWEAPON_H
